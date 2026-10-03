"""Pick and place skills built on Intrinsic planning + LIBERO execution.

Pick:  sync world -> grasp candidates (object-relative) -> Intrinsic IK feasibility of the
       pre-grasp -> Intrinsic PlanTrajectory (free-space, collision checked) to the pre-grasp ->
       execute -> Intrinsic LINEAR plan for the approach (contact with the target object is the
       only permitted contact) -> execute -> close gripper -> grasp verification (finger gap +
       pad contacts) -> attach object in the Intrinsic world -> LINEAR lift with the attached
       object -> verify the object moved with the tcp.
Place: object-relative target -> tcp target via the grasp transform -> plan transport
       (attached object is part of the robot for collision checking) -> LINEAR lowering with a
       contact monitor (stops when the carried object touches its support) -> open gripper ->
       detach -> LINEAR retreat.
"""
from __future__ import annotations

import dataclasses
import os
import re
from typing import Callable, List, Optional, Sequence, Tuple

# Task-3 experiment switch (off: the protocol evaluation of 4c20556 showed pick regressions on
# tasks 1 and 7 while the wider yaw assessment and tilted-for-placement candidates were on)
TILTED_FOR_PLACEMENT = False

import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.env.executor import ExecutionConfig
from libero_intrinsic.intrinsic.client import IntrinsicRequestError
from libero_intrinsic.model import transforms as tf
from libero_intrinsic.skills import geometry as geo
from libero_intrinsic.skills import placement
from libero_intrinsic.skills.base import Skill, SkillContext, SkillResult


@dataclasses.dataclass
class PickParams:
    height_fraction: float = 0.5
    pregrasp_clearance: float = 0.10
    lift_height: float = 0.10
    yaws: Sequence[float] = ()
    max_candidates: int = 8
    approach_dir: Optional[np.ndarray] = None   # side grasps: required insertion direction (roofed container opening)
    close_steps: int = 12
    min_finger_gap: float = 0.004      # summed |finger q| after closing must exceed this
    min_pad_contacts: int = 1
    plan_timeout_s: float = 15.0
    z_offset: float = 0.0
    max_depth: float = 0.045
    open_clearance: float = 0.024      # gripper pre-opening = object width + this (1 cm per control step)
    side_grasp: bool = False           # use horizontal-approach candidates (object goes into a roofed container)


def support_of(env, sync, body: str) -> List[str]:
    """Bodies the object rests on (their top is just below the object's bottom and their footprint
    overlaps it); fingers may touch them while grasping thin objects."""
    box = geo.object_box(env, body)
    out = []
    for b in sync.bodies:
        if b == body:
            continue
        ob = geo.object_box(env, b)
        if -0.005 <= box.bottom_z - ob.top_z <= 0.02 and np.all(np.abs(ob.center_world[:2] - box.center_world[:2]) <= ob.half_extents_world[:2] + box.half_extents_world[:2]):
            out.append(b)
    return out


def nearest_ik(ctx: SkillContext, pos, rot, q_now, cs, n: int = 8, joint_limits=None, posture: str = "nearest"):
    """Intrinsic IK solutions for the pose (collision checked with `cs`), sorted by weighted
    joint distance to q_now (proximal joints weigh more: less arm reconfiguration, which the
    OSC controller tracks far better). `joint_limits` (lo, hi) adds a JointPositionLimits
    constraint to the IK target (posture restriction, e.g. shoulder forward / elbow up)."""
    sols, rid, lat = ctx.client.ik(pos, rot, q_now, max_solutions=n, collision_settings=cs, joint_limits=joint_limits)
    lim = ctx.env.joint_limits()
    safe = [q for q in sols if np.all(np.asarray(q) > lim[:, 0] + 0.06) and np.all(np.asarray(q) < lim[:, 1] - 0.06)]
    sols = safe or sols
    w = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.5])
    if posture == "margin":
        # posture quality: largest distance to the joint limits first (a following LINEAR segment
        # needs room; Intrinsic's linear planner fails near limits/singularities with
        # "Could not solve FinePathIK without excessive change in joint config")
        sols.sort(key=lambda q: -float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q)))))
    else:
        sols.sort(key=lambda q: float(np.sum(w * (np.asarray(q) - q_now) ** 2)))
    return sols, rid, lat


_START_STATE_RE = re.compile(r"Invalid initial joint configuration.*?Left object: (\S+)[^\n]*\n\s*Right objects?: (\S+)", re.S)


def _recover_start_state(ctx: SkillContext, label: str, err: IntrinsicRequestError, cs):
    """Bounded recovery when a plan is rejected because the CURRENT configuration touches an
    object (e.g. a finger still in contact with the object just released, or a pushed body):
    retreat 5 cm straight up with only that contact pair excluded, re-sync, and let the caller
    re-plan once. Returns True when a retreat was executed."""
    m = _START_STATE_RE.search(str(err))
    if not m:
        return False
    a, b = m.group(1).replace("panda.", "").split(".")[0], m.group(2).split(".")[0]
    sync = ctx.sync
    movable = set(sync.bodies) | set(sync.fingers.values()) | {sync.client.robot}
    if b not in movable and a not in movable:
        return False
    rs = ctx.env.robot_state()
    from libero_intrinsic.intrinsic.client import collision_settings
    pairs = sync._base_pairs() + [(sync.client.robot, b), (b, sync.client.robot)] + [(f, b) for f in sync.fingers.values()]
    cs_r = collision_settings(pairs, resolver=sync.client.oref)
    ctx.record(event="start_state_recovery", label=label, pair=[a, b])
    try:
        traj = ctx.client.plan_to_pose(rs.q, rs.tcp_pos + np.array([0, 0, 0.05]), rs.tcp_rot, collision_settings=cs_r,
                                       motion_type="LINEAR", timeout_s=5.0, caller_id=label + "_recover")
        ctx.execute(traj, ExecutionConfig(gripper=0.0), label + "_recover")
    except IntrinsicRequestError as e2:
        # near a joint limit the linear retreat has no path: configuration-space retreat to the
        # nearest IK solution of the same raised pose (same contact pair excluded)
        try:
            sols, _, _ = ctx.client.ik(rs.tcp_pos + np.array([0, 0, 0.05]), rs.tcp_rot, rs.q, max_solutions=8, collision_settings=cs_r)
            w = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.5])
            sols.sort(key=lambda q: float(np.sum(w * (np.asarray(q) - rs.q) ** 2)))
            traj = ctx.client.plan_to_joints(rs.q, sols[0], collision_settings=cs_r, motion_type="ANY", timeout_s=5.0, caller_id=label + "_recover_any")
            ctx.record(event="start_state_recovery_any", label=label)
            ctx.execute(traj, ExecutionConfig(gripper=0.0), label + "_recover_any")
        except (IntrinsicRequestError, IndexError) as e3:
            ctx.record(event="start_state_recovery_failed", label=label, reason=str(e2)[:120] + " | any: " + str(e3)[:80])
            return False
    sync.sync()
    return True


def _plan_and_execute(ctx: SkillContext, label: str, q_start, pos, rot, cs, motion_type="ANY",
                      gripper=-1.0, timeout_s=15.0, contact_monitor=None, time_scale=1.0, joint_limits=None,
                      _retry=True, posture: str = "nearest"):
    try:
        if motion_type == "ANY":
            # free-space motion: plan to the IK solution nearest the current configuration (joint target)
            sols, rid, lat = nearest_ik(ctx, pos, rot, np.asarray(q_start), cs, joint_limits=joint_limits, posture=posture)
            ctx.record(event="ik", label=label + "_goal", n_solutions=len(sols), request_id=rid, latency_s=lat)
            if not sols:
                raise IntrinsicRequestError("ComputeIk", "NOT_FOUND", f"no collision-free IK for {label}", rid)
            traj = ctx.client.plan_to_joints(q_start, sols[0], collision_settings=cs, motion_type="ANY",
                                             timeout_s=timeout_s, caller_id=label)
        else:
            traj = ctx.client.plan_to_pose(q_start, pos, rot, collision_settings=cs, motion_type=motion_type,
                                           timeout_s=timeout_s, caller_id=label)
    except IntrinsicRequestError as e:
        if _retry and "Invalid initial joint configuration" in str(e) and _recover_start_state(ctx, label, e, cs):
            rs = ctx.env.robot_state()
            return _plan_and_execute(ctx, label, rs.q, pos, rot, cs, motion_type, gripper, timeout_s, contact_monitor,
                                     time_scale, joint_limits, _retry=False, posture=posture)
        raise
    ctx.record(event="plan", label=label, trajectory_id=traj.request_id, motion_type=motion_type,
               n_states=int(len(traj.t)), duration_s=traj.duration, latency_s=traj.planning_latency_s,
               target_pos=[float(v) for v in pos])
    cfg = ExecutionConfig(gripper=gripper, contact_monitor=contact_monitor, time_scale=time_scale)
    res = ctx.execute(traj, cfg, label)
    # Post-execution audit: the OSC controller's null-space drift means the executed joint path
    # differs from the planned one; ask Intrinsic whether the configurations actually reached
    # were collision-free under the same collision settings (unintended-collision metric).
    if res.executed_q is not None and len(res.executed_q):
        try:
            sub = res.executed_q[:: max(1, len(res.executed_q) // 20)]
            coll, msg, rid = ctx.client.check_collisions(sub, cs)
            ctx.record(event="executed_path_audit", label=label, trajectory_id=traj.request_id, n_checked=int(len(sub)),
                       collision=bool(coll), msg=msg[:200], request_id=rid)
        except IntrinsicRequestError as e:
            ctx.record(event="executed_path_audit", label=label, error=str(e)[:200])
    return traj, res


class PickSkill(Skill):
    name = "pick"
    max_attempts = 3

    def __init__(self, body: str, params: PickParams = PickParams()):
        self.body = body
        self.p = params
        self._tried: List[str] = []
        self.place_skill = None   # the PlaceSkill that follows (placement-aware grasp ranking), set by the planner
        self.assess_only = False  # when True, attempt() stops after candidate generation/ranking (no motion)
        self.last_assessment = {}

    def preconditions(self, ctx):
        if ctx.sync.attached_bodies():
            return f"gripper already holding {ctx.sync.attached_bodies()}"
        return None

    _last_width = 0.05

    def recover(self, ctx, last):
        # open just enough to free the object (fingers stay clear of neighbours), then withdraw
        ctx.open_gripper(int(np.ceil(min(geo.GRIPPER_MAX_OPENING, self._last_width + self.p.open_clearance) / 0.01)))
        for b in ctx.sync.attached_bodies():
            ctx.sync.detach(b)
        rs = ctx.env.robot_state()
        ctx.sync.sync()
        # retreat straight up 8 cm (LINEAR, target object + support contact allowed)
        try:
            _plan_and_execute(ctx, "pick_recover_retreat", rs.q, rs.tcp_pos + [0, 0, 0.08], rs.tcp_rot,
                              ctx.sync.grasp_collision_settings(self.body, support_bodies=support_of(ctx.env, ctx.sync, self.body)),
                              motion_type="LINEAR", gripper=0.0, timeout_s=5.0)
        except IntrinsicRequestError as e:
            ctx.record(event="recover_plan_failed", reason=str(e))

    def attempt(self, ctx, i):
        env, client, sync = ctx.env, ctx.client, ctx.sync
        sync.sync()
        if self.p.side_grasp:
            cands = [c for c in geo.side_grasp_candidates(env, self.body, approach_dir=self.p.approach_dir) if c.label not in self._tried]
            # reachability ordering: horizontal approaches pointing away from the robot base are the
            # ones the Panda can realise (probe: approach directions facing the base have no IK)
            base = env.robot_state().base_pos
            d = env.body_pose(self.body)[0][:2] - base[:2]
            d = d / (np.linalg.norm(d) + 1e-9)
            for c in cands:
                c.score += 0.08 * float(np.dot(c.approach[:2], d))
            cands.sort(key=lambda c: -c.score)
            pre_clear = 0.06
        else:
            cands = [c for c in geo.grasp_candidates(env, self.body, yaws=self.p.yaws) if c.label not in self._tried]
            pre_clear = self.p.pregrasp_clearance
        if not cands:
            return SkillResult(self.name, False, "no_grasp_candidate")
        # scene-level clearance: fingers (opened to width + clearance) and hand must not hit neighbours
        others = [b for b in sync.bodies if b != self.body]

        supports = support_of(env, sync, self.body)
        plane_z = max((geo.object_box(env, b).top_z for b in supports), default=-np.inf)

        obj_top = geo.object_box(env, self.body).top_z
        blockers = {}

        def clear(cs):
            out = []
            for c in cs:
                opening = min(geo.GRIPPER_MAX_OPENING, c.width + self.p.open_clearance)
                # exact support-plane test (the coarse point cloud of a large table misses thin
                # overlaps): a candidate whose gripper corners dip below the plane is raised by the
                # deficit as long as the pads stay on the object (tilted grasps of low objects)
                # (the finger/pad zones carry +3..5 mm margins, so 4 mm of zone below the plane is tolerated)
                low = float((geo.gripper_corners_tcp(opening) @ c.rot.T + c.pos)[:, 2].min())
                if low < plane_z - 0.004:
                    lift = plane_z - 0.004 - low
                    if c.pos[2] + lift > obj_top - 0.004:   # the pads (tcp-16mm..tcp+10mm) must still overlap the object
                        continue
                    c.pos = c.pos + np.array([0.0, 0.0, lift])
                    c.label = c.label + f"_up{lift * 1000:.0f}"
                ok, blocker = geo.scene_clearance(env, c, opening, others)
                if ok:  # also at the pre-grasp pose (hand higher up, e.g. above a container rim)
                    pre = c.pos - c.approach * pre_clear
                    ok, blocker = geo.hand_clearance(env, pre, c.rot, opening, 0.0, others)
                if ok:
                    out.append(c)
                elif blocker:
                    blockers[blocker] = blockers.get(blocker, 0) + 1
            return out
        cleared = clear(cands)
        n_tilted = 0
        if not cleared:
            # neighbours block every vertical grasp: try approaches tilted away from them
            tilted = [c for c in geo.grasp_candidates(env, self.body, yaws=self.p.yaws, tilts=(np.radians(20), np.radians(35)))
                      if c.label not in self._tried and "tilt" in c.label]
            n_tilted = len(tilted)
            cleared = clear(tilted)
        ctx.record(event="grasp_candidates", n_total=len(cands), n_cleared=len(cleared), n_tilted=n_tilted,
                   blockers=dict(sorted(blockers.items(), key=lambda kv: -kv[1])[:6]))
        ctx.params["last_pick_blockers"] = [b for b, _ in sorted(blockers.items(), key=lambda kv: -kv[1])]
        cands = cleared or cands
        if self.place_skill is not None and len(cands) > 1:
            # placement-aware ranking: grasps that admit a geometrically valid release pose at the
            # placement target come first (e.g. a bowl going into a drawer under a cabinet must be
            # held so that the wide hand stays in front of the cabinet), ties keep the grasp score
            compat, whys = [], []

            def assess(pool):
                for c in pool:
                    try:
                        ok, drop = self.place_skill.grasp_compatible(env, sync, c.pos, c.rot)
                        if not ok:
                            whys.append((c.label, getattr(self.place_skill, "last_compat_why", "")))
                    except Exception as e:   # geometry failure must not block picking
                        ctx.record(event="grasp_placement_check_error", reason=str(e)[:200])
                        ok, drop = True, 0.0
                    compat.append((c, ok, drop))
            # assess enough candidates to cover every yaw family (the score order prefers yaws near 0,
            # but the placement may need the wide hand axis along one direction: task 3 yaw-90 family)
            n_assess = max(self.p.max_candidates, 48 if self.p.side_grasp else 24)
            assess(cands[:n_assess])
            rest = cands[n_assess:]
            if TILTED_FOR_PLACEMENT and not self.p.side_grasp and not any(ok for _, ok, _ in compat) and not any("tilt" in c.label for c, _, _ in compat):
                # no level grasp admits a valid release at the target (e.g. the hand would hit the
                # cabinet above the drawer): a pitched hand carries the object level but keeps the
                # palm away from the obstacle, so tilted pinch grasps join the assessment
                tilted = [c for c in geo.grasp_candidates(env, self.body, yaws=self.p.yaws, tilts=(np.radians(20), np.radians(35)))
                          if c.label not in self._tried and "tilt" in c.label]
                tilted = clear(tilted)
                ctx.record(event="grasp_tilted_for_placement", n_tilted=len(tilted))
                assess(tilted[:32])
            # feasible grasps first, ordered by the drop height after release (1 cm bins: a grasp
            # that lets the object hang deeper into a container is released closer to the floor),
            # ties keep the grasp score order
            def rank_cost(t):
                c, _, d = t
                cost = round(max(d, 0.0), 2)
                if self.p.approach_dir is not None:   # insertion: prefer approaches aligned with the opening normal
                    a = np.asarray(c.approach[:2]); a = a / (np.linalg.norm(a) + 1e-9)
                    cost += 0.1 * float(np.arccos(np.clip(np.dot(a, np.asarray(self.p.approach_dir[:2])), -1, 1)))
                return cost
            good = sorted([t for t in compat if t[1]], key=rank_cost)
            cands = [c for c, _, _ in good] + [c for c, ok, _ in compat if not ok] + rest
            ctx.record(event="grasp_placement_filter", n_checked=len(compat), n_compatible=len(good),
                       first=cands[0].label if cands else "", first_drop_m=float(good[0][2]) if good else None,
                       incompatible_examples=whys[:6], ranked=[(c.label, round(float(d), 3)) for c, _, d in good[:8]])
            self.last_assessment = {"n_cleared": len(cleared), "n_compatible": len(good), "blockers": dict(blockers)}
        if self.assess_only:
            return SkillResult(self.name, False, "assessed", details=dict(self.last_assessment))
        rs = env.robot_state()
        free_cs = sync.free_collision_settings()
        grasp_cs = sync.grasp_collision_settings(self.body, support_bodies=support_of(env, sync, self.body))
        chosen = None
        for c in cands[: self.p.max_candidates]:
            # pre-open the gripper only as far as needed (1 cm per step): keeps the fingers clear of neighbours
            opening = min(geo.GRIPPER_MAX_OPENING, c.width + self.p.open_clearance)
            ctx.close_gripper(9)
            ctx.open_gripper(int(np.ceil(opening / 0.01)))
            sync.sync()
            pre = c.pos - c.approach * pre_clear
            # an IK failure (no solution / only colliding solutions) rejects this candidate only
            try:
                sols, rid, lat = client.ik(pre, c.rot, rs.q, max_solutions=4, collision_settings=free_cs)
            except IntrinsicRequestError as e:
                ctx.record(event="ik", label="pregrasp", grasp=c.label, n_solutions=0, request_id=e.request_id, error=str(e)[:160])
                continue
            ctx.record(event="ik", label="pregrasp", grasp=c.label, n_solutions=len(sols), request_id=rid, latency_s=lat)
            if not sols:
                continue
            try:
                sols2, rid2, lat2 = client.ik(c.pos, c.rot, sols[0], max_solutions=4, collision_settings=grasp_cs)
            except IntrinsicRequestError as e:
                ctx.record(event="ik", label="grasp", grasp=c.label, n_solutions=0, request_id=e.request_id, error=str(e)[:160])
                continue
            ctx.record(event="ik", label="grasp", grasp=c.label, n_solutions=len(sols2), request_id=rid2, latency_s=lat2)
            if not sols2:
                continue
            # Continuity: a feasible grasp endpoint is not a feasible approach path. Among the
            # pre-grasp solutions (plus a home-seeded set), take the first whose LINEAR plan to
            # the grasp pose Intrinsic can produce (dry run, not executed); the free-space motion
            # then targets that joint configuration. (Observed: "FinePathIK ... excessive change
            # in joint config" on the approach when the nearest pre-grasp solution was used.)
            pool = list(sols)
            if "home_q" in ctx.params:
                try:
                    more, _, _ = client.ik(pre, c.rot, ctx.params["home_q"], max_solutions=4, collision_settings=free_cs)
                    pool += [q for q in more if not any(np.allclose(q, s0, atol=1e-3) for s0 in pool)]
                except IntrinsicRequestError:
                    pass
            # configurations the OSC controller can reach first: nearest to the current one
            # (weighted joint distance) among those with a joint-limit margin (observed on task 9:
            # a far pre-grasp branch was reached 0.4-1.0 rad off and the approach then failed)
            lim_ = env.joint_limits()
            w_ = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.5])
            margin_ = lambda q: float(min(np.min(np.asarray(q) - lim_[:, 0]), np.min(lim_[:, 1] - np.asarray(q))))
            # margin-best first, but only among configurations within reach of the controller (weighted
            # joint distance <= 4.0, i.e. the same branch); observed: pure nearest-first ordering made
            # the OSC controller leave the LINEAR approach by 8 cm on task 7, pure margin-first chose
            # a branch 2.3 rad away on task 9
            # Pure nearest-first (the selection ce7685b used through nearest_ik, 10/10 on task 1):
            # a margin preference, even within the branch, chose pre-grasp configurations the OSC
            # controller could not track (task 1 dev state 4: 8 cm tracking error, 0.93 rad joint
            # error). The LINEAR dry-run below only skips infeasible configurations.
            dist_ = lambda q: float(np.sum(w_ * (np.asarray(q) - rs.q) ** 2))
            pool.sort(key=dist_)
            q_pre, n_tested = None, 0
            for q in pool:
                n_tested += 1
                try:
                    client.plan_to_pose(q, c.pos, c.rot, collision_settings=grasp_cs, motion_type="LINEAR", timeout_s=5.0,
                                        caller_id="pick_approach_dryrun")
                    q_pre = q
                    break
                except IntrinsicRequestError:
                    continue
            ctx.record(event="pregrasp_path_check", grasp=c.label, n_solutions=len(pool), n_tested=n_tested, feasible=q_pre is not None)
            if q_pre is None:
                continue
            chosen = (c, pre, q_pre)
            break
        if chosen is None:
            return SkillResult(self.name, False, "no_reachable_grasp")
        c, pre, q_pre = chosen
        self._tried.append(c.label)
        # 1. free-space motion to the pre-grasp pose
        self._last_width = c.width
        if not self.p.side_grasp:
            # top-down picks: the ce7685b path (ANY plan to the IK solution nearest the current
            # configuration, no joint-space settle). The validated-configuration path below was
            # measured to cost tasks 1/7 episodes (4c20556, C2 protocol runs: approach tracking
            # errors of 8 cm after the pre-grasp), while it helps side grasps (task 9).
            traj, res = _plan_and_execute(ctx, "pick_pregrasp", rs.q, pre, c.rot, free_cs, "ANY", 0.0, self.p.plan_timeout_s)
            if not res.ok:
                return SkillResult(self.name, False, f"pregrasp_exec:{res.reason}", details={"grasp": c.label})
        else:
            traj = client.plan_to_joints(rs.q, q_pre, collision_settings=free_cs, motion_type="ANY", timeout_s=self.p.plan_timeout_s,
                                         caller_id="pick_pregrasp")
            ctx.record(event="plan", label="pick_pregrasp", trajectory_id=traj.request_id, motion_type="ANY", n_states=int(len(traj.t)),
                       duration_s=traj.duration, latency_s=traj.planning_latency_s, target_pos=[float(v) for v in pre])
            res = ctx.execute(traj, ExecutionConfig(gripper=0.0), "pick_pregrasp")
            if not res.ok:
                return SkillResult(self.name, False, f"pregrasp_exec:{res.reason}", details={"grasp": c.label})
            # 1b. execution feedback: re-converge in joint space if the reached configuration drifted
            rs = env.robot_state()
            if float(np.max(np.abs(rs.q - q_pre))) > 0.02:
                try:
                    traj = client.plan_to_joints(rs.q, q_pre, collision_settings=free_cs, motion_type="JOINT", timeout_s=5.0, caller_id="pick_pregrasp_settle")
                    ctx.execute(traj, ExecutionConfig(gripper=0.0), "pick_pregrasp_settle")
                except IntrinsicRequestError as e:
                    ctx.record(event="pregrasp_settle_failed", reason=str(e)[:120])
        # 2. linear approach; only contact with the target object is permitted
        rs = env.robot_state()
        try:
            traj, res = _plan_and_execute(ctx, "pick_approach", rs.q, c.pos, c.rot, grasp_cs, "LINEAR", 0.0, self.p.plan_timeout_s)
        except IntrinsicRequestError as e0:
            if "FinePathIK" not in str(e0):
                raise
            # see PushSkill: the OSC null-space drift leaves the arm on a branch from which the
            # linear path IK fails; fall back to a collision-checked configuration-space approach
            ctx.record(event="approach_any_fallback", label="pick_approach", reason=str(e0)[:120])
            rs = env.robot_state()
            traj, res = _plan_and_execute(ctx, "pick_approach_any", rs.q, c.pos, c.rot, grasp_cs, "ANY", 0.0, self.p.plan_timeout_s)
        if not res.ok:
            return SkillResult(self.name, False, f"approach_exec:{res.reason}", details={"grasp": c.label})
        # 3. close and verify
        fq = ctx.close_gripper(self.p.close_steps)
        gap = float(abs(fq[0]) + abs(fq[1]))
        contacts = env.gripper_contacts_with(self.body)
        ctx.record(event="grasp_verify", finger_gap=gap, pad_contacts=contacts, grasp=c.label)
        if gap < self.p.min_finger_gap or contacts < self.p.min_pad_contacts:
            return SkillResult(self.name, False, f"grasp_verify_failed(gap={gap:.4f},contacts={contacts})", details={"grasp": c.label})
        # 4. attach and lift
        sync.attach(self.body)
        obj_before = env.body_pose(self.body)[0].copy()
        rs = env.robot_state()
        # while lifting, the grasped object is still touching whatever it rests on (intentional)
        lift_cs = sync.transport_collision_settings(support_bodies=support_of(env, sync, self.body))
        traj, res = _plan_and_execute(ctx, "pick_lift", rs.q, rs.tcp_pos + [0, 0, self.p.lift_height], rs.tcp_rot,
                                      lift_cs, "LINEAR", +1.0, self.p.plan_timeout_s)
        obj_after = env.body_pose(self.body)[0]
        rs = env.robot_state()
        rel = tf.inv_T(tf.make_T(rs.tcp_pos, rs.tcp_rot)) @ tf.make_T(*env.body_pose(self.body))
        drift = float(np.linalg.norm(rel[:3, 3] - sync.attached[self.body][0]))
        lifted = float(obj_after[2] - obj_before[2])
        contacts = env.gripper_contacts_with(self.body)
        ctx.record(event="lift_verify", lifted_m=lifted, tcp_obj_drift_m=drift, pad_contacts=contacts)
        if not res.ok or lifted < 0.5 * self.p.lift_height or contacts < 1:
            sync.detach(self.body)
            return SkillResult(self.name, False, f"lift_verify_failed(lifted={lifted:.3f},drift={drift:.3f},exec={res.reason})",
                               details={"grasp": c.label})
        return SkillResult(self.name, True, details={"grasp": c.label, "width": c.width, "lifted": lifted, "drift": drift})


@dataclasses.dataclass
class PlaceParams:
    preplace_clearance: float = 0.08     # tcp height above the release height for the transport target
    release_gap: float = 0.015           # object bottom above the support at release if no contact
    open_steps: int = 10
    retreat_height: float = 0.10
    plan_timeout_s: float = 15.0
    settle_steps: int = 6
    hand_below_tcp: float = 0.031        # PandaGripper: the wide hand body ends 31 mm above the tcp (measured from the collision mesh)
    rim_clearance: float = 0.012         # the hand must stay this far above a container's rim


class PlaceSkill(Skill):
    """Place the attached object so that its origin reaches `target_fn()` (world xyz) with the
    object's bottom on the support; `support_bodies` contact is intentional."""
    name = "place"
    max_attempts = 2

    def __init__(self, body: str, target_fn: Callable[[], Tuple[np.ndarray, float]], support_bodies: Sequence[str],
                 params: PlaceParams = PlaceParams(), keep_yaw: bool = True, region: Optional[str] = None):
        self.body = body
        self.target_fn = target_fn  # returns (target_xy_z_of_object_origin, support_top_z) or a ranked list
        self.support = list(support_bodies)
        self.p = params
        self.region = region        # region site (used to rotate the object so it fits a container)
        self._released = False

    def preconditions(self, ctx):
        if self.body not in ctx.sync.attached_bodies():
            return f"{self.body} not attached"
        return None

    def recover(self, ctx, last):
        rs = ctx.env.robot_state()
        ctx.sync.sync()
        try:
            _plan_and_execute(ctx, "place_recover_up", rs.q, rs.tcp_pos + [0, 0, 0.06], rs.tcp_rot,
                              ctx.sync.transport_collision_settings(self.support), "LINEAR", +1.0, 5.0)
        except IntrinsicRequestError as e:
            ctx.record(event="recover_plan_failed", reason=str(e))

    def attempt(self, ctx, i):
        env, client, sync = ctx.env, ctx.client, ctx.sync
        sync.sync()
        if self._released or self.body not in sync.attached:
            # the object was already released by an earlier attempt (only the retreat failed)
            return self._retreat(ctx, SkillResult(self.name, True, "released_earlier"))
        rs = env.robot_state()
        targets = self.target_fn(rs.tcp_rot[:, 0])
        if not isinstance(targets, list):
            targets = [targets]
        # use the attachment as measured now (the object may have shifted in the fingers during
        # the lift: 1.5-2.3 cm for the moka-pot handle grasp), not the one captured at attach time;
        # otherwise the release search predicts the object 1-2 cm off and raises the release
        drift0 = sync.refresh_attachment(self.body)
        ctx.record(event="place_attachment_refresh", drift_m=float(drift0))
        tcp_t_obj_p, tcp_t_obj_R = sync.attached[self.body]
        box = geo.object_box(env, self.body)
        obj_origin_above_bottom = env.body_pose(self.body)[0][2] - box.bottom_z
        R_tcp = rs.tcp_rot
        container = self.region is not None and placement.is_container_region(env, self.region)
        rot_options = [R_tcp]
        if container:
            # rotate the carried object about world z so that its footprint fits the container slot;
            # every equally fitting rotation is a candidate (some violate the wrist joint limit)
            fits = placement.fit_rotations(env, self.region, self.body)
            best_cost = fits[0][1]
            angles = [a for a, cost in fits if cost <= best_cost + 0.005][:6]
            # Drawer regions (owner body on a slide joint): add the rotations that put the hand's
            # closing axis EXACTLY across the drawer, so that the 21.8 cm wide palm lies parallel to
            # the drawer front and its 7.4 cm thickness along the drawer axis. Observed (task 3,
            # protocol states with the drawer open to its limit): the quarter-turn options kept the
            # grasp's 22 deg yaw, the palm corner reached 3.8 cm further toward the cabinet and
            # every deep release spot was rejected (hand vs the cabinet / upper drawer fronts).
            d_axis = placement.drawer_axis_of_region(env, self.region)
            if d_axis is not None:
                lateral = np.cross(np.array([0.0, 0.0, 1.0]), d_axis)
                closing_yaw = float(np.arctan2(R_tcp[1, 0], R_tcp[0, 0]))
                lat_yaw = float(np.arctan2(lateral[1], lateral[0]))
                for k in (0.0, np.pi):
                    a = (lat_yaw + k - closing_yaw + np.pi) % (2 * np.pi) - np.pi
                    if abs(a) < np.radians(100) and not any(abs(a - b) < np.radians(3) for b in angles):
                        angles.append(float(a))
                ctx.record(event="place_lateral_rotation_options", yaw_deg=[float(np.degrees(a)) for a in angles])
            if any(abs(a) > 1e-6 for a in angles):
                rot_options = [R.from_euler("z", a).as_matrix() @ R_tcp for a in angles]
                ctx.record(event="place_fit_rotation", yaw_deg=[float(np.degrees(a)) for a in angles], overhang_m=float(best_cost))
        obj_local = (geo.object_point_cloud(env, self.body, spacing=0.012) - rs.tcp_pos) @ rs.tcp_rot
        # finger-zone width = the measured finger gap (an edge/rim grasp holds a 1 cm wall of a
        # 10 cm object: the fingers open from that gap, not from the object's extent; observed on
        # task 3, where the fully open finger zone reached the drawer's side walls)
        width = float(max(np.sum(np.abs(rs.finger_q)), 0.005))
        targets = self._region_box_targets(ctx, targets, obj_origin_above_bottom)
        if len(rot_options) > 1 or rot_options[0] is not R_tcp:
            # Choose the carried-object rotation by the SAME release search used at place time:
            # the option whose release needs the smallest drop wins (tie-break: joint-limit margin
            # of an IK solution at its pre-place pose). Observed before: a probe pose 15 cm above
            # the target had no IK for any option (hand vs the upper drawers' handles), so the
            # first option was always used and the hand's wide axis lay along the drawer axis.
            cs_probe = sync.transport_collision_settings(self.support)
            lim = env.joint_limits()
            best, best_key, choice = None, None, []
            for ri, Rc in enumerate(rot_options):
                found = self._release_search(env, sync, targets, tcp_t_obj_p, Rc, obj_local, width, obj_origin_above_bottom,
                                             max_raise=0.08, max_candidates=24,
                                             record=lambda ri=ri, **kw: ctx.record(rotation_option=ri, **kw) if kw.get("event") == "place_spot_rejected" else None)
                if found is None:
                    choice.append([None, None])
                    continue
                _, support_z_c, tcp_rel_c, tcp_pre_c, _ = found
                Rl = self.last_release_rot
                drop = float((obj_local @ Rl.T + tcp_rel_c)[:, 2].min() - support_z_c)
                margin = -1.0
                try:
                    sols, _, _ = nearest_ik(ctx, tcp_pre_c, Rl, rs.q, cs_probe, n=4)
                    for q in sols:
                        margin = max(margin, float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q)))))
                except IntrinsicRequestError:
                    pass
                choice.append([round(drop, 3), round(margin, 3)])
                key = (round(drop, 2), -margin)
                if margin > 0 and (best_key is None or key < best_key):
                    best, best_key = Rc, key
            R_tcp = best if best is not None else rot_options[0]
            ctx.record(event="place_rotation_choice", options=choice,
                       chosen=(next(i for i, Rc in enumerate(rot_options) if Rc is best) if best is not None else -1))
        found = self._release_search(env, sync, targets, tcp_t_obj_p, R_tcp, obj_local, width, obj_origin_above_bottom,
                                     record=ctx.record)
        if found is None:
            target_origin, support_z = targets[0]
            release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
            tcp_release = release_origin - R_tcp @ tcp_t_obj_p
            rim_z = max(geo.object_box(env, b).top_z for b in self.support)
            approach = R_tcp[:, 2]
            if support_z < rim_z - 0.02 and approach[2] < -0.9:
                tcp_release[2] = max(tcp_release[2], rim_z + self.p.rim_clearance - geo.hand_lowest_offset(R_tcp))
            tcp_pre = tcp_release - approach * self.p.preplace_clearance
            if approach[2] > -0.9:
                tcp_pre = tcp_pre + np.array([0, 0, 0.02])
        else:
            target_origin, support_z, tcp_release, tcp_pre, _ = found
            R_tcp = self.last_release_rot      # the search may have chosen a pitched hand (release-tilt family)
            approach = R_tcp[:, 2]
        cs_transport = sync.transport_collision_settings(self.support)
        # Transport rule: the carried object must NOT pass through its future support (container
        # walls, drawer, cabinet). Contact with the support is intentional only while lowering.
        # Observed before this rule: the planner swept the book through the caddy wall (task 5).
        # If the object already starts inside/against the support (retry after a partial lowering)
        # the exclusion is kept for this segment, otherwise the start state would be invalid.
        start_clear = self._object_clear_of(env, obj_local, rs.tcp_pos, rs.tcp_rot, self.support, 0.005)
        cs_move = sync.transport_collision_settings(()) if start_clear else cs_transport
        rim_dbg = max(geo.object_box(env, b).top_z for b in self.support)
        ctx.record(event="place_transport_rule", support_checked=bool(start_clear), tcp_pre=[float(v) for v in tcp_pre],
                   tcp_release=[float(v) for v in tcp_release], rim_z=float(rim_dbg),
                   obj_bottom_at_pre=float((obj_local @ R_tcp.T + tcp_pre)[:, 2].min()),
                   obj_bottom_at_release=float((obj_local @ R_tcp.T + tcp_release)[:, 2].min()))
        traj, res = _plan_and_execute(ctx, "place_transport", rs.q, tcp_pre, R_tcp, cs_move, "ANY", +1.0, self.p.plan_timeout_s)
        if not res.ok:
            return SkillResult(self.name, False, f"transport_exec:{res.reason}")
        # Grasp-slip compensation: the object may have shifted/rotated in the fingers during the
        # transport (wrist rotation, swinging). Re-measure tcp_t_obj (and update the attached
        # geometry in the Intrinsic world) and shift the release pose so that the OBJECT, not the
        # stale tcp target, reaches the placement target.
        drift = sync.refresh_attachment(self.body)
        rs = env.robot_state()
        obj_now = env.body_pose(self.body)[0]
        # predicted object position at the release = release tcp + the measured tcp->object offset
        # (the robot is at the PRE-place pose here: for a side approach the pre pose is 8 cm in
        # front of the release, which must not be mistaken for slip; observed on task 9, where the
        # "correction" doubled the insertion depth and drove the hand into the microwave frame)
        predicted = tcp_release + (obj_now - rs.tcp_pos)
        xy_corr = np.array([target_origin[0] - predicted[0], target_origin[1] - predicted[1], 0.0])
        # vertical: keep the planned tcp-to-object bottom distance consistent with the new offset
        box_now = geo.object_box(env, self.body)
        z_corr = (support_z + self.p.release_gap) - (box_now.bottom_z - (rs.tcp_pos[2] - tcp_release[2]))
        ctx.record(event="place_slip_compensation", drift_m=float(drift), xy_correction_m=float(np.linalg.norm(xy_corr)), z_correction_m=float(z_corr))
        if np.linalg.norm(xy_corr) > 0.008 or abs(z_corr) > 0.008:
            tcp_release = tcp_release + xy_corr
            if abs(z_corr) > 0.008 and z_corr > 0:   # only raise (never plan the object into the support)
                tcp_release[2] += z_corr
            tcp_pre2 = tcp_release - approach * self.p.preplace_clearance
            try:
                _, res = _plan_and_execute(ctx, "place_transport_correct", rs.q, tcp_pre2, R_tcp, cs_transport, "LINEAR", +1.0, self.p.plan_timeout_s)
            except IntrinsicRequestError as e:
                ctx.record(event="place_correct_failed", reason=str(e)[:200])
        # lowering with a contact monitor: stop as soon as the carried object touches the support
        def monitor():
            for g1, g2, _ in env.contacts():
                b1 = env.model.body_id2name(env.model.geom_bodyid[env.model.geom_name2id(g1)]) if g1 else ""
                b2 = env.model.body_id2name(env.model.geom_bodyid[env.model.geom_name2id(g2)]) if g2 else ""
                if {b1, b2} & set(self.support) and self.body in (b1, b2):
                    return "support_contact"
            return None
        rs = env.robot_state()
        traj, res = _plan_and_execute(ctx, "place_lower", rs.q, tcp_release, R_tcp, cs_transport, "LINEAR", +1.0,
                                      self.p.plan_timeout_s, contact_monitor=monitor, time_scale=1.5)
        if not res.ok and not res.reason.startswith("contact:"):
            if res.reason.startswith("final_pose_error") and res.final_pos_err < 0.035:
                # blocked just short of the release pose (object touching the container/support):
                # release here, the drop is small
                ctx.record(event="place_lower_short", final_pos_err=res.final_pos_err)
            else:
                return SkillResult(self.name, False, f"lower_exec:{res.reason}")
        if approach[2] > -0.9 and res.ok:
            # side grasp: after entering, lower the object onto the support (contact monitored)
            rs = env.robot_state()
            try:
                _, res2 = _plan_and_execute(ctx, "place_settle", rs.q, rs.tcp_pos - [0, 0, 0.04], rs.tcp_rot, cs_transport, "LINEAR", +1.0,
                                            self.p.plan_timeout_s, contact_monitor=monitor, time_scale=1.5)
            except IntrinsicRequestError as e:
                ctx.record(event="place_settle_failed", reason=str(e)[:200])
        # release: open only as much as needed (fingers stay clear of container walls)
        gap_now = float(np.sum(np.abs(env.robot_state().finger_q)))
        ctx.open_gripper(int(np.ceil(min(geo.GRIPPER_MAX_OPENING, gap_now + 0.03) / 0.01)))
        sync.detach(self.body)
        self._released = True
        ctx.hold(0.0, self.p.settle_steps)
        obj_pos = env.body_pose(self.body)[0]
        in_box = None
        if self.region is not None:   # observation-based check of the goal geometry (origin inside the region box)
            c_v, rot_v, half_v = geo.site_box_world(env, self.region)
            in_box = bool(np.all(np.abs(rot_v.T @ (obj_pos - c_v)) <= half_v))
        ctx.record(event="place_verify", obj_pos=[float(v) for v in obj_pos], target=[float(v) for v in target_origin],
                   xy_err=float(np.linalg.norm(obj_pos[:2] - target_origin[:2])), lower_stop=res.reason, in_region_box=in_box)
        res_ok = in_box is not False
        return self._retreat(ctx, SkillResult(self.name, res_ok, "" if res_ok else "released_outside_region_box",
                                              details={"xy_err": float(np.linalg.norm(obj_pos[:2] - target_origin[:2]))}))

    @staticmethod
    def _box_tops(env, body):
        """Top faces of the collision boxes of `body`: list of (top_z, xy_min, xy_max) in world."""
        m = env.model
        root = m.body_name2id(body)
        pos, rot = env.body_pose(body)
        out = []
        for gi in range(m.ngeom):
            if int(m.geom_bodyid[gi]) != root or (int(m.geom_contype[gi]) == 0 and int(m.geom_conaffinity[gi]) == 0):
                continue
            if int(m.geom_type[gi]) != geo.GEOM_BOX:
                continue
            gpos = np.array(m.geom_pos[gi]); grot = tf.quat_wxyz_to_mat(m.geom_quat[gi]); size = np.array(m.geom_size[gi])
            corners = np.array([[sx * size[0], sy * size[1], sz * size[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            world = (corners @ grot.T + gpos) @ rot.T + pos
            top_z = float(world[:, 2].max())
            top = world[world[:, 2] > top_z - 0.003]
            out.append((top_z, top[:, :2].min(0), top[:, :2].max(0)))
        return out

    def _region_box_targets(self, ctx, targets, obj_origin_above_bottom):
        """Region-box rule. LIBERO's `In` tests the object ORIGIN against the region box (plus
        contact with the container). When the origin of the object resting on the container floor
        lies BELOW the box (task 5: the book's origin is its bottom face, the back-compartment box
        starts 1.2 cm above the floor), lowering it to the floor can never satisfy the goal. The
        valid resting states are on the container's internal structure: this finds a horizontal
        ledge of the support (top face of a collision box) inside the region box's height range
        (the 6.7 cm divider between the back and front compartments) and re-targets the object's
        bottom onto that ledge, offset 4 mm toward the outside of the region, so that after
        release it tips outward and comes to rest leaning on the ledge and the outer wall (origin
        above the ledge, inside the box)."""
        env = ctx.env
        if self.region is None:
            return targets
        c_r, rot_r, half_r = geo.site_box_world(env, self.region)
        box_bottom, box_top = c_r[2] - half_r[2], c_r[2] + half_r[2]
        tops = [t for b in self.support for t in self._box_tops(env, b)]
        t0 = np.asarray(targets[0][0])
        floors = [tz for tz, lo, hi in tops if np.all(t0[:2] >= lo - 0.005) and np.all(t0[:2] <= hi + 0.005) and tz < c_r[2]]
        floor_z = max(floors) if floors else None
        support_z = float(targets[0][1]) if floor_z is None else min(float(targets[0][1]), floor_z)
        rest_origin_z = support_z + obj_origin_above_bottom
        if rest_origin_z >= box_bottom - 0.002:   # resting on the floor keeps the origin inside the box
            return targets
        # region xy rectangle (site frame is axis aligned up to a yaw; use the world AABB of its corners)
        corners = np.array([[sx * half_r[0], sy * half_r[1], 0.0] for sx in (-1, 1) for sy in (-1, 1)]) @ rot_r.T + c_r
        r_lo, r_hi = corners[:, :2].min(0) - 0.015, corners[:, :2].max(0) + 0.015
        ledges = [(tz, lo, hi) for tz, lo, hi in tops
                  if box_bottom + 0.005 < tz < box_top and np.all(hi >= r_lo) and np.all(lo <= r_hi)]
        if not ledges:
            ctx.record(event="place_region_box_rule", rest_origin_z=float(rest_origin_z), box_bottom_z=float(box_bottom), ledge=None)
            return targets
        # prefer ledges whose long side runs along the region's long axis (the object's width is
        # aligned with it by the fit rotation, so it can rest on such a ledge), then the lowest
        r_ext = corners[:, :2].max(0) - corners[:, :2].min(0)
        r_axis = 0 if r_ext[0] >= r_ext[1] else 1
        ledge_z, lo, hi = min(ledges, key=lambda t: (0 if (t[2] - t[1])[r_axis] >= (t[2] - t[1])[1 - r_axis] else 1, t[0]))
        ledge_c = 0.5 * (lo + hi)
        along = np.array([1.0, 0.0]) if (hi - lo)[0] >= (hi - lo)[1] else np.array([0.0, 1.0])
        outward = ledge_c - c_r[:2]
        outward = outward - along * float(np.dot(outward, along))
        outward = outward / (np.linalg.norm(outward) + 1e-9)
        xy = ledge_c + outward * 0.004
        xy = xy + along * float(np.dot(c_r[:2] - xy, along))      # centred along the ledge at the region centre
        new_targets = [(np.array([xy[0], xy[1], ledge_z]), float(ledge_z))]
        ctx.record(event="place_region_box_rule", rest_origin_z=float(rest_origin_z), box_bottom_z=float(box_bottom),
                   ledge=[float(v) for v in new_targets[0][0]], ledge_extent=[float(v) for v in (hi - lo)], floor_z=floor_z)
        return new_targets

    # ------------------------------------------------------------------ geometric release search
    @staticmethod
    def _object_clear_of(env, obj_local, tcp_p, tcp_R, bodies, threshold, min_local_height=None):
        """True when the carried object (points `obj_local` in the tcp frame) placed at
        (tcp_p, tcp_R) keeps at least `threshold` from every body in `bodies`."""
        from scipy.spatial import cKDTree
        pts = obj_local @ tcp_R.T + tcp_p
        if min_local_height is not None:   # only the part of the object above its bottom (container walls)
            pts = pts[pts[:, 2] > pts[:, 2].min() + min_local_height]
            if not len(pts):
                return True
        for b in bodies:
            if np.linalg.norm(env.body_pose(b)[0][:2] - tcp_p[:2]) > 0.8:
                continue
            t = cKDTree(geo.object_point_cloud(env, b, spacing=0.012))
            if t.query(pts, k=1)[0].min() < threshold:
                return False
        return True

    def _release_search(self, env, sync, targets, tcp_t_obj_p, R_tcp, obj_local, width, obj_origin_above_bottom, record=None,
                        max_raise: float = 0.15, max_candidates: int = 10 ** 6):
        """Choose the first placement candidate whose release AND pre-place poses are geometrically
        valid: hand/fingers clear of every other body, carried object clear of every non-support
        body (8 mm) and of the support's walls (5 mm), object above a container rim at the
        pre-place pose. The release pose is raised (1 cm steps, <= 15 cm) until valid.
        Returns (target_origin, support_z, tcp_release, tcp_pre, raised) or None."""
        from scipy.spatial import cKDTree
        rim_z = max(geo.object_box(env, b).top_z for b in self.support)
        hand_low = geo.hand_lowest_offset(R_tcp)
        others = [b for b in sync.bodies if b != self.body]
        non_support = [b for b in others if b not in self.support
                       and np.linalg.norm(env.body_pose(b)[0][:2] - targets[0][0][:2]) < 0.6]
        trees = {b: cKDTree(geo.object_point_cloud(env, b, spacing=0.012)) for b in non_support + list(self.support)}
        # point clouds for the hand/finger clearance test, built once per search (geo.hand_clearance
        # rebuilds them per call, which made the 6-rotation assessment take minutes)
        t_xy = np.asarray(targets[0][0])[:2]
        hand_clouds = {b: geo.object_point_cloud(env, b, spacing=0.012) for b in others
                       if np.linalg.norm(env.body_pose(b)[0][:2] - t_xy) < 0.6}

        def hand_clear(tcp_p, Rm=None):
            half_open_c = opening / 2.0
            for b, pts in hand_clouds.items():
                local = (pts - tcp_p) @ (R_tcp if Rm is None else Rm)
                x, y, z = local[:, 0], local[:, 1], local[:, 2]
                finger = (np.abs(x) >= width / 2 - 0.002) & (np.abs(x) <= half_open_c + 0.027) & (np.abs(y) < geo.FINGER_HALF_Y) & (z > geo.FINGER_Z[0]) & (z < geo.PAD_Z[1])
                palm = (np.abs(x) < geo.HAND_HALF_X) & (np.abs(y) < geo.HAND_HALF_Y) & (z > geo.HAND_Z[0]) & (z < geo.HAND_Z[1])
                if finger.any() or palm.any():
                    if record and os.environ.get("LIBERO_LAB_DEBUG"):
                        sel = finger | palm
                        record(event="hand_block_debug", body=b, tcp=[float(v) for v in tcp_p], n=int(sel.sum()),
                               zone="finger" if finger.any() else "palm", opening=float(opening), width=float(width),
                               world_y=[float(pts[sel][:, 1].min()), float(pts[sel][:, 1].max())], world_z=[float(pts[sel][:, 2].min()), float(pts[sel][:, 2].max())],
                               local_xyz_min=[float(v) for v in local[sel].min(0)], local_xyz_max=[float(v) for v in local[sel].max(0)])
                    return False, b
            return True, ""

        def clear_of(tcp_p, bodies, threshold, walls_only=False, Rm=None):
            pts = obj_local @ (R_tcp if Rm is None else Rm).T + tcp_p
            if walls_only:
                pts = pts[pts[:, 2] > pts[:, 2].min() + 0.02]
                if not len(pts):
                    return True, ""
            for b in bodies:
                if trees[b].query(pts, k=1)[0].min() < threshold:
                    return False, b
            return True, ""
        approach = R_tcp[:, 2]
        world_pts = obj_local @ R_tcp.T
        bottom_off = float(world_pts[:, 2].min())   # object bottom relative to the tcp
        opening = min(geo.GRIPPER_MAX_OPENING, width + 0.02)
        # Footprint centring: placement targets are for the object ORIGIN, but the object's
        # footprint centre may be offset from its origin (the book: ~8 mm), which in a narrow
        # compartment leaves too little wall clearance on one side. Shift the origin target so
        # that the footprint centre lands on the slot, keeping the origin inside the region box.
        fp_off = 0.5 * (world_pts.min(0) + world_pts.max(0)) - R_tcp @ np.asarray(tcp_t_obj_p)
        fp_off = np.array([fp_off[0], fp_off[1], 0.0])
        if self.region is not None:
            c_r, rot_r, half_r = geo.site_box_world(env, self.region)
            # centre the footprint only along region axes that are tight for it (< 4 cm spare):
            # in a wide cavity the hand, not the object, is the limiting body and the origin
            # target (hand position) must stay where the slot search put it
            ext = world_pts.max(0) - world_pts.min(0)
            axes_w = [rot_r[:, i][:2] / (np.linalg.norm(rot_r[:, i][:2]) + 1e-9) for i in range(2)]
            keep = np.zeros(2)
            for i in range(2):
                spare = 2 * half_r[i] - abs(float(np.dot(ext[:2], np.abs(axes_w[i]))))
                if spare < 0.04:
                    keep += axes_w[i] * float(np.dot(fp_off[:2], axes_w[i]))
            fp_off = np.array([keep[0], keep[1], 0.0])
        # Candidate preparation (rim-limited nominal release per candidate)
        prepared = []
        for k, (target_origin, support_z) in enumerate(targets):
            target_origin = np.asarray(target_origin, dtype=float) - fp_off
            if self.region is not None:
                local = rot_r.T @ (target_origin - c_r)
                local[:2] = np.clip(local[:2], -0.8 * half_r[:2], 0.8 * half_r[:2])
                target_origin = c_r + rot_r @ np.array([local[0], local[1], local[2]])
                target_origin[2] = float(targets[k][0][2])
            release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
            tcp_release = release_origin - R_tcp @ tcp_t_obj_p
            container = support_z < rim_z - 0.02
            # Container-aware limit: the wide hand body cannot enter an open-top container (basket,
            # drawer, caddy) from above: keep the hand above the rim. Not for horizontal approaches
            # into a roofed container (microwave): there the hand enters below the roof and the
            # hand/finger clearance checks against the walls apply instead.
            if container and approach[2] < -0.9:
                tcp_min_z = rim_z + self.p.rim_clearance - hand_low
                if tcp_release[2] < tcp_min_z:
                    if record:
                        record(event="place_rim_limit", rim_z=float(rim_z), tcp_release_z=float(tcp_release[2]), tcp_min_z=float(tcp_min_z),
                               drop_height=float(tcp_min_z - tcp_release[2]), candidate=k)
                    tcp_release[2] = tcp_min_z
            prepared.append((k, target_origin, support_z, tcp_release, container))

        def valid(tcp_p, Rm=None):
            """Release pose, pre-place pose and the straight segment between them (5 samples) must
            keep hand/fingers clear of every other body, the carried object clear of non-support
            bodies (8 mm) and of the support's walls (5 mm; at the release only its upper part)."""
            Rm = R_tcp if Rm is None else Rm
            appr = Rm[:, 2]
            ok, blocker = hand_clear(tcp_p, Rm)
            if not ok:
                return False, "hand:" + blocker, None
            ok, blocker = clear_of(tcp_p, non_support, 0.008, Rm=Rm)
            if not ok:
                return False, "obj:" + blocker, None
            ok, blocker = clear_of(tcp_p, self.support, 0.005, walls_only=True, Rm=Rm)
            if not ok:
                return False, "objwall:" + blocker, None
            pre = tcp_p - appr * self.p.preplace_clearance
            if approach[2] > -0.9:            # side grasp: also stay clear above the floor before entering
                pre = pre + np.array([0, 0, 0.02])
            elif container_here and approach[2] < -0.9:   # top grasp: the object must be above the rim before descending
                pre[2] = max(pre[2], rim_z + 0.03 - float((obj_local @ Rm.T)[:, 2].min()))
            for t in (1.0, 0.8, 0.6, 0.4, 0.2):
                q_p = tcp_p + (pre - tcp_p) * t
                ok, blocker = hand_clear(q_p, Rm)
                if ok:
                    ok, blocker = clear_of(q_p, non_support + (list(self.support) if t >= 0.99 else []), 0.005 if t >= 0.99 else 0.008, Rm=Rm)
                if not ok:
                    return False, ("pre:" if t >= 0.99 else f"path{t:.1f}:") + blocker, None
            return True, "", pre
        # Search order: every candidate at the nominal height first, then with the release raised
        # (1 cm steps, <= 15 cm): an unraised release anywhere in the region beats a raised one at
        # the preferred spot (shorter drop).
        last_blockers = {}
        traces = {k: [] for k, *_ in prepared}
        top_off = float(world_pts[:, 2].max())      # object top relative to the tcp
        box_top = (c_r[2] + half_r[2]) if self.region is not None else np.inf
        prepared = prepared[:max_candidates]
        # Release-tilt family (top-down placements into containers): the hand, and with it the
        # carried object, pitched about the closing axis by a few degrees. Observed on task 3: a
        # level hand over a flat, deep bowl position always hits the handles of the drawers above,
        # while a hand pitched 10-20 deg toward the robot clears them; the object then lands on
        # its lower edge from a few millimetres and settles, instead of being dropped 14 cm.
        tilt_opts = [(0.0, R_tcp)]
        if self.region is not None and approach[2] < -0.9 and any(ch for *_, ch in prepared):
            for deg in (10.0, -10.0, 20.0, -20.0):
                tilt_opts.append((deg, R.from_rotvec(R_tcp[:, 0] * np.radians(deg)).as_matrix() @ R_tcp))
        self.last_release_rot = R_tcp
        for raised in np.arange(0.0, max_raise + 0.001, 0.01):
            for k, target_origin, support_z, tcp_release, container_here in prepared:
              for tilt_deg, Rm in tilt_opts:
                if tilt_deg == 0.0:
                    cand = tcp_release + np.array([0, 0, raised])
                else:
                    # same object origin target; the tilted object's lowest point at the release gap
                    bottom_t = float((obj_local @ Rm.T)[:, 2].min())
                    cand = np.array([target_origin[0], target_origin[1], 0.0]) - Rm @ tcp_t_obj_p
                    cand[2] = support_z + self.p.release_gap - bottom_t + raised
                if approach[2] > -0.9 and cand[2] + top_off > box_top + 0.01:
                    continue      # horizontal insertion: the object cannot be lifted out of the region box (roof)
                if raised > 0 and traces[k] and traces[k][0].split(":")[0] in ("obj", "objwall"):
                    # the OBJECT itself overlaps a wall / another body at the landing spot: releasing
                    # higher does not clear the landing (observed on task 3: the bowl dropped 3 cm
                    # onto the drawer's inner front wall and came to rest tilted 30 deg)
                    continue
                ok, blocker, pre = valid(cand, Rm)
                if ok and tilt_deg != 0.0:
                    self.last_release_rot = Rm
                    if record:
                        record(event="place_release_tilted", candidate=k, tilt_deg=float(tilt_deg), raised_m=float(raised))
                if ok:
                    if raised > 0 and record:
                        record(event="place_release_raised", candidate=k, raised_m=float(raised), blockers=traces[k][:16])
                    if record:   # diagnostic trace: why every earlier candidate was rejected (first blocker)
                        record(event="place_release_search", chosen=k, raised_m=float(raised), n_candidates=len(prepared),
                               target=[float(v) for v in target_origin], tcp=[float(v) for v in cand],
                               rejected={str(kk): (traces[kk][0] if traces[kk] else "") for kk, *_ in prepared if traces[kk]},
                               candidates=[[int(kk), [round(float(v), 3) for v in to], [round(float(v), 3) for v in tr]] for kk, to, _, tr, _ in prepared],
                               approach=[float(v) for v in approach], hand_low=float(hand_low))
                    return target_origin, support_z, cand, pre, float(raised)
                traces[k].append(blocker)
                last_blockers[k] = blocker
        if record:
            for k, target_origin, support_z, tcp_release, _ in prepared:
                record(event="place_spot_rejected", candidate=k, blocker=last_blockers.get(k, ""),
                       first_blocker=(traces[k][0] if traces[k] else ""), tcp=[float(v) for v in tcp_release],
                       trace=traces[k][:10])
        return None

    def grasp_compatible(self, env, sync, grasp_pos, grasp_rot, finger_gap: Optional[float] = None) -> Tuple[bool, float]:
        """Placement-aware grasp selection: would this grasp (tcp pose on the object at its
        current pose) admit a valid release pose at the placement target, and how far would the
        object drop after release? Pure geometry (the same release search as at place time,
        every fit rotation considered); no IK. Returns (feasible, cost) with cost = drop height
        + 0.5 x distance of the accepted placement candidate from the preferred slot."""
        obj_p, obj_R = env.body_pose(self.body)
        T_rel = tf.inv_T(tf.make_T(grasp_pos, grasp_rot)) @ tf.make_T(obj_p, obj_R)
        tcp_t_obj_p = T_rel[:3, 3]
        obj_local = (geo.object_point_cloud(env, self.body, spacing=0.012) - grasp_pos) @ grasp_rot
        width = float(max(finger_gap, 0.005)) if finger_gap is not None else float(2 * max(np.abs(obj_local[:, 0]).max(), 0.005))
        box = geo.object_box(env, self.body)
        obj_origin_above_bottom = obj_p[2] - box.bottom_z
        targets = self.target_fn(grasp_rot[:, 0])
        if not isinstance(targets, list):
            targets = [targets]
        rots = [grasp_rot]
        if self.region is not None and placement.is_container_region(env, self.region):
            fits = placement.fit_rotations(env, self.region, self.body)
            angles = [a for a, cost in fits if cost <= fits[0][1] + 0.005][:6]
            rots = [R.from_euler("z", a).as_matrix() @ grasp_rot for a in angles]
        best = None
        self.last_compat_why = ""
        for Rc in rots:
            why = []
            found = self._release_search(env, sync, targets, tcp_t_obj_p, Rc, obj_local, width, obj_origin_above_bottom,
                                         record=lambda **kw: why.append(kw.get("first_blocker", "")) if kw.get("event") == "place_spot_rejected" else None,
                                         max_raise=0.08, max_candidates=24)
            if found is None:
                self.last_compat_why = next((w for w in why if w), "")
                continue
            target_origin, support_z, tcp_release, _, _ = found
            bottom = float((obj_local @ self.last_release_rot.T + tcp_release)[:, 2].min())
            drop = bottom - support_z          # how far the object falls after release
            # distance of the accepted placement candidate from the preferred slot: a grasp that
            # forces the object to the edge of the region (e.g. the front of a drawer, where it
            # later blocks the closing push) ranks below one that admits the preferred spot
            slot_dist = float(np.linalg.norm(np.asarray(target_origin)[:2] - np.asarray(targets[0][0])[:2]))
            score = drop + 0.5 * slot_dist
            if best is None or score < best:
                best = score
            if best < 0.02:
                break        # a near-ideal release exists for this grasp: no need to try more rotations
        return best is not None, (best if best is not None else float("inf"))

    def _retreat(self, ctx, result: SkillResult) -> SkillResult:
        """Withdraw straight up. Contact with the just-released object and with the support/
        container (the open fingers may touch its walls) is intentional while withdrawing."""
        env, sync = ctx.env, ctx.sync
        sync.sync()
        rs = env.robot_state()
        cs_retreat = sync.grasp_collision_settings(self.body, support_bodies=self.support)
        # Release separation (observation-based): a rim/edge grasp can leave the object hooked on
        # a finger after opening (observed: the bowl lifted out of the drawer with the retreat).
        # If finger contacts with the released object persist, move the hand 2.5 cm horizontally
        # away from the object's centre (contact with the object allowed) before withdrawing.
        for k in range(2):
            if env.gripper_contacts_with(self.body) == 0:
                break
            obj_c = geo.object_box(env, self.body).center_world
            away = rs.tcp_pos - obj_c
            away[2] = 0.0
            if np.linalg.norm(away) < 1e-3:
                break
            away = away / np.linalg.norm(away)
            ctx.record(event="release_separation", attempt=k, contacts=int(env.gripper_contacts_with(self.body)))
            try:
                _plan_and_execute(ctx, "place_separate", rs.q, rs.tcp_pos + away * 0.025, rs.tcp_rot, cs_retreat, "LINEAR", 0.0, 5.0)
            except IntrinsicRequestError as e:
                ctx.record(event="release_separation_failed", reason=str(e)[:160])
                break
            sync.sync()
            rs = env.robot_state()
        back = -rs.tcp_rot[:, 2]          # withdraw along the negative approach axis (up, or out of a front opening)
        try:
            traj, res = _plan_and_execute(ctx, "place_retreat", rs.q, rs.tcp_pos + back * self.p.retreat_height, rs.tcp_rot,
                                          cs_retreat, "LINEAR", 0.0, self.p.plan_timeout_s)
            result.details["retreat_ok"] = res.ok
        except IntrinsicRequestError as e:
            result.details["retreat_ok"] = False
            result.details["retreat_error"] = str(e)[:200]
        return result
