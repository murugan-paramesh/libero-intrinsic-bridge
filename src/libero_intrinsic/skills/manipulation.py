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
from typing import Callable, List, Optional, Sequence, Tuple

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


def nearest_ik(ctx: SkillContext, pos, rot, q_now, cs, n: int = 8):
    """Intrinsic IK solutions for the pose (collision checked with `cs`), sorted by weighted
    joint distance to q_now (proximal joints weigh more: less arm reconfiguration, which the
    OSC controller tracks far better)."""
    sols, rid, lat = ctx.client.ik(pos, rot, q_now, max_solutions=n, collision_settings=cs)
    lim = ctx.env.joint_limits()
    safe = [q for q in sols if np.all(np.asarray(q) > lim[:, 0] + 0.06) and np.all(np.asarray(q) < lim[:, 1] - 0.06)]
    sols = safe or sols
    w = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.5])
    sols.sort(key=lambda q: float(np.sum(w * (np.asarray(q) - q_now) ** 2)))
    return sols, rid, lat


def _plan_and_execute(ctx: SkillContext, label: str, q_start, pos, rot, cs, motion_type="ANY",
                      gripper=-1.0, timeout_s=15.0, contact_monitor=None, time_scale=1.0):
    if motion_type == "ANY":
        # free-space motion: plan to the IK solution nearest the current configuration (joint target)
        sols, rid, lat = nearest_ik(ctx, pos, rot, np.asarray(q_start), cs)
        ctx.record(event="ik", label=label + "_goal", n_solutions=len(sols), request_id=rid, latency_s=lat)
        if not sols:
            raise IntrinsicRequestError("ComputeIk", "NOT_FOUND", f"no collision-free IK for {label}", rid)
        traj = ctx.client.plan_to_joints(q_start, sols[0], collision_settings=cs, motion_type="ANY",
                                         timeout_s=timeout_s, caller_id=label)
    else:
        traj = ctx.client.plan_to_pose(q_start, pos, rot, collision_settings=cs, motion_type=motion_type,
                                       timeout_s=timeout_s, caller_id=label)
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
            cands = [c for c in geo.side_grasp_candidates(env, self.body) if c.label not in self._tried]
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

        def clear(cs):
            out = []
            for c in cs:
                opening = min(geo.GRIPPER_MAX_OPENING, c.width + self.p.open_clearance)
                ok, blocker = geo.scene_clearance(env, c, opening, others)
                if ok:  # also at the pre-grasp pose (hand higher up, e.g. above a container rim)
                    pre = c.pos - c.approach * pre_clear
                    ok, blocker = geo.hand_clearance(env, pre, c.rot, opening, 0.0, others)
                if ok:
                    out.append(c)
            return out
        cleared = clear(cands)
        n_tilted = 0
        if not cleared:
            # neighbours block every vertical grasp: try approaches tilted away from them
            tilted = [c for c in geo.grasp_candidates(env, self.body, yaws=self.p.yaws, tilts=(np.radians(20), np.radians(35)))
                      if c.label not in self._tried and "tilt" in c.label]
            n_tilted = len(tilted)
            cleared = clear(tilted)
        ctx.record(event="grasp_candidates", n_total=len(cands), n_cleared=len(cleared), n_tilted=n_tilted)
        cands = cleared or cands
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
            sols, rid, lat = client.ik(pre, c.rot, rs.q, max_solutions=4, collision_settings=free_cs)
            ctx.record(event="ik", label="pregrasp", grasp=c.label, n_solutions=len(sols), request_id=rid, latency_s=lat)
            if not sols:
                continue
            sols2, rid2, lat2 = client.ik(c.pos, c.rot, sols[0], max_solutions=4, collision_settings=grasp_cs)
            ctx.record(event="ik", label="grasp", grasp=c.label, n_solutions=len(sols2), request_id=rid2, latency_s=lat2)
            if sols2:
                chosen = (c, pre)
                break
        if chosen is None:
            return SkillResult(self.name, False, "no_reachable_grasp")
        c, pre = chosen
        self._tried.append(c.label)
        # 1. free-space motion to the pre-grasp
        self._last_width = c.width
        traj, res = _plan_and_execute(ctx, "pick_pregrasp", rs.q, pre, c.rot, free_cs, "ANY", 0.0, self.p.plan_timeout_s)
        if not res.ok:
            return SkillResult(self.name, False, f"pregrasp_exec:{res.reason}", details={"grasp": c.label})
        # 2. linear approach; only contact with the target object is permitted
        rs = env.robot_state()
        traj, res = _plan_and_execute(ctx, "pick_approach", rs.q, c.pos, c.rot, grasp_cs, "LINEAR", 0.0, self.p.plan_timeout_s)
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
        tcp_t_obj_p, tcp_t_obj_R = sync.attached[self.body]
        box = geo.object_box(env, self.body)
        obj_origin_above_bottom = env.body_pose(self.body)[0][2] - box.bottom_z
        R_tcp = rs.tcp_rot
        container = self.region is not None and placement.is_container_region(env, self.region)
        rot_options = [R_tcp]
        if container:
            # rotate the carried object about world z so that its footprint fits the container slot;
            # both rotation directions are candidates (one may violate the wrist joint limit)
            dyaw = placement.fit_rotation(env, self.region, self.body)
            if abs(dyaw) > 1e-6:
                rot_options = [R.from_euler("z", a).as_matrix() @ R_tcp for a in (dyaw, -dyaw) if abs(a) > 1e-6]
                ctx.record(event="place_fit_rotation", yaw_deg=float(np.degrees(dyaw)))
        if len(rot_options) > 1:
            cs_probe = sync.transport_collision_settings(self.support)
            t0, _ = targets[0]
            lim = env.joint_limits()
            best, best_margin = None, -1.0
            for Rc in rot_options:
                probe = np.array([t0[0], t0[1], t0[2] + 0.15]) - Rc @ tcp_t_obj_p  # well above the target
                try:
                    sols, _, _ = nearest_ik(ctx, probe, Rc, rs.q, cs_probe, n=4)
                except IntrinsicRequestError:
                    sols = []
                for q in sols:
                    margin = float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q))))
                    if margin > best_margin:
                        best, best_margin = Rc, margin
            R_tcp = best if best is not None else rot_options[0]
            ctx.record(event="place_rotation_choice", limit_margin_rad=float(best_margin))
        rim_z = max(geo.object_box(env, b).top_z for b in self.support)
        hand_low = geo.hand_lowest_offset(R_tcp)   # lowest hand corner relative to the tcp (rotation aware)
        others = [b for b in sync.bodies if b != self.body]
        width = float(2 * max(np.abs((geo.object_point_cloud(env, self.body) - rs.tcp_pos) @ rs.tcp_rot[:, 0]).max(), 0.005))
        chosen = None
        for k, (target_origin, support_z) in enumerate(targets):
            release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
            tcp_release = release_origin - R_tcp @ tcp_t_obj_p
            # Container-aware limit: the wide hand body cannot enter a container (basket, drawer,
            # microwave, caddy): keep the hand above the rim ...
            if support_z < rim_z - 0.02:
                tcp_min_z = rim_z + self.p.rim_clearance - hand_low
                if tcp_release[2] < tcp_min_z:
                    ctx.record(event="place_rim_limit", rim_z=float(rim_z), tcp_release_z=float(tcp_release[2]), tcp_min_z=float(tcp_min_z),
                               drop_height=float(tcp_min_z - tcp_release[2]), candidate=k)
                    tcp_release[2] = tcp_min_z
            # ... and raise the release pose until hand AND fingers are clear of every other body
            # (geometric search, 1 cm steps, at most 15 cm above the nominal height).
            ok, blocker, raised = False, "", 0.0
            for raised in np.arange(0.0, 0.151, 0.01):
                cand = tcp_release + np.array([0, 0, raised])
                ok, blocker = geo.hand_clearance(env, cand, R_tcp, min(geo.GRIPPER_MAX_OPENING, width + 0.02), width, others)
                if ok:
                    break
            if ok:
                if raised > 0:
                    ctx.record(event="place_release_raised", candidate=k, raised_m=float(raised))
                chosen = (target_origin, support_z, tcp_release + np.array([0, 0, raised]))
                break
            ctx.record(event="place_spot_rejected", candidate=k, blocker=blocker, tcp=[float(v) for v in tcp_release])
        if chosen is None:
            target_origin, support_z = targets[0]
            release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
            tcp_release = release_origin - R_tcp @ tcp_t_obj_p
            if support_z < rim_z - 0.02:
                tcp_release[2] = max(tcp_release[2], rim_z + self.p.rim_clearance - hand_low)
        else:
            target_origin, support_z, tcp_release = chosen
        approach = R_tcp[:, 2]            # tcp z: down for top grasps, horizontal for side grasps
        tcp_pre = tcp_release - approach * self.p.preplace_clearance
        if approach[2] > -0.9:            # side grasp: also stay clear above the floor before entering
            tcp_pre = tcp_pre + np.array([0, 0, 0.02])
        cs_transport = sync.transport_collision_settings(self.support)
        traj, res = _plan_and_execute(ctx, "place_transport", rs.q, tcp_pre, R_tcp, cs_transport, "ANY", +1.0, self.p.plan_timeout_s)
        if not res.ok:
            return SkillResult(self.name, False, f"transport_exec:{res.reason}")
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
        ctx.open_gripper(int(np.ceil((geo.object_box(env, self.body).half_extents_world.max() * 2 + 0.03) / 0.01)))
        sync.detach(self.body)
        self._released = True
        ctx.hold(0.0, self.p.settle_steps)
        obj_pos = env.body_pose(self.body)[0]
        ctx.record(event="place_verify", obj_pos=[float(v) for v in obj_pos], target=[float(v) for v in target_origin],
                   xy_err=float(np.linalg.norm(obj_pos[:2] - target_origin[:2])), lower_stop=res.reason)
        return self._retreat(ctx, SkillResult(self.name, True, details={"xy_err": float(np.linalg.norm(obj_pos[:2] - target_origin[:2]))}))

    def _retreat(self, ctx, result: SkillResult) -> SkillResult:
        """Withdraw straight up. Contact with the just-released object and with the support/
        container (the open fingers may touch its walls) is intentional while withdrawing."""
        env, sync = ctx.env, ctx.sync
        sync.sync()
        rs = env.robot_state()
        cs_retreat = sync.grasp_collision_settings(self.body, support_bodies=self.support)
        back = -rs.tcp_rot[:, 2]          # withdraw along the negative approach axis (up, or out of a front opening)
        try:
            traj, res = _plan_and_execute(ctx, "place_retreat", rs.q, rs.tcp_pos + back * self.p.retreat_height, rs.tcp_rot,
                                          cs_retreat, "LINEAR", 0.0, self.p.plan_timeout_s)
            result.details["retreat_ok"] = res.ok
        except IntrinsicRequestError as e:
            result.details["retreat_ok"] = False
            result.details["retreat_error"] = str(e)[:200]
        return result
