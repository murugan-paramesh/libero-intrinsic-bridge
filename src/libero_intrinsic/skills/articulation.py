"""Articulation-aware skills used by LIBERO-10: turning the flat-stove knob, closing a slide
drawer, closing the microwave door. All are contact-rich pushes/turns planned by Intrinsic
(LINEAR segments) with the pushed body declared as intentional contact; the world is re-synced
between segments because the articulated part moves.
"""
from __future__ import annotations

import dataclasses
from typing import Callable, List, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.env.executor import ExecutionConfig
from libero_intrinsic.intrinsic.client import IntrinsicRequestError
from libero_intrinsic.model import transforms as tf
from libero_intrinsic.skills import geometry as geo
from libero_intrinsic.skills.base import Skill, SkillContext, SkillResult
from libero_intrinsic.skills.manipulation import _recover_start_state, _plan_and_execute


def joint_world_axis_and_anchor(env, joint: str) -> Tuple[np.ndarray, np.ndarray]:
    """World-frame axis direction and a point on the axis for a MuJoCo joint."""
    m, d = env.model, env.data
    jid = m.joint_name2id(joint)
    bid = int(m.jnt_bodyid[jid])
    body = m.body_id2name(bid)
    pos, rot = env.body_pose(body)
    axis = rot @ np.array(m.jnt_axis[jid])
    anchor = pos + rot @ np.array(m.jnt_pos[jid])
    return axis / np.linalg.norm(axis), anchor


class TurnKnobSkill(Skill):
    """Pinch the knob top-down and rotate the tcp about the knob's (vertical) hinge axis."""
    name = "turn_knob"
    max_attempts = 3

    def __init__(self, knob_body: str, joint: str, target_qpos: float, margin_rad: float = 0.35):
        self.body, self.joint, self.target, self.margin = knob_body, joint, target_qpos, margin_rad

    def recover(self, ctx, last):
        ctx.open_gripper(8)
        rs = ctx.env.robot_state()
        ctx.sync.sync()
        try:
            _plan_and_execute(ctx, "knob_recover_up", rs.q, rs.tcp_pos + [0, 0, 0.06], rs.tcp_rot,
                              ctx.sync.grasp_collision_settings(self.body), "LINEAR", -1.0, 5.0)
        except IntrinsicRequestError as e:
            ctx.record(event="recover_plan_failed", reason=str(e))

    def attempt(self, ctx, i):
        env, client, sync = ctx.env, ctx.client, ctx.sync
        ctx.open_gripper(6)
        sync.sync()
        q0 = env.joint_qpos(self.joint)
        axis, anchor = joint_world_axis_and_anchor(env, self.joint)
        cands = geo.grasp_candidates(env, self.body)
        # after attempt failures, try the other yaw candidates
        cands = cands[i:] + cands[:i]
        if not cands:
            return SkillResult(self.name, False, "no_knob_grasp")
        rs = env.robot_state()
        grasp_cs = sync.grasp_collision_settings(self.body)
        free_cs = sync.free_collision_settings()
        chosen = None
        for c in cands[:4]:
            pre = c.pos + np.array([0, 0, 0.08])
            sols, rid, lat = client.ik(pre, c.rot, rs.q, max_solutions=4, collision_settings=free_cs)
            if sols:
                chosen = (c, pre)
                break
        if chosen is None:
            return SkillResult(self.name, False, "knob_unreachable")
        c, pre = chosen
        _, res = _plan_and_execute(ctx, "knob_pregrasp", rs.q, pre, c.rot, free_cs, "ANY", -1.0, 15.0)
        if not res.ok:
            return SkillResult(self.name, False, f"pregrasp_exec:{res.reason}")
        rs = env.robot_state()
        _, res = _plan_and_execute(ctx, "knob_approach", rs.q, c.pos, c.rot, grasp_cs, "LINEAR", -1.0, 10.0)
        if not res.ok:
            return SkillResult(self.name, False, f"approach_exec:{res.reason}")
        ctx.close_gripper(10)
        # rotate about the hinge axis: tcp pose -> rotated about (axis, anchor) by delta
        delta = (self.target + self.margin) - q0
        rs = env.robot_state()
        n_seg = max(int(np.ceil(abs(delta) / 0.5)), 1)
        for k in range(1, n_seg + 1):
            ang = delta * k / n_seg
            Rk = R.from_rotvec(axis * ang).as_matrix()
            p_goal = anchor + Rk @ (rs.tcp_pos - anchor)
            R_goal = Rk @ rs.tcp_rot
            rs_now = env.robot_state()
            try:
                _, res = _plan_and_execute(ctx, f"knob_turn_{k}", rs_now.q, p_goal, R_goal, grasp_cs, "LINEAR", +1.0, 10.0)
            except IntrinsicRequestError as e:
                return SkillResult(self.name, False, f"turn_plan_failed:{e.code}")
            q_now = env.joint_qpos(self.joint)
            ctx.record(event="knob_progress", segment=k, qpos=q_now, target=self.target)
            if q_now >= self.target:
                break
        ctx.open_gripper(8)
        q_now = env.joint_qpos(self.joint)
        rs = env.robot_state()
        sync.sync()
        _plan_and_execute(ctx, "knob_retreat", rs.q, rs.tcp_pos + [0, 0, 0.08], rs.tcp_rot, grasp_cs, "LINEAR", -1.0, 5.0)
        ok = q_now >= self.target
        return SkillResult(self.name, ok, "" if ok else f"knob_qpos={q_now:.3f}<{self.target}", details={"qpos": q_now})


class PushSkill(Skill):
    """Generic push: move the closed gripper to a pre-contact pose (free-space plan), then
    execute LINEAR segments along `path_fn()` (world points for the tcp) keeping the tcp
    orientation; contact with `body` (the pushed part) is intentional. Success is judged by
    `check_fn()` (e.g. joint qpos threshold)."""
    name = "push"
    max_attempts = 3

    def __init__(self, body: str, contact_fn: Callable[[], Tuple[np.ndarray, np.ndarray, List[np.ndarray]]],
                 check_fn: Callable[[], bool], label: str = "push", extra_contact_bodies: Sequence[str] = (),
                 partial_ok: bool = False, progress_fn: Callable[[], float] = None, time_scale: float = 1.0):
        self.body, self.contact_fn, self.check_fn, self.label = body, contact_fn, check_fn, label
        self.time_scale = time_scale          # > 1 slows the push segments (objects riding on the pushed body slide less)
        # partial_ok: a push that cannot be planned to the end but moved the articulation counts
        # as a success (a later stage continues from the new state); progress_fn measures it
        self.partial_ok, self.progress_fn = partial_ok, progress_fn
        self.extra = list(extra_contact_bodies)
        self.name = label

    def recover(self, ctx, last):
        rs = ctx.env.robot_state()
        ctx.sync.sync()
        try:
            _plan_and_execute(ctx, f"{self.label}_recover_up", rs.q, rs.tcp_pos + [0, 0, 0.08], rs.tcp_rot,
                              ctx.sync.grasp_collision_settings(self.body, [(ctx.client.robot, b) for b in self.extra]),
                              "LINEAR", +1.0, 5.0)
        except IntrinsicRequestError as e:
            ctx.record(event="recover_plan_failed", reason=str(e))

    def attempt(self, ctx, i):
        env, client, sync = ctx.env, ctx.client, ctx.sync
        ctx.close_gripper(8)
        sync.sync()
        if self.check_fn():
            return SkillResult(self.name, True, "already_satisfied")
        pre, rot, path = self.contact_fn(ctx)
        progress0 = self.progress_fn() if self.progress_fn is not None else 0.0
        rs = env.robot_state()
        free_cs = sync.free_collision_settings()
        push_cs = sync.grasp_collision_settings(self.body, [(client.robot, b) for b in self.extra])
        jl = ctx.params.pop("push_joint_limits", None)   # posture family chosen by the contact selection (if any)
        lim = env.joint_limits()
        # posture families for the pre-contact (IK JointPositionLimits): the chosen one, unrestricted,
        # shoulder forward (joint 2 >= 0.3 rad), elbow bent (joint 4 <= -1.9 rad)
        families = [jl, None,
                    (np.maximum(lim[:, 0] + 0.05, [-9, 0.3, -9, -9, -9, -9, -9]), lim[:, 1] - 0.05),
                    (lim[:, 0] + 0.05, np.minimum(lim[:, 1] - 0.05, [9, 9, 9, -1.9, 9, 9, 9]))]
        sols = []
        # C6: the numerical IK is seeded; from the configuration the arm is in after the previous
        # skill, every posture family returned only colliding branches on one protocol state
        # (task 3 state 18: link6 vs the wine rack), while the same pose was feasible from other
        # seeds. Extra seeds are tried only when the current configuration yields nothing.
        extra_seeds = [np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]), np.array([0.0, 0.6, 0.0, -1.6, 0.0, 2.2, 0.8])]
        for seed_i, seed in enumerate([rs.q] + extra_seeds):
            for fam in families:
                try:
                    sols, rid, lat = client.ik(pre, rot, seed, max_solutions=8, collision_settings=free_cs, joint_limits=fam)
                except IntrinsicRequestError:
                    sols = []
                ctx.record(event="ik", label=f"{self.label}_pre", n_solutions=len(sols), request_id=rid if sols else "", latency_s=lat if sols else 0.0,
                           seed=seed_i)
                if sols:
                    break
            if sols:
                if seed_i:
                    ctx.record(event="precontact_seed_fallback", label=self.label, seed=seed_i)
                break
        if not sols:
            return SkillResult(self.name, False, "precontact_unreachable")
        # Continuity pre-validation ("feasible endpoint" is not "feasible continuous path"): for
        # each pre-contact IK solution (several seeds and posture families), ask Intrinsic for the
        # LINEAR plan to the first contact pose WITHOUT executing it; the first solution with a
        # feasible plan, preferring the largest joint-limit margin, becomes the free-space goal.
        # (Observed: the nearest/margin-best solution alone failed with "FinePathIK ... excessive
        # change in joint config" on 3/10 protocol states although other solutions admit the path.)
        p0 = path[0] if not isinstance(path[0], tuple) else path[0][0]
        seeds = [rs.q] + ([ctx.params["home_q"]] if "home_q" in ctx.params else [])
        pool = list(sols)
        for seed in seeds:
            for fam in families[1:]:
                try:
                    more, _, _ = client.ik(pre, rot, seed, max_solutions=8, collision_settings=free_cs, joint_limits=fam)
                    pool += [q for q in more if not any(np.allclose(q, s0, atol=1e-3) for s0 in pool)]
                except IntrinsicRequestError:
                    pass
        # Order: configurations the OSC controller can actually reach first, i.e. nearest to the
        # current configuration (weighted joint distance, proximal joints weigh more), among those
        # with a joint-limit margin. Observed (task 9 door): the margin-best solution was 2.3 rad
        # from the current posture; the controller reached the TCP in another branch at the
        # joint-5 limit, from which no LINEAR push segment could be planned.
        w = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.5])
        margin_of = lambda q: float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q))))
        dist_of = lambda q: float(np.sum(w * (np.asarray(q) - rs.q) ** 2))
        pool.sort(key=lambda q: (dist_of(q) > 4.0, margin_of(q) < 0.12, -margin_of(q)))   # same branch first, then margin
        q_goal, n_tested = None, 0
        for q in pool[:12]:
            n_tested += 1
            try:
                client.plan_to_pose(q, p0, rot, collision_settings=push_cs, motion_type="LINEAR", timeout_s=5.0,
                                    caller_id=f"{self.label}_precontact_dryrun")
                q_goal = q
                break
            except IntrinsicRequestError:
                continue
        ctx.record(event="precontact_path_check", n_solutions=len(pool), n_tested=n_tested, feasible=q_goal is not None)
        if q_goal is not None:
            try:
                traj = client.plan_to_joints(rs.q, q_goal, collision_settings=free_cs, motion_type="ANY", timeout_s=15.0,
                                             caller_id=f"{self.label}_precontact")
            except IntrinsicRequestError as e_pc:
                # the arm may still touch the pushed body after a failed attempt: bounded retreat first
                if "Invalid initial joint configuration" not in str(e_pc) or not _recover_start_state(ctx, f"{self.label}_precontact", e_pc, push_cs):
                    raise
                rs = env.robot_state()
                traj = client.plan_to_joints(rs.q, q_goal, collision_settings=free_cs, motion_type="ANY", timeout_s=15.0,
                                             caller_id=f"{self.label}_precontact")
            ctx.record(event="plan", label=f"{self.label}_precontact", trajectory_id=traj.request_id, motion_type="ANY",
                       n_states=int(len(traj.t)), duration_s=traj.duration, latency_s=traj.planning_latency_s, target_pos=[float(v) for v in pre])
            res = ctx.execute(traj, ExecutionConfig(gripper=+1.0), f"{self.label}_precontact")
            rs2 = env.robot_state()
            if res.ok and float(np.max(np.abs(rs2.q - q_goal))) > 0.02:
                # execution feedback: the LINEAR path was validated from q_goal; converge to it in
                # joint space (the OSC controller leaves a few centiradians of error) before pushing
                try:
                    traj2 = client.plan_to_joints(rs2.q, q_goal, collision_settings=free_cs, motion_type="JOINT", timeout_s=5.0,
                                                  caller_id=f"{self.label}_precontact_settle")
                    ctx.execute(traj2, ExecutionConfig(gripper=+1.0), f"{self.label}_precontact_settle")
                except IntrinsicRequestError as e:
                    ctx.record(event="precontact_settle_failed", reason=str(e)[:120])
        else:
            _, res = _plan_and_execute(ctx, f"{self.label}_precontact", rs.q, pre, rot, free_cs, "ANY", +1.0, 15.0, joint_limits=jl,
                                       posture="margin")
        if not res.ok:
            return SkillResult(self.name, False, f"precontact_exec:{res.reason}")
        # Long straight pushes are split into <= 4 cm segments, each planned after a world sync:
        # objects carried along by the pushed body (a bowl inside the drawer) are then at their
        # true poses in the Intrinsic world instead of the stale pre-push poses (observed: the
        # end-pose IK of a one-segment 20 cm drawer push reported finger vs bowl collisions).
        expanded, prev = [], None
        for p in path:
            p_k, rot_k = (p if isinstance(p, tuple) else (p, rot))
            if prev is not None and not isinstance(p, tuple):
                n = int(np.ceil(np.linalg.norm(p_k - prev) / 0.04))
                for j in range(1, n):
                    expanded.append((prev + (p_k - prev) * j / n, rot_k))
            expanded.append((p_k, rot_k))
            prev = p_k
        path = expanded
        prev_ok = False
        for k, p in enumerate(path):
            rs = env.robot_state()
            sync.sync()
            p_k, rot_k = p
            try:
                try:
                    _, res = _plan_and_execute(ctx, f"{self.label}_seg{k}", rs.q, p_k, rot_k, push_cs, "LINEAR", +1.0, 10.0,
                                               time_scale=self.time_scale)
                except IntrinsicRequestError as e0:
                    if "FinePathIK" not in str(e0):
                        raise
                    # later segments: allowed once the previous segment executed (the hand is at the
                    # contact) or the articulation has moved; observed on the microwave door: the
                    # first push segment after the approach fails FinePathIK at the wrist limit
                    if k != 0 and not (prev_ok or (self.progress_fn is not None and self.progress_fn() > progress0 + 0.005)):
                        raise
                    # The benchmark's OSC_POSE controller tracks the TCP pose only; its null space
                    # drifts, so the posture Intrinsic validated for the LINEAR segment is not the
                    # one physically reached (joint error 0.6-1.0 rad observed) and the linear path
                    # IK fails from the reached branch. Fallback for the APPROACH segment only: a
                    # configuration-space (ANY) plan to the same contact pose, collision-checked
                    # with nothing excluded (the contact pose is 1.5 cm outside the panel), i.e. a
                    # non-straight but collision-free approach. Recorded explicitly.
                    # (also for a later segment once the articulation has moved: a short
                    # configuration-space move to the next waypoint keeps pushing; the pushed body
                    # stays excluded from the collision check there, as in the LINEAR segment)
                    ctx.record(event="approach_any_fallback", label=f"{self.label}_seg{k}", reason=str(e0)[:120])
                    _, res = _plan_and_execute(ctx, f"{self.label}_seg{k}_any", rs.q, p_k, rot_k, free_cs if k == 0 else push_cs, "ANY", +1.0, 10.0,
                                               time_scale=self.time_scale)
            except IntrinsicRequestError as e:
                if self.partial_ok and self.progress_fn is not None and self.progress_fn() > progress0 + 0.02:
                    ctx.record(event="push_partial", segment=k, progress=float(self.progress_fn() - progress0), reason=e.code)
                    break
                return SkillResult(self.name, False, f"segment{k}_plan_failed:{e.code}")
            prev_ok = bool(res.ok)
            ctx.record(event="push_progress", segment=k, satisfied=bool(self.check_fn()),
                       progress=float(self.progress_fn()) if self.progress_fn is not None else None)
            if self.check_fn():
                break
        ok = bool(self.check_fn())
        if ok:
            ctx.hold(+1.0, 10)        # keep the contact a moment: a slammed door/drawer settles against its stop
        if self.partial_ok and self.progress_fn is not None and not ok:
            ok = self.progress_fn() > progress0 + 0.02
        rs = env.robot_state()
        sync.sync()
        try:  # withdraw along the negative approach axis (up for top-down pushes, back for side pushes)
            _plan_and_execute(ctx, f"{self.label}_retreat", rs.q, rs.tcp_pos - rs.tcp_rot[:, 2] * 0.08, rs.tcp_rot, push_cs, "LINEAR", +1.0, 5.0)
        except IntrinsicRequestError:
            pass
        sync.sync()
        if ok and not self.partial_ok and not self.check_fn():
            # the goal held while the hand was in contact and was lost after the retreat (observed:
            # the microwave door rebounded from -0.005 to -0.28 rad); the next attempt re-contacts
            ctx.record(event="push_goal_lost_after_retreat", progress=float(self.progress_fn()) if self.progress_fn else None)
            return SkillResult(self.name, False, "push_goal_lost_after_retreat")
        return SkillResult(self.name, ok, "" if ok else "push_goal_not_reached")
