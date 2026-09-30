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

from libero_intrinsic.env.executor import ExecutionConfig
from libero_intrinsic.intrinsic.client import IntrinsicRequestError
from libero_intrinsic.model import transforms as tf
from libero_intrinsic.skills import geometry as geo
from libero_intrinsic.skills.base import Skill, SkillContext, SkillResult


@dataclasses.dataclass
class PickParams:
    height_fraction: float = 0.5
    pregrasp_clearance: float = 0.10
    lift_height: float = 0.10
    yaws: Sequence[float] = ()
    max_candidates: int = 4
    close_steps: int = 12
    min_finger_gap: float = 0.004      # summed |finger q| after closing must exceed this
    min_pad_contacts: int = 1
    plan_timeout_s: float = 15.0
    z_offset: float = 0.0
    max_depth: float = 0.045


def _plan_and_execute(ctx: SkillContext, label: str, q_start, pos, rot, cs, motion_type="ANY",
                      gripper=-1.0, timeout_s=15.0, contact_monitor=None, time_scale=1.0):
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

    def recover(self, ctx, last):
        ctx.open_gripper(10)
        for b in ctx.sync.attached_bodies():
            ctx.sync.detach(b)
        rs = ctx.env.robot_state()
        ctx.sync.sync()
        # retreat straight up 8 cm (LINEAR, target object contact allowed)
        try:
            _plan_and_execute(ctx, "pick_recover_retreat", rs.q, rs.tcp_pos + [0, 0, 0.08], rs.tcp_rot,
                              ctx.sync.grasp_collision_settings(self.body), motion_type="LINEAR", gripper=-1.0,
                              timeout_s=5.0)
        except IntrinsicRequestError as e:
            ctx.record(event="recover_plan_failed", reason=str(e))

    def attempt(self, ctx, i):
        env, client, sync = ctx.env, ctx.client, ctx.sync
        ctx.open_gripper(6)
        sync.sync()
        cands = [c for c in geo.grasp_candidates(env, self.body, yaws=self.p.yaws) if c.label not in self._tried]
        if not cands:
            return SkillResult(self.name, False, "no_grasp_candidate")
        rs = env.robot_state()
        free_cs = sync.free_collision_settings()
        grasp_cs = sync.grasp_collision_settings(self.body)
        chosen = None
        for c in cands[: self.p.max_candidates]:
            pre = c.pos - c.approach * self.p.pregrasp_clearance
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
        traj, res = _plan_and_execute(ctx, "pick_pregrasp", rs.q, pre, c.rot, free_cs, "ANY", -1.0, self.p.plan_timeout_s)
        if not res.ok:
            return SkillResult(self.name, False, f"pregrasp_exec:{res.reason}", details={"grasp": c.label})
        # 2. linear approach; only contact with the target object is permitted
        rs = env.robot_state()
        traj, res = _plan_and_execute(ctx, "pick_approach", rs.q, c.pos, c.rot, grasp_cs, "LINEAR", -1.0, self.p.plan_timeout_s)
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
        lift_cs = sync.transport_collision_settings(support_bodies=ctx.params.get("support_bodies", []))
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
                 params: PlaceParams = PlaceParams(), keep_yaw: bool = True):
        self.body = body
        self.target_fn = target_fn  # returns (target_xy_z_of_object_origin, support_top_z)
        self.support = list(support_bodies)
        self.p = params

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
        rs = env.robot_state()
        target_origin, support_z = self.target_fn(rs.tcp_rot[:, 0])
        tcp_t_obj_p, tcp_t_obj_R = sync.attached[self.body]
        box = geo.object_box(env, self.body)
        obj_origin_above_bottom = env.body_pose(self.body)[0][2] - box.bottom_z
        # desired object origin at release: xy = target, z = support + gap + origin offset
        release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
        # keep the current tcp orientation (top-down); tcp position = obj_origin - R_tcp @ tcp_t_obj_p
        R_tcp = rs.tcp_rot
        tcp_release = release_origin - R_tcp @ tcp_t_obj_p
        # Container-aware limit: the wide hand body cannot enter a container (basket, drawer,
        # microwave, caddy). If the support surface lies below the support object's top (rim),
        # keep the hand above the rim and drop the object from there.
        rim_z = max(geo.object_box(env, b).top_z for b in self.support)
        if support_z < rim_z - 0.02:
            tcp_min_z = rim_z + self.p.rim_clearance - self.p.hand_below_tcp
            if tcp_release[2] < tcp_min_z:
                ctx.record(event="place_rim_limit", rim_z=float(rim_z), tcp_release_z=float(tcp_release[2]), tcp_min_z=float(tcp_min_z),
                           drop_height=float(tcp_min_z - tcp_release[2]))
                tcp_release[2] = tcp_min_z
        tcp_pre = tcp_release + np.array([0, 0, self.p.preplace_clearance])
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
            return SkillResult(self.name, False, f"lower_exec:{res.reason}")
        ctx.open_gripper(self.p.open_steps)
        sync.detach(self.body)
        ctx.hold(-1.0, self.p.settle_steps)
        rs = env.robot_state()
        obj_pos = env.body_pose(self.body)[0]
        ctx.record(event="place_verify", obj_pos=[float(v) for v in obj_pos], target=[float(v) for v in target_origin],
                   xy_err=float(np.linalg.norm(obj_pos[:2] - target_origin[:2])), lower_stop=res.reason)
        # retreat: contact with the just-released object is tolerated while the fingers withdraw
        cs_retreat = sync.grasp_collision_settings(self.body)
        traj, res = _plan_and_execute(ctx, "place_retreat", rs.q, rs.tcp_pos + [0, 0, self.p.retreat_height], rs.tcp_rot,
                                      cs_retreat, "LINEAR", -1.0, self.p.plan_timeout_s)
        return SkillResult(self.name, True, details={"xy_err": float(np.linalg.norm(obj_pos[:2] - target_origin[:2])),
                                                     "retreat_ok": res.ok})
