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
        self.place_skill = None   # the PlaceSkill that follows (placement-aware grasp ranking), set by the planner

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

        supports = support_of(env, sync, self.body)
        plane_z = max((geo.object_box(env, b).top_z for b in supports), default=-np.inf)

        obj_top = geo.object_box(env, self.body).top_z

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
        if self.place_skill is not None and len(cands) > 1:
            # placement-aware ranking: grasps that admit a geometrically valid release pose at the
            # placement target come first (e.g. a bowl going into a drawer under a cabinet must be
            # held so that the wide hand stays in front of the cabinet), ties keep the grasp score
            compat = []
            for c in cands[: max(self.p.max_candidates, 24)]:
                try:
                    ok, drop = self.place_skill.grasp_compatible(env, sync, c.pos, c.rot)
                except Exception as e:   # geometry failure must not block picking
                    ctx.record(event="grasp_placement_check_error", reason=str(e)[:200])
                    ok, drop = True, 0.0
                compat.append((c, ok, drop))
            rest = cands[len(compat):]
            # feasible grasps first, ordered by the drop height after release (1 cm bins: a grasp
            # that lets the object hang deeper into a container is released closer to the floor),
            # ties keep the grasp score order
            good = sorted([t for t in compat if t[1]], key=lambda t: round(max(t[2], 0.0), 2))
            cands = [c for c, _, _ in good] + [c for c, ok, _ in compat if not ok] + rest
            ctx.record(event="grasp_placement_filter", n_checked=len(compat), n_compatible=len(good),
                       first=cands[0].label if cands else "", first_drop_m=float(good[0][2]) if good else None)
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
            angles = [a for a, cost in fits if cost <= best_cost + 0.005][:4]
            if any(abs(a) > 1e-6 for a in angles):
                rot_options = [R.from_euler("z", a).as_matrix() @ R_tcp for a in angles]
                ctx.record(event="place_fit_rotation", yaw_deg=[float(np.degrees(a)) for a in angles], overhang_m=float(best_cost))
        if len(rot_options) > 1 or rot_options[0] is not R_tcp:
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
        obj_local = (geo.object_point_cloud(env, self.body, spacing=0.012) - rs.tcp_pos) @ rs.tcp_rot
        width = float(2 * max(np.abs(obj_local[:, 0]).max(), 0.005))
        targets = self._region_box_targets(ctx, targets, obj_origin_above_bottom)
        found = self._release_search(env, sync, targets, tcp_t_obj_p, R_tcp, obj_local, width, obj_origin_above_bottom,
                                     record=ctx.record)
        if found is None:
            target_origin, support_z = targets[0]
            release_origin = np.array([target_origin[0], target_origin[1], support_z + self.p.release_gap + obj_origin_above_bottom])
            tcp_release = release_origin - R_tcp @ tcp_t_obj_p
            rim_z = max(geo.object_box(env, b).top_z for b in self.support)
            if support_z < rim_z - 0.02:
                tcp_release[2] = max(tcp_release[2], rim_z + self.p.rim_clearance - geo.hand_lowest_offset(R_tcp))
            approach = R_tcp[:, 2]
            tcp_pre = tcp_release - approach * self.p.preplace_clearance
            if approach[2] > -0.9:
                tcp_pre = tcp_pre + np.array([0, 0, 0.02])
        else:
            target_origin, support_z, tcp_release, tcp_pre, _ = found
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
        xy_corr = np.array([target_origin[0] - obj_now[0], target_origin[1] - obj_now[1], 0.0])
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
        ctx.open_gripper(int(np.ceil((geo.object_box(env, self.body).half_extents_world.max() * 2 + 0.03) / 0.01)))
        sync.detach(self.body)
        self._released = True
        ctx.hold(0.0, self.p.settle_steps)
        obj_pos = env.body_pose(self.body)[0]
        ctx.record(event="place_verify", obj_pos=[float(v) for v in obj_pos], target=[float(v) for v in target_origin],
                   xy_err=float(np.linalg.norm(obj_pos[:2] - target_origin[:2])), lower_stop=res.reason)
        return self._retreat(ctx, SkillResult(self.name, True, details={"xy_err": float(np.linalg.norm(obj_pos[:2] - target_origin[:2]))}))

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
        if rest_origin_z >= box_bottom + 0.002:
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

    def _release_search(self, env, sync, targets, tcp_t_obj_p, R_tcp, obj_local, width, obj_origin_above_bottom, record=None):
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

        def clear_of(tcp_p, bodies, threshold, walls_only=False):
            pts = obj_local @ R_tcp.T + tcp_p
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
            # Container-aware limit: the wide hand body cannot enter a container (basket, drawer,
            # microwave, caddy): keep the hand above the rim ...
            if container:
                tcp_min_z = rim_z + self.p.rim_clearance - hand_low
                if tcp_release[2] < tcp_min_z:
                    if record:
                        record(event="place_rim_limit", rim_z=float(rim_z), tcp_release_z=float(tcp_release[2]), tcp_min_z=float(tcp_min_z),
                               drop_height=float(tcp_min_z - tcp_release[2]), candidate=k)
                    tcp_release[2] = tcp_min_z
            # ... and raise the release pose until hand, fingers and the carried object are clear
            ok, blocker, raised, pre, trace = False, "", 0.0, None, []
            for raised in np.arange(0.0, 0.151, 0.01):
                cand = tcp_release + np.array([0, 0, raised])
                if blocker:
                    trace.append(blocker)
                ok, blocker = geo.hand_clearance(env, cand, R_tcp, opening, width, others)
                if not ok:
                    blocker = "hand:" + blocker
                if ok:
                    ok, blocker = clear_of(cand, non_support, 0.008)
                    blocker = blocker and "obj:" + blocker
                if ok:
                    ok, blocker = clear_of(cand, self.support, 0.005, walls_only=True)
                    blocker = blocker and "objwall:" + blocker
                if ok:
                    pre = cand - approach * self.p.preplace_clearance
                    if approach[2] > -0.9:            # side grasp: also stay clear above the floor before entering
                        pre = pre + np.array([0, 0, 0.02])
                    elif container:                   # top grasp: the object must be above the rim before descending
                        pre[2] = max(pre[2], rim_z + 0.03 - bottom_off)
                    ok, blocker = geo.hand_clearance(env, pre, R_tcp, opening, width, others)
                    if ok:
                        ok, blocker = clear_of(pre, non_support + list(self.support), 0.005)
                    if not ok:
                        blocker = "pre:" + blocker
                if ok:
                    break
            if ok:
                if raised > 0 and record:
                    record(event="place_release_raised", candidate=k, raised_m=float(raised), blockers=trace[:16])
                return target_origin, support_z, tcp_release + np.array([0, 0, raised]), pre, float(raised)
            if record:
                record(event="place_spot_rejected", candidate=k, blocker=blocker, tcp=[float(v) for v in tcp_release])
        return None

    def grasp_compatible(self, env, sync, grasp_pos, grasp_rot) -> Tuple[bool, float]:
        """Placement-aware grasp selection: would this grasp (tcp pose on the object at its
        current pose) admit a valid release pose at the placement target, and how far would the
        object drop after release? Pure geometry (the same release search as at place time,
        every fit rotation considered); no IK. Returns (feasible, drop_height_m)."""
        obj_p, obj_R = env.body_pose(self.body)
        T_rel = tf.inv_T(tf.make_T(grasp_pos, grasp_rot)) @ tf.make_T(obj_p, obj_R)
        tcp_t_obj_p = T_rel[:3, 3]
        obj_local = (geo.object_point_cloud(env, self.body, spacing=0.012) - grasp_pos) @ grasp_rot
        width = float(2 * max(np.abs(obj_local[:, 0]).max(), 0.005))
        box = geo.object_box(env, self.body)
        obj_origin_above_bottom = obj_p[2] - box.bottom_z
        targets = self.target_fn(grasp_rot[:, 0])
        if not isinstance(targets, list):
            targets = [targets]
        rots = [grasp_rot]
        if self.region is not None and placement.is_container_region(env, self.region):
            fits = placement.fit_rotations(env, self.region, self.body)
            angles = [a for a, cost in fits if cost <= fits[0][1] + 0.005][:4]
            rots = [R.from_euler("z", a).as_matrix() @ grasp_rot for a in angles]
        best = None
        for Rc in rots:
            found = self._release_search(env, sync, targets, tcp_t_obj_p, Rc, obj_local, width, obj_origin_above_bottom)
            if found is None:
                continue
            target_origin, support_z, tcp_release, _, _ = found
            bottom = float((obj_local @ Rc.T + tcp_release)[:, 2].min())
            drop = bottom - support_z          # how far the object falls after release
            if best is None or drop < best:
                best = drop
        return best is not None, (best if best is not None else float("inf"))

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
