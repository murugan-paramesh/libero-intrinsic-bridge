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
from libero_intrinsic.skills.manipulation import _plan_and_execute


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
                 check_fn: Callable[[], bool], label: str = "push", extra_contact_bodies: Sequence[str] = ()):
        self.body, self.contact_fn, self.check_fn, self.label = body, contact_fn, check_fn, label
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
        pre, rot, path = self.contact_fn()
        rs = env.robot_state()
        free_cs = sync.free_collision_settings()
        push_cs = sync.grasp_collision_settings(self.body, [(client.robot, b) for b in self.extra])
        sols, rid, lat = client.ik(pre, rot, rs.q, max_solutions=4, collision_settings=free_cs)
        ctx.record(event="ik", label=f"{self.label}_pre", n_solutions=len(sols), request_id=rid, latency_s=lat)
        if not sols:
            return SkillResult(self.name, False, "precontact_unreachable")
        _, res = _plan_and_execute(ctx, f"{self.label}_precontact", rs.q, pre, rot, free_cs, "ANY", +1.0, 15.0)
        if not res.ok:
            return SkillResult(self.name, False, f"precontact_exec:{res.reason}")
        for k, p in enumerate(path):
            rs = env.robot_state()
            sync.sync()
            p_k, rot_k = (p if isinstance(p, tuple) else (p, rot))   # path points may carry their own orientation
            try:
                _, res = _plan_and_execute(ctx, f"{self.label}_seg{k}", rs.q, p_k, rot_k, push_cs, "LINEAR", +1.0, 10.0)
            except IntrinsicRequestError as e:
                return SkillResult(self.name, False, f"segment{k}_plan_failed:{e.code}")
            ctx.record(event="push_progress", segment=k, satisfied=bool(self.check_fn()))
            if self.check_fn():
                break
        ok = bool(self.check_fn())
        rs = env.robot_state()
        sync.sync()
        try:  # withdraw along the negative approach axis (up for top-down pushes, back for side pushes)
            _plan_and_execute(ctx, f"{self.label}_retreat", rs.q, rs.tcp_pos - rs.tcp_rot[:, 2] * 0.08, rs.tcp_rot, push_cs, "LINEAR", +1.0, 5.0)
        except IntrinsicRequestError:
            pass
        return SkillResult(self.name, ok, "" if ok else "push_goal_not_reached")
