#!/usr/bin/env python
"""Task 3 development lab (diagnostic only; never counts as an official episode).

Runs the real skill chain up to a stage on a DEVELOPMENT init state, then probes candidate
drawer-closing contacts directly with Intrinsic (collision-checked ComputeIk at the pre-contact,
contact, mid-travel and closed-drawer END poses; LINEAR dry-runs), WITHOUT the box-model
geometric pre-filter of stage_lab.py, and executes the best candidates through the real
PushSkill (state restored between executions).  Every rejection records Intrinsic's collision
pair.  New in this lab: contacts on the drawer's handle bar with the hand pitched forward-down
(the stage-A near-vertical hand is blocked by robot0_link5 vs the cabinet top at the end of
the travel; the stage-B horizontal hand by link5/link6 vs the wine rack and the table)."""
import argparse
import json
import os
import re
import sys
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from stage_lab import Lab, drawer_push_candidates, contacts_of, jsonable  # noqa: E402
from libero_intrinsic.intrinsic.client import IntrinsicRequestError  # noqa: E402
from libero_intrinsic.skills import geometry as geo  # noqa: E402
from libero_intrinsic.skills.articulation import PushSkill  # noqa: E402

DRAWER, JOINT, BOWL = "white_cabinet_1_cabinet_bottom", "white_cabinet_1_bottom_level", "akita_black_bowl_1_main"


def pair_of(err):
    m = re.search(r"Left object: (\S+)\s+Right objects?: (\S+)", str(err))
    if m:
        return m.group(1).replace("panda.", "") + "/" + m.group(2).split(".")[0]
    s = str(err)
    return "kinematic" if "constraints" in s or "reachable" in s else s[:50].replace("\n", " ")


def ik_probe(client, p, rot, cs, seeds, postures, lim, nmax=4):
    """all seeds x postures; returns (solutions, {posture/seed: reason})"""
    out, sols = {}, []
    for pname, jl in postures:
        for si, seed in enumerate(seeds):
            try:
                s, _, _ = client.ik(p, rot, seed, max_solutions=nmax, collision_settings=cs, joint_limits=jl)
                out[f"{pname}/{si}"] = "ok" if s else "no_solution"
                for q in s:
                    if not any(np.allclose(q, q0, atol=1e-3) for q0 in sols):
                        sols.append(np.asarray(q))
            except IntrinsicRequestError as e:
                out[f"{pname}/{si}"] = pair_of(e)
    return sols, out


def pitched_rotation(axis, tilt_deg):
    """hand pointing along `axis` (horizontal unit vector) and down by tilt_deg (as stage_lab 'pitched')"""
    from scipy.spatial.transform import Rotation as R
    yaw = np.arctan2(axis[1], axis[0])
    rot = geo.side_rotation(yaw, 0.0)
    ta = rot[:, 0] if abs(rot[2, 0]) < 0.5 else rot[:, 1]
    sign = 1.0 if np.dot(np.cross(ta, rot[:, 2]), np.array([0, 0, -1.0])) > 0 else -1.0
    return R.from_rotvec(ta * sign * np.radians(tilt_deg)).as_matrix() @ rot


def bowl_push_candidates(env, tilts=(50, 60), dz=(0.015, 0.025), r_in=0.040, y_target=None):
    """Push the bowl deeper into the drawer from INSIDE its cavity: closed fingertips against the
    bowl's far inner wall (relative to the push direction = drawer axis), hand pitched forward-down.
    Travel = distance from the bowl centre to the region centre along the axis (+ 1 cm)."""
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    c_r, rot_r, half_r = geo.site_box_world(env, "white_cabinet_1_bottom_region")
    bowl_c = env.body_pose(BOWL)[0]
    floor_z = geo.object_box(env, DRAWER).bottom_z + 0.0105     # drawer floor top (floor plate 0.918-0.926 on the measured model)
    goal = c_r if y_target is None else y_target
    travel = float(np.dot(goal - bowl_c, axis)) + 0.01
    out = []
    for t in tilts:
        for z in dz:
            rot = pitched_rotation(axis, t)
            contact = np.array([bowl_c[0], bowl_c[1], floor_z + z]) + axis * r_in
            pre = contact - rot[:, 2] * 0.08
            out.append({"family": {"mode": "bowl_inside", "tilt": t, "dz": z}, "pre": pre, "rot": rot, "path": [contact, contact + axis * travel],
                        "axis": axis, "travel": travel})
    return out


def bowl_rim_push_candidates(env, tilts=(0, 20), dz=(-0.003, 0.0), standoff=0.012, travel=0.045):
    """Top-down fingertip push on the bowl's NEAR rim (the rim point nearest the drawer front, which
    stands above the drawer panel while the bowl rests tilted on the inner wall): pushing it along
    the drawer axis slides the bowl's lower wall over the inner wall's top edge until the bowl
    drops level onto the drawer floor, deep enough for its near rim to clear the wall.  Fingers
    closed, closing axis lateral, hand vertical or leaning back by `tilt` deg."""
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    from scipy.spatial.transform import Rotation as R
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    c, Rb = env.body_pose(BOWL)
    local = geo.body_collision_points_local(env.model, BOWL)
    ring = local[local[:, 2] > 0.040]
    world = ring @ Rb.T + c
    n_rim = world[int(np.argmin(world @ axis))]
    yaw = np.arctan2(axis[1], axis[0])
    out = []
    for t in tilts:
        for z in dz:
            rot = geo.top_down_rotation(yaw)                      # closing axis lateral (across the push; top_down_rotation(0) closes along world y)
            ta = rot[:, 0] if abs(np.dot(rot[:, 0], axis)) < 0.5 else rot[:, 1]
            sign = 1.0 if np.dot(np.cross(ta, rot[:, 2]), axis) > 0 else -1.0
            rot = R.from_rotvec(ta * sign * np.radians(t)).as_matrix() @ rot      # fingers lean toward the push (palm back)
            contact = n_rim - axis * standoff + np.array([0, 0, z])
            pre = contact - rot[:, 2] * 0.08
            out.append({"family": {"mode": "bowl_rim", "tilt": t, "dz": z}, "pre": pre, "rot": rot, "path": [contact, contact + axis * travel],
                        "axis": axis, "travel": travel, "n_rim": n_rim})
    return out


def drawer_open_push_candidates(env, dz=(0.028, 0.035), standoff=0.010):
    """Pull the drawer fully open (to its joint limit) by pushing the inner face of its inner front
    wall from INSIDE the empty cavity with the closed fingertips (hand vertical, closing axis across
    the drawer, palm above the side walls).  LIBERO samples the initial opening in [-0.16, -0.14];
    the extra 1-2 cm of cavity in front of the upper handles is what a level bowl needs."""
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    q = env.joint_qpos(JOINT)
    lim_lo = float(env.model.jnt_range[env.model.joint_name2id(JOINT)][0])
    c_r, rot_r, half_r = geo.site_box_world(env, "white_cabinet_1_bottom_region")
    box = geo.object_box(env, DRAWER)
    front_inner = c_r - axis * half_r[1]            # inner face of the inner front wall (region front)
    yaw = np.arctan2(axis[1], axis[0])
    rot = geo.top_down_rotation(yaw)
    travel = float(q - lim_lo) + 0.004
    out = []
    for z in dz:
        contact = np.array([c_r[0], front_inner[1], box.top_z - z]) + axis * standoff
        pre = contact + np.array([0, 0, 0.08])
        out.append({"family": {"mode": "open_inside", "dz": z}, "pre": pre, "rot": rot, "path": [contact, contact - axis * travel], "axis": -axis, "travel": travel})
    return out


def bowl_level_ok(env):
    c_r, rot_r, half_r = geo.site_box_world(env, "white_cabinet_1_bottom_region")
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    from stage_lab import body_tilt_deg
    bowl_c = env.body_pose(BOWL)[0]
    depth = float(np.dot(bowl_c - c_r, axis))       # 0 at the region centre
    return body_tilt_deg(env, BOWL) < 8.0 and depth > -0.025, depth, body_tilt_deg(env, BOWL)


def near_rim_grasp(env, depth=0.010, wall_in=0.008, pitch_deg=40.0, side="x-"):
    """Pinch of the bowl's wall at a LATERAL rim point (max |x - x_c| on the chosen side): the
    closing axis is lateral (world x), so the 21.8 cm wide palm lies ACROSS the drawer and the
    hand's 7.4 cm thickness lies along the drawer axis; the hand is pitched `pitch_deg` forward-down
    (fingers pointing down and along the drawer axis) so the palm leans back, away from the upper
    handles.  The wall's +-x faces are vertical whatever the bowl's tilt about x, so the pinch holds
    the bowl at its current (rear-down) tilt."""
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    lateral = np.cross(np.array([0.0, 0.0, 1.0]), axis)          # world -x for a +y drawer axis
    c, Rb = env.body_pose(BOWL)
    local = geo.body_collision_points_local(env.model, BOWL)
    ring = local[local[:, 2] > 0.040]
    world = ring @ Rb.T + c
    proj = (world - c) @ lateral
    i = int(np.argmax(proj)) if side == "x-" else int(np.argmin(proj))
    n_rim = world[i]
    inward = -lateral if side == "x-" else lateral
    a = np.radians(pitch_deg)
    z_t = axis * np.sin(a) + np.array([0, 0, -np.cos(a)])
    x_t = lateral
    y_t = np.cross(z_t, x_t)
    rot = np.stack([x_t, y_t, z_t], axis=1)
    tcp = n_rim + inward * wall_in + z_t * depth
    return tcp, rot, n_rim, Rb[:, 2]


def probe_insert(lab, seeds, postures, lim, out_rec):
    """regrasp the bowl at its exposed near rim, carry it deeper (rear-down, under the handle bar),
    set it down on its rear bottom edge, release: gravity levels it on the drawer floor."""
    from libero_intrinsic.skills.manipulation import _plan_and_execute
    from libero_intrinsic.skills.articulation import joint_world_axis_and_anchor
    env, client, sync, ctx = lab.env, lab.client, lab.sync, lab.ctx
    axis, _ = joint_world_axis_and_anchor(env, JOINT)
    c_r, rot_r, half_r = geo.site_box_world(env, "white_cabinet_1_bottom_region")
    floor_z = geo.object_box(env, DRAWER).bottom_z + 0.0105
    tcp, rot, n_rim, zb = near_rim_grasp(env, pitch_deg=float(os.environ.get("T3_PITCH", "40")), side=os.environ.get("T3_SIDE", "x+"))
    pre = tcp - rot[:, 2] * 0.06
    rec = {"stage": "insert", "tcp": tcp, "pre": pre, "n_rim": n_rim, "bowl_z": zb}
    free_cs = sync.free_collision_settings()
    bowl_cs = sync.grasp_collision_settings(BOWL, [])
    sols_pre, why_pre = ik_probe(client, pre, rot, free_cs, seeds, postures, lim)
    sols_g, why_g = ik_probe(client, tcp, rot, bowl_cs, seeds, postures, lim)
    rec["ik_pre"] = {"n": len(sols_pre), "reasons": sorted(set(why_pre.values()))}
    rec["ik_grasp"] = {"n": len(sols_g), "reasons": sorted(set(why_g.values()))}
    print("  INSERT ik pre", rec["ik_pre"], "grasp", rec["ik_grasp"], flush=True)
    if not sols_pre or not sols_g:
        rec.update(ok=False, why="ik")
        out_rec.append(lab.log_candidate(rec)); return False
    rs = env.robot_state()
    # partial opening (the drawer's side wall is 1-2 cm beyond the outer finger when fully open)
    ctx.close_gripper(10)
    want = float(os.environ.get("T3_OPEN", "0.045"))
    for _ in range(12):
        fq = ctx.hold(-1.0, 1)
        fq = env.robot_state().finger_q
        if abs(fq[0]) + abs(fq[1]) >= want:
            break
    rec["opening"] = float(abs(fq[0]) + abs(fq[1]))
    sync.sync()
    try:
        _plan_and_execute(ctx, "t3lab_regrasp_pre", rs.q, pre, rot, free_cs, "ANY", 0.0, 15.0)
        rs = env.robot_state()
        _, res = _plan_and_execute(ctx, "t3lab_regrasp_approach", rs.q, tcp, rot, bowl_cs, "LINEAR", 0.0, 8.0)
    except IntrinsicRequestError as e:
        rec.update(ok=False, why="plan:" + pair_of(e)); out_rec.append(lab.log_candidate(rec)); print("  INSERT plan fail", rec["why"], flush=True); return False
    fq = ctx.close_gripper(10)
    gap = float(abs(fq[0]) + abs(fq[1])); contacts = env.gripper_contacts_with(BOWL)
    rec["grasp_verify"] = {"gap": gap, "pad_contacts": contacts, "approach_err": res.final_pos_err if hasattr(res, "final_pos_err") else None}
    print("  INSERT grasp gap %.4f contacts %d" % (gap, contacts), flush=True)
    if contacts < 1 or gap < 0.004:
        rec.update(ok=False, why="grasp_failed"); out_rec.append(lab.log_candidate(rec)); return False
    sync.attach(BOWL)
    carry_cs = sync.grasp_collision_settings(BOWL, [(BOWL, DRAWER), (client.robot, DRAWER)])
    # target bowl pose: same orientation as held (tilted rear-down), lowest point 5 mm above the floor,
    # centre at the region centre along the axis
    c_b, R_b = env.body_pose(BOWL)
    rs = env.robot_state()
    off_p = rs.tcp_rot.T @ (c_b - rs.tcp_pos)           # bowl origin in the tcp frame
    local = geo.body_collision_points_local(env.model, BOWL)
    world = local @ R_b.T + c_b
    lowest = float(world[:, 2].min())
    dz = (floor_z + 0.005) - lowest
    dy = float(np.dot(c_r - c_b, axis)) - 0.005
    try:
        rs = env.robot_state()
        _plan_and_execute(ctx, "t3lab_insert_lift", rs.q, rs.tcp_pos - rot[:, 2] * 0.025, rot, carry_cs, "LINEAR", +1.0, 8.0)
        rs = env.robot_state()
        # recompute precisely from the current bowl pose after the lift
        c_b2, R_b2 = env.body_pose(BOWL)
        world2 = local @ R_b2.T + c_b2
        target = rs.tcp_pos + axis * float(np.dot(c_r - c_b2, axis) + 0.013) + np.array([0, 0, (floor_z + 0.015) - float(world2[:, 2].min())])
        rec["insert_target_tcp"] = target
        _, res2 = _plan_and_execute(ctx, "t3lab_insert_move", rs.q, target, rot, carry_cs, "LINEAR", +1.0, 10.0, time_scale=1.5)
        rec["insert_exec"] = {"ok": res2.ok, "reason": res2.reason, "final_pos_err": getattr(res2, "final_pos_err", None)}
    except IntrinsicRequestError as e:
        rec.update(ok=False, why="insert_plan:" + pair_of(e)); out_rec.append(lab.log_candidate(rec)); print("  INSERT move plan fail", rec["why"], flush=True)
        ctx.open_gripper(8); sync.detach(BOWL); return False
    ctx.open_gripper(8)
    sync.detach(BOWL)
    ctx.hold(-1.0, 10)
    sync.sync()
    rs = env.robot_state()
    try:
        _plan_and_execute(ctx, "t3lab_insert_retreat", rs.q, rs.tcp_pos - rot[:, 2] * 0.08, rot, bowl_cs, "LINEAR", -1.0, 8.0)
    except IntrinsicRequestError as e:
        rec["retreat_fail"] = pair_of(e)
        try:
            rs = env.robot_state()
            _plan_and_execute(ctx, "t3lab_insert_retreat_up", rs.q, rs.tcp_pos + np.array([0, 0, 0.08]), rot, bowl_cs, "LINEAR", -1.0, 8.0)
        except IntrinsicRequestError as e2:
            rec["retreat_fail2"] = pair_of(e2)
    okb, depth, tilt = bowl_level_ok(env)
    from stage_lab import in_region_box
    rec.update(ok=bool(okb), bowl=env.body_pose(BOWL)[0].tolist(), bowl_depth=depth, bowl_tilt=tilt, level_ok=okb,
               in_region=in_region_box(env, "white_cabinet_1_bottom_region", BOWL)[0], bowl_contacts=contacts_of(env, BOWL)[:10])
    out_rec.append(lab.log_candidate(rec))
    print("  INSERT done: level_ok %s depth %.3f tilt %.1f in_region %s bowl %s" % (okb, depth, tilt, rec["in_region"], np.round(rec["bowl"], 3)), flush=True)
    return bool(okb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--init", type=int, default=0)
    ap.add_argument("--until", default="place")
    ap.add_argument("--out", required=True)
    ap.add_argument("--families", default=None)
    ap.add_argument("--execute", type=int, default=2)
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--probe", default="handle", help="handle | chain (bowl levelling push, then handle push)")
    a = ap.parse_args()
    lab = Lab(3, a.init, a.out, video=a.video)
    env, client, sync, ctx = lab.env, lab.client, lab.sync, lab.ctx
    print("skills:", [s.name for s in lab.skills], flush=True)
    opened = []
    if a.probe in ("open", "openchain"):
        lim_lo = float(env.model.jnt_range[env.model.joint_name2id(JOINT)][0])
        print("drawer q0 %.4f (limit %.3f)" % (env.joint_qpos(JOINT), lim_lo), flush=True)
        if env.joint_qpos(JOINT) > lim_lo + 0.003:
            for c in drawer_open_push_candidates(env):
                cand = c
                skill = PushSkill(DRAWER, lambda ctx=None, cand=cand: (cand["pre"], cand["rot"], cand["path"]),
                                  lambda: env.joint_qpos(JOINT) <= lim_lo + 0.003, label="t3lab_open",
                                  extra_contact_bodies=["white_cabinet_1_base"], partial_ok=True,
                                  progress_fn=lambda: -float(env.joint_qpos(JOINT)), time_scale=1.5)
                n0, s0 = len(lab.ctx.log), env.step_count
                res = skill.attempt(lab.ctx, 0)
                ev = lab.ctx.log[n0:]
                rec = {"family": c["family"], "stage": "open", "ok": bool(res.ok), "reason": res.reason, "steps": env.step_count - s0, "drawer_q": float(env.joint_qpos(JOINT)),
                       "executions": [{k: e.get(k) for k in ("label", "ok", "reason", "final_pos_err", "joint_err_final")} for e in ev if e["event"] == "execute"]}
                opened.append(lab.log_candidate(rec))
                print("  OPEN ", c["family"], "ok", res.ok, res.reason[:80], "drawer q %.4f" % env.joint_qpos(JOINT), flush=True)
                if env.joint_qpos(JOINT) <= lim_lo + 0.003:
                    break
    done = lab.run_until(a.until)
    audit = lab.audit([BOWL, "wine_bottle_1_main"], [JOINT], region="white_cabinet_1_bottom_region", region_body=BOWL)
    print("post-stage audit:", json.dumps(audit)[:600], flush=True)
    lab.take_snapshot()
    lim = env.joint_limits()
    fingers = list(sync.fingers.values())
    riders = [BOWL]
    extra_pairs = [(client.robot, "white_cabinet_1_base")] + [(client.robot, b) for b in riders] + [(f, b) for f in fingers for b in riders]
    push_cs = sync.grasp_collision_settings(DRAWER, extra_pairs)
    free_cs = sync.free_collision_settings()
    rs = env.robot_state()
    seeds = [rs.q, ctx.params.get("home_q", rs.q), np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]), np.array([0.0, 0.6, 0.0, -1.6, 0.0, 2.2, 0.8])]
    postures = [("free", None), ("shoulder_fwd", (np.maximum(lim[:, 0] + 0.05, [-9, 0.3, -9, -9, -9, -9, -9]), lim[:, 1] - 0.05)),
                ("elbow_bent", (lim[:, 0] + 0.05, np.minimum(lim[:, 1] - 0.05, [9, 9, 9, -1.9, 9, 9, 9])))]
    if a.families:
        fams = json.loads(a.families)
    else:
        box = geo.object_box(env, DRAWER)
        h_bar = (0.946 - box.bottom_z) / (box.top_z - box.bottom_z)
        fams = [{"mode": "pitched", "tilt": t, "h": round(h_bar + dh, 3), "xo": xo, "over": 0.02}
                for t in (30, 40, 50) for dh in (0.0, 0.12) for xo in (0.0, -0.06)]
    chain = []
    if a.probe == "insert":
        ok0, depth0, tilt0 = bowl_level_ok(env)
        print("bowl level/deep:", ok0, "depth %.3f tilt %.1f" % (depth0, tilt0), flush=True)
        if not ok0:
            probe_insert(lab, seeds, postures, lim, chain)
        lab.take_snapshot()
    if a.probe == "chain":
        ok0, depth0, tilt0 = bowl_level_ok(env)
        print("bowl level/deep:", ok0, "depth %.3f tilt %.1f" % (depth0, tilt0), flush=True)
        if not ok0:
            bowl_cs = sync.grasp_collision_settings(BOWL, [(client.robot, "white_cabinet_1_base")])
            gen = bowl_rim_push_candidates(env) if os.environ.get("T3_BOWL", "rim") == "rim" else bowl_push_candidates(env)
            for c in gen:
                rec = {"family": c["family"], "pre": c["pre"], "contact": c["path"][0], "end": c["path"][1]}
                feasible = True
                for name, p, cs in (("end", c["path"][1], bowl_cs), ("contact", c["path"][0], bowl_cs), ("pre", c["pre"], free_cs)):
                    sols, why = ik_probe(client, p, c["rot"], cs, seeds, postures, lim)
                    rec[f"ik_{name}"] = {"n": len(sols), "reasons": sorted(set(why.values()))}
                    if not sols:
                        feasible = False
                        rec.update(stage="ik", ok=False, why=f"{name}:" + "|".join(sorted(set(why.values()))[:3]))
                        break
                if not feasible:
                    chain.append(lab.log_candidate(rec))
                    print("  BOWL-IK", rec["family"], rec["why"][:150], flush=True)
                    continue
                cand = c
                skill = PushSkill(BOWL, lambda ctx=None, cand=cand: (cand["pre"], cand["rot"], cand["path"]), lambda: bowl_level_ok(env)[0],
                                  label="t3lab_bowl", extra_contact_bodies=["white_cabinet_1_base"], partial_ok=True,
                                  progress_fn=lambda: float(np.dot(env.body_pose(BOWL)[0], cand["axis"])), time_scale=2.0)
                n0, s0 = len(ctx.log), env.step_count
                res = skill.attempt(ctx, 0)
                ev = ctx.log[n0:]
                okb, depth, tilt = bowl_level_ok(env)
                rec.update(stage="executed", ok=bool(res.ok), reason=res.reason, steps=env.step_count - s0, bowl=env.body_pose(BOWL)[0].tolist(),
                           bowl_depth=depth, bowl_tilt=tilt, level_ok=okb,
                           executions=[{k: e.get(k) for k in ("label", "ok", "reason", "final_pos_err", "joint_err_final")} for e in ev if e["event"] == "execute"],
                           events=[{k: (str(v)[:160] if k == "reason" else v) for k, v in e.items() if k in ("event", "label", "reason", "segment", "satisfied", "progress")}
                                   for e in ev if e["event"] in ("approach_any_fallback", "push_progress", "push_partial", "recover_plan_failed", "start_state_recovery")],
                           bowl_contacts=contacts_of(env, BOWL)[:10])
                chain.append(lab.log_candidate(rec))
                print("  BOWL-EXEC", c["family"], "ok", res.ok, res.reason[:80], "depth %.3f tilt %.1f level_ok %s" % (depth, tilt, okb), "bowl", np.round(rec["bowl"], 3), flush=True)
                if okb:
                    break
                lab.restore()
        lab.take_snapshot()      # the levelled state becomes the base state for the handle push
    cands = drawer_push_candidates(env, DRAWER, JOINT, fams)
    results = []
    t0 = time.time()
    for c in cands:
        rec = {"family": c["family"], "pre": c["pre"], "contact": c["path"][0], "end": c["path"][1]}
        rot = c["rot"]
        mid = 0.5 * (c["path"][0] + c["path"][1])
        ok = True
        for name, p, cs in (("end", c["path"][1], push_cs), ("mid", mid, push_cs), ("contact", c["path"][0], push_cs), ("pre", c["pre"], free_cs)):
            sols, why = ik_probe(client, p, rot, cs, seeds, postures, lim)
            rec[f"ik_{name}"] = {"n": len(sols), "reasons": sorted(set(why.values()))}
            if name == "pre":
                pre_sols = sols
            if not sols:
                ok = False
                rec["stage"], rec["ok"], rec["why"] = "ik", False, f"{name}:" + "|".join(sorted(set(why.values()))[:3])
                break
        if not ok:
            results.append(lab.log_candidate(rec))
            print("  IK   ", rec["family"], rec["why"][:150], flush=True)
            continue
        pre_sols.sort(key=lambda q: -float(min(np.min(q - lim[:, 0]), np.min(lim[:, 1] - q))))
        lin_pre, lin_travel, q_pre = "", "", None
        for q in pre_sols[:6]:
            try:
                client.plan_to_pose(q, c["path"][0], rot, collision_settings=push_cs, motion_type="LINEAR", timeout_s=5.0, caller_id="t3lab_pre_dryrun")
                q_pre, lin_pre = q, "ok"
                break
            except IntrinsicRequestError as e:
                lin_pre = e.code + ":" + pair_of(e)
        if q_pre is not None:
            try:
                sols, _, _ = client.ik(c["path"][0], rot, q_pre, max_solutions=4, collision_settings=push_cs)
                client.plan_to_pose(sols[0], c["path"][1], rot, collision_settings=push_cs, motion_type="LINEAR", timeout_s=8.0, caller_id="t3lab_travel_dryrun")
                lin_travel = "ok"
            except IntrinsicRequestError as e:
                lin_travel = e.code + ":" + pair_of(e)
        rec.update(stage="planned", ok=(lin_pre == "ok"), linear_pre=lin_pre, linear_travel=lin_travel)
        results.append(lab.log_candidate(rec))
        print("  PLAN ", rec["family"], "pre:", lin_pre[:40], "travel:", lin_travel[:60], flush=True)
    print("probe time %.0f s" % (time.time() - t0), flush=True)
    ranked = [r for r in results if r.get("stage") == "planned" and r["ok"] and r["linear_travel"] == "ok"] + \
             [r for r in results if r.get("stage") == "planned" and r["ok"] and r["linear_travel"] != "ok"]
    executed = []
    for r in ranked[:a.execute]:
        lab.restore()
        q0 = env.joint_qpos(JOINT)
        fam = r["family"]
        cand = drawer_push_candidates(env, DRAWER, JOINT, [fam])[0]
        skill = PushSkill(DRAWER, lambda ctx=None, cand=cand: (cand["pre"], cand["rot"], cand["path"]), lambda: env.joint_qpos(JOINT) > 0.0,
                          label="t3lab_push", extra_contact_bodies=["white_cabinet_1_base"] + riders, partial_ok=True,
                          progress_fn=lambda: float(env.joint_qpos(JOINT)), time_scale=2.0)
        n0, s0 = len(ctx.log), env.step_count
        res = skill.attempt(ctx, 0)
        ev = ctx.log[n0:]
        rec = {"family": fam, "stage": "executed", "ok": bool(res.ok), "reason": res.reason, "steps": env.step_count - s0,
               "drawer_q_start": q0, "drawer_q_end": float(env.joint_qpos(JOINT)), "closed": bool(env.joint_qpos(JOINT) > 0.0),
               "success_predicate": bool(env.check_success()), "bowl": env.body_pose(BOWL)[0].tolist(),
               "bowl_in_region": lab.audit([BOWL], [JOINT], region="white_cabinet_1_bottom_region", region_body=BOWL).get("in_region_box"),
               "executions": [{k: e.get(k) for k in ("label", "ok", "reason", "final_pos_err", "joint_err_final")} for e in ev if e["event"] == "execute"],
               "events": [{k: (str(v)[:160] if k == "reason" else v) for k, v in e.items() if k in ("event", "label", "reason", "segment", "satisfied", "progress")}
                          for e in ev if e["event"] in ("approach_any_fallback", "push_progress", "push_partial", "precontact_path_check", "recover_plan_failed", "start_state_recovery")],
               "drawer_contacts_end": contacts_of(env, DRAWER)[:12], "bowl_contacts_end": contacts_of(env, BOWL)[:12]}
        executed.append(lab.log_candidate(rec))
        print("  EXEC ", fam, "ok", res.ok, res.reason[:80], "drawer %.3f -> %.3f" % (q0, env.joint_qpos(JOINT)), "success", rec["success_predicate"], "bowl", np.round(rec["bowl"], 3), flush=True)
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(jsonable({"opened": opened, "skills": done, "audit": audit, "chain": chain, "results": results, "executed": executed}), f, indent=1)
    print("\nSUMMARY: %d candidates, %d IK-feasible at all poses, %d with full LINEAR travel, %d executed, %d closed, %d official success" % (
        len(results), sum(1 for r in results if r.get("stage") == "planned"), sum(1 for r in results if r.get("linear_travel") == "ok"),
        len(executed), sum(r["closed"] for r in executed), sum(r["success_predicate"] for r in executed)), flush=True)
    lab.close()


if __name__ == "__main__":
    main()
