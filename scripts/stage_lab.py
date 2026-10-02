#!/usr/bin/env python
"""Development stage lab (NOT part of the evaluation): run the task's real skill sequence up to a
stage, audit the physical state, then search a bounded, geometry-relative candidate family for
the unresolved stage through the pipeline

    geometric filter -> Intrinsic ComputeIk (collision checked) -> Intrinsic LINEAR dry-run of the
    contact path -> physical execution in LIBERO (the genuine PushSkill code path) -> measurement.

Every candidate's rejection reason or measured outcome is appended to <out>/candidates.jsonl so a
failed search is not repeated. Between executed candidates the MuJoCo state is restored to the
post-stage snapshot and the Intrinsic world re-synchronised: this is a development-only device
(the evaluation runner never restores state). Candidates are generated from object/drawer geometry
only; no init-state identity is used anywhere.

Usage:
  python scripts/stage_lab.py --task 3 --init 0 --until place --probe drawer_push --execute 6 --out runs/lab/t3_i0
"""
import argparse
import dataclasses
import json
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]
os.environ.setdefault("MUJOCO_GL", "egl")

from libero_intrinsic.env.executor import TrajectoryExecutor  # noqa: E402
from libero_intrinsic.env.libero_env import list_tasks  # noqa: E402
from libero_intrinsic.eval.runner import DEFAULT_BINARY, EpisodeConfig, TaskSession  # noqa: E402
from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicRequestError, RequestLog  # noqa: E402
from libero_intrinsic.intrinsic.world_sync import WorldSync  # noqa: E402
from libero_intrinsic.skills import geometry as geo  # noqa: E402
from libero_intrinsic.skills.articulation import PushSkill, joint_world_axis_and_anchor  # noqa: E402
from libero_intrinsic.skills.base import SkillContext  # noqa: E402
from libero_intrinsic.skills.planner import build_skill_sequence  # noqa: E402
from libero_intrinsic.skills.task_spec import task_spec_from_env  # noqa: E402


STRICT_RIDERS = os.environ.get("LAB_STRICT", "0") == "1"   # 1: reject candidates by the model of bodies riding in the drawer (else physics decides)


def jsonable(o):
    if isinstance(o, np.ndarray):
        return [float(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, dict):
        return {k: jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    return o


def contacts_of(env, body_root):
    """Active contacts (geom names, dist) touching any geom of body subtree `body_root`."""
    d, m = env.sim.data, env.sim.model
    root = m.body_name2id(body_root)

    def in_sub(bid):
        while bid > 0:
            if bid == root:
                return True
            bid = m.body_parentid[bid]
        return bid == root
    out = []
    for i in range(d.ncon):
        c = d.contact[i]
        if in_sub(m.geom_bodyid[c.geom1]) or in_sub(m.geom_bodyid[c.geom2]):
            out.append((m.geom_id2name(c.geom1), m.geom_id2name(c.geom2), round(float(c.dist), 4)))
    return out


def body_tilt_deg(env, body):
    _, rot = env.body_pose(body)
    return float(np.degrees(np.arccos(np.clip(rot[2, 2], -1, 1))))


def in_region_box(env, region, body):
    c, rot, half = geo.site_box_world(env, region)
    p = env.body_pose(body)[0]
    local = rot.T @ (p - c)
    return bool(np.all(np.abs(local) <= half)), [float(v) for v in local], [float(v) for v in half]


class Lab:
    def __init__(self, task_index, init, out, video=False):
        self.task = list_tasks()[task_index]
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.sess = TaskSession(self.task, out, EpisodeConfig(video=video), binary=DEFAULT_BINARY)
        self.env = self.sess.env
        self.env.reset_to(init)
        self.client = IntrinsicClient(self.sess.server.addresses, self.sess.world_id, RequestLog(os.path.join(out, "requests.jsonl")))
        self.sync = WorldSync(self.env, self.client, self.sess.spec)
        self.sync.sync(verify=True)
        self.executor = TrajectoryExecutor(self.env, self.client)
        self.ctx = SkillContext(self.env, self.client, self.sync, self.executor, 100000)
        self.spec = task_spec_from_env(self.env, self.task.name, self.task.language)
        self.skills = build_skill_sequence(self.spec, self.env, self.ctx)
        self.cand_file = open(os.path.join(out, "candidates.jsonl"), "a")
        self.snapshot = None

    def run_until(self, until):
        done = []
        for s in self.skills:
            r = s.run(self.ctx)
            done.append(dataclasses.asdict(r))
            print(f"  skill {s.name}: ok={r.ok} reason={r.reason!r} steps={self.env.step_count}", flush=True)
            if not r.ok or s.name == until or until in s.name:
                break
        return done

    def take_snapshot(self):
        self.snapshot = (self.env.sim.get_state(), self.env.step_count, list(self.ctx.log))

    def restore(self):
        st, n, log = self.snapshot
        self.env.sim.set_state(st)
        self.env.sim.forward()
        for _ in range(3):
            self.env.step(np.zeros(7))          # settle the controller on the restored state
        self.env.step_count = n
        self.ctx.log = list(log)
        for b in list(self.sync.attached_bodies()):
            self.sync.detach(b)
        self.sync.sync()

    def audit(self, bodies, joints, region=None, region_body=None):
        a = {"step": self.env.step_count, "attached": list(self.sync.attached_bodies()),
             "robot_q": self.env.robot_state().q, "tcp": self.env.robot_state().tcp_pos}
        for b in bodies:
            p, _ = self.env.body_pose(b)
            a[b] = {"pos": p, "tilt_deg": body_tilt_deg(self.env, b), "contacts": contacts_of(self.env, b),
                    "gripper_contacts": self.env.gripper_contacts_with(b)}
        for j in joints:
            a[j] = float(self.env.joint_qpos(j))
        if region and region_body:
            a["in_region_box"] = in_region_box(self.env, region, region_body)
        a["success"] = bool(self.env.check_success())
        return jsonable(a)

    def log_candidate(self, rec):
        rec = jsonable(rec)
        rec["t"] = time.time()
        self.cand_file.write(json.dumps(rec) + "\n")
        self.cand_file.flush()
        return rec

    def close(self):
        self.cand_file.close()
        self.sess.close()


# ------------------------------------------------------------------------- drawer push family
def drawer_push_candidates(env, drawer_body, joint, families):
    """Geometry-relative contact candidates for closing a drawer along its prismatic axis."""
    axis, _ = joint_world_axis_and_anchor(env, joint)
    q = env.joint_qpos(joint)
    box = geo.object_box(env, drawer_body)
    pts = geo.body_collision_points_local(env.model, drawer_body) @ box.rot.T + box.pos
    proj = pts @ axis
    front = box.center_world - axis * (proj.max() - proj.min()) / 2     # front face centre line
    lateral = np.cross(np.array([0.0, 0.0, 1.0]), axis)
    yaw = np.arctan2(axis[1], axis[0])
    travel = abs(q) + 0.03
    out = []
    for fam in families:
        mode, tilt, h, xo = fam["mode"], fam["tilt"], fam["h"], fam["xo"]
        z = box.bottom_z + h * (box.top_z - box.bottom_z)
        contact = np.array([front[0], front[1], z]) + lateral * xo - axis * 0.015
        if mode in ("topdown_along", "topdown_lateral"):
            rot = geo.top_down_rotation(yaw if mode == "topdown_along" else yaw + np.pi / 2)
            ta = rot[:, 1] if mode == "topdown_along" else rot[:, 0]
            sign = 1.0 if np.dot(np.cross(ta, rot[:, 2]), axis) > 0 else -1.0
            rot = R.from_rotvec(ta * sign * np.radians(tilt)).as_matrix() @ rot
            pre = contact - rot[:, 2] * 0.10
        elif mode == "horizontal":
            rot = geo.side_rotation(yaw, fam.get("roll", 0.0))
            pre = contact - axis * 0.08
        elif mode == "pitched":       # hand pointing forward-down at `tilt` deg below horizontal
            rot = geo.side_rotation(yaw, fam.get("roll", 0.0))
            ta = rot[:, 0] if abs(rot[2, 0]) < 0.5 else rot[:, 1]
            sign = 1.0 if np.dot(np.cross(ta, rot[:, 2]), np.array([0, 0, -1.0])) > 0 else -1.0
            rot = R.from_rotvec(ta * sign * np.radians(tilt)).as_matrix() @ rot
            pre = contact - rot[:, 2] * 0.08
        else:
            raise ValueError(mode)
        path = [contact, contact + axis * travel]
        out.append({"family": fam, "pre": pre, "rot": rot, "path": path, "axis": axis, "travel": travel})
    return out


def hand_hits(tcp_pos, rot, pts, opening=0.01):
    local = (pts - tcp_pos) @ rot
    x, y, z = local[:, 0], local[:, 1], local[:, 2]
    finger = (np.abs(x) <= opening / 2 + 0.027) & (np.abs(y) < geo.FINGER_HALF_Y) & (z > geo.FINGER_Z[0]) & (z < geo.PAD_Z[1])
    palm = (np.abs(x) < geo.HAND_HALF_X) & (np.abs(y) < geo.HAND_HALF_Y) & (z > geo.HAND_Z[0]) & (z < geo.HAND_Z[1])
    return bool(finger.any() or palm.any())


def probe_drawer_push(lab, drawer_body, joint, families, execute_n, others, riders=()):
    env, client, sync, ctx = lab.env, lab.client, lab.sync, lab.ctx
    lim = env.joint_limits()
    free_cs = sync.free_collision_settings()
    push_cs = sync.grasp_collision_settings(drawer_body, [])
    cands = drawer_push_candidates(env, drawer_body, joint, families)
    results = []
    for c in cands:
        rec = {"family": c["family"], "pre": c["pre"], "contact": c["path"][0], "end": c["path"][1], "stage": "geometry"}
        rot = c["rot"]
        # 1. geometric filter: closed hand (opening 1 cm) clear of the other bodies at contact/mid/end
        ok, why = True, ""
        for name, p, frac in (("contact", c["path"][0], 0.0), ("mid", 0.5 * (c["path"][0] + c["path"][1]), 0.5), ("end", c["path"][1], 1.0)):
            ok, blocker = geo.hand_clearance(env, p, rot, 0.01, 0.0, [b for b in others if b not in riders])
            if ok and STRICT_RIDERS:
                for b in riders:          # bodies inside the drawer travel with it
                    pts = geo.object_point_cloud(env, b, spacing=0.012) + c["axis"] * c["travel"] * frac
                    if hand_hits(p, rot, pts):
                        ok, blocker = False, b + "(moved)"
                        break
            if not ok:
                why = f"{name}:hand:{blocker}"
                break
        if not ok:
            rec.update(stage="geometry", ok=False, why=why)
            results.append(lab.log_candidate(rec))
            print("  GEO  ", rec["family"], why, flush=True)
            continue
        # 2. Intrinsic IK (collision checked) at pre (free), contact, mid, end (drawer excluded)
        rs = env.robot_state()
        seeds = [rs.q, ctx.params.get("home_q", rs.q), np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]), np.array([0.0, 0.6, 0.0, -1.6, 0.0, 2.2, 0.8])]
        postures = [("free", None), ("shoulder_fwd", (np.maximum(lim[:, 0] + 0.05, [-9, 0.3, -9, -9, -9, -9, -9]), lim[:, 1] - 0.05)),
                    ("elbow_bent", (lim[:, 0] + 0.05, np.minimum(lim[:, 1] - 0.05, [9, 9, 9, -1.9, 9, 9, 9])))]
        ik_ok, ik_why, pre_sols = False, {}, []
        for pname, jl in postures:
            for si, seed in enumerate(seeds):
                why_p = ""
                for name, p, cs in ((("end", c["path"][1], push_cs),) if STRICT_RIDERS else ()) + (("contact", c["path"][0], push_cs), ("pre", c["pre"], free_cs)):
                    try:
                        sols, _, _ = client.ik(p, rot, seed, max_solutions=4, collision_settings=cs, joint_limits=jl)
                        if not sols:
                            why_p = f"{name}:no_solution"
                            break
                        if name == "pre":
                            pre_sols = sols
                    except IntrinsicRequestError as e:
                        s = str(e)
                        import re
                        m = re.search(r"Left object: (\S+)\s+Right objects?: (\S+)", s)
                        why_p = f"{name}:" + (f"collision:{m.group(1).replace('panda.', '')}/{m.group(2).split('.')[0]}" if m else ("kinematic" if "constraints" in s else e.code))
                        break
                ik_why[f"{pname}/seed{si}"] = why_p or "ok"
                if not why_p:
                    ik_ok = True
                    rec["posture"] = pname
                    rec["seed"] = si
                    break
            if ik_ok:
                break
        rec["ik"] = ik_why
        if not ik_ok:
            rec.update(stage="ik", ok=False, why=sorted(set(ik_why.values()))[:4])
            results.append(lab.log_candidate(rec))
            print("  IK   ", rec["family"], rec["why"], flush=True)
            continue
        # 3. LINEAR dry-runs: pre -> contact (from each pre solution, best margin first) and contact -> end
        pre_sols.sort(key=lambda q: -float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q)))))
        lin_pre, lin_travel, q_pre = "", "", None
        for q in pre_sols[:6]:
            try:
                client.plan_to_pose(q, c["path"][0], rot, collision_settings=push_cs, motion_type="LINEAR", timeout_s=5.0, caller_id="lab_pre_dryrun")
                q_pre = q
                lin_pre = "ok"
                break
            except IntrinsicRequestError as e:
                lin_pre = e.code + ":" + str(e)[:60].replace("\n", " ")
        if q_pre is not None:
            try:
                sols, _, _ = client.ik(c["path"][0], rot, q_pre, max_solutions=4, collision_settings=push_cs)
                qc = sols[0]
                client.plan_to_pose(qc, c["path"][1], rot, collision_settings=push_cs, motion_type="LINEAR", timeout_s=8.0, caller_id="lab_travel_dryrun")
                lin_travel = "ok"
            except IntrinsicRequestError as e:
                lin_travel = e.code + ":" + str(e)[:80].replace("\n", " ")
        rec.update(linear_pre=lin_pre, linear_travel=lin_travel)
        rec["stage"] = "planned"
        rec["ok"] = (lin_pre == "ok")
        results.append(lab.log_candidate(rec))
        print("  PLAN ", rec["family"], "pre:", lin_pre[:40], "travel:", lin_travel[:60], flush=True)
    # 4. physical execution of the best-ranked candidates (planned ones first, then IK-only)
    ranked = [r for r in results if r["stage"] == "planned" and r["ok"] and r["linear_travel"] == "ok"] + \
             [r for r in results if r["stage"] == "planned" and r["ok"] and r["linear_travel"] != "ok"] + \
             [r for r in results if r["stage"] == "planned" and not r["ok"]]
    executed = []
    for r in ranked[:execute_n]:
        lab.restore()
        q0 = env.joint_qpos(joint)
        fam = r["family"]
        cand = drawer_push_candidates(env, drawer_body, joint, [fam])[0]
        skill = PushSkill(drawer_body, lambda ctx=None, cand=cand: (cand["pre"], cand["rot"], cand["path"]),
                          lambda: env.joint_qpos(joint) > 0.0, label="lab_push", partial_ok=False)
        n0 = len(ctx.log)
        t0 = env.step_count
        res = skill.attempt(ctx, 0)
        ev = ctx.log[n0:]
        ex = [e for e in ev if e["event"] == "execute"]
        prog = [(e["segment"], float(env.joint_qpos(joint))) for e in ev if e["event"] == "push_progress"]
        rec = {"family": fam, "stage": "executed", "ok": bool(res.ok), "reason": res.reason, "steps": env.step_count - t0,
               "drawer_q_start": q0, "drawer_q_end": float(env.joint_qpos(joint)), "closed": bool(env.joint_qpos(joint) > 0.0),
               "success_predicate": bool(env.check_success()),
               "executions": [{k: e.get(k) for k in ("label", "ok", "reason", "final_pos_err", "final_rot_err_deg", "joint_err_final", "joint_err_max")} for e in ex],
               "events": [{k: (str(v)[:120] if k == "reason" else v) for k, v in e.items() if k in ("event", "label", "reason", "segment", "satisfied")}
                          for e in ev if e["event"] in ("approach_any_fallback", "push_progress", "precontact_path_check", "precontact_settle_failed", "recover_plan_failed", "start_state_recovery")],
               "drawer_contacts_end": contacts_of(env, drawer_body)[:12], "robot_q_end": env.robot_state().q}
        executed.append(lab.log_candidate(rec))
        print("  EXEC ", fam, "ok", res.ok, res.reason, "drawer", round(q0, 3), "->", round(float(env.joint_qpos(joint)), 3), "success", rec["success_predicate"], flush=True)
    return results, executed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", type=int, required=True)
    ap.add_argument("--init", type=int, default=0)
    ap.add_argument("--until", default="place")
    ap.add_argument("--probe", default="drawer_push")
    ap.add_argument("--execute", type=int, default=4)
    ap.add_argument("--out", required=True)
    ap.add_argument("--families", default=None, help="JSON list of family dicts (default: built-in grid)")
    a = ap.parse_args()
    lab = Lab(a.task, a.init, a.out)
    print("skills:", [s.name for s in lab.skills])
    done = lab.run_until(a.until)
    drawer_body, joint = "white_cabinet_1_cabinet_bottom", "white_cabinet_1_bottom_level"
    audit = lab.audit(["akita_black_bowl_1_main", "wine_bottle_1_main"], [joint], region="white_cabinet_1_bottom_region", region_body="akita_black_bowl_1_main")
    print("post-stage audit:", json.dumps(audit)[:1500])
    with open(os.path.join(a.out, "post_stage_audit.json"), "w") as f:
        json.dump({"skills": done, "audit": audit, "events": jsonable(lab.ctx.log)}, f, indent=1)
    lab.take_snapshot()
    if a.probe == "drawer_push":
        if a.families:
            fams = json.loads(a.families)
        else:
            fams = []
            for mode, tilts in (("topdown_along", (20, 35)), ("topdown_lateral", (20, 35)), ("horizontal", (0,)), ("pitched", (30, 45))):
                for tilt in tilts:
                    for h in (0.45, 0.7):
                        for xo in (0.0, -0.05, -0.09):
                            fams.append({"mode": mode, "tilt": tilt, "h": h, "xo": xo})
        others = [b for b in lab.sync.bodies if b != drawer_body] + ["white_cabinet_1_base", "white_cabinet_1_cabinet_middle", "white_cabinet_1_cabinet_top"]
        others = [b for b in others if b in [lab.env.model.body_id2name(i) for i in range(lab.env.model.nbody)]]
        riders = [b for b in lab.sync.bodies if in_region_box(lab.env, "white_cabinet_1_bottom_region", b)[0]]
        print("bodies riding in the drawer:", riders)
        results, executed = probe_drawer_push(lab, drawer_body, joint, fams, a.execute, others, riders)
        print("\nSUMMARY: %d candidates, %d planned (%d with full travel), %d executed, %d closed" % (
            len(results), sum(1 for r in results if r["stage"] == "planned" and r["ok"]),
            sum(1 for r in results if r["stage"] == "planned" and r.get("linear_travel") == "ok"),
            len(executed), sum(1 for r in executed if r["closed"])))
    lab.close()


if __name__ == "__main__":
    main()
