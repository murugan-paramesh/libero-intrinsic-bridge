#!/usr/bin/env python
"""Milestone check: robot-model agreement between Intrinsic Core and LIBERO/MuJoCo.

1. Start libero_planner_server on the Task-N SDF world.
2. For N random joint configurations inside the LIBERO joint limits: set them in MuJoCo, read the
   world pose of gripper0_grip_site; ask Intrinsic ComputeFk for root_t_tcp; compare.
3. For M of those TCP poses: ask Intrinsic ComputeIk (seeded elsewhere), set the solution in MuJoCo
   and verify the resulting TCP pose (independent validation of IK through the LIBERO robot).
4. Collision check sanity: a configuration that intersects the table must be reported in collision.
Writes a JSON report with max/mean errors and the acceptance tolerances.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]
os.environ.setdefault("MUJOCO_GL", "egl")

from libero_intrinsic.env.libero_env import ARM_JOINTS, LiberoEnv, list_tasks  # noqa: E402
from libero_intrinsic.eval.runner import DEFAULT_BINARY  # noqa: E402
from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicRequestError, IntrinsicServer, RequestLog, collision_settings  # noqa: E402
from libero_intrinsic.intrinsic.world_sync import WorldSync  # noqa: E402
from libero_intrinsic.model import transforms as tf  # noqa: E402
from libero_intrinsic.model.scene_to_sdf import SceneToSdf  # noqa: E402

POS_TOL, ROT_TOL_DEG = 1e-4, 0.01     # FK agreement (same kinematic parameters; only float error expected)
IK_POS_TOL, IK_ROT_TOL_DEG = 2e-3, 0.5  # numeric IK convergence tolerance


def set_q(env, q):
    for k, jn in enumerate(ARM_JOINTS):
        env.data.set_joint_qpos(jn, q[k])
    env.sim.forward()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", type=int, default=0)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--n-ik", type=int, default=40)
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "dev", "kinematics_validation.json"))
    ap.add_argument("--binary", default=DEFAULT_BINARY)
    a = ap.parse_args()
    task = list_tasks()[a.task]
    env = LiberoEnv(task, camera_hw=(64, 64))
    env.reset_to(0)
    out_dir = os.path.dirname(a.out)
    spec = SceneToSdf(env, os.path.join(out_dir, f"kinval_task{a.task}_world")).write("world")
    with IntrinsicServer(a.binary, {"kinval": spec.sdf_path}, os.path.join(out_dir, "kinval_server")) as srv:
        log = RequestLog(os.path.join(out_dir, "kinval_requests.jsonl"))
        client = IntrinsicClient(srv.addresses, "kinval", log)
        sync = WorldSync(env, client, spec)
        sync.sync(verify=True)
        lim = env.joint_limits()
        rng = np.random.default_rng(0)
        rs0 = env.robot_state()
        q_home = rs0.q.copy()
        # ---- FK
        fk_pos, fk_rot, fk_lat = [], [], []
        samples = []
        for i in range(a.n):
            q = rng.uniform(lim[:, 0], lim[:, 1]) if i > 0 else q_home
            set_q(env, q)
            rs = env.robot_state()
            t0 = time.perf_counter()
            p, R_ = client.fk(q)
            fk_lat.append(time.perf_counter() - t0)
            fk_pos.append(np.linalg.norm(p - rs.tcp_pos))
            fk_rot.append(tf.rot_error_deg(R_, rs.tcp_rot))
            samples.append((q, rs.tcp_pos.copy(), rs.tcp_rot.copy()))
        # ---- IK (collision-free w.r.t. the world as synced at the home configuration)
        ik_pos, ik_rot, ik_lat, ik_fail, ik_coll = [], [], [], 0, 0
        cs = collision_settings(disable_all=True)  # pure kinematic validation first
        for k in range(a.n_ik):
            q_true, p_t, R_t = samples[k]
            seed = rng.uniform(lim[:, 0], lim[:, 1])
            try:
                t0 = time.perf_counter()
                sols, rid, lat = client.ik(p_t, R_t, seed, max_solutions=4, collision_settings=cs)
                ik_lat.append(time.perf_counter() - t0)
            except IntrinsicRequestError as e:
                ik_fail += 1
                continue
            if not sols:
                ik_fail += 1
                continue
            set_q(env, sols[0])
            rs = env.robot_state()
            ik_pos.append(np.linalg.norm(rs.tcp_pos - p_t))
            ik_rot.append(tf.rot_error_deg(rs.tcp_rot, R_t))
            if np.any(sols[0] < lim[:, 0] - 1e-6) or np.any(sols[0] > lim[:, 1] + 1e-6):
                ik_fail += 1
        # ---- collision check sanity
        set_q(env, q_home)
        sync.sync()
        free_cs = collision_settings()
        home_coll, msg_home, _ = client.check_collisions([q_home], free_cs)
        # push the arm down into the table: joint2 forward + joint4 open (empirically intersects the table)
        q_bad = q_home.copy(); q_bad[1] = min(lim[1, 1], 1.4); q_bad[3] = max(lim[3, 0], -1.2)
        set_q(env, q_bad)
        mj_contacts_bad = [c for c in env.contacts() if any(("robot0" in (n or "") or "gripper0" in (n or "")) for n in c[:2])]
        bad_coll, msg_bad, _ = client.check_collisions([q_bad], free_cs)
        set_q(env, q_home)
        report = {
            "task": task.name, "n_fk": a.n, "fk_pos_err_max_m": float(np.max(fk_pos)), "fk_pos_err_mean_m": float(np.mean(fk_pos)),
            "fk_rot_err_max_deg": float(np.max(fk_rot)), "fk_rot_err_mean_deg": float(np.mean(fk_rot)),
            "fk_latency_mean_s": float(np.mean(fk_lat)), "fk_tolerance": {"pos_m": POS_TOL, "rot_deg": ROT_TOL_DEG},
            "fk_pass": bool(np.max(fk_pos) < POS_TOL and np.max(fk_rot) < ROT_TOL_DEG),
            "n_ik": a.n_ik, "ik_solved": len(ik_pos), "ik_failures": ik_fail,
            "ik_pos_err_max_m": float(np.max(ik_pos)) if ik_pos else None, "ik_pos_err_mean_m": float(np.mean(ik_pos)) if ik_pos else None,
            "ik_rot_err_max_deg": float(np.max(ik_rot)) if ik_rot else None, "ik_rot_err_mean_deg": float(np.mean(ik_rot)) if ik_rot else None,
            "ik_latency_mean_s": float(np.mean(ik_lat)) if ik_lat else None, "ik_tolerance": {"pos_m": IK_POS_TOL, "rot_deg": IK_ROT_TOL_DEG},
            "ik_pass": bool(ik_pos and np.max(ik_pos) < IK_POS_TOL and np.max(ik_rot) < IK_ROT_TOL_DEG),
            "collision_home": {"intrinsic": home_coll, "msg": msg_home[:300]},
            "collision_arm_in_table": {"intrinsic": bad_coll, "msg": msg_bad[:300], "mujoco_robot_contacts": len(mj_contacts_bad)},
            "robot_base_pos": spec.robot_base_pos.tolist(), "sdf": spec.sdf_path, "addresses": srv.addresses.__dict__,
            "n_requests": len(log.records),
        }
    env.close()
    os.makedirs(out_dir, exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
