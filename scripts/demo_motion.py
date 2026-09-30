#!/usr/bin/env python
"""Milestone: execute one Intrinsic-planned, collision-checked motion in LIBERO.

Plans (PlanTrajectory) from the current configuration to a pre-grasp pose 10 cm above the first
goal object of the task, executes it through the OSC_POSE controller, records tracking error and
a video, and stores the Intrinsic request/response protos.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]
os.environ.setdefault("MUJOCO_GL", "egl")

from libero_intrinsic.env.executor import ExecutionConfig, TrajectoryExecutor  # noqa: E402
from libero_intrinsic.env.libero_env import LiberoEnv, list_tasks  # noqa: E402
from libero_intrinsic.eval.runner import DEFAULT_BINARY, VideoRecorder  # noqa: E402
from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicServer, RequestLog  # noqa: E402
from libero_intrinsic.intrinsic.world_sync import WorldSync  # noqa: E402
from libero_intrinsic.model.scene_to_sdf import SceneToSdf  # noqa: E402
from libero_intrinsic.skills import geometry as geo  # noqa: E402
from libero_intrinsic.skills.task_spec import task_spec_from_env  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", type=int, default=0)
    ap.add_argument("--init", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "dev", "demo_motion"))
    ap.add_argument("--binary", default=DEFAULT_BINARY)
    a = ap.parse_args()
    task = list_tasks()[a.task]
    os.makedirs(a.out, exist_ok=True)
    env = LiberoEnv(task, camera_hw=(256, 256))
    env.reset_to(a.init)
    spec = SceneToSdf(env, os.path.join(a.out, "world")).write("world")
    with IntrinsicServer(a.binary, {"demo": spec.sdf_path}, os.path.join(a.out, "server")) as srv:
        log = RequestLog(os.path.join(a.out, "intrinsic_requests.jsonl"), dump_protos=True)
        client = IntrinsicClient(srv.addresses, "demo", log)
        sync = WorldSync(env, client, spec)
        print("sync:", sync.sync(verify=True))
        ts = task_spec_from_env(env, task.name, task.language)
        obj = ts.goal[0].args[0]
        body = env.object_root_body(obj)
        cands = geo.grasp_candidates(env, body)
        c = cands[0]
        pre = c.pos + np.array([0, 0, 0.10])
        rs = env.robot_state()
        print(f"target object {obj}: grasp {c.label} width {c.width:.3f}; pre-grasp {pre.round(3)}")
        sols, rid, lat = client.ik(pre, c.rot, rs.q, max_solutions=4, collision_settings=sync.free_collision_settings())
        print(f"IK: {len(sols)} solutions in {lat:.3f}s ({rid})")
        traj = client.plan_to_pose(rs.q, pre, c.rot, collision_settings=sync.free_collision_settings(), timeout_s=20)
        print(f"PlanTrajectory {traj.request_id}: {len(traj.t)} states, {traj.duration:.2f}s, latency {traj.planning_latency_s:.2f}s")
        video = VideoRecorder(env, os.path.join(a.out, "demo_motion.mp4"))
        ex = TrajectoryExecutor(env, client, on_step=video)
        res = ex.execute(traj, ExecutionConfig())
        video.close()
        rs = env.robot_state()
        summary = {"task": task.name, "init": a.init, "object": obj, "grasp": c.label, "pregrasp_target": pre.tolist(),
                   "trajectory_id": traj.request_id, "planned_duration_s": traj.duration, "planning_latency_s": traj.planning_latency_s,
                   "n_states": int(len(traj.t)), "ik_solutions": len(sols),
                   "execution": {k: v for k, v in res.__dict__.items() if k != "log"},
                   "final_tcp": rs.tcp_pos.tolist(), "final_q": rs.q.tolist(), "planned_final_q": traj.q[-1].tolist(),
                   "video": video.path, "requests": len(log.records), "addresses": srv.addresses.__dict__}
        with open(os.path.join(a.out, "summary.json"), "w") as f:
            json.dump(summary, f, indent=1)
        np.save(os.path.join(a.out, "planned_trajectory_q.npy"), traj.q)
        np.save(os.path.join(a.out, "executed_log.npy"), np.array([[l["t"], l["pos_err"], l["rot_err_deg"], l["joint_err"]] for l in res.log]))
        print(json.dumps(summary["execution"], indent=1))
    env.close()


if __name__ == "__main__":
    main()
