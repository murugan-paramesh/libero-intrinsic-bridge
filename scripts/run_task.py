#!/usr/bin/env python
"""Run one LIBERO-10 task with the Intrinsic-planned classical controller.

Examples:
  python scripts/run_task.py --task 0 --inits 0 1 2 --out runs/dev/task0
  python scripts/run_task.py --task 0 --inits 0 --no-video --dump-protos
"""
import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]
os.environ.setdefault("MUJOCO_GL", "egl")

from libero_intrinsic.env.libero_env import list_tasks  # noqa: E402
from libero_intrinsic.eval.runner import DEFAULT_BINARY, EpisodeConfig, TaskSession  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", type=int, required=True)
    ap.add_argument("--inits", type=int, nargs="+", default=[0])
    ap.add_argument("--out", default=None)
    ap.add_argument("--binary", default=DEFAULT_BINARY)
    ap.add_argument("--budget", type=int, default=1200)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--dump-protos", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    task = list_tasks()[a.task]
    out = a.out or os.path.join(ROOT, "runs", "dev", f"task{a.task}")
    cfg = EpisodeConfig(step_budget=a.budget, video=not a.no_video, dump_protos=a.dump_protos, seed=a.seed)
    sess = TaskSession(task, out, cfg, binary=a.binary)
    try:
        for i in a.inits:
            rec = sess.run_episode(i)
            print(json.dumps({k: rec.get(k) for k in ["run_id", "task_index", "init_state_index", "success", "steps",
                                                       "failure_stage", "failure_reason", "error", "wall_time_s"]}))
            print("  intrinsic requests:", rec["intrinsic"].get("requests_by_rpc"), "plan latencies:",
                  [round(x, 2) for x in rec["intrinsic"].get("plan_latency_s", [])])
    finally:
        sess.close()


if __name__ == "__main__":
    main()
