#!/usr/bin/env python
"""Run the frozen evaluation protocol from a YAML config and write the results table."""
import argparse
import json
import os
import sys
import time

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]
os.environ.setdefault("MUJOCO_GL", "egl")

from libero_intrinsic.env.libero_env import list_tasks  # noqa: E402
from libero_intrinsic.eval.report import load_episodes, markdown_table, summarize  # noqa: E402
from libero_intrinsic.eval.runner import DEFAULT_BINARY, EpisodeConfig, TaskSession  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(ROOT, "configs", "eval_frozen.yaml"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--tasks", type=int, nargs="*", default=None, help="subset of tasks (reported as a subset)")
    ap.add_argument("--inits", type=int, nargs="*", default=None, help="override init states (dev runs)")
    ap.add_argument("--binary", default=DEFAULT_BINARY)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    out = a.out or os.path.join(ROOT, "runs", "eval", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    tasks = a.tasks if a.tasks is not None else cfg["tasks"]
    inits = a.inits if a.inits is not None else cfg["eval_init_states"]
    with open(os.path.join(out, "protocol.json"), "w") as f:
        json.dump({"config": cfg, "config_path": a.config, "tasks": tasks, "init_states": inits, "started": time.time()}, f, indent=1)
    all_tasks = list_tasks()
    for ti in tasks:
        task = all_tasks[ti]
        sess = TaskSession(task, os.path.join(out, f"task{ti}"),
                           EpisodeConfig(step_budget=cfg["step_budget"], video=cfg.get("video", True), seed=cfg["seed"]), binary=a.binary)
        try:
            for i in inits:
                rec = sess.run_episode(i)
                print(json.dumps({k: rec.get(k) for k in ["run_id", "task_index", "init_state_index", "success", "steps",
                                                           "failure_stage", "failure_reason", "error", "wall_time_s"]}), flush=True)
        finally:
            sess.close()
        s = summarize(load_episodes(out))
        with open(os.path.join(out, "results.md"), "w") as f:
            f.write(markdown_table(s) + "\n")
        with open(os.path.join(out, "results.json"), "w") as f:
            json.dump(s, f, indent=1)
    print(markdown_table(summarize(load_episodes(out))))


if __name__ == "__main__":
    main()
