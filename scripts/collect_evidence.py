#!/usr/bin/env python
"""Copy representative evaluation episodes (one successful and one failed per task, when they
exist) with their videos, records and RPC logs into evidence/eval_episodes, plus a manifest."""
import glob
import json
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
from libero_intrinsic.eval.report import load_episodes, classify  # noqa: E402


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "eval")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "evidence", "eval_episodes")
    os.makedirs(out, exist_ok=True)
    eps = load_episodes(run_dir)
    manifest = []
    for ti in sorted({e["task_index"] for e in eps}):
        for kind in ("success", "failure", "infra_error"):
            cands = [e for e in eps if e["task_index"] == ti and classify(e) == kind]
            if not cands:
                continue
            e = sorted(cands, key=lambda x: x["init_state_index"])[0]
            src = os.path.dirname(e["_path"])
            dst = os.path.join(out, f"task{ti}_{kind}_{e['run_id']}")
            os.makedirs(dst, exist_ok=True)
            for f in ("episode.json", "intrinsic_requests.jsonl", "agentview.mp4"):
                if os.path.exists(os.path.join(src, f)):
                    shutil.copy(os.path.join(src, f), dst)
            manifest.append({"task_index": ti, "task_name": e["task_name"], "kind": kind, "run_id": e["run_id"],
                             "init_state_index": e["init_state_index"], "steps": e["steps"], "success": e["success"],
                             "failure_stage": e.get("failure_stage"), "failure_reason": (e.get("failure_reason") or "")[:160],
                             "error": e.get("error"), "dir": os.path.relpath(dst, ROOT),
                             "revision": e["revisions"]["libero_intrinsic_bridge"]})
    with open(os.path.join(out, "MANIFEST.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    print(json.dumps(manifest, indent=1))


if __name__ == "__main__":
    main()
