#!/usr/bin/env python3
"""Archive an evaluation run into the repository: episode records and protocol files only (no
videos, no per-request logs), plus the generated results table, so that the numbers reported are
reproducible from committed artefacts."""
import glob
import json
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src")]


def main():
    src, dst = sys.argv[1], sys.argv[2]
    os.makedirs(dst, exist_ok=True)
    n = 0
    for p in sorted(glob.glob(os.path.join(src, "**", "episode.json"), recursive=True)):
        run_id = os.path.basename(os.path.dirname(p))
        os.makedirs(os.path.join(dst, run_id), exist_ok=True)
        shutil.copy(p, os.path.join(dst, run_id, "episode.json"))
        n += 1
    for p in sorted(glob.glob(os.path.join(src, "*", "protocol.json"))):
        shutil.copy(p, os.path.join(dst, f"protocol_{os.path.basename(os.path.dirname(p))}.json"))
    from libero_intrinsic.eval.report import load_episodes, markdown_table, summarize
    eps = load_episodes(dst)
    summ = summarize(eps)
    json.dump(summ, open(os.path.join(dst, "results.json"), "w"), indent=1)
    open(os.path.join(dst, "results.md"), "w").write(markdown_table(summ) + "\n")
    print(f"archived {n} episode records to {dst}")


if __name__ == "__main__":
    main()
