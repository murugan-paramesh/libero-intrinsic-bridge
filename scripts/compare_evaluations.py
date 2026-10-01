#!/usr/bin/env python3
"""Per-task before/after comparison of two evaluation runs (directories containing episode.json
records, e.g. evaluations/baseline_76267bf and runs/eval_<rev>). Both runs must use the same
protocol (same tasks and init states); the script checks that and reports every episode with its
denominator. Writes a markdown table (and optionally JSON)."""
import argparse
import glob
import json
import math
import os
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [os.path.join(ROOT, "src")]


def wilson(s, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = s / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def load(run_dir):
    eps = {}
    for p in sorted(glob.glob(os.path.join(run_dir, "**", "episode.json"), recursive=True)):
        e = json.load(open(p))
        eps[(int(e["task_index"]), int(e["init_state_index"]))] = e
    return eps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("before")
    ap.add_argument("after")
    ap.add_argument("--labels", nargs=2, default=["before", "after"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    b, c = load(a.before), load(a.after)
    keys_b, keys_c = set(b), set(c)
    if keys_b != keys_c:
        print(f"WARNING: episode sets differ: only-before={sorted(keys_b - keys_c)} only-after={sorted(keys_c - keys_b)}")
    per = defaultdict(lambda: {"nb": 0, "sb": 0, "nc": 0, "sc": 0, "gain": [], "loss": [], "name": ""})
    for k, e in b.items():
        t = per[k[0]]
        t["nb"] += 1
        t["sb"] += int(bool(e.get("success")))
        t["name"] = e.get("task_name", "")
    for k, e in c.items():
        t = per[k[0]]
        t["nc"] += 1
        t["sc"] += int(bool(e.get("success")))
        if k in b:
            if e.get("success") and not b[k].get("success"):
                t["gain"].append(k[1])
            if b[k].get("success") and not e.get("success"):
                t["loss"].append(k[1])
    lines = [f"| task | {a.labels[0]} | {a.labels[1]} | delta | newly solved (init) | newly failed (init) |", "|---|---|---|---|---|---|"]
    tb = tc = nb = nc = 0
    for ti in sorted(per):
        t = per[ti]
        tb += t["sb"]; tc += t["sc"]; nb += t["nb"]; nc += t["nc"]
        lines.append(f"| {ti} {t['name'][:50]} | {t['sb']}/{t['nb']} | {t['sc']}/{t['nc']} | {t['sc'] - t['sb']:+d} | {t['gain'] or '-'} | {t['loss'] or '-'} |")
    lb, lc = wilson(tb, nb), wilson(tc, nc)
    lines.append(f"| **all** | **{tb}/{nb}** (CI {100 * lb[0]:.0f}-{100 * lb[1]:.0f}%) | **{tc}/{nc}** (CI {100 * lc[0]:.0f}-{100 * lc[1]:.0f}%) | {tc - tb:+d} | | |")
    md = "\n".join(lines)
    print(md)
    if a.out:
        with open(a.out, "w") as f:
            f.write(md + "\n")
        with open(os.path.splitext(a.out)[0] + ".json", "w") as f:
            json.dump({"before": a.before, "after": a.after, "per_task": {str(k): v for k, v in per.items()},
                       "total_before": [tb, nb], "total_after": [tc, nc]}, f, indent=1)


if __name__ == "__main__":
    main()
