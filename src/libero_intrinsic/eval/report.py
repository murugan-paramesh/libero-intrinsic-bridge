"""Aggregate episode records into metrics and a results table (no cherry-picking: every
episode.json under a run directory is counted; infrastructure errors are reported separately
from unsuccessful episodes and both are shown with their denominator treatment)."""
from __future__ import annotations

import glob
import json
import math
import os
from collections import defaultdict
from typing import Dict, List, Tuple


def wilson_interval(k: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def load_episodes(run_dir: str) -> List[dict]:
    eps = []
    for p in sorted(glob.glob(os.path.join(run_dir, "**", "episode.json"), recursive=True)):
        with open(p) as f:
            e = json.load(f)
        e["_path"] = p
        eps.append(e)
    return eps


def classify(e: dict) -> str:
    if e.get("error"):
        return "infra_error"
    return "success" if e.get("success") else "failure"


def failure_category(e: dict) -> str:
    if e.get("error"):
        return "infra:" + e["error"].split(":")[0] if ":" in e["error"] else "infra"
    if e.get("success"):
        return "success"
    stage = e.get("failure_stage", "unknown")
    reason = e.get("failure_reason", "")
    for key in ["no_grasp_candidate", "no_reachable_grasp", "grasp_verify_failed", "lift_verify_failed", "tracking_error",
                "timeout", "budget", "intrinsic:PlanTrajectory", "intrinsic:ComputeIk", "transport_exec", "lower_exec",
                "precondition", "push_goal", "knob", "joint_limit", "final_pose_error", "contact"]:
        if key in reason:
            return f"{stage.split(':')[0]}:{key}"
    return f"{stage.split(':')[0]}:{reason[:30]}" if reason else "goal_not_reached"


def summarize(episodes: List[dict]) -> Dict:
    by_task = defaultdict(list)
    for e in episodes:
        by_task[(e["task_index"], e["task_name"])].append(e)
    rows = []
    tot = {"n": 0, "success": 0, "failure": 0, "infra": 0}
    for (ti, name), eps in sorted(by_task.items()):
        n = len(eps)
        s = sum(1 for e in eps if classify(e) == "success")
        infra = sum(1 for e in eps if classify(e) == "infra_error")
        valid = n - infra
        lo, hi = wilson_interval(s, n)
        lat = [l for e in eps for l in e.get("intrinsic", {}).get("plan_latency_s", [])]
        plan_fail = sum(e.get("intrinsic", {}).get("plan_failures", 0) for e in eps)
        plan_to = sum(e.get("intrinsic", {}).get("plan_timeouts", 0) for e in eps)
        n_plans = len(lat)
        steps = [e.get("steps", 0) for e in eps]
        execs = [ev for e in eps for ev in e.get("events", []) if ev.get("event") == "execute"]
        pos_err = [ev["pos_err_max"] for ev in execs]
        joint_err = [ev["joint_err_max"] for ev in execs]
        grasp_fail = sum(1 for e in eps for ev in e.get("events", []) if ev.get("event") == "skill_attempt" and ev.get("skill") == "pick" and not ev.get("ok"))
        recoveries = sum(sk.get("recoveries", 0) for e in eps for sk in e.get("skills", []))
        cats = defaultdict(int)
        for e in eps:
            if classify(e) != "success":
                cats[failure_category(e)] += 1
        rows.append({
            "task_index": ti, "task_name": name, "episodes": n, "successes": s, "failures": valid - s, "infra_errors": infra,
            "success_rate_all": s / n if n else 0.0, "ci95_all": (lo, hi),
            "success_rate_valid": s / valid if valid else 0.0,
            "success_within_600": sum(1 for e in eps if classify(e) == "success" and e.get("steps", 1e9) <= 600),
            "n_plans": n_plans, "plan_latency_mean_s": sum(lat) / n_plans if n_plans else 0.0,
            "plan_latency_max_s": max(lat) if lat else 0.0, "plan_failures": plan_fail, "plan_timeouts": plan_to,
            "steps_mean": sum(steps) / n if n else 0.0,
            "track_pos_err_max_m": max(pos_err) if pos_err else 0.0, "track_pos_err_mean_m": sum(pos_err) / len(pos_err) if pos_err else 0.0,
            "track_joint_err_max_rad": max(joint_err) if joint_err else 0.0,
            "grasp_attempt_failures": grasp_fail, "recoveries": recoveries, "failure_categories": dict(cats),
        })
        tot["n"] += n; tot["success"] += s; tot["infra"] += infra; tot["failure"] += valid - s
    lo, hi = wilson_interval(tot["success"], tot["n"])
    return {"per_task": rows, "overall": {**tot, "success_rate_all": tot["success"] / tot["n"] if tot["n"] else 0.0, "ci95_all": (lo, hi),
                                          "success_rate_valid": tot["success"] / (tot["n"] - tot["infra"]) if tot["n"] - tot["infra"] else 0.0}}


def markdown_table(summary: Dict) -> str:
    lines = ["| # | task | episodes | success | fail | infra | rate (all) | 95% CI | plans | plan lat mean/max (s) | plan fail/timeout | steps mean | max tcp track err (m) | grasp fails | recoveries |",
             "|---|------|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in summary["per_task"]:
        lo, hi = r["ci95_all"]
        lines.append(f"| {r['task_index']} | {r['task_name'][:60]} | {r['episodes']} | {r['successes']} | {r['failures']} | {r['infra_errors']} | "
                     f"{100*r['success_rate_all']:.0f}% | [{100*lo:.0f}, {100*hi:.0f}] | {r['n_plans']} | {r['plan_latency_mean_s']:.2f}/{r['plan_latency_max_s']:.2f} | "
                     f"{r['plan_failures']}/{r['plan_timeouts']} | {r['steps_mean']:.0f} | {r['track_pos_err_max_m']:.3f} | {r['grasp_attempt_failures']} | {r['recoveries']} |")
    o = summary["overall"]
    lo, hi = o["ci95_all"]
    lines.append(f"| all | | {o['n']} | {o['success']} | {o['failure']} | {o['infra']} | {100*o['success_rate_all']:.0f}% | [{100*lo:.0f}, {100*hi:.0f}] | | | | | | | |")
    return "\n".join(lines)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    eps = load_episodes(a.run_dir)
    s = summarize(eps)
    md = markdown_table(s)
    print(md)
    print("\nFailure categories:")
    for r in s["per_task"]:
        print(f"  task {r['task_index']}: {r['failure_categories']}")
    if a.out:
        with open(a.out, "w") as f:
            json.dump(s, f, indent=1)
        with open(os.path.splitext(a.out)[0] + ".md", "w") as f:
            f.write(md + "\n")


if __name__ == "__main__":
    main()
