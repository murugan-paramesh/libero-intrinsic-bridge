from libero_intrinsic.eval.report import summarize, wilson_interval, classify, failure_category


def _ep(ti, success, error=None, steps=100, reason="", stage=""):
    return {"task_index": ti, "task_name": f"task{ti}", "success": success, "error": error, "steps": steps,
            "intrinsic": {"plan_latency_s": [0.5, 1.0], "plan_failures": 0, "plan_timeouts": 0},
            "events": [], "skills": [], "failure_reason": reason, "failure_stage": stage}


def test_wilson():
    lo, hi = wilson_interval(0, 0)
    assert (lo, hi) == (0.0, 0.0)
    lo, hi = wilson_interval(10, 10)
    assert lo > 0.65 and hi == 1.0
    lo, hi = wilson_interval(5, 10)
    assert 0.23 < lo < 0.25 and 0.76 < hi < 0.78


def test_denominators_separate_infra_errors():
    eps = [_ep(0, True), _ep(0, False, reason="grasp_verify_failed(gap)", stage="pick:x"), _ep(0, False, error="infrastructure:x")]
    s = summarize(eps)
    r = s["per_task"][0]
    assert r["episodes"] == 3 and r["successes"] == 1 and r["failures"] == 1 and r["infra_errors"] == 1
    assert abs(r["success_rate_all"] - 1 / 3) < 1e-9 and abs(r["success_rate_valid"] - 0.5) < 1e-9
    assert r["failure_categories"] == {"pick:grasp_verify_failed": 1, "infra:infrastructure": 1}
    assert s["overall"]["n"] == 3 and s["overall"]["infra"] == 1


def test_classify():
    assert classify(_ep(0, True)) == "success"
    assert classify(_ep(0, False)) == "failure"
    assert classify(_ep(0, True, error="boom")) == "infra_error"  # an error never counts as success
    assert failure_category(_ep(0, False, reason="timeout", stage="place:obj")) == "place:timeout"
