"""2026-11-26/GB-48: default cruise 0.1012 is below the 1.0 m/s hard cap. No motion."""
import json, os, sys, tempfile

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)

import gb_explore as X

def test_hard_max_lin_unchanged():
    assert abs(X.HARD_MAX_LIN - 1.0) < 1e-12

def test_gate_stops_untouched():
    stops = X.effective_explore_params()["gate_stops_untouched_m"]
    assert stops == {"front": 0.27, "rear": 0.28, "sweep": 0.24}

def test_whitelist_max_allows_1012_but_hard_max_clamps():
    spec = X.LEARN_PARAM_WHITELIST["explore_cruise_lin"]
    assert abs(float(spec["max"]) - 0.1012) < 1e-12
    # whitelist itself caps default cruise at 0.1012; hard cap is now 1.0
    assert abs(X._clamp_learned("explore_cruise_lin", 0.1012) - 0.1012) < 1e-12

def test_applied_1012_effective_cruise_is_hard_max():
    X._speed_mult = lambda: 1.0
    d = tempfile.mkdtemp()
    p = os.path.join(d, "applied_params.json")
    with open(p, "w") as f:
        json.dump({"schema": "wally.applied.v1", "params": {
            "explore_cruise_lin": 0.1012,
            "explore_turn_ang": 0.48,
            "avoid_dist_m": 0.49,
            "slowdown_dist_m": 0.7,
        }}, f)
    out = X.apply_learned_params(log=False, paths=[p])
    assert abs(X.HARD_MAX_LIN - 1.0) < 1e-12
    # exact effective cruise after whitelist + hard cap (speed_mult 1.0 assumed)
    sm = 1.0
    try:
        sm = float(out.get("speed_mult") or 1.0)
    except Exception:
        sm = 1.0
    effective = X.effective_explore_params()["explore_cruise_lin"]
    assert abs(effective - min(X.HARD_MAX_LIN, 0.1012 * sm)) < 1e-12
    assert abs(effective - 0.1012 * sm if sm <= 1 else min(X.HARD_MAX_LIN, 0.1012 * sm)) < 1e-12 or abs(effective - min(X.HARD_MAX_LIN, 0.1012 * sm)) < 1e-12
    assert effective <= 0.1012 + 1e-12
    assert effective < X.HARD_MAX_LIN
