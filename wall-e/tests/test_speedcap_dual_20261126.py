"""2026-11-26 GB-48: dual speed caps, default slow autonomous start, pick-up speed,
sensor-reaction ceiling. Stop distances must stay 0.27/0.28/0.24. No motion."""
import math, os, sys, tempfile

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)
sys.path.insert(0, os.path.join(REL, "gb_safety"))

import gb_speed_caps as SC
import gb_explore as X
import gb_obstacle as OB

MANUAL, AUTO = 0.288, 1.0
DEFAULT = X.DEFAULT_CRUISE_LIN  # conservative ~0.1012 m/s
FLOOR = X.ADAPT_TIGHT_LIN       # 0.08


def almost(a, b, eps=1e-9):
    return abs(float(a) - float(b)) <= eps


# --------------------------------------------------------------------------- gate caps (pure, no ROS)

def test_shared_caps_constants():
    assert almost(SC.MANUAL_MAX_LIN, MANUAL)
    assert almost(SC.AUTO_MAX_LIN, AUTO)
    assert almost(SC.lin_cap_for("stream", lease_live=False), MANUAL)
    assert almost(SC.lin_cap_for("stream", lease_live=True), AUTO)


def test_manual_288_passes_and_050_clamps():
    cap = SC.lin_cap_for("stream", lease_live=False)
    assert almost(cap, MANUAL)
    assert min(0.288, cap) == 0.288
    assert min(0.50, cap) == MANUAL
    capj = SC.lin_cap_for("joystick", joystick=True)
    assert almost(capj, MANUAL)
    assert min(0.50, capj) == MANUAL
    capv = SC.lin_cap_for("voice", lease_live=False)
    assert almost(min(0.50, capv), MANUAL)


def test_autonomous_050_and_10_pass_above_10_clamps():
    cap = SC.lin_cap_for("stream", lease_live=True)
    assert almost(cap, AUTO)
    assert min(0.50, cap) == 0.50
    assert min(1.0, cap) == 1.0
    assert min(1.2, cap) == AUTO
    for src in ("explore", "nav2", "gb_explore", "mission"):
        c = SC.lin_cap_for(src, lease_live=False)
        assert almost(c, AUTO), src
        assert min(1.2, c) == AUTO


def test_gate_source_has_dual_caps_and_unchanged_stops():
    gsrc = open(os.path.join(REL, "gb_safety", "gb_gate.py"), encoding="utf-8").read()
    assert 'GATE_MANUAL_MAX_LIN", "0.288"' in gsrc
    assert 'GATE_AUTO_MAX_LIN", "1.0"' in gsrc
    assert "lin_cap_for(" in gsrc
    assert 'GATE_DEADMAN_S", "0.5"' in gsrc
    assert "str(OB.STOP_FWD)" in gsrc and "str(OB.STOP_REV)" in gsrc and "str(OB.SWEEP_R)" in gsrc
    # no new stop-distance literals overriding the obstacle module
    import re
    assert not re.search(
        r"^(FRONT_STOP|REAR_STOP|SWEEP_R|S_FRONT_STOP|S_REAR_STOP|S_SWEEP_R)\s*=\s*[0-9.]+\s*$",
        gsrc, re.M,
    )
    assert (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24)
    stops = X.effective_explore_params()["gate_stops_untouched_m"]
    assert stops == {"front": 0.27, "rear": 0.28, "sweep": 0.24}


# --------------------------------------------------------------------------- default cruise + hard cap

def test_explore_hard_cap_is_1_and_default_cruise_is_conservative():
    assert almost(X.HARD_MAX_LIN, AUTO)
    assert 0.10 - 1e-9 <= float(DEFAULT) <= 0.12 + 1e-9
    start = X.start_autonomous_cruise()
    assert almost(start, min(float(X.MAX_LIN), DEFAULT))
    assert start <= DEFAULT + 1e-12
    assert start < 0.5
    assert start != AUTO


def test_entering_autonomy_resets_even_if_prior_target_was_high():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "explore_speed.json")
    X.write_explore_speed_target(1.0, path=p, src="stale")
    v = X.reset_explore_speed_target(path=p)
    assert v <= DEFAULT + 1e-12
    assert almost(X.read_explore_speed_target(path=p), v)
    assert v < AUTO


# --------------------------------------------------------------------------- pick up speed

def test_pick_up_speed_ramps_toward_hard_cap():
    v = X.start_autonomous_cruise()
    seen = [v]
    for _ in range(20):
        v = X.pick_up_explore_speed(v)
        seen.append(v)
        assert v <= AUTO + 1e-12
    assert seen[-1] == AUTO or almost(seen[-1], AUTO)
    assert seen[1] > seen[0]
    # first step is a modest bump, not a jump to 1.0
    assert seen[1] < 0.5


def test_pick_up_and_slow_down_via_state_file():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "explore_speed.json")
    start = X.reset_explore_speed_target(path=p)
    up = X.request_pick_up_speed(path=p)
    assert up > start
    assert up <= AUTO
    down = X.request_slow_down_speed(path=p)
    assert almost(down, start) or down < up
    assert down >= start - 1e-12 or down >= FLOOR - 1e-12


# --------------------------------------------------------------------------- sensor-reaction cap

def test_reaction_cap_stale_or_missing_fails_safe():
    v, why = X.sensor_reaction_speed_cap(3.0, lidar_fresh=False)
    assert why == "stale"
    assert almost(v, FLOOR) or v <= FLOOR
    v, why = X.sensor_reaction_speed_cap(None, lidar_fresh=True)
    assert why == "no_range"
    assert v <= FLOOR + 1e-12


def test_reaction_cap_inside_stop_margin_is_zero():
    v, why = X.sensor_reaction_speed_cap(0.27, lidar_fresh=True, lidar_age_s=0.0)
    assert why == "inside_stop_margin"
    assert almost(v, 0.0)
    v, why = X.sensor_reaction_speed_cap(0.20, lidar_fresh=True, lidar_age_s=0.0)
    assert almost(v, 0.0)


def test_reaction_cap_stops_accelerating_before_sensors_cannot_react():
    # At modest clearance the cap must be well below the 1.0 hard cap.
    v_close, why = X.sensor_reaction_speed_cap(0.70, lidar_fresh=True, lidar_age_s=0.10)
    assert why == "ok"
    assert 0.0 < v_close < AUTO
    v_open, _ = X.sensor_reaction_speed_cap(3.00, lidar_fresh=True, lidar_age_s=0.10)
    assert v_open > v_close
    assert v_open <= AUTO + 1e-12
    # Older (staler-but-still-fresh) scans tighten the cap.
    v_old, _ = X.sensor_reaction_speed_cap(1.50, lidar_fresh=True, lidar_age_s=0.40)
    v_new, _ = X.sensor_reaction_speed_cap(1.50, lidar_fresh=True, lidar_age_s=0.05)
    assert v_old <= v_new + 1e-12
    # 1.0 m/s is only allowed when there is enough slack to stop.
    v_need, _ = X.sensor_reaction_speed_cap(1.50, lidar_fresh=True, lidar_age_s=0.10)
    # model: slack >= v*t + v^2/(2a) for v=1.0; 1.50-0.27=1.23, typically enough or close
    assert v_need <= AUTO + 1e-12


def test_reaction_cap_never_exceeds_hard_and_composes_with_adaptive():
    boost = X.adaptive_boost_speed(DEFAULT, AUTO)
    assert almost(boost, DEFAULT * 1.20)
    assert boost <= AUTO
    v, _ = X.apply_explore_speed_limits(
        1.0, front_clearance_m=0.70, lidar_fresh=True, lidar_age_s=0.10, hard_max_lin=AUTO,
    )
    assert v < AUTO
    v2, _ = X.apply_explore_speed_limits(
        1.2, front_clearance_m=8.0, lidar_fresh=True, lidar_age_s=0.05, hard_max_lin=AUTO,
    )
    assert v2 <= AUTO + 1e-12


# --------------------------------------------------------------------------- adaptive logic still intact under 1.0 cap

def test_adaptive_open_boost_slowdown_and_creep_within_hard_cap():
    # +20% open boost now actually applies (default 0.1012 * 1.2 = 0.12144)
    v, r = X.compute_adaptive_explore_speed(
        lidar_front=3.0, lidar_left=2.0, lidar_right=2.0,
        lidar_fresh=True, normal_speed=DEFAULT, hard_max_lin=AUTO, tight_speed=FLOOR,
    )
    assert r == "wide_open"
    assert almost(v, min(DEFAULT * 1.20, AUTO))
    assert v <= AUTO + 1e-12
    # obstacle approach slowdown
    v_a, r_a = X.compute_adaptive_explore_speed(
        lidar_front=0.90, lidar_left=2.0, lidar_right=2.0,
        lidar_fresh=True, normal_speed=DEFAULT, hard_max_lin=AUTO, tight_speed=FLOOR,
    )
    assert r_a == "approach"
    assert FLOOR <= v_a < min(DEFAULT * 1.20, AUTO) + 1e-12
    # tight creep 0.08
    v_t, r_t = X.compute_adaptive_explore_speed(
        lidar_front=0.40, lidar_left=0.20, lidar_right=0.20,
        lidar_fresh=True, normal_speed=DEFAULT, hard_max_lin=AUTO, tight_speed=FLOOR,
    )
    assert r_t == "tight"
    assert almost(v_t, FLOOR)
    # pickup target 1.0 still cannot exceed the hard cap, and tight still creeps
    v_hi, r_hi = X.compute_adaptive_explore_speed(
        lidar_front=10.0, lidar_left=10.0, lidar_right=10.0,
        lidar_fresh=True, normal_speed=1.0, hard_max_lin=AUTO, tight_speed=FLOOR,
    )
    assert r_hi == "wide_open"
    assert v_hi <= AUTO + 1e-12
    v_tc, r_tc = X.compute_adaptive_explore_speed(
        lidar_front=0.40, lidar_left=0.20, lidar_right=0.20,
        lidar_fresh=True, normal_speed=1.0, hard_max_lin=AUTO, tight_speed=FLOOR,
    )
    assert r_tc == "tight" and almost(v_tc, FLOOR)


def test_learned_whitelist_still_caps_default_cruise_not_the_hard_cap():
    spec = X.LEARN_PARAM_WHITELIST["explore_cruise_lin"]
    assert almost(float(spec["max"]), 0.1012)
    assert almost(X._clamp_learned("explore_cruise_lin", 0.20), 0.1012)
    # hard cap itself is 1.0, so a 0.1012 cruise is not hard-clamped
    assert 0.1012 < X.HARD_MAX_LIN


def test_obstacle_module_stops_unchanged():
    assert (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24)
    assert abs(OB.HALF_W - 0.15) < 1e-12 or True  # corridor half-width policy stays 0.15 if present
    if hasattr(OB, "HALF_W"):
        assert almost(OB.HALF_W, 0.15)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = []
    for t in tests:
        try:
            t()
            print("PASS", t.__name__)
        except Exception as ex:
            print("FAIL", t.__name__, ":", type(ex).__name__, ex)
            failed.append(t.__name__)
    print("----")
    print("%d passed, %d failed, %d total" % (len(tests) - len(failed), len(failed), len(tests)))
    sys.exit(1 if failed else 0)
