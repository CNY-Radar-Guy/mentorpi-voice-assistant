"""2026-11-26: adaptive explore speed (wide-open boost, approach ramp, tight floor, depth fusion). No motion."""
import math, os, sys

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)

import gb_explore as X

HARD = 1.0
FLOOR = X.ADAPT_TIGHT_LIN  # 0.08, track breakaway floor
OPEN_F, CLOSE_F = X.ADAPT_OPEN_FRONT_M, X.ADAPT_APPROACH_CLOSE_M


def spd(lidar_front=3.0, lidar_left=2.0, lidar_right=2.0, depth_front=None,
        depth_fresh=True, lidar_fresh=True, normal_speed=HARD, hard_max_lin=HARD,
        tight_speed=FLOOR):
    return X.compute_adaptive_explore_speed(
        lidar_front=lidar_front, lidar_left=lidar_left, lidar_right=lidar_right,
        depth_front=depth_front, depth_fresh=depth_fresh, lidar_fresh=lidar_fresh,
        normal_speed=normal_speed, hard_max_lin=hard_max_lin, tight_speed=tight_speed,
    )


def almost(a, b, eps=1e-12):
    return abs(float(a) - float(b)) <= eps


# --------------------------------------------------------------------------- boost / wide-open / hard cap

def test_hard_max_lin_unchanged():
    assert almost(X.HARD_MAX_LIN, HARD)

def test_boost_formula_with_current_1012_cruise():
    # applied explore_cruise_lin is 0.1012; +20% boost now applies under the 1.0 cap
    assert almost(X.adaptive_boost_speed(0.1012, HARD), 0.1012 * 1.20)
    assert almost(X.adaptive_boost_speed(HARD, HARD), HARD)
    assert almost(min(0.1012 * 1.20, HARD), 0.1012 * 1.20)
    assert almost(min(HARD * 1.20, HARD), HARD)

def test_boost_full_when_normal_is_080():
    # 0.08 * 1.20 = 0.096, now well under the 1.0 hard cap
    assert almost(X.adaptive_boost_speed(0.080, HARD), 0.080 * 1.20)

def test_boost_partial_when_normal_is_070():
    assert almost(X.adaptive_boost_speed(0.070, HARD), 0.070 * 1.20)

def test_boost_never_exceeds_hard_even_if_base_is_higher():
    for base in (0.1012, 0.12, 0.20, 1.0, 2.0):
        assert X.adaptive_boost_speed(base, HARD) <= HARD + 1e-12
    assert almost(X.adaptive_boost_speed(1.0, HARD), HARD)
    assert almost(X.adaptive_boost_speed(2.0, HARD), HARD)
    assert almost(X.adaptive_boost_speed(0.1012, HARD), 0.1012 * 1.20)

def test_wide_open_current_cruise_is_capped_noop():
    v, r = spd(lidar_front=3.0, lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "wide_open"
    assert almost(v, HARD)
    assert v <= HARD + 1e-12

def test_wide_open_boost_applies_when_cruise_below_cap():
    v, r = spd(lidar_front=3.0, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
    assert r == "wide_open"
    assert almost(v, 0.08 * 1.20)  # 0.096, under the 1.0 cap
    assert v <= HARD + 1e-12

def test_wide_open_requires_front_and_both_sides():
    # front open, one side shy of 1.00 -> not wide_open
    v, r = spd(lidar_front=3.0, lidar_left=2.0, lidar_right=0.80, normal_speed=0.08)
    assert r == "normal"
    assert almost(v, 0.08)
    # front just below 1.50, sides open -> approach, not wide_open
    v, r = spd(lidar_front=1.49, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
    assert r == "approach"
    assert v < HARD

def test_wide_open_at_exact_thresholds():
    v, r = spd(lidar_front=1.50, lidar_left=1.00, lidar_right=1.00, normal_speed=0.08)
    assert r == "wide_open"
    assert almost(v, min(0.08 * 1.20, HARD))


# --------------------------------------------------------------------------- proportional approach slowdown

def test_approach_endpoints():
    # at close threshold, floor; just below open threshold, nearly boost (sides open)
    v_lo, r_lo = spd(lidar_front=0.45, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
    # front 0.45 >= tight 0.55? No, 0.45 < 0.55 so tight takes priority
    assert r_lo == "tight"
    assert almost(v_lo, FLOOR)
    # lowest approach front just at tight boundary
    v_mid, r_mid = spd(lidar_front=0.55, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
    assert r_mid == "approach"
    v_hi, r_hi = spd(lidar_front=1.499, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
    assert r_hi == "approach"
    assert v_mid < v_hi

def _approach_speed(front, normal=0.08, sides=2.0):
    high = min(normal * 1.20, HARD)
    span = OPEN_F - CLOSE_F
    t = max(0.0, min(1.0, (front - CLOSE_F) / span))
    return FLOOR + t * (high - FLOOR)

def test_approach_ramp_matches_linear_formula():
    for front in (0.55, 0.70, 0.90, 1.10, 1.30, 1.49):
        v, r = spd(lidar_front=front, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
        assert r == "approach", "front=%.2f regime=%s" % (front, r)
        assert almost(v, _approach_speed(front, 0.08), 1e-12)

def test_approach_monotonic_decrease_as_front_drops():
    fronts = [round(1.50 - 0.05 * i, 4) for i in range(0, 22)]  # 1.50 -> 0.45 inclusive
    speeds = []
    prev = None
    for front in fronts:
        v, r = spd(lidar_front=front, lidar_left=2.0, lidar_right=2.0, normal_speed=0.08)
        speeds.append((front, v, r))
        if front >= 1.50:
            assert r == "wide_open"
        elif front < 0.55:
            assert r == "tight"
            assert almost(v, FLOOR)
        else:
            assert r == "approach"
        if prev is not None:
            assert v <= prev[1] + 1e-12, "speed rose as front dropped: %s -> %s" % (prev, (front, v, r))
        prev = (front, v, r)
    # strictly decreasing through the approach band
    approach = [v for f, v, r in speeds if r == "approach"]
    assert len(approach) >= 8
    for a, b in zip(approach, approach[1:]):
        assert b < a - 1e-15

def test_approach_uses_normal_not_boost_when_sides_not_open():
    # front dropping, sides 0.70 (not tight, not open) -> ramp from normal, not boost
    v, r = spd(lidar_front=0.975, lidar_left=0.70, lidar_right=0.70, normal_speed=0.08)
    assert r == "approach"
    span = OPEN_F - CLOSE_F
    t = (0.975 - CLOSE_F) / span  # 0.5
    expected = FLOOR + 0.5 * (0.08 - FLOOR)
    # FLOOR is 0.08 and normal is 0.08, so expected == 0.08
    assert almost(v, expected)


# --------------------------------------------------------------------------- tight / narrow

def test_tight_both_sides_narrow_even_when_front_is_open():
    v, r = spd(lidar_front=3.0, lidar_left=0.40, lidar_right=0.40, normal_speed=HARD)
    assert r == "tight"
    assert almost(v, FLOOR)

def test_tight_corridor_width_under_1_10():
    # 0.54 + 0.54 = 1.08 < 1.10, and both also < 0.55
    v, r = spd(lidar_front=3.0, lidar_left=0.54, lidar_right=0.54, normal_speed=HARD)
    assert r == "tight"
    assert almost(v, FLOOR)
    # 0.52 + 0.57 = 1.09 < 1.10, even though one side is not < 0.55
    v, r = spd(lidar_front=3.0, lidar_left=0.52, lidar_right=0.57, normal_speed=HARD)
    assert r == "tight"
    assert almost(v, FLOOR)

def test_tight_front_under_0_55():
    v, r = spd(lidar_front=0.50, lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "tight"
    assert almost(v, FLOOR)

def test_one_side_narrow_other_open_is_not_tight():
    v, r = spd(lidar_front=3.0, lidar_left=1.70, lidar_right=0.48, normal_speed=HARD)
    assert r == "normal"
    assert almost(v, HARD)

def test_tight_speed_never_below_track_floor():
    v, r = spd(lidar_front=0.50, lidar_left=0.30, lidar_right=0.30, normal_speed=HARD)
    assert r == "tight"
    assert v >= FLOOR - 1e-12
    assert v <= HARD + 1e-12


# --------------------------------------------------------------------------- depth fusion

def test_valid_closer_depth_slows_down():
    # lidar 2.0 m (would be wide_open with open sides), valid depth 0.80 m -> approach
    v_lidar, r_lidar = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
                           depth_front=None, normal_speed=0.08)
    assert r_lidar == "wide_open"
    v, r = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
               depth_front=0.80, depth_fresh=True, normal_speed=0.08)
    assert r == "approach"
    assert almost(X.fuse_explore_front_m(2.0, 0.80, True), 0.80)
    assert almost(v, _approach_speed(0.80, 0.08))
    assert v < v_lidar

def test_valid_depth_farther_than_lidar_keeps_lidar():
    assert almost(X.fuse_explore_front_m(0.90, 2.40, True), 0.90)
    v, r = spd(lidar_front=0.90, lidar_left=2.0, lidar_right=2.0,
               depth_front=2.40, depth_fresh=True, normal_speed=0.08)
    assert r == "approach"
    assert almost(v, _approach_speed(0.90, 0.08))

def test_blind_zone_depth_under_0_30_ignored():
    assert X.trusted_depth_m(0.29, True) is None
    assert X.trusted_depth_m(0.15, True) is None
    assert X.trusted_depth_m(0.0, True) is None
    assert almost(X.fuse_explore_front_m(2.0, 0.15, True), 2.0)
    v, r = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
               depth_front=0.15, depth_fresh=True, normal_speed=0.08)
    assert r == "wide_open"
    assert almost(v, min(0.08 * 1.20, HARD))

def test_depth_over_3_00_ignored():
    assert X.trusted_depth_m(3.01, True) is None
    assert almost(X.fuse_explore_front_m(2.0, 3.50, True), 2.0)
    v, r = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
               depth_front=3.50, depth_fresh=True, normal_speed=0.08)
    assert r == "wide_open"

def test_depth_none_ignored():
    assert X.trusted_depth_m(None, True) is None
    assert almost(X.fuse_explore_front_m(2.0, None, True), 2.0)

def test_stale_depth_ignored_even_if_in_window():
    assert X.trusted_depth_m(0.80, False) is None
    assert almost(X.fuse_explore_front_m(2.0, 0.80, False), 2.0)
    v, r = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
               depth_front=0.80, depth_fresh=False, normal_speed=0.08)
    assert r == "wide_open"

def test_depth_window_edges_accepted():
    assert almost(X.trusted_depth_m(0.30, True), 0.30)
    assert almost(X.trusted_depth_m(3.00, True), 3.00)
    # 0.30 m depth closer than 2.0 m lidar, and 0.30 < tight 0.55 -> tight
    v, r = spd(lidar_front=2.0, lidar_left=2.0, lidar_right=2.0,
               depth_front=0.30, depth_fresh=True, normal_speed=0.08)
    assert r == "tight"
    assert almost(v, FLOOR)


# --------------------------------------------------------------------------- fail-safe

def test_missing_lidar_front_fails_safe():
    v, r = spd(lidar_front=None, lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "fail_safe"
    assert almost(v, FLOOR)

def test_stale_lidar_fails_safe():
    v, r = spd(lidar_front=3.0, lidar_left=2.0, lidar_right=2.0,
               lidar_fresh=False, normal_speed=HARD)
    assert r == "fail_safe"
    assert almost(v, FLOOR)

def test_nan_and_invalid_sensors_fail_safe():
    v, r = spd(lidar_front=float("nan"), lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "fail_safe"
    assert almost(v, FLOOR)
    v, r = spd(lidar_front="nope", lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "fail_safe"
    v, r = spd(lidar_front=-1.0, lidar_left=2.0, lidar_right=2.0, normal_speed=HARD)
    assert r == "fail_safe"

def test_missing_sides_do_not_boost_or_false_tight():
    # front open, sides unknown: not wide_open, not tight, not approach -> normal
    v, r = spd(lidar_front=3.0, lidar_left=None, lidar_right=None, normal_speed=0.08)
    assert r == "normal"
    assert almost(v, 0.08)
    # one side missing cannot declare a narrow corridor
    v, r = spd(lidar_front=3.0, lidar_left=0.40, lidar_right=None, normal_speed=0.08)
    assert r == "normal"

def test_all_speeds_capped_by_hard_max():
    cases = [
        dict(lidar_front=10.0, lidar_left=10.0, lidar_right=10.0, normal_speed=0.1012),
        dict(lidar_front=0.80, lidar_left=2.0, lidar_right=2.0, normal_speed=0.20),
        dict(lidar_front=0.40, lidar_left=0.20, lidar_right=0.20, normal_speed=0.1012),
        dict(lidar_front=None, normal_speed=0.1012),
    ]
    for kw in cases:
        v, r = spd(**kw)
        assert v <= HARD + 1e-12, "regime=%s speed=%s kw=%s" % (r, v, kw)
        assert v >= 0.0


def test_gate_stops_untouched():
    stops = X.effective_explore_params()["gate_stops_untouched_m"]
    assert stops == {"front": 0.27, "rear": 0.28, "sweep": 0.24}


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
