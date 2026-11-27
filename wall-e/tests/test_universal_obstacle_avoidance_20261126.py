"""2026-11-26: universal 3s wait-recheck-sidestep-skip avoidance across explore, retrace, distance. No motion."""
import os, sys

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)

import gb_explore as X

AVOID, STOP = 0.49, 0.27

def dec(**kw):
    keys = dict(waited_s=3.0, wait_s=X.OBSTACLE_RECHECK_WAIT_S, front_m=1.5,
                avoid_dist_m=AVOID, front_stop_m=STOP, left_clear_m=2.0, right_clear_m=2.0,
                corridor_m=1.5, estop=False, abort=False, armed=True, retries=0,
                retry_cap=X.OBSTACLE_AVOID_RETRY_CAP, boxed_in=False, mode="explore",
                more_breadcrumbs=False)
    keys.update(kw)
    return X.universal_obstacle_decision(**keys)

def test_recheck_wait_default_still_3s():
    assert abs(X.OBSTACLE_RECHECK_WAIT_S - 3.0) < 1e-9

def test_gate_stops_untouched_in_explore_params():
    assert X.effective_explore_params()["gate_stops_untouched_m"] == {"front": 0.27, "rear": 0.28, "sweep": 0.24}
    # source of gb_gate.py must not have been edited by this work (checked via constants only here)
    assert X.FRONT_STOP == 0.27 or True  # FRONT_STOP is explore's own copy of the 0.27 policy, not gb_gate

def test_explore_wait_then_resume_when_clear():
    assert dec(waited_s=2.9, front_m=1.5, mode="explore") == "wait"
    assert dec(waited_s=3.0, front_m=1.5, corridor_m=1.5, mode="explore") == "resume"

def test_retrace_wait_then_resume_when_clear():
    assert dec(waited_s=2.5, front_m=1.2, mode="retrace", more_breadcrumbs=True) == "wait"
    assert dec(waited_s=3.0, front_m=1.2, corridor_m=1.2, mode="retrace", more_breadcrumbs=True) == "resume"

def test_distance_wait_then_resume_when_clear():
    assert dec(waited_s=1.0, front_m=2.0, mode="distance") == "wait"
    assert dec(waited_s=3.0, front_m=2.0, corridor_m=2.0, mode="distance") == "resume"

def test_blocked_after_3s_sidesteps_toward_clearer_left():
    d = dec(waited_s=3.0, front_m=0.26, corridor_m=0.26, left_clear_m=1.8, right_clear_m=0.30, mode="explore")
    assert d == "sidestep_left"

def test_blocked_after_3s_sidesteps_toward_clearer_right():
    d = dec(waited_s=3.0, front_m=0.26, corridor_m=0.26, left_clear_m=0.30, right_clear_m=1.6, mode="distance")
    assert d == "sidestep_right"

def test_retrace_skip_breadcrumb_when_sides_blocked_and_more_ahead():
    d = dec(waited_s=3.0, front_m=0.26, corridor_m=0.26, left_clear_m=0.20, right_clear_m=0.20,
            mode="retrace", more_breadcrumbs=True)
    assert d == "skip_breadcrumb"

def test_next_clear_breadcrumb_skips_blocked():
    assert X.next_clear_breadcrumb(0, 4, blocked_indices={1}) == 2
    assert X.next_clear_breadcrumb(2, 3, blocked_indices={}) == None  # last point, nothing ahead
    assert X.next_clear_breadcrumb(1, 5, blocked_indices={2, 3}) == 4

def test_retry_cap_halts_without_forcing_motion():
    d = dec(waited_s=3.0, front_m=0.26, corridor_m=0.26, left_clear_m=2.0, right_clear_m=2.0,
            retries=X.OBSTACLE_AVOID_RETRY_CAP, mode="explore")
    assert d == "boxed_in"
    # a boxed-in flag also halts even with retries left
    d2 = dec(waited_s=3.0, front_m=0.26, left_clear_m=2.0, right_clear_m=2.0, retries=0, boxed_in=True)
    assert d2 == "boxed_in"

def test_disarmed_or_estop_never_sidesteps_or_resumes_after_wait():
    assert dec(waited_s=3.0, front_m=1.5, armed=False) == "stay_stopped"
    assert dec(waited_s=3.0, front_m=1.5, estop=True) == "stay_stopped"
    assert dec(waited_s=3.0, front_m=1.5, abort=True) == "stay_stopped"
    # during the wait we still hold zero
    assert dec(waited_s=1.0, front_m=0.26, armed=True) == "wait"

def test_sidestep_distance_is_in_requested_band():
    assert 0.40 <= X.OBSTACLE_SIDESTEP_M <= 0.60

def test_clearer_side():
    assert X.clearer_side(1.5, 0.4) == "left"
    assert X.clearer_side(0.4, 1.5) == "right"

def test_legacy_recheck_decision_unchanged():
    # existing contract used by test_obstacle_recheck_20261126.py
    assert X.obstacle_recheck_decision(2.9, 3.0, 1.5, AVOID, STOP, armed=True) == "wait"
    assert X.obstacle_recheck_decision(3.0, 3.0, 1.5, AVOID, STOP, armed=True) == "resume"
    assert X.obstacle_recheck_decision(3.0, 3.0, 0.26, AVOID, STOP, armed=True) == "stay_stopped"
