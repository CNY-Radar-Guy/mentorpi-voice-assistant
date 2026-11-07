"""Last ROS check before a motor command is handed to the controller.

Zeros pass. A nonzero command passes only inside a live lease, under the
provisional per-track cap, and never for a place this program cannot detect.
Direct serial writes do not come here. This module publishes nothing.
"""
import os
import gb_motor_cap as cap

ZERO = ((1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0))


def _zero_like(pairs):
    return all(abs(rps) <= 1e-9 for _mid, rps in pairs)


def admit(pairs, now, lease=None, maint=False):
    """Return (speeds, decision). Rejected motion becomes four zero speeds."""
    decision = {
        "hardware_top_speed_verified": False,
        "provisional_software_cap": True,
        "half_rps": cap.half_rps(),
        "firmware_cap": False,
        "serial_bypass": True,
        "all_writer_pause": False,
        "physical_stop_verified": False,
        "wheel_feedback": False,
        "repair": None,
    }
    clamped = cap.clamp_motors(pairs)
    if clamped is None:
        decision.update(admitted=False, reason="malformed")
        decision["repair"] = _repair("The motor command was malformed or non-finite and was replaced with zeros.")
        return list(ZERO), decision
    if _zero_like(clamped):
        decision.update(admitted=True, reason="zero")
        return clamped, decision
    if maint:
        decision.update(admitted=False, reason="maintenance")
        decision["repair"] = _repair("A nonzero motor command arrived while the maintenance hold is on. It was replaced with zeros.")
        return list(ZERO), decision
    if not lease:
        decision.update(admitted=False, reason="no_lease")
        decision["repair"] = _repair("A nonzero motor command had no live lease. It was replaced with zeros.")
        return list(ZERO), decision
    if lease.get("condition_kind") == "place" or (
            lease.get("condition") and lease.get("condition_enforced") is False and lease.get("condition_kind") != "told"):
        decision.update(admitted=False, reason="unenforced_condition")
        decision["repair"] = _repair("The stop condition cannot be detected, so the motion was not started.")
        return list(ZERO), decision
    try:
        deadline = float(lease.get("deadline"))
    except (TypeError, ValueError):
        deadline = None
    if deadline is None or float(now) > deadline:
        decision.update(admitted=False, reason="deadline")
        decision["repair"] = _repair("The lease deadline passed. The nonzero command was replaced with zeros.")
        return list(ZERO), decision
    changed = any(abs(float(a[1]) - float(b[1])) > 1e-9 for a, b in zip(pairs, clamped))
    decision.update(admitted=True, reason="clamped" if changed else "within_provisional_cap")
    if changed:
        decision["repair"] = _repair("A motor was above the provisional half-speed ceiling and was clamped. This is not a verified hardware maximum.")
    return clamped, decision


def _repair(text):
    return {
        "explanation": text,
        "approved": False,
        "applied": False,
        "retest": "supervised",
        "interlock": False,
    }


def maint_held(path=None):
    path = path or os.environ.get("GB_MAINT_FILE")
    return bool(path) and os.path.exists(path)
