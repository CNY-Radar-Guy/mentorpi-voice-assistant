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


def claim_ok(claim, lease):
    """A raw motor list cannot borrow an explore lease. Source and action must match."""
    if not isinstance(claim, dict) or not lease or lease.get("revoked"):
        return False
    if claim.get("lease_id") != lease.get("id"):
        return False
    if claim.get("source") != lease.get("source") or claim.get("source") not in ("voice", "gate"):
        return False
    if claim.get("action") != lease.get("action") or claim.get("action") != "bounded_look":
        return False
    return True


def admit(pairs, now, lease=None, maint=False, claim=None):
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
    if not lease or not claim_ok(claim, lease):
        decision.update(admitted=False, reason="no_matching_lease")
        decision["repair"] = _repair("A nonzero motor command did not match the live lease source and action. It was replaced with zeros.")
        return list(ZERO), decision
    try:
        lin = float(claim.get("lin"))
        ang = float(claim.get("ang"))
    except (TypeError, ValueError):
        lin = ang = None
    if lin is None or not cap.motors_match_twist(pairs, lin, ang):
        decision.update(admitted=False, reason="direction")
        decision["repair"] = _repair("The motor command did not match the leased twist, so it was not allowed to borrow the lease. It was replaced with zeros.")
        return list(ZERO), decision
    if lease.get("condition_kind") == "place" or (
            lease.get("condition") and lease.get("condition_enforced") is False and lease.get("condition_kind") != "told"):
        decision.update(admitted=False, reason="unenforced_condition")
        decision["repair"] = _repair("The stop condition cannot be detected, so the motion was not started.")
        return list(ZERO), decision
    try:
        deadline = float(lease.get("mono_deadline", lease.get("deadline")))
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


class ControllerHook(object):
    """Runs inside the controller process, not inside gb-actuation.

    After a nonzero admission, idle_tick writes zeros once the command is older
    than EXPIRE_S. A dead gb-actuation timer cannot do that. Watchdog zeros are
    accepted only when every motor is zero, so the watchdog cannot become a
    second nonzero writer.
    """

    EXPIRE_S = 0.5

    def __init__(self):
        self.last_nonzero = None
        self.serial = []

    def set_motor_state(self, pairs, now, lease=None, maint=False, claim=None):
        speeds, decision = admit(pairs, now, lease, maint=maint, claim=claim)
        self.serial.append(list(speeds))
        if any(abs(r) > 1e-9 for _i, r in speeds):
            self.last_nonzero = float(now)
        else:
            self.last_nonzero = None
        decision["boundary"] = "controller_callback"
        decision["enforced_on_robot"] = False
        return list(speeds), decision

    def idle_tick(self, now):
        if self.last_nonzero is None:
            return None, {"reason": "idle", "enforced_on_robot": False, "serial_bypass": True}
        if float(now) - self.last_nonzero <= self.EXPIRE_S:
            return None, {"reason": "still_fresh", "enforced_on_robot": False, "serial_bypass": True}
        self.serial.append(list(ZERO))
        self.last_nonzero = None
        return list(ZERO), {
            "reason": "stale_nonzero_zeroed",
            "boundary": "controller_callback",
            "enforced_on_robot": False,
            "serial_bypass": True,
            "physical_stop_verified": False,
        }

    def watchdog_zero(self, pairs):
        clamped = cap.clamp_motors(pairs)
        if clamped is None or not _zero_like(clamped):
            self.serial.append(list(ZERO))
            return list(ZERO), {"reason": "watchdog_nonzero_rejected", "serial_bypass": True}
        self.serial.append(list(clamped))
        self.last_nonzero = None
        return list(clamped), {"reason": "watchdog_zero", "serial_bypass": True}


RAW_MOTOR = "/ros_robot_controller/set_motor"
ADMITTED_TOPIC = "/gb/set_motor_admitted"
CONTAINER_MAINT = "/home/ubuntu/shared/gb_safety/MAINT"


def watchdog_zero_reachable(subscriptions):
    """The controller must still receive the raw motor topic after any extra subscription.

    A remap that replaces ~/set_motor with only the public admitted topic drops
    the watchdog's direct zero. That plan is not reachable.
    """
    names = set(subscriptions or ())
    return RAW_MOTOR in names


def naive_remap_subscriptions():
    """The rejected plan: one subscription, the public topic, raw zeros gone."""
    return {ADMITTED_TOPIC}


def kept_watchdog_subscriptions():
    """Raw zeros stay. The public topic is not proof; the callback rechecks."""
    return {RAW_MOTOR, ADMITTED_TOPIC}


def board_command(raw_pairs, admitted_pairs, now, lease, maint, claim, hook):
    """What the controller process may write. gb-actuation being dead is irrelevant.

    All-zero on the raw topic is the watchdog path and clears a remembered
    nonzero. A nonzero raw command is not a second writer. Admitted nonzero
    still has to match the lease. After the last admitted nonzero, idle_tick
    in this same object writes zeros without a timer inside gb-actuation.
    """
    if raw_pairs is not None:
        raw = cap.clamp_motors(raw_pairs)
        if raw is not None and _zero_like(raw):
            hook.last_nonzero = None
            hook.serial.append(list(raw))
            return list(raw), {
                "reason": "watchdog_raw_zero",
                "writer": "watchdog_or_idle",
                "alternate_nonzero": False,
                "enforced_on_robot": False,
                "serial_bypass": True,
                "physical_stop_verified": False,
                "boundary": "controller_callback",
            }
        if admitted_pairs is None:
            hook.serial.append(list(ZERO))
            return list(ZERO), {
                "reason": "raw_nonzero_rejected",
                "writer": None,
                "alternate_nonzero": False,
                "enforced_on_robot": False,
                "serial_bypass": True,
                "boundary": "controller_callback",
            }
    if admitted_pairs is None:
        return hook.idle_tick(now)
    speeds, decision = hook.set_motor_state(admitted_pairs, now, lease, maint=maint, claim=claim)
    decision["alternate_nonzero"] = False
    decision["public_topic_bypass"] = True
    return speeds, decision


def install_plan(maint_present, hook_loaded, raw_zero_kept, watchdog_active,
                 host_python=False, flat_tree=False, container_maint=True):
    """A partial or host-python install does not enable the unit and does not touch the hold."""
    base = {"enable": False, "start_unit": False, "restart_controller": False,
            "clear_maint": False, "stop_watchdog": False, "remap_only": False}
    if host_python or flat_tree or not container_maint:
        base.update(ok=False, reason="host_python_or_flat_tree_or_host_maint_path")
        return base
    if not maint_present or not watchdog_active or not raw_zero_kept:
        base.update(ok=False, reason="hold_watchdog_or_raw_zero_missing")
        return base
    if not hook_loaded:
        base.update(ok=False, reason="controller_hook_not_in_process")
        return base
    base.update(ok=True, reason="files_ready_supervised_start_not_performed")
    return base


def supervised_controller_step(approved, hook_loaded, raw_zero_kept, maint_present, watchdog_active):
    """Restarting the controller drops the motor link. This run does not do it."""
    base = {"restart_controller": False, "clear_maint": False, "stop_watchdog": False,
            "remap_only": False, "start_unit": False}
    if not approved:
        base.update(ok=False, reason="controller_link_interruption_needs_supervised_activation")
        return base
    if not (hook_loaded and raw_zero_kept and maint_present and watchdog_active):
        base.update(ok=False, reason="supervised_step_missing_hook_zero_hold_or_watchdog")
        return base
    base.update(ok=True, start_unit=True, restart_controller=True,
                reason="supervised_only_raw_zero_kept_hook_rechecks_admitted")
    return base


def rollback_plan(raw_zero_kept, hook_loaded, maint_present, watchdog_active, naive_remap=False):
    """Do not put stock nonzero writers back on the raw motor topic."""
    base = {"clear_maint": False, "stop_watchdog": False, "restore_raw_nonzero_writers": False,
            "remove_raw_zero": False, "retest": "supervised", "maint_is_hardware_inhibit": False,
            "restart_controller": False}
    if not maint_present or not watchdog_active:
        base.update(ok=False, stop_actuation=False, reason="hold_or_watchdog_missing")
        return base
    if naive_remap or not raw_zero_kept:
        base.update(ok=False, stop_actuation=False,
                    reason="refusing to restore the raw motor route; that would reopen stock writers, and MAINT is not a hardware inhibit")
        return base
    if not hook_loaded:
        base.update(ok=False, stop_actuation=False,
                    reason="controller hook is not loaded, so stopping admission would not by itself zero a board holding the last speed")
        return base
    base.update(ok=True, stop_actuation=True,
                reason="actuation can stop; the controller hook still times out to zero; the raw zero subscription stays")
    return base


def maint_held(path=None):
    path = path or os.environ.get("GB_MAINT_FILE")
    return bool(path) and os.path.exists(path)
