"""Partial intent record. This is not a motor interlock.

Stock nodes can still publish /controller/cmd_vel and /ros_robot_controller/set_motor.
Odometry in the capture is integrated from the last command, not from the wheels.
Nothing here publishes a twist, clears MAINT, or edits another program.
A person approves a note. The note does not apply a patch.
"""
import json, os, time

BYPASSES = (
    "lidar_app", "line_following", "object_tracking", "hand_gesture",
    "odom_publisher", "rosbridge_websocket", "gb_guard",
)


def status_path():
    explicit = os.environ.get("GB_INTENT_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    shared = os.environ.get("GB_SHARED_DIR")
    if shared:
        return os.path.join(os.path.expanduser(shared), "intent_status.json")
    return "/home/pi/docker/tmp/gb_safety/intent_status.json"


def is_bounded_lease(text):
    """A room look with or without a later stop. Not a map mission and not an open-ended drive."""
    t = " ".join((text or "").lower().split())
    if not t or "map this room" in t:
        return False
    return ("look around" in t) or ("drive around" in t) or (" explore" in " " + t) or t.startswith("explore")


def open_lease(text, now, cap_s=45.0):
    """A fixed lease. A later stop clause is stored and does not extend the lease.

    'stop when I tell you' is kept as a told-stop. 'stop at the door' is kept
    as a place clause and is not enforced: there is no place detector.
    """
    try:
        cap = float(cap_s)
    except (TypeError, ValueError):
        cap = 45.0
    cap = max(5.0, min(45.0, cap))
    now = float(now)
    import gb_estop
    clause = gb_estop.future_condition(text)
    kind = None
    if clause:
        kind = "place" if " at " in (" " + clause) else "told"
    return {
        "id": "look-%d" % int(now * 1000),
        "kind": "bounded_look",
        "text": text,
        "t": now,
        "deadline": now + cap,
        "cap_s": cap,
        "indefinite": False,
        "stop_when_told": kind == "told" or clause is None,
        "condition": clause,
        "condition_kind": kind,
        "condition_enforced": False if kind == "place" else bool(kind == "told"),
        "approved": False,
        "interlock": False,
        "all_writer_pause": False,
    }


def pending_condition(text, now):
    """Store a later stop when no look was asked. Does not move."""
    import gb_estop
    clause = gb_estop.future_condition(text)
    if not clause:
        return None
    kind = "place" if " at " in (" " + clause) else "told"
    return {
        "kind": "pending_condition",
        "text": text,
        "t": float(now),
        "condition": clause,
        "condition_kind": kind,
        "condition_enforced": False if kind == "place" else True,
        "motion": False,
        "indefinite": False,
        "interlock": False,
        "all_writer_pause": False,
    }


def compare(lease, now, observations=()):
    """Expected behavior against what was observed. This does not publish a stop.

    A zero command-derived odom twist does not prove the wheels stopped.
    """
    now = float(now)
    past = now > float(lease["deadline"])
    told = any(item.get("standalone_stop") for item in observations or ())
    ran_past = []
    bypass_nz = []
    commanded = False
    odom_zero = True
    saw_odom = False
    command_derived = False
    for item in observations or ():
        if item.get("nz"):
            commanded = True
            if past or told:
                ran_past.append(item.get("node") or item.get("topic"))
            if item.get("node") in BYPASSES:
                bypass_nz.append(item.get("node"))
        if "odom_vx" in item or "odom_wz" in item:
            saw_odom = True
            if abs(float(item.get("odom_vx") or 0)) > 1e-6 or abs(float(item.get("odom_wz") or 0)) > 1e-6:
                odom_zero = False
            if item.get("odom_source") == "command":
                command_derived = True
    return {
        "expected": "stop" if past or told else "within_lease",
        "past_deadline": past,
        "stop_when_told": bool(told),
        "ran_past_deadline": ran_past,
        "all_writer_pause": False,
        "pause_reason": "stock publishers are not paused; this compare does not stop them",
        "bypass_nonzero": bypass_nz,
        "physical_stop_verified": False,
        "stall_unverified": bool(commanded and saw_odom and odom_zero and command_derived),
        "hardware_interlock": "unknown",
        "wheel_feedback": False,
        "indefinite": False,
        "interlock": False,
        "partial": True,
    }


def repair_record(lease, comparison, explanation):
    """An advisory note a person can accept later. Nothing is applied."""
    return {
        "lease_id": lease.get("id"),
        "explanation": explanation,
        "approved": False,
        "applied": False,
        "interlock": False,
        "all_writer_pause": False,
        "physical_stop_verified": False,
        "hardware_interlock": "unknown",
        "expected": comparison.get("expected"),
        "past_deadline": comparison.get("past_deadline"),
    }


def note(kind, text, approved=False):
    """Remember one request. approved False means a person has not accepted a repair."""
    return {
        "id": "%s-%d" % (kind, int(time.time() * 1000)),
        "kind": kind,
        "text": text,
        "t": time.time(),
        "approved": bool(approved),
        "interlock": False,
    }


def command_has_intent(message, intents):
    """True only when a nonzero command names an existing intent. Absence is a report, not a stop."""
    if not message or not message.get("nz"):
        return True
    ident = message.get("intent_id")
    if not ident:
        return False
    return any(item.get("id") == ident for item in intents or [])


def observe(messages, intents):
    """Say what was seen. interlock is false while the listed nodes can publish around the gate."""
    naked = []
    for message in messages or []:
        if message.get("nz") and not command_has_intent(message, intents):
            naked.append({
                "topic": message.get("topic"),
                "node": message.get("node"),
                "intent_id": message.get("intent_id"),
            })
    return {
        "interlock": False,
        "partial": True,
        "note": "read-only report; stock publishers are not blocked and wheel speed is not measured",
        "bypasses": list(BYPASSES),
        "intent_free": naked,
        "approval": "a person must accept any repair; this file does not apply one",
        "wheel_feedback": False,
    }


def approve(item, accepted):
    """Record a person's decision. The result has no velocity."""
    out = dict(item)
    out["approved"] = bool(accepted)
    out["interlock"] = False
    out.pop("lin", None)
    out.pop("ang", None)
    return out


def write_status(report, path=None):
    path = path or status_path()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    body = dict(report)
    body["t"] = time.time()
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(body, f)
    os.replace(tmp, path)
    return path


def rollback_steps():
    """Stop the observer only. The maintenance hold is not part of this unit."""
    return [
        "systemctl stop gb-intent",
        "systemctl disable gb-intent",
        "do not remove MAINT",
        "do not clear the maintenance hold",
        "do not publish a twist",
    ]
