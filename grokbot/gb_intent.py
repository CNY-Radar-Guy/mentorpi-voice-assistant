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
        "source": "voice",
        "action": "bounded_look",
        "revoked": False,
    }


_BOOK = {"lease": None}


def lease_path():
    explicit = os.environ.get("GB_LEASE_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    import gb_paths
    return os.path.join(gb_paths.shared_dir(), "lease.json")


def envelope_path():
    explicit = os.environ.get("GB_GATE_ENVELOPE")
    if explicit:
        return os.path.expanduser(explicit)
    import gb_paths
    return os.path.join(gb_paths.shared_dir(), "gate_envelope.json")


def _atomic(path, body):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(body, f)
    os.replace(tmp, path)
    return path


def write_lease(lease, path=None):
    return _atomic(path or lease_path(), dict(lease))


def read_lease(path=None):
    path = path or lease_path()
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(body, dict) or not body.get("id"):
        return None
    return body


def write_envelope(intent_id, lin, ang, now, path=None):
    """The gate's current twist. A raw motor list cannot invent this."""
    return _atomic(path or envelope_path(), {
        "intent_id": intent_id,
        "lin": float(lin),
        "ang": float(ang),
        "mono": float(now),
        "source": "gate",
        "action": "bounded_look",
        "cleared": False,
    })


def clear_envelope(now, why, path=None):
    return _atomic(path or envelope_path(), {
        "cleared": True,
        "mono": float(now),
        "why": why,
        "source": "gate",
    })


def read_envelope(now, path=None, fresh_s=0.5):
    path = path or envelope_path()
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(body, dict) or body.get("cleared"):
        return None
    try:
        age = float(now) - float(body.get("mono"))
        lin = float(body.get("lin"))
        ang = float(body.get("ang"))
    except (TypeError, ValueError):
        return None
    if age < 0 or age > float(fresh_s):
        return None
    if body.get("source") != "gate" or body.get("action") != "bounded_look" or not body.get("intent_id"):
        return None
    body["lin"] = lin
    body["ang"] = ang
    return body


def claim_from(lease, envelope, now):
    """Voice lease plus a fresh gate twist. An explore lease alone is not a claim."""
    if not lease or lease.get("revoked") or not envelope:
        return None
    if envelope.get("intent_id") != lease.get("id"):
        return None
    if lease.get("source") != "voice" or lease.get("action") != "bounded_look":
        return None
    try:
        deadline = float(lease.get("mono_deadline", lease.get("deadline")))
        if float(now) > deadline:
            return None
    except (TypeError, ValueError):
        return None
    return {
        "lease_id": lease["id"],
        "source": "voice",
        "action": "bounded_look",
        "lin": envelope["lin"],
        "ang": envelope["ang"],
    }


def issue_lease(text, now, cap_s=45.0):
    """One monotonic deadline. A second request before it ends does not move it.

    `now` is the caller's clock. Voice and the container node pass time.monotonic()
    so a wall-clock step cannot extend the lease. The same number is not comparable
    across hosts that do not share CLOCK_MONOTONIC.
    """
    now = float(now)
    cur = _BOOK["lease"]
    if cur and not cur.get("revoked"):
        try:
            deadline = float(cur.get("mono_deadline", cur.get("deadline")))
            if now <= deadline:
                return cur
        except (TypeError, ValueError):
            pass
    lease = open_lease(text, now, cap_s)
    lease["mono"] = now
    lease["mono_deadline"] = now + float(lease["cap_s"])
    lease["deadline"] = lease["mono_deadline"]
    _BOOK["lease"] = lease
    try:
        write_lease(lease)
    except OSError:
        pass
    return lease


def active_lease():
    return _BOOK["lease"]


def revoke_active(now, why):
    """Stop cancels the lease. The deadline can move earlier and never later."""
    cur = _BOOK["lease"]
    if not cur:
        return None
    now = float(now)
    try:
        cur["deadline"] = min(float(cur["deadline"]), now)
    except (TypeError, ValueError):
        cur["deadline"] = now
    cur["revoked"] = True
    cur["revoke_why"] = why
    cur["mono_deadline"] = cur["deadline"]
    try:
        write_lease(cur)
        clear_envelope(now, why)
    except OSError:
        pass
    return cur


def propose_repair(explanation):
    return {
        "id": "repair-%d" % int(time.time() * 1000),
        "explanation": explanation,
        "state": "proposed",
        "approved": False,
        "applied": False,
        "approver": None,
        "retest": None,
        "interlock": False,
    }


def accept_repair(record, approver):
    """A named person accepts the proposal. Nothing is applied and nothing moves."""
    if not record or not approver or str(approver).strip().lower() == "auto":
        return None
    out = dict(record)
    out["approved"] = True
    out["approver"] = str(approver)
    out["state"] = "approved"
    out["applied"] = False
    return out


def record_retest(record, passed, supervisor):
    """Supervised result after approval. Still not an automatic patch."""
    if not record or not record.get("approved") or not supervisor or str(supervisor).strip().lower() == "auto":
        return None
    out = dict(record)
    out["retest"] = "passed" if passed else "failed"
    out["state"] = "retest_passed" if passed else "retest_failed"
    out["supervisor"] = str(supervisor)
    out["applied"] = False
    return out


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
