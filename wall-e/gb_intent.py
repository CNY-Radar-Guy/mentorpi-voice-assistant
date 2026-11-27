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
