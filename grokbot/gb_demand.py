"""Demand file. Follow and pointing turn the camera worker on before any motion.

The RGB guard used to subscribe only while the gate was already moving, so a
follow request could not see a person until the robot moved, and it would not
move until it saw a person. A fresh demand file breaks that cycle. It is not
permission to drive.
"""
import json, os, time

MODES = ("follow", "point", "view", "acquire")
VIEW_TTL = 2.0
ACQUIRE_TTL = 20.0
VIEW_DUTY = 0.12


def view_period(infer_s, duty=VIEW_DUTY, min_s=2.0, max_s=8.0):
    """Seconds between view-only detections. Duty is a share of one core, not the machine."""
    try:
        infer = float(infer_s)
    except (TypeError, ValueError):
        infer = 0.7
    if not (infer > 0):
        infer = 0.7
    return min(float(max_s), max(float(min_s), infer / float(duty)))


def moves(mode):
    """Follow and point are motor modes. View and acquire only wake the camera."""
    return mode in ("follow", "point")


def tracker_wants(mode, moving=False):
    """Camera subscription. Acquire does not require motion or a viewer."""
    return bool(moving or mode in ("follow", "point", "view", "acquire"))


def tracker_period(mode, moving, infer_s):
    """Acquire and view stay on the low duty. Follow while active may run at 1 Hz."""
    if mode in ("view", "acquire") and not moving:
        return view_period(infer_s)
    return 1.0


def write_demand(path, mode, now=None):
    if mode not in MODES and mode != "idle":
        raise ValueError(mode)
    now = time.time() if now is None else float(now)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    body = {"t": now, "mode": mode}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(body, f)
    os.replace(tmp, path)
    return body


def read_demand(path, now=None, ttl=None):
    now = time.time() if now is None else float(now)
    try:
        with open(path) as f:
            body = json.load(f)
        age = now - float(body["t"])
        mode = body.get("mode")
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if ttl is None:
        ttl = VIEW_TTL if mode == "view" else (ACQUIRE_TTL if mode == "acquire" else 30.0)
    if age < 0 or age > float(ttl) or mode not in MODES:
        return None
    return mode
