"""Demand file. Follow and pointing turn the camera worker on before any motion.

The RGB guard used to subscribe only while the gate was already moving, so a
follow request could not see a person until the robot moved, and it would not
move until it saw a person. A fresh demand file breaks that cycle. It is not
permission to drive.
"""
import json, os, time

MODES = ("follow", "point")


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


def read_demand(path, now=None, ttl=30.0):
    now = time.time() if now is None else float(now)
    try:
        with open(path) as f:
            body = json.load(f)
        age = now - float(body["t"])
        mode = body.get("mode")
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if age < 0 or age > float(ttl) or mode not in MODES:
        return None
    return mode
