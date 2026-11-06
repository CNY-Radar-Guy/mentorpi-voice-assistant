"""Maintenance lock. Separate from the normal stop latch and from a safety fault.

Nothing in the 15 s ESTOP timer, a voice restart, resume, or a movement command
clears this file. Clear it only with clear_lock(explicit=True) after checks.
"""
import json, os, time

NAME = "MAINT"


def default_path():
    base = os.environ.get("GB_DIR") or os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, "data", NAME)


def is_set(path=None):
    return os.path.lexists(path or default_path())


def blocks_velocity(maint_present, estop_present=False):
    """Maintenance blocks every source, including joystick. A normal stop does too."""
    return bool(maint_present or estop_present)


def write_lock(paths, now=None):
    """Create the lock only where it is absent. Never rewrites or removes one."""
    now = time.time() if now is None else float(now)
    body = json.dumps({
        "t": now,
        "src": "maintenance",
        "kind": "maintenance",
        "why": "deploy interlock; movement and resume do not clear this",
    }).encode()
    created = []
    for path in paths:
        if not path or os.path.lexists(path):
            continue
        if os.path.basename(path) != NAME:
            continue
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        try:
            os.write(fd, body)
        finally:
            os.close(fd)
        created.append(path)
    return created


def clear_lock(path, explicit=False):
    """Remove a maintenance file only when the caller passed the post-check flag.

    Refuses any path whose name is not MAINT, so an ESTOP file cannot be deleted here.
    """
    if not explicit or not path or os.path.basename(path) != NAME:
        return False
    try:
        os.remove(path)
    except FileNotFoundError:
        return False
    except OSError:
        return False
    return True
