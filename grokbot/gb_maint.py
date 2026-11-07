"""Maintenance lock. Separate from the normal stop latch and from a safety fault.

Nothing in the 15 s ESTOP timer, a voice restart, resume, or a movement command
clears this file. Clear it only with clear_lock(explicit=True) after checks.
"""
import json, os, time

NAME = "MAINT"


def default_path():
    import gb_paths
    return os.path.join(gb_paths.data_dir(), NAME)


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


def _read(path):
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    return body if isinstance(body, dict) else None


def _rewrite(path, body):
    payload = json.dumps(body).encode()
    fd = os.open(path, os.O_RDWR)
    try:
        import fcntl
        fcntl.flock(fd, fcntl.LOCK_EX)
        os.ftruncate(fd, 0)
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, payload)
    finally:
        os.close(fd)


def arm_bench(path, success=False, now=None):
    """After a finished deploy, mark that interlock as a bench hold.

    A wake cannot do this. Failure, a fault lock, an in-progress deploy, and an
    untyped file stay as they are. Does not create a lock and does not touch ESTOP.
    """
    if not success or not path or os.path.basename(path) != NAME or not os.path.lexists(path):
        return False
    body = _read(path)
    if not body:
        return False
    if body.get("scope") == "bench-test" and body.get("armed") == "post-deploy" and body.get("kind") == "bench":
        return True
    if body.get("scope") in ("deployment-in-progress", "fault") or body.get("kind") == "fault":
        return False
    why = str(body.get("why") or "")
    if body.get("kind") != "maintenance" or body.get("src") != "maintenance" or "deploy interlock" not in why:
        return False
    now = time.time() if now is None else float(now)
    body.update({
        "t": now,
        "src": "bench",
        "kind": "bench",
        "scope": "bench-test",
        "armed": "post-deploy",
        "release": "wake-and-john",
        "why": "bench hold after a checked deploy; only a fresh wake plus I am John clears it",
    })
    _rewrite(path, body)
    return True


def release_own_deploy_hold(path, not_before=None):
    """Remove only a deploy interlock this activation created. Does not arm a bench hold.

    Leaves a fault, an unknown file, an older lock, and every ESTOP file alone.
    """
    if not path or os.path.basename(path) != NAME or not os.path.lexists(path):
        return False
    body = _read(path) or {}
    if body.get("kind") != "maintenance" or body.get("src") != "maintenance":
        return False
    if "deploy interlock" not in str(body.get("why") or ""):
        return False
    if body.get("scope") in ("fault", "deployment-in-progress") or body.get("kind") == "fault":
        return False
    if not_before is not None:
        try:
            if float(body.get("t") or 0) + 2 < float(not_before):
                return False
        except (TypeError, ValueError):
            return False
    try:
        os.remove(path)
    except OSError:
        return False
    return True


def note_bench_wake(now=None, source="voice"):
    """Remember a fresh spoken wake. A model or replayed source does not count."""
    if source != "voice":
        return False
    now = time.time() if now is None else float(now)
    _BENCH["wake_at"] = now
    _BENCH["source"] = source
    return True


def release_bench(paths, john_ok, now=None, source="voice", model=False):
    """Remove a post-deploy bench hold only. ESTOP and every other lock stay.

    Returns the paths removed. Does not publish velocity or enable autonomy.
    """
    now = time.time() if now is None else float(now)
    wake_at = _BENCH.get("wake_at")
    if (model or source != "voice" or not john_ok or _BENCH.get("source") != "voice"
            or wake_at is None or now + 0.05 < float(wake_at) or now - float(wake_at) > 30.0):
        return []
    removed = []
    for path in paths or []:
        if not path or os.path.basename(path) != NAME or not os.path.lexists(path):
            continue
        body = _read(path) or {}
        if body.get("kind") != "bench" or body.get("scope") != "bench-test" or body.get("armed") != "post-deploy":
            continue
        try:
            os.remove(path)
        except OSError:
            continue
        removed.append(path)
    return removed


_BENCH = {"wake_at": None, "source": None}


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
