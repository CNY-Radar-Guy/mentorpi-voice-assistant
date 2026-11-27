"""gb_floor.py - 2026-11-25 noestop (John, voice 14:02 ET): on the FLOOR Wally never goes into a latched emergency stop.

Rule: whenever data/learn/mode.json does not say "bench" (ground, or unreadable), every path that used to write a
persistent ESTOP / MAINT latch (gate fault counter, hard tilt, remote ESTOP button, spoken "emergency stop", clap,
SSH touch) does a NORMAL stop instead: zero now + the gate's 10 Hz zero for 2 s, end the mission, disarm (exactly like
a spoken "stop"), log the cause. No latch file is written and nothing blocks re-arming: "wake up Wally" always works.
A latch file that still appears in ground mode is treated as a one-shot stop and archived with its cause logged.
Bench mode keeps the old latching behaviour (John's bench rule: latches there are cleared by the assistant).

Unchanged: obstacle stop distances (gb_obstacle 0.27/0.28/0.24), slowdown/avoid, deadman 0.5 s, speed caps,
wake-phrase arming, watchdog zeroing on stale/dead sources, back-off after a bump.

Works on the host (reads mode.json directly) and in the container (reads FLOOR_MODE.json, refreshed every 1 s by
gb_safety_run.sh). Missing / stale mode information counts as floor (no latch).
"""
import json, os, time

E = os.environ.get
HOST_MODE_FILE = E("GB_MODE_FILE", "/home/pi/grokbot/data/learn/mode.json")
MIRROR_FRESH_S = 5.0
_CACHE = {"t": 0.0, "mode": None}


def shared_dir():
    d = E("GB_SHARED_DIR")
    if d: return os.path.expanduser(d)
    for d in ("/home/pi/docker/tmp/gb_safety", "/home/ubuntu/shared/gb_safety"):
        if os.path.isdir(d): return d
    return "/home/pi/docker/tmp/gb_safety"


def mirror_path(): return os.path.join(shared_dir(), "FLOOR_MODE.json")
def stop_event_path(): return os.path.join(shared_dir(), "FLOOR_STOP")
def log_path(): return os.path.join(shared_dir(), "floor_stop.log")


def _read_mode(path, fresh_s=None):
    try:
        if fresh_s is not None and time.time() - os.stat(path).st_mtime > fresh_s:
            return None
        with open(path) as f:
            d = json.load(f)
        m = d.get("mode") if isinstance(d, dict) else None
        return str(m).strip().lower() if m else None
    except (OSError, ValueError):
        return None


def mode(max_age=0.5):
    """'bench', 'ground', or 'unknown'. Host mode.json wins; else the container mirror (fresh <= 5 s)."""
    now = time.time()
    if _CACHE["mode"] is not None and now - _CACHE["t"] < max_age:
        return _CACHE["mode"]
    m = None
    if os.path.exists(HOST_MODE_FILE):
        m = _read_mode(HOST_MODE_FILE)
    if m is None:
        m = _read_mode(mirror_path(), MIRROR_FRESH_S)
    m = m or "unknown"
    _CACHE.update(t=now, mode=m)
    return m


def no_latch(max_age=0.5):
    """True on the floor (anything but bench): never write or honour a persistent ESTOP / MAINT latch."""
    return mode(max_age) != "bench"


def log(msg):
    line = time.strftime("%Y-%m-%d %H:%M:%S") + " [floor] " + str(msg)
    try:
        with open(log_path(), "a") as f: f.write(line + "\n")
    except OSError:
        pass
    return line


def request_stop(src, why, was="estop"):
    """Ask for a normal stop through the shared dir: the gate stops on it (once), the host launcher ends missions,
    disarms and logs, then archives the event. Never a latch."""
    d = {"t": time.time(), "ts": time.strftime("%Y-%m-%d %H:%M:%S"), "src": str(src)[:40], "why": str(why)[:200],
         "was": was, "kind": "floor-normal-stop"}
    p = stop_event_path()
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = "%s.tmp.%d" % (p, os.getpid())
        with open(tmp, "w") as f: json.dump(d, f)
        os.replace(tmp, p)
    except OSError as ex:
        log("stop event write failed: %s" % ex)
    log("NORMAL STOP (no latch) src=%s was=%s why=%s" % (d["src"], was, d["why"]))
    return d


def archive(path, why="ground mode: latch not honoured", archive_dir=None):
    """Move a latch file aside (never delete it) and log its cause. Returns the new path or None."""
    if not path or not os.path.lexists(path):
        return None
    try:
        with open(path) as f: body = f.read(2000)
    except OSError:
        body = ""
    d = archive_dir or os.path.join(os.path.dirname(path), "floor_archive")
    try:
        os.makedirs(d, exist_ok=True)
        dst = os.path.join(d, "%s.floor-archived-%s-%d" % (os.path.basename(path), time.strftime("%Y%m%d-%H%M%S"),
                                                         int((time.time() % 1) * 1000)))
        os.replace(path, dst)
    except OSError as ex:
        log("archive failed %s: %s" % (path, ex))
        return None
    log("ARCHIVED %s -> %s (%s) cause=%s" % (path, dst, why, body.strip().replace("\n", " ")[:300]))
    return dst
