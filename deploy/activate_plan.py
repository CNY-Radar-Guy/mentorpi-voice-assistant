#!/usr/bin/env python3
"""Decisions for a safety restart. The shell script is what the parent runs on the Pi.

This module does not talk to Docker and does not delete an ESTOP file.
"""
import json, os, sys, time


def parse_env(text):
    vals = {}
    for line in (text or "").splitlines():
        s = line.strip()
        if not s or s.startswith("#") or "=" not in s:
            continue
        key, val = s.split("=", 1)
        vals[key.strip()] = val.strip().strip('"').strip("'")
    return vals


def voice_restart_allowed(env_text, estop_present=False):
    """Empty greeting fields are not enough. The suppress switch must be set.

    A reload is optional. The default activation leaves grokbot running so the
    spoken stop listener stays up. This function does not clear a latch.
    """
    del estop_present
    vals = parse_env(env_text)
    if vals.get("SUPPRESS_STARTUP_SPEECH", "0") != "1":
        return False, "SUPPRESS_STARTUP_SPEECH=1 is required; empty greeting fields are not a guarantee"
    if vals.get("ALLOW_AUTONOMY", "0") == "1":
        return False, "ALLOW_AUTONOMY=1 is not part of this activation"
    if vals.get("ESTOP_AUTOCLEAR_S", "0") not in ("0", ""):
        return False, "ESTOP_AUTOCLEAR_S must be 0 before a voice reload"
    return True, "startup speech is suppressed; a normal stop latch is not cleared by the reload"


def write_transition_latch(paths, now=None):
    """Create a latch only at paths that do not exist. Never removes or rewrites one.

    Returns the list of paths created.
    """
    now = time.time() if now is None else float(now)
    body = json.dumps({
        "t": now,
        "src": "activation",
        "why": "safety restart transition; this script does not clear the latch",
    })
    created = []
    for path in paths:
        if not path or os.path.lexists(path):
            continue
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        try:
            os.write(fd, body.encode())
        finally:
            os.close(fd)
        created.append(path)
    return created


def within_window(elapsed_s, limit=2.0):
    try:
        return float(elapsed_s) <= float(limit)
    except (TypeError, ValueError):
        return False


PLAN = (
    "Read live docker inspect .Mounts and, when readable, /proc/1/mountinfo. Abort with no service changes if /home/pi/docker/tmp is not the live source of /home/ubuntu/shared.",
    "Write the maintenance lock on the host and the shared mount. Do not create or delete data/ESTOP.",
    "Start a zero Twist publisher on /controller/cmd_vel before gb-gate or gb-watchdog is restarted.",
    "Restart gb-gate and gb-watchdog only. Both Python processes must be visible within 2 seconds, and the maintenance lock must still be present. Leave grokbot running so the spoken stop listener stays up.",
    "A voice reload is separate and only when SUPPRESS_STARTUP_SPEECH=1 and ESTOP_AUTOCLEAR_S=0. Empty STARTUP_GREETING is not that switch. The reload must not clear ESTOP or MAINT.",
    "The maintenance lock stays until an explicit post-check clear. Resume, a drive command, and the old 15 second timer do not clear it.",
    "Do not start a follow, do not play a readiness phrase, and do not move web_video_server. Port 8080 stays unmet.",
)


def main(argv):
    if len(argv) < 2:
        print("\n".join(PLAN))
        return 0
    cmd = argv[1]
    if cmd == "plan":
        print("\n".join(PLAN))
        return 0
    if cmd == "voice-ok":
        path = argv[2] if len(argv) > 2 else ""
        text = ""
        if path and os.path.isfile(path):
            with open(path) as f:
                text = f.read()
        estop = len(argv) > 3 and os.path.lexists(argv[3])
        ok, reason = voice_restart_allowed(text, estop)
        print(reason)
        return 0 if ok else 1
    if cmd == "latch":
        created = write_transition_latch(argv[2:])
        print("created " + " ".join(created) if created else "unchanged")
        return 0
    if cmd == "maint":
        import os as _os
        sys.path.insert(0, _os.path.join(_os.path.dirname(__file__), "..", "grokbot"))
        import gb_maint
        created = gb_maint.write_lock(argv[2:])
        print("created " + " ".join(created) if created else "unchanged")
        return 0 if any(_os.path.lexists(p) for p in argv[2:]) else 1
    print("unknown command", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
