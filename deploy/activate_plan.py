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
    """A grokbot restart stays silent and does not release a latch by itself.

    Default ESTOP_AUTOCLEAR_S is 15. Starting the voice process while a latch
    exists would have AutoClear delete that latch and speak. This plan refuses
    that unless auto-clear is already off.
    """
    vals = parse_env(env_text)
    if vals.get("STARTUP_GREETING", ""):
        return False, "STARTUP_GREETING would speak on restart"
    if vals.get("TTS_WARMUP", "0") == "1":
        return False, "TTS_WARMUP=1 would speak on restart"
    if vals.get("HEALTH_SPEAK", "0") == "1":
        return False, "HEALTH_SPEAK=1 would speak on restart"
    if vals.get("ALLOW_AUTONOMY", "0") == "1":
        return False, "ALLOW_AUTONOMY=1 is not part of this activation"
    if estop_present and vals.get("ESTOP_AUTOCLEAR_S", "15") != "0":
        return False, "ESTOP is latched and ESTOP_AUTOCLEAR_S is not 0; starting grokbot would auto-clear it and speak"
    return True, "silent startup preserved; this plan does not clear an ESTOP latch"


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
    "Start a zero Twist publisher on /controller/cmd_vel before any safety process is stopped.",
    "If data/ESTOP is absent, stop grokbot first so the rising-edge reactor cannot speak, then create the latch. Never rewrite or delete an existing latch.",
    "Restart gb-gate and gb-watchdog only. Both Python processes must be visible within 2 seconds.",
    "If that window is missed, leave the latch in place and do not start grokbot.",
    "Do not restart grokbot from this script. A restart can speak, and the default 15 second auto-clear would release the latch.",
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
    print("unknown command", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
