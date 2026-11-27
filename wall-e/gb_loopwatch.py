#!/usr/bin/env python3
"""gb_loopwatch.py - Wally service restart-loop / stuck-cycle watchdog (host, systemd: gb-loopwatch.service).

Added 2026-11-24 ~04:05 PT on John's authorization. It watches; it never drives.

* Non-motion units (voice/wake/TTS/LLM bridge = grokbot, range-health publisher = gb-health):
  - restart loop (>= LOOP_N restarts inside LOOP_W s)  -> stop the unit + alert (it is cycling, more restarts won't help)
  - unit 'failed' (systemd gave up) or stuck (alive but heartbeat/HTTP stalled) -> restart with backoff
    (RESTART_BACKOFF s), at most RESTART_CAP restarts per hour, then stop + alert.
* Motion / safety units (gb-gate, gb-watchdog, gb-guard, gb-rgbguard, grokbot-lowv, button_scan, docker MentorPi,
  ROS bringup): ALERT ONLY + one zero-velocity burst (gb_estop.zero_now). Never restarted, stopped or edited here.
* Never writes, clears or deletes ESTOP / MAINT / CLEAR_REQ files, never touches gb_gate, never changes any watchdog.
Alerts: journal (stdout), status JSON (STATUS_FILES), spoken line via the voice service's /internal/say (rate-limited).
Thresholds (derived from tonight's logs, see docs/loopwatch.md): LOOP_N=4 in LOOP_W=600 s, boot grace 180 s.
"""
import json, os, subprocess, sys, time, urllib.request
from collections import deque

E = os.environ.get
DRY = E("LOOPWATCH_DRY", "0") == "1"
POLL_S = float(E("LOOPWATCH_POLL_S", "30"))
LOOP_N = int(E("LOOPWATCH_LOOP_N", "4")); LOOP_W = float(E("LOOPWATCH_LOOP_W", "600"))
BOOT_GRACE_S = float(E("LOOPWATCH_BOOT_GRACE", "180"))
RESTART_BACKOFF = (60.0, 120.0, 240.0); RESTART_CAP = 3          # per rolling hour, non-motion units only
STUCK_POLLS = 3                                                 # consecutive bad polls before a unit counts as stuck
SAY_EVERY_S = 300.0
SH = "/home/pi/docker/tmp/gb_safety"
STATUS_FILES = (E("LOOPWATCH_STATUS", "/home/pi/grokbot/data/loopwatch.json"), SH + "/loopwatch.json")
SAY_URL = "http://127.0.0.1:%s/internal/say" % E("WEB_PORT", "8000")
VOICE_HEALTH = "http://127.0.0.1:%s/health" % E("WEB_PORT", "8000")
CONTAINER = "MentorPi"

NONMOTION = {   # unit -> stuck probe ("http" url | ("file", path, max_age_s) | None)
    "grokbot.service": ("http", VOICE_HEALTH),
    "gb-health.service": ("file", SH + "/health.json", 30.0),
}
SAFETY = {      # alert only
    "gb-gate.service": ("file", SH + "/sensors.json", 10.0),
    "gb-watchdog.service": ("file", SH + "/watchdog_state.json", 10.0),
    "gb-guard.service": None, "gb-rgbguard.service": None, "grokbot-lowv.service": None, "button_scan.service": None,
}
FORBIDDEN = ("ESTOP", "MAINT", "CLEAR_REQ")                     # paths this program must never write or remove


def log(*a):
    print(time.strftime("%Y-%m-%d %H:%M:%S"), "[loopwatch]", *a, flush=True)


# ------------------------------------------------------------------ pure logic (unit-tested with fixtures)
def restarts_in_window(hist, now, window=LOOP_W):
    """hist: deque of (t, NRestarts). Restarts counted inside the last `window` seconds."""
    inside = [n for t, n in hist if now - t <= window]
    if len(inside) < 2: return 0
    return max(0, inside[-1] - inside[0])

def is_loop(hist, now, uptime_s, n=LOOP_N, window=LOOP_W, grace=BOOT_GRACE_S):
    if uptime_s < grace: return False
    return restarts_in_window(hist, now, window) >= n

def backoff_ok(attempts, now):
    """attempts: list of restart times (ours). -> (allowed, reason)."""
    recent = [t for t in attempts if now - t < 3600]
    if len(recent) >= RESTART_CAP: return False, "cap %d/h reached" % RESTART_CAP
    if recent and now - recent[-1] < RESTART_BACKOFF[min(len(recent), len(RESTART_BACKOFF)) - 1]:
        return False, "backoff"
    return True, "ok"

def decide(unit, cls, active, loop, stuck, attempts, now):
    """-> list of actions: ('alert', msg) / ('zero',) / ('restart', unit) / ('stop', unit). Pure function."""
    acts = []
    if cls == "safety":
        if loop: acts += [("alert", "%s is in a restart loop" % unit), ("zero",)]
        elif active == "failed": acts += [("alert", "%s has failed" % unit), ("zero",)]
        elif stuck: acts += [("alert", "%s looks stuck (heartbeat stale)" % unit), ("zero",)]
        return acts
    if loop:
        return [("stop", unit), ("alert", "%s was restarting in a loop, so I stopped it" % unit)]
    if active == "failed" or stuck:
        ok, why = backoff_ok(attempts, now)
        if ok: return [("restart", unit), ("alert", "%s %s, restarting it" % (unit, "failed" if active == "failed" else "was stuck"))]
        if why.startswith("cap"):
            return [("stop", unit), ("alert", "%s kept failing (%s), so I stopped it" % (unit, why))]
    return acts


# ------------------------------------------------------------------ probes
def _run(cmd, timeout=10):
    try: return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception as ex: return subprocess.CompletedProcess(cmd, 99, "", str(ex))

def systemd_state(units):
    r = _run(["systemctl", "show", "-p", "Id,ActiveState,NRestarts"] + list(units))
    out, cur = {}, {}
    for line in (r.stdout or "").splitlines() + [""]:
        if not line.strip():
            if cur.get("Id"): out[cur["Id"]] = cur
            cur = {}; continue
        k, _, v = line.partition("="); cur[k] = v
    return out

def file_stale(path, max_age, now):
    try: d = json.load(open(path)); t = float(d.get("t") or d.get("ts") or os.path.getmtime(path))
    except Exception:
        try: t = os.path.getmtime(path)
        except OSError: return True
    return now - t > max_age

def http_ok(url, timeout=4):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r: return r.status == 200
    except Exception: return False

def docker_state():
    r = _run(["docker", "inspect", "-f", "{{.State.Running}} {{.RestartCount}}", CONTAINER])
    try: run, rc = r.stdout.split(); return run == "true", int(rc)
    except Exception: return False, -1

def bringup_alive():
    return _run(["docker", "exec", CONTAINER, "pgrep", "-f", "ros2 launch bringup"], 8).returncode == 0

def uptime_s():
    try: return float(open("/proc/uptime").read().split()[0])
    except Exception: return 1e9


# ------------------------------------------------------------------ actions
_last_say = {}
def act(a, now):
    if a[0] == "alert":
        log("ALERT:", a[1])
        if now - _last_say.get(a[1], 0) >= SAY_EVERY_S:
            _last_say[a[1]] = now
            if DRY: log("DRY would say:", a[1]); return
            try:
                req = urllib.request.Request(SAY_URL, data=json.dumps({"text": "Watchdog: " + a[1].replace(".service", "") + "."}).encode(),
                                             headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=20)
            except Exception as ex: log("say failed:", ex)
    elif a[0] == "zero":
        if DRY: log("DRY would send zero velocity"); return
        try:
            sys.path.insert(0, "/home/pi/grokbot/releases/e48d3d8"); import gb_estop
            gb_estop.zero_now("loopwatch"); log("zero velocity sent")
        except Exception as ex: log("zero failed:", ex)
    elif a[0] in ("restart", "stop"):
        unit = a[1]
        if unit not in NONMOTION: log("REFUSED %s on non-allowlisted unit %s" % (a[0], unit)); return
        if DRY: log("DRY would systemctl", a[0], unit); return
        r = _run(["systemctl", a[0], unit], 60); log("systemctl", a[0], unit, "rc", r.returncode, (r.stderr or "")[-160:])

def write_status(st):
    for p in STATUS_FILES:
        try:
            with open(p + ".tmp", "w") as f: json.dump(st, f)
            os.replace(p + ".tmp", p)
        except Exception as ex: log("status write failed", p, ex)


def main():
    log("start dry=%s poll=%.0fs loop=%d restarts/%.0fs boot_grace=%.0fs nonmotion=%s safety(alert-only)=%s" % (
        DRY, POLL_S, LOOP_N, LOOP_W, BOOT_GRACE_S, sorted(NONMOTION), sorted(SAFETY)))
    hist = {u: deque(maxlen=200) for u in list(NONMOTION) + list(SAFETY) + ["docker:" + CONTAINER]}
    bad = {u: 0 for u in hist}; attempts = {u: [] for u in NONMOTION}; alerts = deque(maxlen=50); k = 0
    while True:
        now = time.time(); up = uptime_s(); k += 1
        sd = systemd_state(list(NONMOTION) + list(SAFETY)); units = {}
        for u in list(NONMOTION) + list(SAFETY):
            s = sd.get(u, {}); active = s.get("ActiveState", "unknown")
            try: hist[u].append((now, int(s.get("NRestarts", "0"))))
            except ValueError: pass
            probe = NONMOTION.get(u) if u in NONMOTION else SAFETY.get(u)
            badnow = False
            if active == "active" and probe and up >= BOOT_GRACE_S:
                badnow = (not http_ok(probe[1])) if probe[0] == "http" else file_stale(probe[1], probe[2], now)
            bad[u] = bad[u] + 1 if badnow else 0
            loop = is_loop(hist[u], now, up); stuck = bad[u] >= STUCK_POLLS
            cls = "nonmotion" if u in NONMOTION else "safety"
            acts = decide(u, cls, active, loop, stuck, attempts.get(u, []), now)
            for a in acts:
                act(a, now)
                if a[0] == "restart": attempts[u].append(now); bad[u] = 0
                if a[0] == "stop": hist[u].clear()
                if a[0] == "alert": alerts.append({"t": now, "msg": a[1]})
            units[u] = {"active": active, "restarts_window": restarts_in_window(hist[u], now), "loop": loop,
                        "stuck_polls": bad[u], "class": cls}
        run, rc = docker_state(); dk = "docker:" + CONTAINER
        if rc >= 0: hist[dk].append((now, rc))
        dloop = is_loop(hist[dk], now, up, n=2)
        brk = bringup_alive() if (k % 2 == 1 and run) else None
        for msg in (["container %s is not running" % CONTAINER] if not run else []) + \
                   (["container %s is restarting in a loop" % CONTAINER] if dloop else []) + \
                   (["ROS bringup is not running in the container"] if brk is False and up >= BOOT_GRACE_S else []):
            act(("alert", msg), now); act(("zero",), now); alerts.append({"t": now, "msg": msg})
        units[dk] = {"running": run, "restart_count": rc, "loop": dloop, "bringup": brk}
        write_status({"t": now, "dry": DRY, "uptime_s": round(up), "thresholds": {"loop_n": LOOP_N, "loop_w_s": LOOP_W,
                      "boot_grace_s": BOOT_GRACE_S, "stuck_polls": STUCK_POLLS, "poll_s": POLL_S, "restart_cap_per_h": RESTART_CAP},
                      "units": units, "recent_alerts": list(alerts)[-10:]})
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
