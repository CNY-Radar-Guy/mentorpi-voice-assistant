#!/usr/bin/env python3
"""Remote helpers run on Wally via SSH stdin: python3 - <op> [args...].
Read-only and safety-preserving. Never creates, clears, or tests ESTOP latches.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ARMED_PATH = Path("/home/pi/docker/tmp/gb_safety/ARMED")
ARM_LOG = Path("/home/pi/docker/tmp/gb_safety/arm.log")
HOST_ESTOP = Path("/home/pi/grokbot/data/ESTOP")
HOST_MAINT = Path("/home/pi/grokbot/data/MAINT")
BOX_ESTOP = Path("/home/pi/docker/tmp/gb_safety/ESTOP")
MODE_PATH = Path("/home/pi/grokbot/data/learn/mode.json")
BENCH_GUARD_OFF = Path("/home/pi/grokbot/data/BENCH_GUARD_OFF")
BENCH_GUARD_BACKUP_DIR = Path("/home/pi/grokbot/data/backups")
MAPVIEW_STATE_URL = "http://127.0.0.1:8095/api/state"
BATTERY_PATH = Path("/home/pi/grokbot/data/battery.json")
HEALTH_PATH = Path("/home/pi/docker/tmp/gb_safety/health.json")
SENSORS_PATH = Path("/home/pi/docker/tmp/gb_safety/sensors.json")
RELEASE_HINTS = [
    Path("/home/pi/grokbot/releases/e48d3d8"),
    Path("/home/pi/grokbot"),
]
UNITS = [
    "grokbot",
    "gb-gate",
    "gb-guard",
    "gb-health",
    "gb-armguard",
    "gb-watchdog",
    "gb-mapview",
    "gb-loopwatch",
    "gb-rgbguard",
    "grokbot-lowv",
    "wally-session-logger",
]


def out(obj, code=0):
    print(json.dumps(obj, default=str))
    raise SystemExit(code)


def read_text(p: Path, n=8000):
    try:
        return p.read_text(errors="replace")[:n]
    except Exception:
        return None


def read_json(p: Path):
    t = read_text(p)
    if not t:
        return None
    try:
        return json.loads(t)
    except Exception:
        return {"raw": t[:500]}


def tail_file(p: Path, n=12):
    t = read_text(p, 250000)
    if not t:
        return []
    return t.splitlines()[-n:]


def is_active(unit: str) -> str:
    try:
        r = subprocess.run(
            ["systemctl", "is-active", unit],
            capture_output=True,
            text=True,
            timeout=3,
        )
        return (r.stdout or r.stderr or "").strip()
    except Exception as e:
        return "error: %s" % e


def journal(unit: str, n=10):
    try:
        r = subprocess.run(
            ["journalctl", "-u", unit, "-n", str(n), "--no-pager", "-o", "short-iso"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return [ln for ln in (r.stdout or "").splitlines() if ln.strip()][-n:]
    except Exception:
        return []


def http_json(url, method="GET", body=None, timeout=8, headers=None):
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    hdrs = {"Content-Type": "application/json"} if body is not None else {}
    if headers:
        hdrs.update(headers)
    req = urllib.request.Request(url, data=data, method=method, headers=hdrs)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode(errors="replace")
            try:
                parsed = json.loads(raw) if raw else None
            except Exception:
                parsed = {"raw": raw[:800]}
            return {"ok": 200 <= resp.status < 300, "status": resp.status, "body": parsed}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def port_open(port: int) -> bool:
    s = socket.socket()
    s.settimeout(1.0)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except Exception:
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass


def is_live_explore_cmd(cmd):
    """True only for a Python process whose argv includes the gb_explore.py script.

    grep/sed/zsh/journalctl (or python -c) that merely mention the filename must not match.
    """
    if not cmd:
        return False
    args = [a for a in str(cmd).replace("\0", " ").split() if a]
    if not args:
        return False
    exe = args[0].rstrip("/").split("/")[-1].lower()
    if not (exe == "python" or exe.startswith("python3") or exe.startswith("python2")):
        return False
    if "-c" in args:
        return False
    for a in args[1:]:
        base = a.rstrip("/").split("/")[-1]
        if base == "gb_explore.py":
            return True
    return False


def explore_pids():
    found = []
    try:
        for name in os.listdir("/proc"):
            if not name.isdigit():
                continue
            try:
                cmd = Path("/proc/%s/cmdline" % name).read_bytes().replace(b"\0", b" ").decode(errors="replace")
            except Exception:
                continue
            if is_live_explore_cmd(cmd):
                found.append({"pid": int(name), "cmd": cmd.strip()[:200]})
    except Exception:
        pass
    return found


def estop_present() -> bool:
    return HOST_ESTOP.exists() or BOX_ESTOP.exists()


def append_arm_log(line: str):
    try:
        ARM_LOG.parent.mkdir(parents=True, exist_ok=True)
        with ARM_LOG.open("a", encoding="utf-8") as f:
            f.write("%s %s\n" % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), line))
    except Exception:
        pass


def release_dirs():
    dirs = []
    for p in RELEASE_HINTS:
        if p.exists():
            dirs.append(p)
    rel = Path("/home/pi/grokbot/releases")
    if rel.exists():
        for child in sorted(rel.iterdir(), key=lambda c: c.stat().st_mtime, reverse=True):
            if child.is_dir() and child not in dirs:
                dirs.append(child)
    return dirs


def gate_stop():
    """Ask the motion gate for a normal stop. Does not set or clear ESTOP."""
    last_err = None
    for d in release_dirs():
        if not (d / "gb_estop.py").exists() and not (d / "gb_estop.py").is_file():
            # still try import if path has the module
            pass
        sys.path.insert(0, str(d))
        try:
            import gb_estop  # type: ignore

            if hasattr(gb_estop, "gate_request"):
                return {"ok": True, "via": str(d), "result": gb_estop.gate_request({"op": "stop", "src": "jarvis"})}
            last_err = "gb_estop.gate_request missing in %s" % d
        except Exception as e:
            last_err = str(e)
        finally:
            if sys.path and sys.path[0] == str(d):
                sys.path.pop(0)
    return {"ok": False, "error": last_err or "gate stop helper not found"}


def stop_explore():
    stopped = []
    for item in explore_pids():
        pid = item["pid"]
        if pid == os.getpid() or pid == os.getppid():
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            stopped.append(item)
        except Exception as e:
            item["error"] = str(e)
            stopped.append(item)
    return stopped



def has_flag(args, name):
    for a in args or []:
        if not isinstance(a, str):
            continue
        if a == name:
            return True
        if a.startswith(name + "=") and a.split("=", 1)[1] not in ("0", "false", "False", ""):
            return True
    return False


def positional_args(args):
    skip = {"dry_run"}
    out_args = []
    for a in args or []:
        if not isinstance(a, str):
            continue
        if a in skip or a.startswith("dry_run=") or a.startswith("actor=") or a.startswith("stop_after=") or a.startswith("name="):
            continue
        out_args.append(a)
    return out_args


def flag_value(args, name, default=None):
    for a in args or []:
        if isinstance(a, str) and a.startswith(name + "="):
            return a.split("=", 1)[1]
    return default


def archive_bench_guard_off():
    if not BENCH_GUARD_OFF.exists():
        return {"present": False, "removed": False}
    try:
        BENCH_GUARD_BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        dest = BENCH_GUARD_BACKUP_DIR / ("BENCH_GUARD_OFF.bak-jarvis-%s" % time.strftime("%Y%m%d-%H%M%S"))
        BENCH_GUARD_OFF.replace(dest)
        return {"present": True, "removed": True, "archived": str(dest)}
    except Exception as e:
        return {"present": True, "removed": False, "error": str(e)}


def docker_mentorpi(docker_lines):
    for ln in docker_lines or []:
        name = ln.split("\t")[0].strip() if "\t" in ln else (ln.split()[0] if ln else "")
        if name.lower().replace("_", "") in ("mentorpi",):
            return {"name": name, "running": True, "status": ln}
    return {"name": "MentorPi", "running": False, "status": "not running"}


def mapping_mod():
    last_err = None
    for d in release_dirs():
        if not (d / "gb_mapping.py").exists():
            continue
        sys.path.insert(0, str(d))
        try:
            import gb_mapping  # type: ignore
            return gb_mapping, str(d)
        except Exception as e:
            last_err = str(e)
        finally:
            if sys.path and sys.path[0] == str(d):
                sys.path.pop(0)
    return None, last_err


def mapping_running():
    mod, _err = mapping_mod()
    if mod and hasattr(mod, "running"):
        try:
            return bool(mod.running())
        except Exception:
            return False
    return False


def guarded_arm_via_gb_arm(src, actor):
    """Use the Pi gb_arm helper so provenance and latches are honoured."""
    last_err = None
    for d in release_dirs():
        sys.path.insert(0, str(d))
        try:
            import gb_arm  # type: ignore
            actor_n = gb_arm.normalize_actor(actor, src) if hasattr(gb_arm, "normalize_actor") else actor
            ok = bool(gb_arm.arm(src, actor=actor_n))
            st = gb_arm.state() if hasattr(gb_arm, "state") else {"armed": gb_arm.is_armed()}
            armed = bool(st.get("armed") if isinstance(st, dict) else gb_arm.is_armed())
            return {
                "ok": ok,
                "armed": armed,
                "actor": actor_n,
                "src": src,
                "via": str(d),
                "state": st if isinstance(st, dict) else None,
                "error": None if ok else "gb_arm.arm refused (ESTOP/MAINT latch or write failed)",
            }
        except Exception as e:
            last_err = str(e)
        finally:
            if sys.path and sys.path[0] == str(d):
                sys.path.pop(0)
    return {"ok": False, "armed": ARMED_PATH.exists(), "error": last_err or "gb_arm not found"}


def cmd_status(_args):
    try:
        docker = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}\t{{.Status}}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        docker_lines = [ln for ln in (docker.stdout or "").splitlines() if ln.strip()]
    except Exception as e:
        docker_lines = ["error: %s" % e]
    armed_info = read_json(ARMED_PATH)
    if armed_info is None and ARMED_PATH.exists():
        armed_info = read_text(ARMED_PATH)
    payload = {
        "ok": True,
        "host": socket.gethostname(),
        "checked_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "battery": read_json(BATTERY_PATH),
        "mode": read_json(MODE_PATH),
        "armed": ARMED_PATH.exists(),
        "armed_info": armed_info,
        "estop_latched": estop_present(),
        "estop_host": HOST_ESTOP.exists(),
        "estop_box": BOX_ESTOP.exists(),
        "maint": HOST_MAINT.exists(),
        "voice_port_open": port_open(8000),
        "voice_health": http_json("http://127.0.0.1:8000/health", timeout=4),
        "services": {u: is_active(u) for u in UNITS},
        "docker": docker_lines,
        "explore_running": bool(explore_pids()),
        "explore_pids": explore_pids(),
        "health": read_json(HEALTH_PATH),
        "sensors": read_json(SENSORS_PATH),
        "recent_arm_log": tail_file(ARM_LOG, 8),
        "recent_grokbot_log": journal("grokbot", 10),
        "recent_gate_log": tail_file(Path("/home/pi/docker/tmp/gb_safety/gate.log"), 6),
    }

    mapview = http_json(MAPVIEW_STATE_URL, timeout=3)
    mv = (mapview or {}).get("body") if isinstance((mapview or {}).get("body"), dict) else {}
    health = payload.get("health") if isinstance(payload.get("health"), dict) else {}
    sensors = payload.get("sensors") if isinstance(payload.get("sensors"), dict) else {}
    sens = sensors.get("sensors") if isinstance(sensors.get("sensors"), dict) else {}
    lidar_s = sens.get("lidar") if isinstance(sens.get("lidar"), dict) else {}
    depth_s = sens.get("depth") if isinstance(sens.get("depth"), dict) else {}
    odom_s = sens.get("odom") if isinstance(sens.get("odom"), dict) else {}
    fresh = sens.get("fresh") if isinstance(sens.get("fresh"), dict) else {}
    mentor = docker_mentorpi(docker_lines)
    front = mv.get("front_m")
    if front is None:
        front = sensors.get("front") if sensors.get("front") is not None else lidar_s.get("fwd")
    payload["bench_guard_off"] = bool(BENCH_GUARD_OFF.exists() or sensors.get("bench_guard_off"))
    payload["lidar_points"] = mv.get("lidar_points")
    payload["front_m"] = front
    payload["pose"] = mv.get("pose")
    payload["odom"] = odom_s or None
    payload["lidar"] = {
        "points": mv.get("lidar_points"),
        "front_m": front,
        "online": health.get("lidar_online") if "lidar_online" in health else fresh.get("lidar"),
        "fresh": fresh.get("lidar"),
        "topic": health.get("lidar_topic"),
    }
    payload["depth"] = {
        "online": health.get("depth_online") if "depth_online" in health else fresh.get("depth"),
        "topic_online": health.get("depth_topic_online"),
        "metric_valid": health.get("depth_metric_valid"),
        "fresh": fresh.get("depth"),
        "fwd": depth_s.get("fwd"),
        "valid": depth_s.get("valid"),
    }
    payload["mapping_running"] = mapping_running()
    payload["mentorpi"] = mentor
    payload["mapview"] = {
        "ok": bool(mapview and mapview.get("ok")),
        "url": "http://192.168.50.125:8095/",
        "map": mv.get("map"),
        "rosbridge": mv.get("rosbridge"),
    }
    svcs = payload.get("services") or {}
    payload["core_services"] = {
        "gb-gate": svcs.get("gb-gate"),
        "gb-watchdog": svcs.get("gb-watchdog"),
        "grokbot": svcs.get("grokbot"),
        "gb-mapview": svcs.get("gb-mapview"),
        "MentorPi": "active" if mentor.get("running") else "inactive",
    }
    out(payload)


def parse_actor(args, default="remote_script"):
    for a in args or []:
        if isinstance(a, str) and a.startswith("actor="):
            val = a.split("=", 1)[1].strip()
            if val:
                return val
    return default


def guarded_disarm_via_gb_arm(why, actor):
    """Use the Pi gb_arm helper so mission leases are honoured. Never blindly unlinks ARMED."""
    last_err = None
    for d in release_dirs():
        sys.path.insert(0, str(d))
        try:
            import gb_arm  # type: ignore

            rec = gb_arm.disarm_result(why, actor=actor)
            rec = dict(rec or {})
            rec["via"] = str(d)
            rec["ok"] = bool(rec.get("ok", not rec.get("blocked")))
            return rec
        except Exception as e:
            last_err = str(e)
        finally:
            if sys.path and sys.path[0] == str(d):
                sys.path.pop(0)
    return {"ok": False, "blocked": False, "error": last_err or "gb_arm not found", "removed": False}


def cmd_stop(args=None):
    actor = parse_actor(args, "remote_script")
    was_armed = ARMED_PATH.exists()
    info = read_text(ARMED_PATH, 500) if was_armed else None
    rec = guarded_disarm_via_gb_arm("remote stop (%s)" % actor, actor)
    blocked = bool(rec.get("blocked"))
    explore = [] if blocked else stop_explore()
    gate = gate_stop()
    if blocked:
        append_arm_log("remote stop BLOCKED by mission lease actor=%s prev=%s rec=%s" % (actor, info, rec))
    elif rec.get("removed"):
        append_arm_log("remote stop: disarmed actor=%s prev=%s" % (actor, info or ""))
    out(
        {
            "ok": True,
            "stopped_explore": explore,
            "gate_stop": gate,
            "was_armed": was_armed,
            "disarmed": (not blocked) and (not ARMED_PATH.exists()),
            "disarm_blocked": blocked,
            "disarm_actor": actor,
            "disarm": rec,
            "disarm_error": rec.get("error"),
            "estop_touched": False,
            "note": "Normal stop only. Mission lease blocks remote_script cleanup from disarming a live voice-owned mission. ESTOP files were not created, cleared, or tested.",
        }
    )

def cmd_disarm(args=None):
    actor = parse_actor(args, "remote_script")
    was_armed = ARMED_PATH.exists()
    info = read_text(ARMED_PATH, 500) if was_armed else None
    rec = guarded_disarm_via_gb_arm("remote disarm (%s)" % actor, actor)
    if rec.get("blocked"):
        append_arm_log("remote disarm BLOCKED by mission lease actor=%s prev=%s rec=%s" % (actor, info, rec))
        out({
            "ok": True,
            "was_armed": was_armed,
            "disarmed": False,
            "disarm_blocked": True,
            "disarm_actor": actor,
            "previous": info,
            "disarm": rec,
            "note": "Mission lease blocked this non-emergency remote disarm; Wall-E stays armed.",
        })
    if rec.get("error") and not rec.get("removed") and was_armed and ARMED_PATH.exists():
        out({"ok": False, "error": "Could not disarm: %s" % rec.get("error"), "was_armed": True, "disarm": rec}, 1)
    if rec.get("removed"):
        append_arm_log("remote disarm: removed ARMED actor=%s prev=%s" % (actor, info or ""))
    out({"ok": True, "was_armed": was_armed, "disarmed": not ARMED_PATH.exists(), "disarm_blocked": False,
         "disarm_actor": actor, "previous": info, "disarm": rec})


def cmd_arm(args=None):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    if estop_present():
        out(
            {
                "ok": False,
                "error": "Wally has an emergency-stop latch. Jarvis will not arm him and will not clear that latch.",
                "estop_host": HOST_ESTOP.exists(),
                "estop_box": BOX_ESTOP.exists(),
                "actor": actor,
            },
            1,
        )
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "arm",
            "actor": actor,
            "src": "jarvis-ui: John asked to arm",
            "currently_armed": ARMED_PATH.exists(),
            "note": "Dry run only. ARMED was not written and Wall-E was not armed.",
        })
    rec = guarded_arm_via_gb_arm("jarvis-ui: John asked to arm", actor)
    if rec.get("ok"):
        append_arm_log("jarvis arm: actor=%s via=%s" % (actor, rec.get("via")))
        out({"ok": True, "armed": True, "actor": actor, "armed_info": {"src": rec.get("src"), "actor": rec.get("actor")}, "arm": rec})
    out({"ok": False, "armed": ARMED_PATH.exists(), "actor": actor, "arm": rec, "error": rec.get("error") or "Arm refused"}, 1)


def cmd_mode(args):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    pos = positional_args(args)
    wanted = (pos[0] if pos else "get").strip().lower()
    current = read_json(MODE_PATH) or {}
    if wanted in ("", "get", "status"):
        out({"ok": True, "mode": current, "path": str(MODE_PATH), "bench_guard_off": BENCH_GUARD_OFF.exists(), "actor": actor})
    if wanted not in ("bench", "ground"):
        out({"ok": False, "error": "Mode must be bench or ground, or get."}, 1)
    new = dict(current) if isinstance(current, dict) else {}
    new["previous_mode"] = current.get("mode") if isinstance(current, dict) else None
    new["mode"] = wanted
    new["set_by"] = "Jarvis, on John's request"
    new["updated_by"] = "jarvis"
    new["since"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    new["updated_at"] = new["since"]
    new["note"] = "Mode set through Jarvis control_wally tool"
    new["ground_requires"] = "John's explicit word only"
    new["actor"] = actor
    guard_plan = "archive" if wanted == "ground" and BENCH_GUARD_OFF.exists() else ("leave_absent" if not BENCH_GUARD_OFF.exists() else "leave_present")
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "set_mode",
            "mode": new,
            "previous": current,
            "bench_guard_off_action": guard_plan,
            "note": "Dry run only. mode.json and BENCH_GUARD_OFF were not changed.",
        })
    try:
        MODE_PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp = MODE_PATH.with_suffix(".json.tmp-jarvis")
        tmp.write_text(json.dumps(new, indent=2) + chr(10))
        tmp.replace(MODE_PATH)
    except Exception as e:
        out({"ok": False, "error": "Could not write mode.json: %s" % e}, 1)
    guard = {"present": False, "removed": False}
    if wanted == "ground":
        guard = archive_bench_guard_off()
    out({
        "ok": True,
        "mode": new,
        "previous": current,
        "bench_guard_off": BENCH_GUARD_OFF.exists(),
        "bench_guard": guard,
        "actor": actor,
    })


def cmd_say(args):
    text = " ".join(args).strip()
    if not text:
        out({"ok": False, "error": "Missing text to say."}, 1)
    r = http_json("http://127.0.0.1:8000/internal/say", method="POST", body={"text": text[:500]}, timeout=20)
    out({"ok": bool(r.get("ok")), "voice": r, "text": text[:500]}, 0 if r.get("ok") else 1)


def cmd_ask(args):
    # args: text..., optionally preceded by flags robot=1 dry_run=1
    robot = True
    dry_run = True
    parts = []
    for a in args:
        if a.startswith("robot="):
            robot = a.split("=", 1)[1] not in ("0", "false", "False")
        elif a.startswith("dry_run="):
            dry_run = a.split("=", 1)[1] not in ("0", "false", "False")
        else:
            parts.append(a)
    text = " ".join(parts).strip()
    if not text:
        out({"ok": False, "error": "Missing text to ask."}, 1)
    # Always keep dry_run true here unless explicitly disabled by the node wrapper.
    r = http_json(
        "http://127.0.0.1:8000/api/chat",
        method="POST",
        body={"text": text[:2000], "robot": robot, "dry_run": dry_run},
        timeout=90,
    )
    out({"ok": bool(r.get("ok")), "chat": r, "text": text[:2000], "robot": robot, "dry_run": dry_run}, 0 if r.get("ok") else 1)



VOICE_CHAT_URL = "http://127.0.0.1:8000/api/chat"
DEFAULT_EXPLORE_PHRASE = "explore the room"


def build_explore_launch(args=None):
    """Return the Jarvis explore launch plan. Never a bare `python3 gb_explore.py` help run.

    Starting the mission through grokbot's /api/chat keeps lidar_alive(), AUTO_EXPLORE,
    ALLOW_MOTION, cap_s/stall_s/return/save_map, and the mission lease inside the
    long-lived grokbot process — the same path voice already uses. A standalone
    `python3 gb_explore.py` only prints help; `python3 gb_explore.py --run ...`
    can false-negative on lidar_alive() when rosbridge is not already held by grokbot.
    """
    args = list(args or [])
    seconds = None
    pos = positional_args(args)
    if pos:
        try:
            seconds = float(pos[0])
        except Exception:
            seconds = None
    if seconds is not None and seconds > 0:
        phrase = "explore the room for %g seconds" % seconds
    else:
        phrase = DEFAULT_EXPLORE_PHRASE
        seconds = None
    dry = has_flag(args, "dry_run")
    actor = parse_actor(args, "jarvis")
    body = {"text": phrase, "robot": (not dry), "dry_run": bool(dry)}
    return {
        "launch_path": "grokbot_api_chat",
        "method": "POST",
        "url": VOICE_CHAT_URL,
        "body": body,
        "phrase": phrase,
        "seconds": seconds,
        "dry_run": bool(dry),
        "actor": actor,
        "argv": ["POST", VOICE_CHAT_URL, phrase],
        "cli": None,
        "uses_run_flags": True,
    }


def find_explore():
    for d in release_dirs():
        p = d / "gb_explore.py"
        if p.exists():
            return p
    return None


def cmd_explore(args):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    launch = build_explore_launch(args)
    if estop_present():
        out({"ok": False, "error": "Wally has an emergency-stop latch. Jarvis will not start exploring and will not clear that latch."}, 1)
    if not ARMED_PATH.exists():
        out({"ok": False, "error": "Wally is not armed. Ask John to arm him or include a wake before starting motion."}, 1)
    already = explore_pids()
    if already:
        out({"ok": True, "already_running": True, "explore_pids": already, "note": "Explore appears to be running already."})
    script = find_explore()
    seconds = launch["seconds"]
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "explore_start",
            "launch_path": launch["launch_path"],
            "url": launch["url"],
            "phrase": launch["phrase"],
            "body": launch["body"],
            "script": str(script) if script else None,
            "seconds": seconds,
            "actor": actor,
            "armed": True,
            "note": "Dry run only. Exploration was not started and Wall-E was not moved.",
        })
    if not port_open(8000):
        out({"ok": False, "error": "Wally's voice service is not listening on port 8000, so Jarvis cannot start explore through grokbot."}, 1)
    r = http_json(launch["url"], method="POST", body=launch["body"], timeout=90)
    body = r.get("body") if isinstance(r, dict) else None
    err = None
    if isinstance(r, dict) and r.get("error"):
        err = r.get("error")
    if isinstance(body, dict) and body.get("error"):
        err = body.get("error")
    ok = bool(isinstance(r, dict) and r.get("ok")) and not err
    try:
        append_arm_log("jarvis explore: api/chat phrase=%r dry=%s ok=%s actor=%s" % (launch["phrase"], dry, ok, actor))
    except Exception:
        pass
    out({
        "ok": ok,
        "started": bool(ok and not dry),
        "launch_path": launch["launch_path"],
        "phrase": launch["phrase"],
        "seconds": seconds,
        "armed": True,
        "actor": actor,
        "chat": body,
        "error": None if ok else (err or "explore start via grokbot /api/chat failed"),
    }, 0 if ok else 1)


def cmd_move(args):
    if estop_present():
        out({"ok": False, "error": "Wally has an emergency-stop latch. Jarvis will not move him and will not clear that latch."}, 1)
    if not ARMED_PATH.exists():
        out({"ok": False, "error": "Wally is not armed. Ask John to arm him or include a wake before starting motion."}, 1)
    if explore_pids():
        out({"ok": False, "error": "Wally is currently running an exploration mission. Stop it explicitly before sending a separate move."}, 1)
    direction = (args[0] if args else "forward").strip().lower()
    try:
        seconds = float(args[1]) if len(args) > 1 else 0.8
    except Exception:
        seconds = 0.8
    seconds = max(0.2, min(seconds, 2.0))
    # Conservative speeds well under Wally's usual caps (cruise ~0.08, turn 0.30-0.48).
    lin, ang = 0.0, 0.0
    if direction in ("forward", "fwd", "go"):
        lin = 0.06
    elif direction in ("back", "backward", "reverse"):
        lin = -0.06
    elif direction in ("left",):
        ang = 0.25
    elif direction in ("right",):
        ang = -0.25
    else:
        out({"ok": False, "error": "Direction must be forward, back, left, or right."}, 1)
    last_err = None
    for d in release_dirs():
        sys.path.insert(0, str(d))
        try:
            import gb_estop  # type: ignore

            if hasattr(gb_estop, "gate_request"):
                result = gb_estop.gate_request(
                    {"op": "timed", "lin": lin, "ang": ang, "dur": seconds, "src": "jarvis", "id": "jarvis-move"}
                )
                out({"ok": True, "direction": direction, "lin": lin, "ang": ang, "seconds": seconds, "result": result, "via": str(d)})
            last_err = "gb_estop.gate_request missing in %s" % d
        except Exception as e:
            last_err = str(e)
        finally:
            if sys.path and sys.path[0] == str(d):
                sys.path.pop(0)
    out({"ok": False, "error": last_err or "No gate request helper found for a timed move."}, 1)



def cmd_explore_stop(args=None):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    running = explore_pids()
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "explore_stop",
            "explore_pids": running,
            "actor": actor,
            "note": "Dry run only. Would SIGTERM explore processes and request a normal gate stop. Would not disarm.",
        })
    stopped = stop_explore()
    gate = gate_stop()
    out({
        "ok": True,
        "stopped_explore": stopped,
        "was_running": bool(running),
        "explore_running": bool(explore_pids()),
        "gate_stop": gate,
        "disarmed": False,
        "armed": ARMED_PATH.exists(),
        "actor": actor,
        "note": "Stopped exploration only. Armed state was left unchanged.",
    })


def cmd_map_status(args=None):
    mod, err = mapping_mod()
    running = False
    via = None
    if mod:
        via = err if False else mapping_mod()[1]
        try:
            running = bool(mod.running()) if hasattr(mod, "running") else False
        except Exception as e:
            err = str(e)
    else:
        via = None
    out({
        "ok": True,
        "mapping_running": running,
        "via": via,
        "error": None if mod else err,
        "explore_running": bool(explore_pids()),
    })


def cmd_map_start(args=None):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    if estop_present():
        out({"ok": False, "error": "Wally has an emergency-stop latch. Jarvis will not start mapping and will not clear that latch."}, 1)
    mod, err = mapping_mod()
    if not mod:
        out({"ok": False, "error": "Could not load gb_mapping.py: %s" % (err or "not found")}, 1)
    already = False
    try:
        already = bool(mod.running())
    except Exception:
        already = False
    if already:
        out({"ok": True, "already_running": True, "mapping_running": True, "via": err if False else mapping_mod()[1], "note": "Mapping (slam_toolbox) already appears to be running."})
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "map_start",
            "via": mapping_mod()[1],
            "actor": actor,
            "note": "Dry run only. slam_toolbox was not started and Wall-E was not moved.",
        })
    try:
        result = mod.start()
    except Exception as e:
        out({"ok": False, "error": "Failed to start mapping: %s" % e}, 1)
    if isinstance(result, tuple):
        ok, msg = result[0], result[1] if len(result) > 1 else ""
    else:
        ok, msg = bool(result), str(result)
    out({"ok": bool(ok), "started": bool(ok), "message": msg, "mapping_running": mapping_running(), "actor": actor}, 0 if ok else 1)


def cmd_map_stop(args=None):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    mod, err = mapping_mod()
    if not mod:
        out({"ok": False, "error": "Could not load gb_mapping.py: %s" % (err or "not found")}, 1)
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "map_stop",
            "mapping_running": mapping_running(),
            "actor": actor,
            "note": "Dry run only. Mapping was not stopped.",
        })
    try:
        result = mod.stop()
    except Exception as e:
        out({"ok": False, "error": "Failed to stop mapping: %s" % e}, 1)
    if isinstance(result, tuple):
        ok, msg = result[0], result[1] if len(result) > 1 else ""
    else:
        ok, msg = bool(result), str(result)
    out({"ok": bool(ok), "stopped": bool(ok), "message": msg, "mapping_running": mapping_running(), "actor": actor}, 0 if ok else 1)


def cmd_map_save(args=None):
    actor = parse_actor(args, "jarvis")
    dry = has_flag(args, "dry_run")
    name = flag_value(args, "name")
    pos = positional_args(args)
    if not name and pos:
        name = pos[0]
    stop_after = True
    raw_sa = flag_value(args, "stop_after")
    if raw_sa is not None:
        stop_after = raw_sa not in ("0", "false", "False", "no")
    mod, err = mapping_mod()
    if not mod:
        out({"ok": False, "error": "Could not load gb_mapping.py: %s" % (err or "not found")}, 1)
    if dry:
        out({
            "ok": True,
            "dry_run": True,
            "would": "map_save",
            "name": name,
            "stop_after": stop_after,
            "mapping_running": mapping_running(),
            "actor": actor,
            "note": "Dry run only. No map was saved and mapping was not stopped.",
        })
    try:
        result = mod.save(name=name, stop_after=stop_after)
    except Exception as e:
        out({"ok": False, "error": "Failed to save map: %s" % e}, 1)
    if isinstance(result, tuple):
        ok = result[0]
        msg = result[1] if len(result) > 1 else ""
        info = result[2] if len(result) > 2 else None
    else:
        ok, msg, info = bool(result), str(result), None
    out({"ok": bool(ok), "message": msg, "info": info, "mapping_running": mapping_running(), "actor": actor}, 0 if ok else 1)


def main():
    if len(sys.argv) < 2:
        out({"ok": False, "error": "Missing op. Use status, stop, disarm, arm, mode, say, ask, explore, explore_stop, map_start, map_stop, map_save, map_status, or move."}, 1)
    op = sys.argv[1].strip().lower().replace("-", "_")
    args = sys.argv[2:]
    ops = {
        "status": cmd_status,
        "stop": cmd_stop,
        "disarm": cmd_disarm,
        "arm": cmd_arm,
        "mode": cmd_mode,
        "say": cmd_say,
        "ask": cmd_ask,
        "explore": cmd_explore,
        "explore_stop": cmd_explore_stop,
        "map_start": cmd_map_start,
        "map_stop": cmd_map_stop,
        "map_save": cmd_map_save,
        "map_status": cmd_map_status,
        "move": cmd_move,
    }
    if op not in ops:
        out({"ok": False, "error": "Unknown op %s" % op}, 1)
    ops[op](args)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as e:
        out({"ok": False, "error": "remote.py failed: %s" % e}, 1)

