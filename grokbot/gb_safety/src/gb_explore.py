#!/usr/bin/env python3
"""gb_explore.py - autonomous missions for the MentorPi T1: "map this room and come back", "drive N feet and come back",
"drive around for N minutes and come back".

Two halves in one file:
  * host side (imported by grokbot_voice.py, runs as pi): parse_plan() turns words into a plan, request() checks the
    safety gates (AUTO_EXPLORE=1 AND ALLOW_MOTION=1 AND gb-guard active AND LiDAR alive), starts slam_toolbox through
    gb_mapping (explore missions), optionally starts a speed-capped Nav2 stack, launches the ROS node inside the
    MentorPi container as user ubuntu, supervises it (heartbeats both ways), saves the map, and announces the result.
  * node side (python3 gb_explore.py --node ..., inside the container): rclpy node that drives /controller/cmd_vel.
    Every command passes send(): hard caps 0.096 m/s and 0.64 rad/s, LiDAR front/rear stop, stale scan/odom -> zero.
    Abort file / SIGINT / SIGTERM / stale host heartbeat -> repeated zero Twist. A bash wrapper runs
    "--zero" after the node exits for any reason (even SIGKILL), and the host also sends zero bursts over rosbridge.
Dry run (--dry): identical logic, but nothing is published on /controller/cmd_vel; planned commands are logged.
"""
import json, math, os, re, signal, subprocess, sys, threading, time

import gb_paths
E = os.environ.get
HARD_MAX_LIN, HARD_MAX_ANG = 0.096, 0.64                       # never exceeded, whatever the env says
MAX_LIN = min(HARD_MAX_LIN, abs(float(E("EXPLORE_MAX_LIN", "0.08"))))
MAX_ANG = min(HARD_MAX_ANG, abs(float(E("EXPLORE_MAX_ANG", "0.48"))))
def _speed_mult():
    """Voice speed setting (50-100 %). Container: speed.json written by the host at launch; host: gb_caps."""
    for p in ("/home/ubuntu/shared/grokbot/explore/speed.json", os.path.expanduser("~/grokbot/data/caps.json")):
        try:
            d = json.load(open(p)); v = float(d.get("mult", d.get("speed_pct", 100) / 100.0))
            return max(0.5, min(1.0, v))
        except Exception:
            continue
    return 1.0
SPEED_MULT = _speed_mult()
MAX_LIN = min(HARD_MAX_LIN, max(0.04, MAX_LIN * SPEED_MULT))
MAX_ANG = min(HARD_MAX_ANG, max(0.30, MAX_ANG * SPEED_MULT))   # tracks need ~0.3 rad/s to turn in place
MAX_DIST_M = min(5.0, float(E("EXPLORE_MAX_DIST_M", "5")))
MAX_WANDER_S = min(300.0, float(E("EXPLORE_MAX_WANDER_S", "300")))
EXPLORE_CAP_S = float(E("EXPLORE_CAP_S", "240"))            # default hard time cap for "map this room"
MAX_EXPLORE_S = min(600.0, float(E("EXPLORE_MAX_S", "600")))
STALL_S = float(E("EXPLORE_STALL_S", "30"))
CONTAINER = E("ROS_CONTAINER", "MentorPi")
HOST_SHARED = os.path.expanduser(E("ROS_SHARED_HOST", "/home/pi/docker/tmp"))
CONT_SHARED = E("ROS_SHARED_CONTAINER", "/home/ubuntu/shared")
HOST_DIR = os.path.join(HOST_SHARED, "grokbot", "explore"); CONT_DIR = CONT_SHARED + "/grokbot/explore"
ROSRUN = CONT_SHARED + "/grokbot/rosrun.sh"
CMD_VEL = "/controller/cmd_vel"
GATE_IN = "/gb/cmd_in"                                       # non-zero motion goes through gb_gate (deadman 0.5 s, caps, ESTOP)
ESTOP_CONT = "/home/ubuntu/shared/gb_safety/ESTOP"
ESTOP_HOST = gb_paths.host_estop()

class _GateRoute:
    """Publisher shim: every command -> gate stream; zero commands ALSO straight to the controller (always allowed)."""
    def __init__(self, gate, direct): self.gate, self.direct = gate, direct
    def publish(self, m):
        self.gate.publish(m)
        if m.linear.x == 0 and m.angular.z == 0 and m.linear.y == 0:
            self.direct.publish(m)
NAV_OUT = "/gb_explore/nav_cmd"                              # Nav2 output is remapped here, never to /cmd_vel
FRONT_STOP = float(E("EXPLORE_FRONT_STOP", "0.27"))          # m from LiDAR centre: no forward motion closer than this
REAR_STOP = float(E("EXPLORE_REAR_STOP", "0.20"))
AVOID_DIST = float(E("EXPLORE_AVOID_DIST", "0.45"))          # wander: start turning away below this (was 0.45; gate stops at 0.35)
# gb-guard mission bypass flag (same file seen from host and container). Written by the node on every heartbeat,
# removed by the node's finally and by the host's finally; gb-guard ignores it when stale / pid dead / dry / aborting.
GUARD_FLAG_HOST = os.path.join(HOST_SHARED, "gb_guard", "mission_active")
GUARD_FLAG_CONT = CONT_SHARED + "/gb_guard/mission_active"

def _clear_guard_flag(path):
    try: os.remove(path)
    except OSError: pass

def _now(): return time.strftime("%Y-%m-%d %H:%M:%S")

# ===================================================================================== parsing (pure python)
_UNITS = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9}
_TEENS = {"ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
          "seventeen": 17, "eighteen": 18, "nineteen": 19}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fourty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80,
         "ninety": 90}
_UNIT_WORDS = (r"(?:feet|foot|ft|meters?|metres?|m|yards?|yds?|inch(?:es)?|centimet(?:er|re)s?|cm|minutes?|mins?|"
               r"seconds?|secs?|s|hours?)")
DIST_RX = re.compile(r"(\d+(?:\.\d+)?)\s*(feet|foot|ft|meters?|metres?|m|yards?|yds?|inch(?:es)?|centimet(?:er|re)s?|cm)\b")
TIME_RX = re.compile(r"(\d+(?:\.\d+)?)\s*(minutes?|mins?|seconds?|secs?|s|hours?)\b")
_TO_M = {"f": 0.3048, "m": 1.0, "y": 0.9144, "i": 0.0254, "c": 0.01}

def normalize_numbers(t):
    t = t.lower().replace("-", " ").replace(",", " ")
    t = re.sub(r"(\d)\s*(ft|m|cm|s)\b", r"\1 \2", t)
    t = re.sub(r"\b(" + "|".join(_TENS) + r")\s+(" + "|".join(k for k in _UNITS if k != "zero") + r")\b",
               lambda m: str(_TENS[m.group(1)] + _UNITS[m.group(2)]), t)
    allw = {**_UNITS, **_TEENS, **_TENS}
    t = re.sub(r"\b(" + "|".join(allw) + r")\b", lambda m: str(allw[m.group(1)]), t)
    t = re.sub(r"\b(\d+(?:\.\d+)?) and (?:a )?half\b", lambda m: str(float(m.group(1)) + 0.5), t)
    t = re.sub(r"\b(?:a |1 )?half (?:a |an |of a |1 )?(?=" + _UNIT_WORDS + r"\b)", "0.5 ", t)
    t = re.sub(r"\ba couple(?: of)?\b", "2", t); t = re.sub(r"\ba few\b", "3", t)
    t = re.sub(r"\b(?:a|an)\s+(?=(?:foot|meter|metre|yard|minute|min|second|sec)\b)", "1 ", t)
    t = re.sub(r"\b(\d+) point (\d+)\b", r"\1.\2", t)
    return re.sub(r"\s+", " ", t).strip()

_RET = re.compile(r"\b(come back|coming back|came back|return|returns|returning|go back|head back|get back|drive back|"
                  r"turn around and|back here|back to (?:me|here|the start|your start|start|where)|back where|"
                  r"where you start|where you began|starting (?:point|spot|place|position|pose)|park|parking|"
                  r"original (?:spot|position|place)|come home|go home|head home|get home|back home|home base|"
                  r"come right back|bring yourself back)\b")
_VERB = re.compile(r"\b(drive|driving|go|going|move|moving|roll|rolling|head|travel|roam|roaming|wander|wandering|cruise|"
                   r"cruising|explore|exploring|scout|tour|patrol|run|look around|get around|walk|ride|zoom|trundle)\b")
_MAPV = re.compile(r"\b(map|maps|mapping|mapped|floor ?plan|slam|chart|survey)\b")
_AREA = re.compile(r"\b(room|rooms|around|area|space|place|house|apartment|office|kitchen|garage|surroundings|"
                   r"environment|here|this|everything|floor|lab|yourself|on your own|autonomous(?:ly)?)\b")

def parse_plan(text):
    """Local parser. Returns a plan dict or None (not a mission)."""
    t = (text or "").lower().strip().rstrip(".!?")
    t = re.sub(r"^(?:hey |hi |ok |okay )?(?:grok ?bot|grok|groc|grog|g ?bot|gee bot|robot)[, ]+", "", t).strip()
    t = re.sub(r"^(?:can you|could you|would you|will you|i want you to|i'd like you to|please|go ahead and)\s+", "", t)
    t = re.sub(r"^(?:please)\s+", "", t)
    if re.match(r"^(stop|halt|cancel|abort|quit|end|freeze|don't|do not|never mind)\b", t):
        return None
    if re.match(r"^(save|delete|show|list|where|what|how|load|forget)\b", t):
        return None
    if re.search(r"\b(using|on|with|from|in) (?:the|my|our|a|that|saved) map\b", t):
        return None
    n = normalize_numbers(t)
    n = re.sub(r"\b(?:nap|nab|mop|mapp?ed out|math) (?=(?:the|this|my|our|a) (?:room|area|space|house|office|kitchen|garage|lab)\b)", "map ", n)
    ret = bool(_RET.search(n))
    verb = bool(_VERB.search(n)); mapv = bool(_MAPV.search(n)); expl = bool(re.search(r"\bexplor", n))
    dm = DIST_RX.search(n); tm = TIME_RX.search(n)
    if re.search(r"\b(backwards?|in reverse|reverse|back up)\b", n) and dm:
        return None                                        # only forward distance missions
    no_save = bool(re.search(r"\b(don'?t|do not|no need to|without) sav", n))
    if (mapv and (verb or ret or _AREA.search(n))) or (expl and not dm):
        if re.match(r"^(start|begin) (?:the )?(mapping|slam)\b", n) and not (verb or ret):
            return None                                    # manual mapping intent handles this
        if re.match(r"^(?:make|build|create) a map$", n):
            return None
        cap = EXPLORE_CAP_S
        if tm: cap = _secs(tm)
        return {"mission": "explore", "cap_s": max(20.0, min(MAX_EXPLORE_S, cap)), "requested_s": _secs(tm) if tm else None,
                "return": True, "save_map": not no_save, "stall_s": STALL_S}
    if dm and (verb or ret):
        d = _meters(dm)
        if d <= 0: return None
        return {"mission": "distance", "distance_m": round(min(MAX_DIST_M, d), 3), "requested_m": round(d, 3),
                "requested_text": dm.group(0), "clamped": d > MAX_DIST_M, "return": ret, "park": ret}
    if tm and verb and (ret or re.search(r"\b(around|wander|wandering|roam|roaming|cruise|cruising|about|patrol)\b", n)):
        s = _secs(tm)
        if s <= 0: return None
        return {"mission": "wander", "duration_s": round(min(MAX_WANDER_S, s), 1), "requested_s": s,
                "requested_text": tm.group(0), "clamped": s > MAX_WANDER_S, "return": ret}
    return None

def _meters(m):
    v = float(m.group(1)); u = m.group(2)
    return v * (0.01 if u.startswith("c") else 0.0254 if u.startswith("i") else 0.3048 if u.startswith("f")
                else 0.9144 if u.startswith("y") else 1.0)

def _secs(m):
    v = float(m.group(1)); u = m.group(2)
    return v * (3600 if u.startswith("h") else 60 if u.startswith("m") else 1)

def plan_from_router(d):
    """Plan from a Grok router JSON {"intent":"mission","mission":..,"distance_m":..,"duration_s":..,"return":..}."""
    mis = str(d.get("mission") or "").lower()
    def f(k):
        try: return float(d.get(k))
        except Exception: return None
    ret = bool(d.get("return", mis == "explore"))
    if mis in ("explore", "map", "mapping"):
        s = f("duration_s")
        return {"mission": "explore", "cap_s": max(20.0, min(MAX_EXPLORE_S, s or EXPLORE_CAP_S)), "requested_s": s,
                "return": True, "save_map": d.get("save_map", True) is not False, "stall_s": STALL_S}
    if mis == "distance":
        m = f("distance_m")
        if not m or m <= 0: return None
        return {"mission": "distance", "distance_m": round(min(MAX_DIST_M, m), 3), "requested_m": m,
                "requested_text": f"{m:g} m", "clamped": m > MAX_DIST_M, "return": ret, "park": ret}
    if mis in ("wander", "time", "duration"):
        s = f("duration_s")
        if not s or s <= 0: return None
        return {"mission": "wander", "duration_s": round(min(MAX_WANDER_S, s), 1), "requested_s": s,
                "requested_text": f"{s:g} s", "clamped": s > MAX_WANDER_S, "return": ret}
    return None

def _dur_words(s):
    s = float(s)
    if s >= 60 and abs(s / 60 - round(s / 60)) < 1e-6: return f"{int(round(s / 60))} minute" + ("s" if round(s / 60) != 1 else "")
    if s >= 60: return f"{s / 60:.1f} minutes"
    return f"{int(round(s))} seconds"

def describe_plan(p):
    if p["mission"] == "follow":
        return {"follow": f"follow {p['target']} at about {p['stop_m']:.1f} meters",
                "come": f"find {p['target']} and stop about {p['stop_m']:.0f} meter away",
                "object": f"drive to the {p['target']} and stop short of it"}.get(p.get("kind"), "follow")
    if p["mission"] == "explore":
        return (f"drive around slowly and map this room for up to {_dur_words(p['cap_s'])} (I stop early when the map "
                f"stops growing for {int(p['stall_s'])} seconds), " + ("save the map, " if p["save_map"] else "") +
                "then come back to where I started and stop")
    if p["mission"] == "distance":
        ft = p["distance_m"] / 0.3048
        s = f"drive forward {p['distance_m']:.2f} meters ({ft:.1f} feet)"
        if p.get("clamped"): s += f", capped from {p['requested_text']} to my 5 meter limit"
        return s + (", stop, turn around, come back to the start and park facing the way I started" if p["return"]
                    else ", then stop")
    s = f"drive around slowly, avoiding obstacles, for {_dur_words(p['duration_s'])}"
    if p.get("clamped"): s += f" (capped from {p['requested_text']} to my 5 minute limit)"
    return s + (", then come back to where I started and stop" if p["return"] else ", then stop")

# ===================================================================================== host side (pi)
_S = {"proc": None, "thread": None, "plan": None, "dry": False, "abort": threading.Event(), "phase": "idle",
      "t0": 0.0, "last": None, "nav2": None}
_LOCK = threading.Lock()

def _hlog(*a):
    try:
        import gb_store
        p = os.path.join(gb_store.DATA, "explore.log")
    except Exception:
        p = os.path.expanduser("~/grokbot/data/explore.log")
    line = _now() + " " + " ".join(str(x) for x in a)
    print("[explore]", line, flush=True)
    with open(p, "a") as f: f.write(line + "\n")

def _flag(k): return str(os.environ.get(k, "0")).strip() == "1"

def _guard_active():
    if E("EXPLORE_REQUIRE_GUARD", "1") == "0": return True
    try:
        return subprocess.run(["systemctl", "is-active", "gb-guard"], capture_output=True, text=True,
                              timeout=5).stdout.strip() == "active"
    except Exception:
        return False

def _dx(args, timeout=20, user="ubuntu"):
    return subprocess.run(["docker", "exec", "-u", user, CONTAINER] + args, capture_output=True, text=True, timeout=timeout)

def running():
    th = _S["thread"]; return bool(th and th.is_alive())

def status_text():
    if running():
        return (f"I'm on a {_S['plan']['mission']} mission{' (dry run)' if _S['dry'] else ''}, phase {_S['phase']}, "
                f"{int(time.time() - _S['t0'])} seconds in. Clap twice any time to end it.")
    return _S["last"] or "No mission has run since I started."

def _zero_host():
    try:
        import gb_robot; gb_robot.stop(); return True
    except Exception as ex:
        _hlog("host zero burst failed:", ex); return False

def abort(reason="stop"):
    """Abort a running mission now. Safe to call any time. Returns True if a mission was running."""
    was = running()
    _S["abort"].set()
    try:
        os.makedirs(HOST_DIR, exist_ok=True)
        with open(os.path.join(HOST_DIR, "abort"), "w") as f: f.write(reason)
    except Exception: pass
    if was:
        _hlog("ABORT requested:", reason)
        try:
            pid = open(os.path.join(HOST_DIR, "node.pid")).read().strip()
            if pid: _dx(["kill", "-INT", pid], 5)
        except Exception: pass
        if not _S["dry"]:
            threading.Thread(target=_zero_host, daemon=True).start()
    return was

def request(plan, dry=False, store=None, announce=None, cli_dry=False):
    """Gate + start a mission in the background. Returns the immediate spoken reply."""
    desc = describe_plan(plan)
    _hlog(f"request dry={dry} plan={json.dumps(plan)}")
    if os.path.exists(ESTOP_HOST):
        _hlog("refused: ESTOP latched"); return "I'm emergency-latched, so I won't start a mission. Say resume first."
    if running():
        return "I'm already on a mission. Tell me to end it first."
    if cli_dry and not dry:
        raise ValueError("cli_dry requires dry=True")
    if not _flag("AUTO_EXPLORE") and not cli_dry:
        return (f"Auto explore is switched off, so I won't move. I understood: {desc}. "
                "To allow it, set AUTO_EXPLORE=1 and ALLOW_MOTION=1 in my settings.")
    if not dry and not _flag("ALLOW_MOTION"):
        return f"Driving is switched off (ALLOW_MOTION=0), so I won't move. I understood: {desc}."
    try:
        import gb_caps; blk = gb_caps.motion_block("mission")
    except Exception as ex:
        blk = f"I couldn't read my sensor settings ({ex}), so I won't drive on my own."
    if blk:
        return blk
    if not dry and not _guard_active():
        return "My obstacle guard service isn't running, so I won't drive on my own. Start gb-guard first."
    try:
        import gb_features
        if not gb_features.lidar_alive():
            return "My LiDAR isn't publishing scans, so I can't do that safely right now."
    except Exception as ex:
        _hlog("lidar check failed:", ex)
    _S["abort"].clear(); _S.pop("abort_t", None)
    _S.update(plan=plan, dry=dry, phase="starting", t0=time.time())
    th = threading.Thread(target=_run, args=(plan, dry, store, announce), daemon=True, name="gb_explore")
    _S["thread"] = th; th.start()
    return ("Dry run, I won't actually move. " if dry else "Okay. ") + f"I'll {desc}. Clap twice any time to end it."

def _write_launch_files():
    os.makedirs(HOST_DIR, exist_ok=True)
    me = os.path.abspath(__file__)
    dst = os.path.join(HOST_DIR, "gb_explore.py")
    src = open(me).read()
    if not os.path.exists(dst) or open(dst).read() != src:
        with open(dst + ".tmp", "w") as f: f.write(src)
        os.replace(dst + ".tmp", dst)
    import hashlib
    h1 = hashlib.md5(src.encode()).hexdigest()[:10]; h2 = hashlib.md5(open(dst).read().encode()).hexdigest()[:10]
    if h1 != h2: raise RuntimeError("container copy of gb_explore.py is stale")
    _hlog(f"container copy refreshed md5={h2}")
    try:
        import gb_caps; m = gb_caps.speed_mult()
    except Exception:
        m = 1.0
    with open(os.path.join(HOST_DIR, "speed.json"), "w") as f: json.dump({"mult": m}, f)
    _hlog(f"speed setting {int(m * 100)}% for this mission")
    with open(os.path.join(HOST_DIR, "gb_nav2.launch.py"), "w") as f: f.write(NAV2_LAUNCH)

def nav2_start(timeout=60):
    """Start the speed-capped Nav2 (no map_server/amcl: slam_toolbox supplies /map and map->odom). True if active."""
    _write_launch_files()
    r = _dx(["bash", "-lc", f"{ROSRUN} python3 {CONT_DIR}/gb_explore.py --prep-nav2"], 40)
    if r.returncode != 0:
        _hlog("nav2 prep failed:", (r.stdout + r.stderr)[-300:]); return False
    logf = open(os.path.join(os.path.dirname(_logpath()), "explore_nav2.log"), "ab")
    subprocess.Popen(["docker", "exec", "-u", "ubuntu", CONTAINER, "bash", "-lc",
                      f"echo $$ > {CONT_DIR}/nav2.pid; exec {ROSRUN} ros2 launch {CONT_DIR}/gb_nav2.launch.py"],
                     stdout=logf, stderr=logf, stdin=subprocess.DEVNULL, start_new_session=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(2)
        try: r = _dx(["bash", "-lc", f"{ROSRUN} ros2 lifecycle get /bt_navigator"], 15)
        except subprocess.TimeoutExpired:
            _hlog("nav2 lifecycle query timed out (CPU busy?) - retrying"); continue
        if "active" in r.stdout and "inactive" not in r.stdout:
            pubs = _dx(["bash", "-lc", f"{ROSRUN} ros2 topic info -v /cmd_vel"], 15).stdout
            nav_pub = re.findall(r"Node name: (\S+)\s*\nNode namespace: \S+\s*\nTopic type: \S+\s*\nEndpoint type: PUBLISHER", pubs)
            if nav_pub:
                _hlog("SAFETY: something publishes /cmd_vel after nav2 start:", nav_pub, "- stopping nav2")
                nav2_stop(); return False
            _hlog(f"nav2 active after {time.time() - t0:.0f}s")
            return True
    _hlog("nav2 did not become active in time"); nav2_stop(); return False

def nav2_running():
    return _dx(["pgrep", "-f", "gb_nav2_container"], 5).returncode == 0

def nav2_stop():
    try:
        pid = open(os.path.join(HOST_DIR, "nav2.pid")).read().strip()
        if pid: _dx(["kill", "-INT", pid], 5)
    except Exception: pass
    for _ in range(16):
        time.sleep(0.5)
        if not nav2_running(): break
    else:
        _dx(["pkill", "-TERM", "-f", "gb_nav2_container"], 5); time.sleep(1)
        _dx(["pkill", "-KILL", "-f", "gb_nav2_container"], 5)
    _dx(["pkill", "-INT", "-f", "gb_nav2.launch.py"], 5)
    ok = not nav2_running(); _hlog("nav2 stopped ok=", ok); return ok

def _logpath():
    try:
        import gb_store; return os.path.join(gb_store.DATA, "explore.log")
    except Exception:
        return os.path.expanduser("~/grokbot/data/explore.log")

def _run(plan, dry, store, announce):
    report = None; started_slam = False; nav2 = False; res = {}
    try:
        _write_launch_files()
        for fn in ("abort", "result.json", "hb", "node.pid", "host_hb"):
            try: os.remove(os.path.join(HOST_DIR, fn))
            except OSError: pass
        _clear_guard_flag(GUARD_FLAG_HOST)
        if plan["mission"] == "explore":
            import gb_mapping
            _S["phase"] = "start_slam"
            if gb_mapping.running():
                _hlog("slam already running - reusing it")
            else:
                ok, msg = gb_mapping.start(); started_slam = ok
                _hlog("slam start:", ok, msg)
                if not ok:
                    report = "I couldn't start mapping, so I didn't move. " + msg; return
            if E("EXPLORE_NAV2", "0") != "0" and not _S["abort"].is_set():   # 2026-11-05: default OFF — reactive camera-first
                _S["phase"] = "start_nav2"
                nav2 = nav2_start(); _S["nav2"] = nav2
                _hlog("nav2 available:", nav2)
        if _S["abort"].is_set():
            report = "Stopped before I started moving."; return
        _S["phase"] = "running"
        with open(os.path.join(HOST_DIR, "host_hb"), "w") as f: f.write(str(time.time()))
        args = f"--node --plan '{json.dumps(plan)}'" + (" --dry" if dry else "") + (" --nav2" if nav2 else "")
        cmd = (f"{ROSRUN} python3 {CONT_DIR}/gb_explore.py {args}; rc=$?; "
               f"{ROSRUN} python3 {CONT_DIR}/gb_explore.py --zero" + (" --dry" if dry else "") + "; exit $rc")
        logf = open(os.path.join(os.path.dirname(_logpath()), "explore_ros.log"), "ab")
        logf.write(f"\n===== {_now()} {args}\n".encode()); logf.flush()
        proc = subprocess.Popen(["docker", "exec", "-u", "ubuntu", CONTAINER, "bash", "-lc", cmd], stdout=logf,
                                stderr=logf, stdin=subprocess.DEVNULL, start_new_session=True)
        _S["proc"] = proc
        hbf = os.path.join(HOST_DIR, "hb"); hhb = os.path.join(HOST_DIR, "host_hb")
        limit = _plan_limit(plan) + 120; t0 = time.time(); seen_hb = False; wd = None
        abort_file = os.path.join(HOST_DIR, "abort")
        def _aborting(): return _S["abort"].is_set() or os.path.exists(abort_file)
        while proc.poll() is None:
            with open(hhb, "w") as f: f.write(str(time.time()))
            try:
                st = json.load(open(hbf)); seen_hb = True; _S["phase"] = st.get("phase", "?")
                if time.time() - st["t"] > 2.0 and wd is None and not _aborting():
                    wd = "node heartbeat stale"
            except Exception:
                if time.time() - t0 > 25 and not seen_hb: wd = "node never started"
            if time.time() - t0 > limit: wd = "host time limit"
            if _aborting() and wd is None:
                ta = _S.setdefault("abort_t", time.time())
                if time.time() - ta > 8: wd = "node did not exit 8 s after stop"
            if wd:
                _hlog("WATCHDOG:", wd, "- aborting node and sending zero velocity")
                abort("watchdog: " + wd)
                if not dry: _zero_host()
                try: proc.wait(10)
                except Exception:
                    _dx(["pkill", "-INT", "-f", "gb_explore.py --node"], 5)
                break
            time.sleep(0.25)
        try: proc.wait(15)
        except Exception: pass
        if not dry: _zero_host()                           # belt and braces: zero over rosbridge too
        try: res = json.load(open(os.path.join(HOST_DIR, "result.json")))
        except Exception:
            if _aborting() and not wd:
                res = {"outcome": "aborted", "reason": "a stop command", "phases": ["?"] if seen_hb else []}
            else:
                res = {"outcome": "error", "reason": wd or "node exited without a result", "phases": ["?"] if seen_hb else []}
        _hlog("node result:", json.dumps(res)[:1500])
        report = _report(plan, res, dry)
    except Exception as ex:
        _hlog("mission error:", repr(ex)); report = f"My mission failed: {ex}. I stopped."
        if not dry: _zero_host()
    finally:
        _clear_guard_flag(GUARD_FLAG_HOST)                 # node gone (any path): gb-guard fully active again
        try:
            if plan["mission"] == "explore":
                import gb_mapping
                _S["phase"] = "save_map"
                if gb_mapping.running() and plan.get("save_map", True) and res.get("phases"):
                    ok, msg, info = gb_mapping.save(time.strftime("explore_%Y%m%d_%H%M%S"), stop_after=started_slam,
                                                    store=store)
                    report = (report or "") + " " + msg
                    _hlog("map save:", ok, msg)
                elif started_slam:
                    gb_mapping.stop()
            if nav2 or nav2_running(): nav2_stop()          # also catches a half-started Nav2 after an error
        except Exception as ex:
            _hlog("cleanup error:", repr(ex))
        if not dry: _zero_host()
        _S["phase"] = "idle"; _S["last"] = report
        _hlog("REPORT:", report)
        if announce and report:
            try: announce(report)
            except Exception as ex: _hlog("announce failed:", ex)

def _plan_limit(p):
    if p["mission"] == "follow": return p["cap_s"] + 120
    if p["mission"] == "explore": return p["cap_s"] + 240
    if p["mission"] == "distance": return p["distance_m"] / 0.05 * 2 + 120
    return p["duration_s"] + 240

def _report(plan, r, dry):
    pre = "Dry run finished. " if dry else ""
    oc = r.get("outcome"); why = r.get("reason", "")
    if oc == "aborted":
        why = re.sub(r"^stop command.*", "a stop command", why); why = re.sub(r"^signal \d+", "a stop command", why)
        return pre + f"I stopped because of {why}. Motors are at zero."
    if oc == "error":
        return pre + f"Something went wrong ({why}), so I stopped. Motors are at zero."
    parts = []
    m = plan["mission"]
    if m == "explore":
        parts.append(f"I explored for {int(r.get('explore_s', 0))} seconds and the map covers about "
                     f"{r.get('known_m2', 0):.1f} square meters ({r.get('end_reason', '')}).")
    elif m == "follow":
        n = plan.get("target"); er = r.get("end_reason")
        parts.append({"lost": f"I lost {n}.", "arrived": f"I'm here, about {plan.get('stop_m', 1):.0f} meter from {'you' if plan.get('kind') == 'come' else n}.",
                      "not_found": f"I couldn't find {n}.", "time": "Time's up, so I stopped following."}.get(er, "I stopped."))
    elif m == "distance":
        parts.append(f"I drove {r.get('dist_out_m', 0):.2f} meters" +
                     (f", stopping early because something was in the way" if r.get("blocked_out") else "") + ".")
    else:
        parts.append(f"I drove around for {int(r.get('wander_s', 0))} seconds, about {r.get('path_m', 0):.1f} meters.")
    if r.get("returned") is not None:
        e = r.get("return_error_m")
        if r.get("returned"):
            parts.append(f"I'm back where I started, within {100 * (e or 0):.0f} centimeters" +
                         (" and parked facing the original way." if r.get("parked") else "."))
        else:
            parts.append(f"I couldn't get all the way back ({r.get('return_reason', '')}); I'm about "
                         f"{(e or 0):.1f} meters from the start.")
    parts.append("Done.")
    return pre + " ".join(parts)

# ===================================================================================== Nav2 launch (container)
NAV2_LAUNCH = r'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
D = os.path.dirname(os.path.abspath(__file__))
P = os.path.join(D, "nav2_params.yaml"); C = os.path.join(D, "nav2_controller.yaml")
OUT = "/gb_explore/nav_cmd"   # gb_explore relays this to /controller/cmd_vel with hard caps + LiDAR stop
def generate_launch_description():
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    names = ["controller_server", "smoother_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"]
    return LaunchDescription([
        Node(package="rclcpp_components", executable="component_container_isolated", name="gb_nav2_container",
             parameters=[P, {"autostart": True}], output="screen"),
        LoadComposableNodes(target_container="gb_nav2_container", composable_node_descriptions=[
            ComposableNode(package="nav2_controller", plugin="nav2_controller::ControllerServer", name="controller_server",
                           parameters=[P, C], remappings=tf + [("cmd_vel", "/gb_explore/cmd_vel_nav")]),
            ComposableNode(package="nav2_smoother", plugin="nav2_smoother::SmootherServer", name="smoother_server",
                           parameters=[P], remappings=tf),
            ComposableNode(package="nav2_planner", plugin="nav2_planner::PlannerServer", name="planner_server",
                           parameters=[P], remappings=tf),
            ComposableNode(package="nav2_behaviors", plugin="behavior_server::BehaviorServer", name="behavior_server",
                           parameters=[P], remappings=tf + [("cmd_vel", OUT)]),
            ComposableNode(package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator", name="bt_navigator",
                           parameters=[P], remappings=tf),
            ComposableNode(package="nav2_velocity_smoother", plugin="nav2_velocity_smoother::VelocitySmoother",
                           name="velocity_smoother", parameters=[P],
                           remappings=tf + [("cmd_vel", "/gb_explore/cmd_vel_nav"), ("cmd_vel_smoothed", OUT)]),
            ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                           name="lifecycle_manager_navigation",
                           parameters=[{"use_sim_time": False, "autostart": True, "node_names": names, "bond_timeout": 8.0}]),
        ]),
    ])
'''

def prep_nav2():
    """(container) write speed-capped copies of Hiwonder's nav2_params.yaml + nav2_controller_dwb.yaml."""
    import yaml
    src = "/home/ubuntu/ros2_ws/src/navigation/config"
    P = yaml.safe_load(open(os.path.join(src, "nav2_params.yaml")))
    C = yaml.safe_load(open(os.path.join(src, "nav2_controller_dwb.yaml")))
    for k in ("amcl", "amcl_rclcpp_node", "map_server", "map_saver", "controller_server", "controller_server_rclcpp_node"):
        P.pop(k, None)
    bt = P["bt_navigator"]["ros__parameters"]
    if bt.get("plugin_lib_names") is None: bt.pop("plugin_lib_names", None)
    vs = P["velocity_smoother"]["ros__parameters"]
    vs["max_velocity"] = [MAX_LIN, 0.0, MAX_ANG]; vs["min_velocity"] = [-MAX_LIN, 0.0, -MAX_ANG]
    vs["max_accel"] = [1.0, 0.0, 2.0]; vs["max_decel"] = [-1.0, 0.0, -2.0]
    bs = P["behavior_server"]["ros__parameters"]
    bs["max_rotational_vel"] = MAX_ANG; bs["min_rotational_vel"] = min(0.3, MAX_ANG); bs["rotational_acc_lim"] = 2.0
    for cm in (("local_costmap", "local_costmap"), ("global_costmap", "global_costmap")):
        rp = P[cm[0]][cm[1]]["ros__parameters"]; rp["robot_radius"] = float(E("NAV2_ROBOT_RADIUS", "0.22"))
        rp.pop("footprint", None)
        # gb_gate guard is always on (stream stop 0.25 m ahead / 0.20 m behind): keep plans wider than that so Nav2
        # rarely meets the guard; if it does, the controller stalls -> progress checker -> BT recovery + replanning.
        il = rp.get("inflation_layer")
        if isinstance(il, dict):
            il["inflation_radius"] = float(E("NAV2_INFLATION_R", "0.50")); il["cost_scaling_factor"] = float(E("NAV2_COST_SCALE", "4.0"))
    cs = C["controller_server"]["ros__parameters"]
    cs["progress_checker"].update(required_movement_radius=0.2, movement_time_allowance=15.0)
    fp = cs["FollowPath"]
    fp.update(max_vel_x=MAX_LIN, max_speed_xy=MAX_LIN, max_vel_theta=MAX_ANG, min_vel_x=0.0, acc_lim_x=1.0,
              decel_lim_x=-1.0, acc_lim_theta=2.0, decel_lim_theta=-2.0, debug_trajectory_details=False)
    # local costmap params also go to the controller (it owns local_costmap)
    os.makedirs(CONT_DIR, exist_ok=True)
    yaml.safe_dump(P, open(os.path.join(CONT_DIR, "nav2_params.yaml"), "w"), default_flow_style=None, sort_keys=False)
    yaml.safe_dump(C, open(os.path.join(CONT_DIR, "nav2_controller.yaml"), "w"), default_flow_style=None, sort_keys=False)
    print("nav2 params written: lin", MAX_LIN, "ang", MAX_ANG)

# ===================================================================================== node side (container)
def _wrap(a): return math.atan2(math.sin(a), math.cos(a))

def zero_only(dry=False, n=20):
    _clear_guard_flag(GUARD_FLAG_CONT)                     # wrapper runs this after the node exits, even after SIGKILL
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()
    nd = rclpy.create_node("gb_explore_zero")
    pub = nd.create_publisher(Twist, CMD_VEL, 10)
    time.sleep(0.3)
    for _ in range(n):
        if not dry: pub.publish(Twist())
        time.sleep(0.05)
    print(_now(), "[zero] sent", 0 if dry else n, "zero Twists" + (" (dry: none published)" if dry else ""), flush=True)
    nd.destroy_node(); rclpy.shutdown()

class Abort(Exception):
    pass

def node_main(plan, dry, use_nav2):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    import tf2_ros
    from std_msgs.msg import String
    import sys
    sys.path.insert(0, "/home/ubuntu/shared/gb_safety")
    import gb_obstacle as OB          # 2026-11-05: same footprint corridor / ramp as the gate (copied there by gb_safety_run.sh)

    os.makedirs(CONT_DIR, exist_ok=True)
    with open(os.path.join(CONT_DIR, "node.pid"), "w") as f: f.write(str(os.getpid()))
    stop_flag = {"why": None}
    def _sig(sig, frm): stop_flag["why"] = f"signal {sig}"
    try:                                   # keep the ROS context alive on SIGINT so the final zero burst can publish
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=None)
    signal.signal(signal.SIGINT, _sig); signal.signal(signal.SIGTERM, _sig)

    class X(Node):
        def __init__(self):
            super().__init__("gb_explore")
            self.pub = _GateRoute(self.create_publisher(Twist, GATE_IN, 10), self.create_publisher(Twist, CMD_VEL, 10))
            self.scan = None; self.scan_t = 0.0; self.odom = None; self.odom_t = 0.0
            self.map = None; self.map_t = 0.0; self.nav_cmd = (0.0, 0.0, 0.0)
            self.create_subscription(LaserScan, "/scan_raw", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, mq)
            self.create_subscription(Twist, NAV_OUT, self.on_nav, 10)
            self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
            self.sect = None; self.gst = None; self.gst_t = 0.0
            self.create_subscription(String, "/gb/gate_status", self.on_gst, 10)   # gate's FUSED front (LiDAR + depth)
        def on_gst(self, m):
            try: self.gst = json.loads(m.data); self.gst_t = time.time()
            except Exception: pass
        def on_scan(self, m):
            r = np.asarray(m.ranges, dtype=float)
            a = np.array([_wrap(m.angle_min + i * m.angle_increment) for i in range(len(r))])
            ok = np.isfinite(r) & (r >= max(0.08, m.range_min)) & (r <= m.range_max)
            def mn(lo, hi):
                s = ok & (a >= lo) & (a <= hi); return float(r[s].min()) if s.any() else float("inf")
            def mean(lo, hi):
                s = ok & (a >= lo) & (a <= hi); return float(np.clip(r[s], 0, 3).mean()) if s.any() else 3.0
            d = math.radians
            self.sect = {"front": mn(-d(25), d(25)), "fl": mn(d(25), d(70)), "fr": mn(-d(70), -d(25)),
                         "left_mean": mean(d(20), d(100)), "right_mean": mean(-d(100), -d(20)),
                         "rear": min(mn(d(150), d(180)), mn(-d(180), -d(150))), "wide_front": mn(-d(45), d(45))}
            try:                                             # footprint corridor (the +/-25 deg cone misses the track corners)
                v = OB.lidar_analyze(m.ranges, m.angle_min, m.angle_increment, max(0.08, m.range_min), m.range_max)
                self.sect.update(corr=v.fwd, corr_rev=v.rev, sweep=v.sweep, blind=v.blind_f)
            except Exception:
                self.sect.update(corr=0.0, corr_rev=0.0, sweep=0.0, blind=1.0)   # fail safe: treat as blocked
            self.scan_t = time.time()
        def on_odom(self, m):
            p = m.pose.pose; q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom = (p.position.x, p.position.y, yaw); self.odom_t = time.time()
        def on_map(self, m):
            self.map = m; self.map_t = time.time()
        def on_nav(self, m):
            self.nav_cmd = (m.linear.x, m.angular.z, time.time())
        def map_pose(self):
            try:
                t = self.tfb.lookup_transform("map", "base_footprint", rclpy.time.Time())
                q = t.transform.rotation
                return (t.transform.translation.x, t.transform.translation.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            except Exception:
                return None

    nd = X()
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(nd)
    threading.Thread(target=ex.spin, daemon=True).start()
    res = {"mission": plan["mission"], "dry": dry, "nav2": use_nav2, "phases": [], "outcome": "error", "reason": "",
           "cmds_published": 0, "cmds_planned": 0}
    cmdlog = open(os.path.join(CONT_DIR, "cmdlog.jsonl"), "w")
    st = {"phase": "init", "last": None, "last_log": 0.0, "t0": time.time(), "blocked_since": None}

    def log(*a):
        print(_now(), "[node]", *a, flush=True)

    def phase(p):
        if st["phase"] != p:
            log("PHASE", st["phase"], "->", p)
            res["phases"].append(p); st["phase"] = p

    def hb():
        with open(os.path.join(CONT_DIR, "hb.tmp"), "w") as f:
            json.dump({"t": time.time(), "phase": st["phase"]}, f)
        os.replace(os.path.join(CONT_DIR, "hb.tmp"), os.path.join(CONT_DIR, "hb"))
        try:                                   # keep gb-guard's mission bypass alive (fail-safe: stale -> guard on)
            with open(GUARD_FLAG_CONT + ".tmp", "w") as f:
                json.dump({"pid": os.getpid(), "t": time.time(), "mission": plan.get("mission"), "dry": bool(dry)}, f)
            os.replace(GUARD_FLAG_CONT + ".tmp", GUARD_FLAG_CONT)
        except OSError:
            pass

    def checks():
        if stop_flag["why"]: raise Abort(stop_flag["why"])
        if os.path.exists(ESTOP_CONT): raise Abort("ESTOP latched")
        if os.path.exists(os.path.join(CONT_DIR, "abort")):
            raise Abort("stop command (" + open(os.path.join(CONT_DIR, "abort")).read().strip() + ")")
        try:
            hh = float(open(os.path.join(CONT_DIR, "host_hb")).read())
            if time.time() - hh > 3.0: raise Abort("host watchdog (voice service heartbeat stale)")
        except (OSError, ValueError):
            if time.time() - st["t0"] > 10: raise Abort("host watchdog (no heartbeat file)")
        hb()

    def send(lin, ang, why=""):
        """THE only path to the motors. Caps, LiDAR stop, staleness stop, logging."""
        checks()
        lin = max(-MAX_LIN, min(MAX_LIN, float(lin))); ang = max(-MAX_ANG, min(MAX_ANG, float(ang)))
        now = time.time(); s = nd.sect or {}
        if now - nd.scan_t > 0.6 or now - nd.odom_t > 0.6:
            lin, ang, why = 0.0, 0.0, "STALE " + ("scan" if now - nd.scan_t > 0.6 else "odom")
        elif lin > 0 and s.get("front", 9) < FRONT_STOP:
            lin, why = 0.0, why + " FRONT-BLOCK %.2f" % s.get("front", 9)
        elif lin < 0 and s.get("rear", 9) < REAR_STOP:
            lin, why = 0.0, why + " REAR-BLOCK %.2f" % s.get("rear", 9)
        if lin != 0:                                         # corridor + gate's fused distance, speed ramp (0.04 -> max)
            gf = (nd.gst or {}).get("front") if now - nd.gst_t < 0.5 else None
            if lin > 0:
                dd = min(s.get("corr", 0.0), gf if gf is not None else 9.0)
                cap = OB.ramp(dd, MAX_LIN)
                if cap <= 0: lin, why = 0.0, why + " CORRIDOR-BLOCK %.2f" % dd
                elif lin > cap: lin, why = cap, why + " ramp %.2f" % dd
            else:
                cap = OB.ramp(s.get("corr_rev", 0.0), MAX_LIN, OB.STOP_REV)
                if cap <= 0: lin, why = 0.0, why + " REAR-CORRIDOR-BLOCK %.2f" % s.get("corr_rev", 0.0)
                elif -lin > cap: lin = -cap
        assert abs(lin) <= HARD_MAX_LIN + 1e-9 and abs(ang) <= HARD_MAX_ANG + 1e-9
        m = Twist(); m.linear.x = lin; m.angular.z = ang
        if not dry:
            try:
                nd.pub.publish(m); res["cmds_published"] += 1
            except Exception as e:                      # e.g. RCLError: context invalid -> stop the mission
                raise Abort(f"publish failed ({type(e).__name__}: {str(e)[:80]})")
        res["cmds_planned"] += 1
        key = (round(lin, 2), round(ang, 2), st["phase"])
        if key != st["last"] or now - st["last_log"] > 2.0:
            o = nd.odom or (0, 0, 0)
            rec = {"t": round(now - st["t0"], 2), "phase": st["phase"], "lin": round(lin, 3), "ang": round(ang, 3),
                   "why": why.strip(), "odom": [round(o[0], 3), round(o[1], 3), round(o[2], 3)],
                   "front": round(s.get("front", -1), 2), "pub": not dry}
            cmdlog.write(json.dumps(rec) + "\n"); cmdlog.flush()
            log(("PUB " if not dry else "DRY ") + json.dumps(rec))
            st["last"] = key; st["last_log"] = now
        return lin

    def zero(n=10, why="zero"):
        for _ in range(n):
            send(0.0, 0.0, why); time.sleep(0.05)

    def wait_inputs(t=10):
        t0 = time.time()
        while time.time() - t0 < t:
            checks()
            gst_ok = nd.gst and time.time() - nd.gst_t < 1.0 and (nd.gst.get("fusion") or {}).get("fresh", {}).get("depth")
            if nd.odom and time.time() - nd.odom_t < 0.5 and ((nd.sect and time.time() - nd.scan_t < 0.5) or gst_ok):
                return
            time.sleep(0.1)
        raise Abort("no LiDAR/depth scan or odometry")

    def dist(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])

    # ---------------------------------------------------------------- primitives
    def rotate_to(yaw_target, label, timeout=20.0, tol=0.08):
        phase(label); t0 = time.time()
        while time.time() - t0 < timeout:
            err = _wrap(yaw_target - nd.odom[2])
            if abs(err) < tol: break
            w = max(0.25, min(MAX_ANG, 1.2 * abs(err))) * (1 if err > 0 else -1)
            send(0.0, w, f"rotate err={err:.2f}"); time.sleep(0.1)
        zero(5, "rotate done")
        return abs(_wrap(yaw_target - nd.odom[2]))

    def go_to_points(points, label, timeout, final_tol=0.10):
        """Follow odom waypoints; rotate in place when badly misaligned (label 'turn_around' for the first big turn)."""
        t0 = time.time(); blocked = 0.0; i = 0
        while i < len(points):
            if time.time() - t0 > timeout: return False, "took too long"
            o = nd.odom; tgt = points[i]
            last = i == len(points) - 1
            d = dist(o, tgt)
            if d < (final_tol if last else 0.18):
                i += 1; continue
            err = _wrap(math.atan2(tgt[1] - o[1], tgt[0] - o[0]) - o[2])
            if abs(err) > 0.45:
                phase("turn_around" if abs(err) > 2.0 and "turn_around" not in res["phases"] else label + "_turn")
                send(0.0, max(0.3, min(MAX_ANG, 1.2 * abs(err))) * (1 if err > 0 else -1), f"align err={err:.2f}")
            else:
                phase(label)
                v = max(0.04, min(MAX_LIN, 0.6 * d))
                got = send(v, max(-MAX_ANG, min(MAX_ANG, 1.5 * err)), f"to wp{i} d={d:.2f}")
                if got == 0.0:
                    blocked += 0.1
                    if blocked > 15: return False, "path blocked by an obstacle"
                else:
                    blocked = max(0.0, blocked - 0.05)
            time.sleep(0.1)
        zero(5, "arrived")
        return True, "ok"

    def wander_step(w):
        """Camera-first reactive walk (10 Hz). Depth+LiDAR fused via gate status; bump recovery without aborting."""
        import random
        s = nd.sect; now = time.time(); o = nd.odom or (0, 0, 0)
        gf = (nd.gst or {}).get("front") if now - nd.gst_t < 0.5 else None
        fus = (nd.gst or {}).get("fusion") or {}
        depth_f = fus.get("depth_fwd"); depth_f = float(depth_f) if depth_f is not None else 9.0
        front = min(s.get("front", 9), s.get("corr", 9), gf if gf is not None else 9.0, depth_f)
        # ---- detect a new gate trip / bump (IMU jolt/tilt, stall, corridor block while we were driving)
        trips = int((nd.gst or {}).get("trips") or 0)
        last = (nd.gst or {}).get("last_trip") or ""
        if trips > w.get("trips_seen", 0):
            w["trips_seen"] = trips
            if any(k in last.lower() for k in ("bump", "tilt", "stall", "obstacle", "jolt")):
                yaw = o[2]; w.setdefault("bumps", []).append((o[0], o[1], yaw, now))
                w["bumps"] = [b for b in w["bumps"] if now - b[3] < 120]
                w["hold_until"] = now + 0.8                                   # brief pause then go around (was 2.5)
                # pick a DIFFERENT recovery than last time
                prev = w.get("last_recover")
                choices = ["turn_opp", "turn_same", "back"]
                if prev in choices: choices.remove(prev)
                # avoid headings that recently bumped
                def toward_bump(d):
                    for bx, by, byaw, _t in w["bumps"][-6:]:
                        if dist(o, (bx, by)) < 0.55 and abs(_wrap(byaw - (o[2] + d * 1.2))) < 0.9: return True
                    return False
                pick = None
                for c in random.sample(choices, len(choices)):
                    if c == "back" and s.get("corr_rev", 0) < OB.STOP_REV + 0.08: continue
                    if c.startswith("turn") and toward_bump(1 if "opp" not in c else -1): continue
                    pick = c; break
                pick = pick or "turn_opp"
                w["last_recover"] = pick
                w["recover_until"] = now + (3.5 if pick.startswith("turn") else 2.0)
                if pick == "turn_opp": w["dir"] = -(w.get("dir") or (1 if s["left_mean"] >= s["right_mean"] else -1))
                elif pick == "turn_same": w["dir"] = w.get("dir") or (1 if s["left_mean"] >= s["right_mean"] else -1)
                else: w["dir"] = 0                                            # back up
                w["escapes_recent"] = w.get("escapes_recent", 0) + 1
                log("bump recover:", pick, "dir", w["dir"], "front", round(front, 2), "trip", last[:80])
        # ---- held after bump
        if w.get("hold_until", 0) > now:
            send(0.0, 0.0, "hold after bump"); return
        # ---- recovery action
        if w.get("recover_until", 0) > now:
            if w.get("dir") == 0:
                if s.get("corr_rev", 0) > OB.STOP_REV + 0.05:
                    send(-OB.V_MIN, 0.0, "recover back"); return
                w["dir"] = 1 if s["left_mean"] >= s["right_mean"] else -1
            send(0.0, w["dir"] * MAX_ANG * 0.85, "recover turn"); return
        # ---- boxed in after many recoveries: try a bigger escape (never end the mission, never latch)
        recent = [b for b in w.get("bumps", []) if now - b[3] < 45]
        if len(recent) >= 5:
            xs = [b[0] for b in recent]; ys = [b[1] for b in recent]
            if (max(xs) - min(xs)) < 0.40 and (max(ys) - min(ys)) < 0.40 and w.get("path", 0) - w.get("path_at_stuck", 0) < 0.20:
                w["path_at_stuck"] = w.get("path", 0)
                w["bumps"] = w["bumps"][-2:]                                 # forget older dead-ends; keep trying
                w["dir"] = 1 if random.random() < 0.5 else -1
                w["recover_until"] = now + 5.0; w["hold_until"] = now + 0.4
                w["last_recover"] = "turn_opp"
                log("boxed in -> wide turn, still exploring")
                send(0.0, w["dir"] * MAX_ANG, "wide turn around clutter"); return
        # ---- normal escape from close obstacle
        if w.get("escape_until", 0) > now:
            if front > AVOID_DIST + 0.20 and s.get("corr", 0.0) > AVOID_DIST + 0.15 and now - w.get("escape_start", 0) > 0.8:
                w["escape_until"] = 0
            elif s.get("sweep", 0.0) < OB.SWEEP_R and s.get("corr_rev", 0.0) > OB.STOP_REV + 0.05:
                send(-OB.V_MIN, 0.0, "back out (turn sweep blocked %.2f)" % s.get("sweep", 0.0)); return
            else:
                send(0.0, w.get("dir", 1) * MAX_ANG * 0.8, "escape turn"); return
        if front < AVOID_DIST or s.get("wide_front", 9) < AVOID_DIST * 0.7:
            # prefer the side that does NOT point at a recent bump
            left_ok = not any(dist(o, (b[0], b[1])) < 0.5 and abs(_wrap(b[2] - (o[2] + 1.2))) < 1.0 for b in w.get("bumps", [])[-5:])
            right_ok = not any(dist(o, (b[0], b[1])) < 0.5 and abs(_wrap(b[2] - (o[2] - 1.2))) < 1.0 for b in w.get("bumps", [])[-5:])
            if left_ok and (s["left_mean"] >= s["right_mean"] or not right_ok): w["dir"] = 1
            elif right_ok: w["dir"] = -1
            else: w["dir"] = 1 if s["left_mean"] >= s["right_mean"] else -1
            w["escape_until"] = now + 3.5; w["escape_start"] = now
            send(0.0, w["dir"] * MAX_ANG * 0.8, "obstacle %.2f (depth/lidar) -> turn %s" % (front, "left" if w["dir"] > 0 else "right"))
            return
        if now - w.get("drift_t", 0) > 6.0:
            w["drift"] = random.uniform(-0.25, 0.25); w["drift_t"] = now; w["escapes_recent"] = 0
        side = max(-0.4, min(0.4, 0.6 * (min(s["fl"], 1.5) - min(s["fr"], 1.5))))
        # speed from fused front (camera+lidar); OB.ramp applied again in send()
        v = MAX_LIN * max(0.35, min(1.0, (front - AVOID_DIST) / 0.6))
        if w.get("chk") is None or now - w["chk"][0] > 4.0:
            if w.get("chk") and dist(o, w["chk"][1]) < 0.03 and w.get("fwd"):
                w["dir"] = 1 if s["left_mean"] >= s["right_mean"] else -1
                w["escape_until"] = now + 3.0; w["escape_start"] = now
                send(0.0, w["dir"] * MAX_ANG * 0.8, "stuck -> turn"); w["chk"] = (now, o); return
            w["chk"] = (now, o)
        w["fwd"] = send(v, w.get("drift", 0) + side, "wander") > 0
        crumbs = w.setdefault("crumbs", [])
        if not crumbs or dist(o, crumbs[-1]) > 0.25: crumbs.append((o[0], o[1]))
        w["path"] = w.get("path", 0.0) + (dist(o, w["prev"]) if w.get("prev") else 0.0); w["prev"] = o

    def known_m2():
        m = nd.map
        if m is None: return 0.0, 0
        g = np.asarray(m.data, dtype=np.int16)
        k = int((g >= 0).sum()); return k * m.info.resolution ** 2, k

    def retrace(crumbs, start, label="return"):
        """drive back along the breadcrumb trail (reverse), then to start"""
        pts = list(reversed(crumbs)) + [(start[0], start[1])]
        # thin: keep points >= 0.3 m apart
        thin = []
        for p in pts:
            if not thin or dist(p, thin[-1]) >= 0.3: thin.append(p)
        if dist(thin[-1], start) > 0.01: thin.append((start[0], start[1]))
        length = sum(dist(thin[i], thin[i + 1]) for i in range(len(thin) - 1)) + dist(nd.odom, thin[0])
        return go_to_points(thin, label, timeout=max(60.0, 3.0 * length / MAX_LIN))

    # ---------------------------------------------------------------- Nav2 helpers
    nav = {"client": None}
    def nav_client():
        if nav["client"] is None:
            from nav2_msgs.action import NavigateToPose
            nav["cls"] = NavigateToPose
            nav["client"] = ActionClient(nd, NavigateToPose, "navigate_to_pose")
        return nav["client"]

    def nav_goto(x, y, yaw, label, timeout):
        """send a NavigateToPose goal, relay its (capped) output; returns (ok, why)"""
        cl = nav_client()
        if not cl.wait_for_server(timeout_sec=5.0): return False, "nav2 action server not available"
        g = nav["cls"].Goal(); ps = PoseStamped(); ps.header.frame_id = "map"
        ps.header.stamp = nd.get_clock().now().to_msg()
        ps.pose.position.x = float(x); ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2); ps.pose.orientation.w = math.cos(yaw / 2); g.pose = ps
        fut = cl.send_goal_async(g); t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 5: return False, "goal not accepted"
            zero(1, "wait goal accept")
        gh = fut.result()
        if not gh.accepted: return False, "goal rejected"
        rf = gh.get_result_async(); phase(label)
        try:
            while not rf.done():
                if time.time() - t0 > timeout:
                    gh.cancel_goal_async(); return False, "timeout"
                lin, ang, t = nd.nav_cmd
                if time.time() - t > 0.5: lin = ang = 0.0
                send(lin, ang, "nav2"); time.sleep(0.1)
        except Abort:
            gh.cancel_goal_async(); raise
        stt = rf.result().status
        zero(3, "nav goal end")
        return stt == 4, f"nav2 status {stt}"            # 4 = SUCCEEDED

    def frontier_goal(blacklist):
        m = nd.map; mp = nd.map_pose()
        if m is None or mp is None: return None
        w, h, r = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        g = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        free = (g >= 0) & (g < 25); unk = g < 0
        nb = np.zeros_like(unk)
        nb[1:, :] |= unk[:-1, :]; nb[:-1, :] |= unk[1:, :]; nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
        ys, xs = np.nonzero(free & nb)
        if len(xs) == 0: return None
        bins = {}
        for x, y in zip(xs.tolist(), ys.tolist()):
            bins.setdefault((x // 6, y // 6), []).append((x, y))
        best = None
        for cells in bins.values():
            if len(cells) < 4: continue
            cx = sum(c[0] for c in cells) / len(cells); cy = sum(c[1] for c in cells) / len(cells)
            gx, gy = min(cells, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
            wx, wy = ox + (gx + 0.5) * r, oy + (gy + 0.5) * r
            d = math.hypot(wx - mp[0], wy - mp[1])
            if d < 0.4 or any(math.hypot(wx - b[0], wy - b[1]) < 0.5 for b in blacklist): continue
            score = d - 0.02 * len(cells)
            if best is None or score < best[0]: best = (score, wx, wy, len(cells))
        return best

    # ---------------------------------------------------------------- missions
    try:
        phase("wait_inputs"); wait_inputs()
        start = nd.odom; res["start_odom"] = [round(v, 3) for v in start]
        log("plan", json.dumps(plan), "dry" if dry else "LIVE", "nav2" if use_nav2 else "no-nav2",
            "caps lin", MAX_LIN, "ang", MAX_ANG)
        zero(4, "start")
        mis = plan["mission"]
        if mis == "distance":
            D = float(plan["distance_m"]); phase("out"); t0 = time.time(); blocked = 0.0
            res["blocked_out"] = False
            while True:
                o = nd.odom; trav = dist(o, start)
                if trav >= D - 0.02: break
                if time.time() - t0 > D / 0.04 + 20: res["reason_out"] = "time"; break
                err = _wrap(start[2] - o[2])
                v = max(0.04, min(MAX_LIN, D - trav + 0.03))
                got = send(v, max(-0.5, min(0.5, 1.5 * err)), f"out {trav:.2f}/{D:.2f}")
                if got == 0.0:
                    blocked += 0.1
                    if blocked > 5.0: res["blocked_out"] = True; break
                else: blocked = 0.0
                time.sleep(0.1)
            res["dist_out_m"] = round(dist(nd.odom, start), 3)
            phase("stop"); zero(20, "stop at far point"); time.sleep(0.5)
            if plan.get("return"):
                ok, why = go_to_points([(start[0], start[1])], "return", timeout=res["dist_out_m"] / 0.04 + 40,
                                       final_tol=0.08)
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    err = rotate_to(start[2], "park"); res["parked"] = err < 0.15
                res["return_error_m"] = round(dist(nd.odom, start), 3)
        elif mis == "follow":
            # person/object follow: target written by the host tracker (gb_perceive) as robot-relative fwd/left.
            # All motion goes through send() (caps, LiDAR stop). Never forward inside the person standoff.
            tf_path = os.path.join(CONT_DIR, "target.json"); stop_d = float(plan.get("stop_m", 1.2))
            hard = float(plan.get("standoff_m", 0.8)); once = bool(plan.get("once")); T = float(plan.get("cap_s", 300))
            lost_s = float(plan.get("lost_s", 5.0)); t0 = time.time(); last_seen = None; turned = 0.0; end = "time"
            phase("follow")
            while time.time() - t0 < T:
                try:
                    with open(tf_path) as f: tg = json.load(f)
                except Exception: tg = None
                now = time.time()
                if tg and tg.get("seen") and now - float(tg.get("t", 0)) < 1.5:
                    last_seen = now; fw, lf = float(tg["fwd"]), float(tg["left"])
                    d = math.hypot(fw, lf); b = math.atan2(lf, fw)
                    if once and abs(d - stop_d) < 0.2 and abs(b) < 0.2: end = "arrived"; break
                    ang = max(-MAX_ANG, min(MAX_ANG, 1.2 * b))
                    lin = 0.0 if (d <= max(stop_d, hard) or abs(b) > 0.5) else min(MAX_LIN, 0.5 * (d - stop_d))
                    send(lin, ang, f"follow d={d:.2f} b={b:.2f}")
                elif last_seen is None and plan.get("search") and turned < 2 * math.pi:
                    w = min(0.3, MAX_ANG); send(0.0, w, "search"); turned += w * 0.1
                elif last_seen is None and now - t0 > lost_s + (4 * math.pi / 0.3 if plan.get("search") else 0):
                    end = "not_found"; break
                elif last_seen is not None and now - last_seen > lost_s:
                    end = "lost"; break
                else:
                    zero(1, "follow wait")
                time.sleep(0.1)
            res["end_reason"] = end; res["target"] = plan.get("target"); phase("stop"); zero(20, "follow done")
        elif mis == "wander":
            phase("wander"); w = {}; t0 = time.time(); T = float(plan["duration_s"])
            while time.time() - t0 < T:
                wander_step(w); time.sleep(0.1)
            res["wander_s"] = round(time.time() - t0, 1); res["path_m"] = round(w.get("path", 0.0), 2)
            phase("stop"); zero(20, "wander time up")
            if plan.get("return"):
                ok, why = retrace(w.get("crumbs", []), start)
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    err = rotate_to(start[2], "park"); res["parked"] = err < 0.15
                res["return_error_m"] = round(dist(nd.odom, start), 3)
        elif mis == "explore":
            t_map = time.time()
            while nd.map is None and time.time() - t_map < 15:
                zero(1, "waiting for /map"); time.sleep(0.2)
            start_map = None
            for _ in range(30):
                start_map = nd.map_pose()
                if start_map: break
                zero(1, "waiting for map tf"); time.sleep(0.1)
            res["start_map"] = [round(v, 3) for v in start_map] if start_map else None
            cap = float(plan["cap_s"]); stall = float(plan.get("stall_s", STALL_S))
            hist = []; t0 = time.time(); w = {}; end = "time cap reached"; blacklist = []; mode = "nav2-frontier" if use_nav2 else "reactive"
            res["explore_mode"] = mode; fails = 0
            phase("explore")
            while True:
                el = time.time() - t0
                m2, k = known_m2(); hist.append((time.time(), k))
                hist = [hk for hk in hist if time.time() - hk[0] <= stall + 1]
                if el >= cap: end = "time cap reached"; break
                if el > stall + 10 and len(hist) > 2 and time.time() - hist[0][0] >= stall - 1:
                    grow = k - hist[0][1]
                    if grow < max(40, 0.02 * k):
                        end = f"map stopped growing for {int(stall)} seconds"; break
                if mode == "nav2-frontier":
                    fg = frontier_goal(blacklist)
                    if fg is None:
                        if el < 20:                       # map too small yet: wander a little first
                            for _ in range(20): wander_step(w); time.sleep(0.1)
                            continue
                        end = "no unexplored frontiers left"; break
                    _, gx, gy, sz = fg; mp = nd.map_pose() or (0, 0, 0)
                    log(f"frontier goal ({gx:.2f},{gy:.2f}) size {sz}")
                    ok, why = nav_goto(gx, gy, math.atan2(gy - mp[1], gx - mp[0]), "explore",
                                       timeout=min(45.0, max(5.0, cap - el)))
                    phase("explore")
                    if not ok:
                        blacklist.append((gx, gy)); fails += 1; log("frontier failed:", why)
                        if fails >= 4 and fails > len(blacklist) // 2 + 2:
                            log("nav2 keeps failing -> reactive explore"); mode = "reactive"; res["explore_mode"] = "nav2->reactive"
                    else:
                        fails = 0
                    o = nd.odom; cr = w.setdefault("crumbs", [])
                    if not cr or dist(o, cr[-1]) > 0.25: cr.append((o[0], o[1]))
                else:
                    wander_step(w); time.sleep(0.1)
            res["explore_s"] = round(time.time() - t0, 1); res["end_reason"] = end
            res["known_m2"] = round(known_m2()[0], 2); res["path_m"] = round(w.get("path", 0.0), 2)
            log("explore finished:", end, "known m2", res["known_m2"])
            phase("stop"); zero(20, "explore done")
            if plan.get("return", True):
                ok = False; why = "not tried"
                if use_nav2 and start_map:
                    ok, why = nav_goto(start_map[0], start_map[1], start_map[2], "return", timeout=150)
                    res["return_via"] = "nav2"
                    log("nav2 return:", ok, why)
                if not ok:
                    res["return_via"] = "odom-retrace"
                    ok, why = retrace(w.get("crumbs", []), start)
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    err = rotate_to(start[2], "park"); res["parked"] = err < 0.15
                res["return_error_m"] = round(dist(nd.odom, start), 3)
        res["outcome"] = "done"; phase("done")
    except Abort as a:
        res["outcome"] = "aborted"; res["reason"] = str(a); log("ABORT:", a)
    except Exception as e:
        import traceback; traceback.print_exc()
        res["outcome"] = "error"; res["reason"] = repr(e)
    finally:
        _clear_guard_flag(GUARD_FLAG_CONT)                  # re-arm gb-guard first, before anything else
        def _write_result():
            try:
                res["end_odom"] = [round(v, 3) for v in nd.odom] if nd.odom else None
                res["elapsed_s"] = round(time.time() - st["t0"], 1)
                with open(os.path.join(CONT_DIR, "result.json.tmp"), "w") as f: json.dump(res, f)
                os.replace(os.path.join(CONT_DIR, "result.json.tmp"), os.path.join(CONT_DIR, "result.json"))
            except Exception as e:
                log("result write failed:", e)
        _write_result()                                     # before anything else can fail
        m = Twist()
        for _ in range(20):                                 # always end with repeated zero Twist
            try:
                if not dry: nd.pub.publish(m)
            except Exception as e:
                log("final zero publish failed:", e); break
            time.sleep(0.05)
        log("final zero burst", "skipped (dry)" if dry else "sent x20")
        res["final_zero"] = "dry" if dry else "sent"
        _write_result()
        _clear_guard_flag(GUARD_FLAG_CONT)
        try: cmdlog.close()
        except Exception: pass
        try: os.remove(os.path.join(CONT_DIR, "node.pid"))
        except OSError: pass
        try:
            ex.shutdown(); nd.destroy_node(); rclpy.shutdown()
        except Exception: pass
    return 0 if res["outcome"] == "done" else 1

# ===================================================================================== CLI
def main(argv):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", action="store_true"); ap.add_argument("--zero", action="store_true")
    ap.add_argument("--prep-nav2", action="store_true"); ap.add_argument("--dry", action="store_true")
    ap.add_argument("--nav2", action="store_true"); ap.add_argument("--plan")
    ap.add_argument("--parse", nargs="+", help="print the plan for each phrase")
    ap.add_argument("--run", help="(host) run a mission for this phrase in the foreground")
    ap.add_argument("--stop-after", type=float, default=0, help="with --run --dry: send stop after N seconds")
    a = ap.parse_args(argv)
    if a.parse:
        for p in a.parse:
            pl = parse_plan(p); print(json.dumps({"text": p, "plan": pl, "says": describe_plan(pl) if pl else None}))
        return 0
    if a.zero: zero_only(a.dry); return 0
    if a.prep_nav2: prep_nav2(); return 0
    if a.node: return node_main(json.loads(a.plan), a.dry, a.nav2)
    if a.run:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        pl = parse_plan(a.run)
        if not pl: print("not a mission:", a.run); return 2
        if not a.dry and not _flag("AUTO_EXPLORE"):
            print("refusing: CLI live runs need AUTO_EXPLORE=1 in the environment; use --dry"); return 2
        print(json.dumps(pl))
        print(request(pl, dry=a.dry, announce=lambda t: print("ANNOUNCE:", t), cli_dry=a.dry), flush=True)
        t0 = time.time(); stopped = False
        while running():
            if a.stop_after and not stopped and time.time() - t0 >= a.stop_after:
                print(f"STOP at +{time.time() - t0:.1f}s:", abort("stop command (cli test)"), flush=True); stopped = True
            time.sleep(0.5)
        return 0
    ap.print_help(); return 2

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
