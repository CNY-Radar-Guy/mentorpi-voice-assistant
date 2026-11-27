#!/usr/bin/env python3
"""gb_explore.py - autonomous missions for the MentorPi T1: "map this room and come back", "drive N feet and come back",
"drive around for N minutes and come back".

Two halves in one file:
  * host side (imported by grokbot_voice.py, runs as pi): parse_plan() turns words into a plan, request() checks the
    safety gates (AUTO_EXPLORE=1 AND ALLOW_MOTION=1 AND gb-guard active AND LiDAR alive), starts slam_toolbox through
    gb_mapping (explore missions), optionally starts a speed-capped Nav2 stack, launches the ROS node inside the
    MentorPi container as user ubuntu, supervises it (heartbeats both ways), saves the map, and announces the result.
  * node side (python3 gb_explore.py --node ..., inside the container): rclpy node that drives /controller/cmd_vel.
    Every command passes send(): hard caps 1.0 m/s (autonomous) and 0.64 rad/s, LiDAR front/rear stop, stale scan/odom -> zero.
    Return-to-start uses the SLAM map frame (TF map->base_link, IMU yaw) with live LiDAR avoidance;
    command-odometry breadcrumb retrace is a last resort only when no map pose is available.
    Abort file / SIGINT / SIGTERM / stale host heartbeat -> repeated zero Twist. A bash wrapper runs
    "--zero" after the node exits for any reason (even SIGKILL), and the host also sends zero bursts over rosbridge.
Dry run (--dry): identical logic, but nothing is published on /controller/cmd_vel; planned commands are logged.
"""
import json, math, os, re, signal, subprocess, sys, threading, time

import gb_paths
E = os.environ.get
HARD_MAX_LIN, HARD_MAX_ANG = 1.0, 0.64                         # autonomous hard cap; default cruise stays ~0.10-0.12
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
ARMED_CONT = "/home/ubuntu/shared/gb_safety/ARMED"            # 2026-11-25 wake-phrase arming flag (gb_arm.py)
ESTOP_HOST = gb_paths.host_estop()

class _GateRoute:
    """Publisher shim: every command -> gate stream; zero commands ALSO straight to the controller (always allowed)."""
    def __init__(self, gate, direct): self.gate, self.direct = gate, direct
    def publish(self, m):
        if (m.linear.x or m.angular.z or m.linear.y) and not os.path.exists(ARMED_CONT):   # 2026-11-25 arm
            if not getattr(self, "_disarmed", False):
                self._disarmed = True; print(_now(), "[ARM] disarmed: zeroing motion, aborting mission", flush=True)
                try:                                         # 2026-11-25 noestop: keep a floor-stop cause already written
                    if not os.path.exists(CONT_DIR + "/abort"): open(CONT_DIR + "/abort", "w").write("disarmed")
                except Exception: pass
            m = type(m)()
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


_EXPLORE_NEAR_MISS = (
    (re.compile(r"\bit'?s\s+florida(?:\s+the)?\s+room\b", re.I), "explore the room"),
    (re.compile(r"\bflorida(?:\s+the)?\s+room\b", re.I), "explore the room"),
    (re.compile(r"\bexpire(?:s|d)?(?:\s+the|\s+this|\s+my|\s+our)?\s+room\b", re.I), "explore the room"),
    (re.compile(r"\bit'?s\s+flora(?:\s+the)?\s+room\b", re.I), "explore the room"),
)
_WAKE_LEAD = re.compile(
    r"^(?:(?:hey|ok|okay|hi)\s+)?(?:wally|wallie|walley)\s*[,: ]+|"
    r"^(?:wake\s+up\s+wally|wally\s+wake\s+up|hey\s+wally)\s*[,: ]+",
    re.I,
)


def normalize_explore_command(text):
    """Strip a leading wake phrase and rewrite high-confidence Whisper near-misses of 'explore the room'.

    Conservative on purpose: 'more of the room' is not rewritten (too ambiguous).
    """
    t = (text or "").strip()
    t = _WAKE_LEAD.sub("", t, count=1).strip()
    for rx, repl in _EXPLORE_NEAR_MISS:
        t = rx.sub(repl, t)
    return t


def parse_plan(text):
    """Local parser. Returns a plan dict or None (not a mission)."""
    t = (text or "").lower().strip().rstrip(".!?")
    t = normalize_explore_command(t)
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
    # 2026-11-25 navfix: "drive around the room" / "navigate (around)" / "go around on your own" = explore (John 12:29)
    navv = bool(re.search(r"\bnavigat", n)) or (verb and bool(re.search(
        r"\b(around (?:the |this |my |our )?(?:room|house|place|area|space|office|floor)|on your own|by yourself|"
        r"autonomous(?:ly)?)\b", n)))
    if navv and not dm and not tm and not re.search(r"\b(to|toward|towards) (?:the |my |your )?\w+$", n):
        expl = True
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

# 2026-11-26 explorefix: host-side node watchdog. After Popen, wait HB_STARTUP_GRACE_S
# (15 s) before the tight 2 s heartbeat timeout begins so wait_inputs / SLAM posegraph
# load can spike CPU without a false "node heartbeat stale" abort. "finishing" keeps 8 s.
# The 25 s "node never started" guard is unchanged.
HB_STARTUP_GRACE_S = 15.0
HB_STALE_S = 2.0
HB_FINISH_STALE_S = 8.0
HB_NEVER_STARTED_S = 25.0

def clear_node_heartbeat(host_dir):
    """Remove leftover node hb files so the host watchdog cannot see a previous run's timestamp."""
    removed = []
    for fn in ("hb", "hb.tmp"):
        p = os.path.join(host_dir, fn)
        try:
            os.remove(p)
            removed.append(fn)
        except OSError:
            pass
    return removed

def host_heartbeat_stale(now, hb_t, phase, t0, seen_hb=True,
                         grace_s=None, stale_s=None, finish_s=None):
    """True when the node heartbeat should abort the mission.

    During the first grace_s seconds after Popen (default 15), the 2 s timeout is
    not applied. After the grace period, driving phases use 2 s and 'finishing'
    uses 8 s. The 25 s 'never started' guard is separate (seen_hb is False).
    """
    if not seen_hb:
        return False
    grace_s = HB_STARTUP_GRACE_S if grace_s is None else float(grace_s)
    stale_s = HB_STALE_S if stale_s is None else float(stale_s)
    finish_s = HB_FINISH_STALE_S if finish_s is None else float(finish_s)
    try:
        now = float(now); hb_t = float(hb_t); t0 = float(t0)
    except (TypeError, ValueError):
        return True
    if now - t0 < grace_s:
        return False
    limit = finish_s if (phase or "") == "finishing" else stale_s
    return (now - hb_t) > limit

def report_abort_reason(why):
    """Spoken/report reason for an aborted mission. Watchdog aborts stay watchdog, not 'a stop command'."""
    w = (why or "").strip() or "an unknown reason"
    if "watchdog" in w.lower():
        m = re.search(r"watchdog:\s*([^)]+?)(?:\)\s*)?$", w, re.I)
        if m:
            detail = m.group(1).strip()
            return detail if detail.lower().startswith("watchdog") else ("watchdog: " + detail)
        return w
    w = re.sub(r"^stop command.*", "a stop command", w, flags=re.I)
    w = re.sub(r"^signal \d+", "a stop command", w, flags=re.I)
    return w

def _hb_write(path):
    """2026-11-24 hbatomic: write the host heartbeat atomically (tmp + os.replace). The old open(path, "w") truncated
    the file before writing, so the container node could read an empty file (float("") -> ValueError) and abort with
    'host watchdog (no heartbeat file)' while the host was alive (run 2026-11-24 03:27:51 PT). Timeouts unchanged."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f: f.write(str(time.time()))
    os.replace(tmp, path)

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

def request(plan, dry=False, store=None, announce=None, cli_dry=False, actor=None):
    """Gate + start a mission in the background. Returns the immediate spoken reply."""
    desc = describe_plan(plan)
    _hlog(f"request dry={dry} plan={json.dumps(plan)}")
    if os.path.exists(ESTOP_HOST):
        _hlog("refused: ESTOP latched"); return "I'm emergency-latched, so I won't start a mission. Say resume first."
    if not dry:                                       # 2026-11-25 arm: no mission until "wake up Wally"
        import gb_arm
        if not gb_arm.is_armed():
            _hlog("refused: disarmed (say wake up Wally)"); return gb_arm.refuse("explore mission", plan)
    apply_learned_params(log=True)
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
    if not dry:
        try:
            import gb_arm
            owner = gb_arm.normalize_actor(actor) if actor else "voice"
            gb_arm.set_mission_lease(owner=owner, pid=os.getpid(), reason="explore request",
                                     mission=(plan or {}).get("mission"))
        except Exception as _ex:
            _hlog("mission lease set failed:", _ex)
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
    try:                                                   # 2026-11-27 fusion stuck-recovery module next to the node
        fsrc = os.path.join(os.path.dirname(me), "gb_fusion_recovery.py"); fdst = os.path.join(HOST_DIR, "gb_fusion_recovery.py")
        if os.path.exists(fsrc):
            fb = open(fsrc).read()
            if not os.path.exists(fdst) or open(fdst).read() != fb:
                with open(fdst + ".tmp", "w") as f: f.write(fb)
                os.replace(fdst + ".tmp", fdst)
    except Exception as _fx:
        _hlog("fusion module copy failed (LiDAR-only recovery):", _fx)
    try:
        import gb_caps; m = gb_caps.speed_mult()
    except Exception:
        m = 1.0
    with open(os.path.join(HOST_DIR, "speed.json"), "w") as f: json.dump({"mult": m}, f)
    _hlog(f"speed setting {int(m * 100)}% for this mission")
    try:
        _def = reset_explore_speed_target()
        _hlog("explore cruise reset to default %.3f m/s (hard cap %.3f)" % (_def, HARD_MAX_LIN))
    except Exception as _ex:
        _hlog("explore cruise reset skipped:", _ex)
    # 2026-11-25 learnapply: give the container node the same approved file (host path is not mounted there)
    try:
        for _lp in _learned_param_paths():
            if _lp.startswith(HOST_DIR):
                continue
            if os.path.isfile(_lp):
                _dst = os.path.join(HOST_DIR, "applied_params.json")
                _txt = open(_lp, encoding="utf-8").read()
                with open(_dst + ".tmp", "w", encoding="utf-8") as _f: _f.write(_txt)
                os.replace(_dst + ".tmp", _dst)
                break
    except Exception as _ex:
        _hlog("applied_params copy skipped:", _ex)
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
                ok, msg = gb_mapping.start(autonomous=True); started_slam = ok   # 2026-11-23 fix 3: honest message
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
        _hb_write(os.path.join(HOST_DIR, "host_hb"))   # 2026-11-24 hbatomic: atomic, no torn reads
        args = f"--node --plan '{json.dumps(plan)}'" + (" --dry" if dry else "") + (" --nav2" if nav2 else "")
        cmd = (f"{ROSRUN} python3 {CONT_DIR}/gb_explore.py {args}; rc=$?; "
               f"{ROSRUN} python3 {CONT_DIR}/gb_explore.py --zero" + (" --dry" if dry else "") + "; exit $rc")
        logf = open(os.path.join(os.path.dirname(_logpath()), "explore_ros.log"), "ab")
        logf.write(f"\n===== {_now()} {args}\n".encode()); logf.flush()
        # 2026-11-26 explorefix: drop any leftover hb from before/during SLAM start so the
        # watchdog cannot treat a previous run's timestamp as the current node heartbeat.
        clear_node_heartbeat(HOST_DIR)
        proc = subprocess.Popen(["docker", "exec", "-u", "ubuntu", CONTAINER, "bash", "-lc", cmd], stdout=logf,
                                stderr=logf, stdin=subprocess.DEVNULL, start_new_session=True)
        _S["proc"] = proc
        hbf = os.path.join(HOST_DIR, "hb"); hhb = os.path.join(HOST_DIR, "host_hb")
        limit = _plan_limit(plan) + 120; t0 = time.time(); seen_hb = False; wd = None
        abort_file = os.path.join(HOST_DIR, "abort")
        def _aborting(): return _S["abort"].is_set() or os.path.exists(abort_file)
        while proc.poll() is None:
            _hb_write(hhb)                         # 2026-11-24 hbatomic
            try:
                st = json.load(open(hbf)); seen_hb = True; _S["phase"] = st.get("phase", "?")
                if wd is None and not _aborting() and host_heartbeat_stale(
                        time.time(), st.get("t"), st.get("phase"), t0, seen_hb=True):
                    wd = "node heartbeat stale"
            except Exception:
                if time.time() - t0 > HB_NEVER_STARTED_S and not seen_hb: wd = "node never started"
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
                try:
                    import gb_arm
                    res.update(gb_arm.session_abort_fields())
                except Exception:
                    pass
            else:
                res = {"outcome": "error", "reason": wd or "node exited without a result", "phases": ["?"] if seen_hb else []}
        if wd:
            if not isinstance(res, dict):
                res = {}
            res["outcome"] = "aborted"
            res["reason"] = "watchdog: " + wd
        _hlog("node result:", json.dumps(res)[:1500])
        report = _report(plan, res, dry)
    except Exception as ex:
        _hlog("mission error:", repr(ex)); report = f"My mission failed: {ex}. I stopped."
        if not dry: _zero_host()
    finally:
        try:
            import gb_arm
            gb_arm.clear_mission_lease("mission end")
        except Exception:
            pass
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
        why = report_abort_reason(why)
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

def _gate_code_dir():
    """2026-11-17 allguard: folder of the gb_gate.py the running gate was started from (same container), so the node
    imports the SAME gb_obstacle as the gate instead of the stale /home/ubuntu/shared/gb_safety copy."""
    d = E("GB_OBSTACLE_DIR")
    if d and os.path.exists(os.path.join(d, "gb_obstacle.py")): return d
    try: pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError: return None
    for p in pids:
        try: args = open("/proc/%s/cmdline" % p, "rb").read().split(b"\0")
        except OSError: continue
        for a in args:
            a = a.decode("utf-8", "replace")
            if a.endswith("/gb_gate.py") and os.path.exists(os.path.join(os.path.dirname(a), "gb_obstacle.py")):
                return os.path.dirname(a)
    return None

BOXED_S = float(E("EXPLORE_BOXED_S", "3.0"))     # front, reverse and turn all blocked this long -> end the mission
STUCK_S = float(E("EXPLORE_STUCK_S", "8.0"))     # every nonzero command refused this long -> end the mission
# 2026-11-23 fix 3 (tripstop): after a gate trip the blocked direction is NOT re-commanded for TRIP_HOLD_S.
# 2026-11-27: TRIP_LIMIT / TRIP_WIN_S are kept for logging only. Transient obstacle / turn-sweep /
# soft-stall stops reposition and resume; they do not hard-abort the mission. A hard abort fires only
# after REPOSITION_FAIL_LIMIT consecutive failed repositions with no progress, or a real ESTOP/fault.
TRIP_HOLD_S = float(E("EXPLORE_TRIP_HOLD_S", "4.0")); TRIP_LIMIT = int(E("EXPLORE_TRIP_LIMIT", "3"))
TRIP_WIN_S = float(E("EXPLORE_TRIP_WIN_S", "20.0"))

# 2026-11-26 John (voice): when an object in front triggers a stop during explore/mapping, wait 3 s,
# recheck, resume the current task if the way is safely clear, otherwise stay stopped (do not force motion).
# He first said 5 s, then refined to the 3-second variant. BOXED_S is already 3 s but that ends a boxed-in
# mission; this wait is a recheck, not an abort. gb_gate stops 0.27/0.28/0.24 stay untouched.
OBSTACLE_RECHECK_WAIT_S = float(E("EXPLORE_OBSTACLE_RECHECK_S", "3.0"))

def is_front_obstacle_trip(why):
    """True when the gate trip is an object ahead, not a tilt/jolt/stall recovery case."""
    t = (why or "").lower()
    if any(k in t for k in ("tilt", "jolt", "track stall", "no progress")):
        return False
    return ("ahead" in t) or ("obstacle" in t) or ("blind ahead" in t)

def obstacle_front_is_clear(front_m, avoid_dist_m, front_stop_m, estop=False, abort=False, armed=True):
    """Safely clear to resume explore: armed, no estop/stop, and front at/above avoid and the untouched gate stop.
    Does not change gb_gate stops (0.27 / 0.28 / 0.24)."""
    if estop or abort or not armed:
        return False
    try:
        front = float(front_m)
    except (TypeError, ValueError):
        return False
    if front != front:  # NaN
        return False
    need = max(float(avoid_dist_m), float(front_stop_m))
    return front >= need

def obstacle_recheck_decision(waited_s, wait_s, front_m, avoid_dist_m, front_stop_m,
                              estop=False, abort=False, armed=True):
    """After a front obstacle stop: 'wait' until wait_s, then 'resume' if clear else 'stay_stopped'."""
    if float(waited_s) < float(wait_s):
        return "wait"
    if obstacle_front_is_clear(front_m, avoid_dist_m, front_stop_m, estop=estop, abort=abort, armed=armed):
        return "resume"
    return "stay_stopped"


# --- BEGIN universal obstacle avoidance (2026-11-26 John) ---
# Used by EVERY driving mode (explore, wander, retrace/return-to-start, distance/waypoints).
# Sequence: stop, wait OBSTACLE_RECHECK_WAIT_S (3.0 s), recheck live front/corridor;
# resume if clear and armed with no ESTOP/stop; else guarded turn toward the clearer
# LiDAR side at explore_turn_ang (MAX_ANG), short 0.4-0.6 m sidestep while the corridor
# stays clear, and in retrace skip an unreachable breadcrumb. Bounded retry cap: if
# boxed in, halt safely and never force motion. gb_gate stops 0.27/0.28/0.24 untouched.
OBSTACLE_SIDESTEP_M = float(E("EXPLORE_OBSTACLE_SIDESTEP_M", "0.50"))
OBSTACLE_AVOID_RETRY_CAP = int(float(E("EXPLORE_OBSTACLE_AVOID_RETRIES", "3")))

def clearer_side(left_clear_m, right_clear_m, tie="left"):
    """Return 'left' or 'right' from live LiDAR side clearance."""
    try:
        L = float(left_clear_m)
    except (TypeError, ValueError):
        L = float("nan")
    try:
        R = float(right_clear_m)
    except (TypeError, ValueError):
        R = float("nan")
    L_ok, R_ok = (L == L), (R == R)
    if L_ok and R_ok:
        if L > R: return "left"
        if R > L: return "right"
        return tie
    if L_ok: return "left"
    if R_ok: return "right"
    return tie

def side_is_usable(clear_m, front_stop_m, open_min_m=0.40):
    try:
        v = float(clear_m)
    except (TypeError, ValueError):
        return False
    if v != v:
        return False
    return v >= max(float(front_stop_m), float(open_min_m))

def corridor_is_clear(corridor_m, front_m, avoid_dist_m, front_stop_m):
    need = max(float(avoid_dist_m), float(front_stop_m))
    try:
        f = float(front_m)
        c = float(corridor_m) if corridor_m is not None else f
    except (TypeError, ValueError):
        return False
    if f != f or c != c:
        return False
    return min(c, f) >= need

def next_clear_breadcrumb(index, n_points, blocked_indices=None):
    """Next breadcrumb index after `index` that is not marked blocked, or None."""
    blocked = set(blocked_indices or [])
    for j in range(int(index) + 1, int(n_points)):
        if j not in blocked:
            return j
    return None

def begin_obstacle_wait(state, now, wait_s=None):
    st = dict(state or {})
    st["obstacle_wait_start"] = float(now)
    st["obstacle_wait_until"] = float(now) + float(wait_s if wait_s is not None else OBSTACLE_RECHECK_WAIT_S)
    st["obstacle_stay_stopped"] = False
    return st

def universal_obstacle_decision(waited_s, wait_s, front_m, avoid_dist_m, front_stop_m,
                                left_clear_m=None, right_clear_m=None, corridor_m=None,
                                estop=False, abort=False, armed=True,
                                retries=0, retry_cap=None, boxed_in=False,
                                mode="explore", more_breadcrumbs=False):
    """Unified decision for explore, retrace, and distance/waypoint modes.

    Returns: wait, resume, sidestep_left, sidestep_right, skip_breadcrumb, stay_stopped, boxed_in.
    After the retry cap or when boxed in, never returns a motion action.
    """
    if retry_cap is None:
        retry_cap = OBSTACLE_AVOID_RETRY_CAP
    if float(waited_s) < float(wait_s):
        return "wait"
    if estop or abort or not armed:
        return "stay_stopped"
    if obstacle_front_is_clear(front_m, avoid_dist_m, front_stop_m,
                               estop=estop, abort=abort, armed=armed) and \
            corridor_is_clear(corridor_m if corridor_m is not None else front_m,
                              front_m, avoid_dist_m, front_stop_m):
        return "resume"
    # Still blocked after the 3 s recheck.
    if boxed_in or int(retries) >= int(retry_cap):
        return "boxed_in"
    left_ok = side_is_usable(left_clear_m, front_stop_m)
    right_ok = side_is_usable(right_clear_m, front_stop_m)
    retrace = (mode or "") in ("retrace", "return", "go_to_points")
    if retrace and more_breadcrumbs and not left_ok and not right_ok:
        return "skip_breadcrumb"
    if left_ok or right_ok:
        side = clearer_side(left_clear_m if left_ok else None,
                            right_clear_m if right_ok else None)
        if side == "right" and right_ok:
            return "sidestep_right"
        if side == "left" and left_ok:
            return "sidestep_left"
        return "sidestep_right" if right_ok else "sidestep_left"
    if retrace and more_breadcrumbs:
        return "skip_breadcrumb"
    return "stay_stopped"
# --- END universal obstacle avoidance ---


def trip_dirs(why):
    """Which motion directions a gate trip reason blocks: subset of {'fwd', 'rev', 'turn'}."""
    t = (why or "").lower(); d = set()
    if "turn sweep" in t or "turn refused" in t or "turn stall" in t: d.add("turn")
    if "behind" in t or "reverse refused" in t: d.add("rev")
    if "ahead" in t or "track stall" in t or "jolt" in t or "tilt" in t or "blind ahead" in t or \
            ("no progress" in t and "reversing" not in t): d.add("fwd")         # navstd: reverse no-progress holds rev only
    if not d and ("bump" in t or "obstacle" in t): d.update(("fwd", "turn"))
    return d

BACKOFF_MARGIN = 0.05            # navstd: recovery back-off only with rear >= rear stop + 5 cm and rear LiDAR not blind

# 2026-11-27: after a transient obstacle / turn-sweep / soft-stall stop, automatically
# reposition (gated back-away + open-side turn) and resume exploration. If a single
# reposition does not free Wall-E (corner / tight spot), keep making continuous
# small, slow multi-direction nudges — gentle reverse / arc / left / right micro-moves
# guided by live LiDAR sector clearance — WITHOUT coming to a full stop and waiting
# between nudges, until a path opens, then resume exploration.
# Transient stops do NOT count toward a hard abort, even if more than TRIP_LIMIT occur
# in TRIP_WIN_S. A hard abort fires only after REPOSITION_FAIL_LIMIT consecutive
# failed nudge cycles with no meaningful clearance/odometry progress, or a real
# ESTOP/hardware fault. All velocity commands still go through send() -> gb_gate
# (stops 0.27/0.28/0.24 and caps 0.288/1.0 untouched).
REPOSITION_BACK_V = -0.07          # m/s; in the requested [-0.08, -0.06] band (send() may lift to lin_floor 0.08)
REPOSITION_BACK_S = 1.2
REPOSITION_TURN_S = 1.5
REPOSITION_CLEAR_WAIT_S = 0.5      # unused as a dwell; kept for env/compat only
REPOSITION_FAIL_LIMIT = 5          # consecutive failed nudge/reposition cycles with no progress -> hard abort
REPOSITION_PROGRESS_M = 0.05       # odom travel or front-clearance gain that counts as success
REPOSITION_SWEEP_NEED_M = 0.24     # must match the untouched turn-sweep stop; never weaken it

# Continuous micro-nudges (John 2026-11-27). Linear stays in 0.06-0.07 m/s so it
# is safely above STALL_MIN_CMD = 0.05 (gb_obstacle.py) and cannot false-stall,
# and stays well under the 0.288 manual / 1.0 autonomous caps. Angular is kept
# low and always paired with that linear component (no in-place creep).
NUDGE_LIN = 0.065
NUDGE_LIN_MIN = 0.06
NUDGE_LIN_MAX = 0.07
NUDGE_ANG = 0.20
NUDGE_S = 0.40
STALL_MIN_CMD_NUDGE = 0.05         # mirror of gb_obstacle.STALL_MIN_CMD; do not command below this


def is_hard_fault_trip(why):
    """True for ESTOP / hardware / power faults that must end the mission, not be retried."""
    t = (why or "").lower()
    return any(k in t for k in (
        "estop", "e-stop", "e stop", "emergency stop",
        "hardware", "fault", "low voltage", "low-voltage", "low batt",
    ))


def is_transient_obstacle_trip(why):
    """Front obstacle stop, turn-sweep guard block, or soft stall — recover, don't abort."""
    t = (why or "").lower()
    if is_hard_fault_trip(why):
        return False
    if any(k in t for k in ("tilt", "jolt")):
        return False
    if "turn sweep" in t or "turn refused" in t or "turn stall" in t:
        return True
    if "track stall" in t or "soft stall" in t or "no progress" in t or "stall" in t:
        return True
    if is_front_obstacle_trip(why):
        return True
    if "behind" in t or "reverse refused" in t:
        return True
    if "obstacle" in t or "bump" in t:
        return True
    return False


def handle_explore_trip(why):
    """Classify a new gate trip for the explore loop.

    Returns: 'hard_abort' | 'reposition' | 'bump_recover' | 'hold'
    Transient obstacle/turn-sweep/soft-stall trips always 'reposition' (never a
    3-in-20s abort). Tilt/jolt keep the existing bump-recovery path.
    """
    if is_hard_fault_trip(why):
        return "hard_abort"
    if is_transient_obstacle_trip(why):
        return "reposition"
    t = (why or "").lower()
    if any(k in t for k in ("bump", "tilt", "stall", "obstacle", "jolt")):
        return "bump_recover"
    return "hold"


def rear_safe_to_back(rear_m, corr_rev_m, rear_stop_m, margin_m=BACKOFF_MARGIN,
                      blind_r=0.0, blind_frac_stop=0.5,
                      estop=False, abort=False, armed=True):
    """True when a brief gated backup is allowed: armed, no ESTOP/stop, rear and
    rear-corridor both above rear_stop + margin, and the rear LiDAR is not blind."""
    if estop or abort or not armed:
        return False
    try:
        rear = float(rear_m)
        corr = float(corr_rev_m) if corr_rev_m is not None else rear
        blind = float(blind_r or 0.0)
    except (TypeError, ValueError):
        return False
    if rear != rear or corr != corr:
        return False
    need = float(rear_stop_m) + float(margin_m)
    return rear >= need and corr >= need and blind < float(blind_frac_stop)


def open_side_dir(left_m, right_m, tie=1):
    """+1 = left, -1 = right, matching open_dir()."""
    side = clearer_side(left_m, right_m, tie="left" if tie >= 0 else "right")
    return 1 if side == "left" else -1


def _finite(v, default=0.0):
    try:
        x = float(v)
        return x if x == x else default
    except (TypeError, ValueError):
        return default


def clamp_nudge_lin(v):
    """Keep a nonzero linear nudge in [NUDGE_LIN_MIN, NUDGE_LIN_MAX], above STALL_MIN_CMD."""
    try:
        x = float(v)
    except (TypeError, ValueError):
        return 0.0
    if x == 0 or x != x:
        return 0.0
    a = min(NUDGE_LIN_MAX, max(NUDGE_LIN_MIN, abs(x)))
    if a < STALL_MIN_CMD_NUDGE:
        a = NUDGE_LIN_MIN
    return math.copysign(a, x)


def reposition_progress(start_xy, now_xy, start_front, now_front, start_sweep, now_sweep,
                        front_stop_m=0.27, sweep_need_m=REPOSITION_SWEEP_NEED_M,
                        progress_m=REPOSITION_PROGRESS_M, avoid_dist_m=None):
    """True if the maneuver gained meaningful odometry travel or clearance."""
    moved = 0.0
    try:
        moved = math.hypot(float(now_xy[0]) - float(start_xy[0]),
                           float(now_xy[1]) - float(start_xy[1]))
    except (TypeError, ValueError, IndexError):
        moved = 0.0
    if moved >= float(progress_m):
        return True
    nf, sf = _finite(now_front), _finite(start_front)
    ns, ss = _finite(now_sweep), _finite(start_sweep)
    if nf >= float(front_stop_m) and (nf - sf) >= float(progress_m):
        return True
    if ns >= float(sweep_need_m) and (ns - ss) >= 0.02:
        return True
    need = float(front_stop_m)
    if avoid_dist_m is not None:
        need = max(need, float(avoid_dist_m))
    if nf >= need and ns >= float(sweep_need_m):
        return True
    return False


def clear_enough_to_resume(front_m, sweep_m, front_stop_m=0.27,
                           sweep_need_m=REPOSITION_SWEEP_NEED_M, avoid_dist_m=None):
    """True when front and turn-sweep are back above the untouched gate stops."""
    need = float(front_stop_m)
    if avoid_dist_m is not None:
        try:
            need = max(need, float(avoid_dist_m))
        except (TypeError, ValueError):
            pass
    return _finite(front_m) >= need and _finite(sweep_m) >= float(sweep_need_m)


def reposition_abort_decision(consecutive_failures, fail_limit=REPOSITION_FAIL_LIMIT,
                              progressed=False, hard_fault=False):
    """Decide whether a finished reposition/nudge cycle should resume, retry, or hard-abort.

    A successful cycle (progressed=True) resets the consecutive-failure
    counter and does NOT count toward a hard abort. A single failed nudge or
    a single failed cycle never aborts. Returns
    (action, reason, new_failure_count) where action is 'resume' | 'retry' | 'abort'.
    """
    if hard_fault:
        return "abort", "hard fault / ESTOP", int(consecutive_failures)
    if progressed:
        return "resume", "recovered via reposition", 0
    n = int(consecutive_failures) + 1
    limit = int(fail_limit)
    if n >= limit:
        return "abort", "truly stuck after %d failed reposition attempts" % n, n
    return "retry", "reposition failed, will retry (%d/%d)" % (n, limit), n


def start_reposition(now, odom, front_m, sweep_m, open_dir_value):
    """Fresh reposition-state dict for a newly detected transient gate trip."""
    try:
        od = tuple(odom) if odom is not None else (0.0, 0.0, 0.0)
        if len(od) < 2:
            od = (0.0, 0.0, 0.0)
    except (TypeError, ValueError):
        od = (0.0, 0.0, 0.0)
    front = _finite(front_m, 0.0)
    sweep = _finite(sweep_m, 0.0)
    d = int(open_dir_value) if open_dir_value in (1, -1) else 1
    return {
        "active": True,
        "phase": "choose",
        "start": float(now),
        "start_odom": (float(od[0]), float(od[1]), float(od[2]) if len(od) > 2 else 0.0),
        "start_front": front,
        "start_sweep": sweep,
        "backed": False,
        "tried_dirs": [],
        "dir": d,
        "back_until": 0.0,
        "turn_until": 0.0,
        "clear_until": 0.0,
        "deadline": float(now) + REPOSITION_BACK_S + REPOSITION_TURN_S + 1.0,
        "fails": 0,
        "nudge_tried": [],
        "nudge_name": None,
        "nudge_until": 0.0,
        "cycle_odom": (float(od[0]), float(od[1])),
        "cycle_front": front,
        "cycle_sweep": sweep,
    }


def nudge_candidates(sensors, caps):
    """Feasible low-speed micro-nudges, ranked by live LiDAR sector clearance (best first).

    Every candidate has |lin| in [0.06, 0.07] (above STALL_MIN_CMD = 0.05). Forward
    nudges are offered only when front is at/above the untouched 0.27 m stop;
    reverse nudges only when rear is above the 0.28 m stop + margin. Angular is
    low and always paired with that linear component. The caller still routes
    every command through send() / gb_gate, which enforces the stops.
    """
    front_stop = float(caps.get("front_stop", 0.27))
    rear_stop = float(caps.get("rear_stop", 0.28))
    margin = float(caps.get("margin", BACKOFF_MARGIN))
    blind_frac = float(caps.get("blind_frac_stop", 0.5))
    lin = clamp_nudge_lin(caps.get("nudge_lin", NUDGE_LIN))
    if lin == 0:
        lin = NUDGE_LIN
    lin = clamp_nudge_lin(lin if lin > 0 else -lin)
    ang = abs(_finite(caps.get("nudge_ang", NUDGE_ANG), NUDGE_ANG))
    front = _finite(sensors.get("front"))
    rear = _finite(sensors.get("rear"))
    corr_rev = _finite(sensors.get("corr_rev"), rear)
    left = _finite(sensors.get("left"))
    right = _finite(sensors.get("right"))
    can_back = rear_safe_to_back(
        rear, corr_rev, rear_stop, margin, sensors.get("blind_r", 0.0), blind_frac,
        estop=caps.get("estop", False), abort=caps.get("abort", False),
        armed=caps.get("armed", True))
    can_fwd = front >= front_stop
    cands = []
    if can_back:
        score = min(rear, corr_rev)
        # Prefer a straight back in a tight corner to gain turn-sweep room, then arcs
        # toward the more open side.
        cands.append({"name": "back", "lin": -lin, "ang": 0.0, "score": score + 0.15})
        cands.append({"name": "back_left", "lin": -lin, "ang": ang, "score": score + 0.05 + left})
        cands.append({"name": "back_right", "lin": -lin, "ang": -ang, "score": score + 0.05 + right})
    if can_fwd:
        cands.append({"name": "arc_left", "lin": lin, "ang": ang, "score": front + left})
        cands.append({"name": "arc_right", "lin": lin, "ang": -ang, "score": front + right})
        cands.append({"name": "fwd", "lin": lin, "ang": 0.0, "score": front * 0.5})
    cands.sort(key=lambda c: (-c["score"], c["name"]))
    return cands


def _nudge_holds(cand, sensors, caps):
    """Release explore-side trip holds only for directions that are actually clear."""
    pop = []
    front_stop = float(caps.get("front_stop", 0.27))
    sweep_need = float(caps.get("sweep_need", REPOSITION_SWEEP_NEED_M))
    if cand.get("lin", 0) < 0:
        pop.append("rev")
    elif cand.get("lin", 0) > 0 and _finite(sensors.get("front")) >= front_stop:
        pop.append("fwd")
    if cand.get("ang", 0) != 0 and _finite(sensors.get("sweep")) >= sweep_need:
        pop.append("turn")
    return pop


def _cmd(event, lin=0.0, ang=None, ang_dir=0, why="", pop_holds=None, fails=0, reason=""):
    return {
        "event": event, "lin": float(lin), "ang": ang, "ang_dir": int(ang_dir or 0),
        "why": why, "pop_holds": list(pop_holds or []), "fails": int(fails), "reason": reason,
    }


def _nudge_tick(rp, now, sensors, caps):
    """One tick of continuous multi-direction nudging. Never inserts a zero-velocity
    dwell between two feasible nudges. A zero command is returned only when no
    legal move exists (both ends below the untouched gate stops) or we are
    recovering / aborting / disarmed."""
    front_stop = float(caps.get("front_stop", 0.27))
    sweep_need = float(caps.get("sweep_need", REPOSITION_SWEEP_NEED_M))
    progress_m = float(caps.get("progress_m", REPOSITION_PROGRESS_M))
    avoid_dist = caps.get("avoid_dist")
    fail_limit = int(caps.get("fail_limit", REPOSITION_FAIL_LIMIT))
    nudge_s = float(caps.get("nudge_s", NUDGE_S))
    fails = int(rp.get("fails") or 0)
    odom = sensors.get("odom") or rp.get("start_odom") or (0.0, 0.0, 0.0)
    front = sensors.get("front")
    sweep_f = _finite(sensors.get("sweep"))

    def _progress(base_xy, base_front, base_sweep):
        return reposition_progress(
            base_xy, odom, base_front, front, base_sweep, sweep_f,
            front_stop, sweep_need, progress_m, avoid_dist)

    # If the corridor is open again, resume exploration immediately (one settling
    # zero at the transition to explore is OK; it is not a between-nudge dwell).
    if clear_enough_to_resume(front, sweep_f, front_stop, sweep_need, avoid_dist) or (
            _progress(rp.get("start_odom"), rp.get("start_front"), rp.get("start_sweep"))
            and _finite(front) >= front_stop and sweep_f >= sweep_need):
        rp["fails"] = 0
        rp["active"] = False
        rp["phase"] = "done"
        return _cmd("recovered", why="reposition: recovered, resume explore",
                    pop_holds=["fwd", "turn"], fails=0, reason="recovered via reposition")

    cands = nudge_candidates(sensors, caps)
    tried = list(rp.get("nudge_tried") or [])
    current = rp.get("nudge_name")
    until = float(rp.get("nudge_until") or 0)
    cur = next((c for c in cands if c["name"] == current), None)

    # Keep commanding the current feasible nudge until its slice ends — no stop.
    if cur and now < until:
        return _cmd("running", lin=cur["lin"], ang=cur["ang"],
                    why="nudge %s" % cur["name"],
                    pop_holds=_nudge_holds(cur, sensors, caps),
                    fails=fails, reason="nudging")

    # Slice ended or current direction became illegal: pick the next untried
    # candidate immediately (no zero-velocity gap).
    nxt = next((c for c in cands if c["name"] not in tried), None)
    if nxt is None:
        # A full multi-direction cycle finished (or no legal candidates).
        cyc_xy = rp.get("cycle_odom") or rp.get("start_odom")
        progressed = _progress(cyc_xy, rp.get("cycle_front"), rp.get("cycle_sweep"))
        if progressed:
            # Reset the consecutive-failure counter and keep nudging toward the
            # opening, still without a dwell, unless we can already resume
            # (handled above).
            rp["fails"] = 0
            fails = 0
            rp["nudge_tried"] = []
            rp["cycle_odom"] = (float(odom[0]), float(odom[1])) if odom is not None else cyc_xy
            rp["cycle_front"] = front
            rp["cycle_sweep"] = sweep_f
            nxt = cands[0] if cands else None
            if nxt is not None:
                rp["nudge_name"] = nxt["name"]
                rp["nudge_until"] = float(now) + nudge_s
                rp["nudge_tried"] = [nxt["name"]]
                return _cmd("running", lin=nxt["lin"], ang=nxt["ang"],
                            why="nudge %s (progress, continue)" % nxt["name"],
                            pop_holds=_nudge_holds(nxt, sensors, caps),
                            fails=0, reason="recovered via reposition")
        action, reason, n = reposition_abort_decision(
            fails, fail_limit, progressed=False,
            hard_fault=bool(caps.get("estop") or caps.get("abort") or caps.get("hard_fault")))
        rp["fails"] = n
        if action == "abort":
            rp["active"] = False
            rp["phase"] = "aborted"
            return _cmd("abort", why="reposition: " + reason, fails=n, reason=reason)
        # New cycle immediately — no stop if a legal nudge exists.
        rp["nudge_tried"] = []
        rp["cycle_odom"] = (float(odom[0]), float(odom[1])) if odom is not None else cyc_xy
        rp["cycle_front"] = front
        rp["cycle_sweep"] = sweep_f
        if not cands:
            # Genuinely boxed: no legal lin in either direction. Holding zero is
            # a safety stop, not a dwell between nudges. Re-check next slice.
            rp["nudge_name"] = None
            rp["nudge_until"] = float(now) + nudge_s
            return _cmd("retry", why="nudge: no legal move this cycle (%s)" % reason,
                        fails=n, reason=reason)
        nxt = cands[0]
        rp["nudge_name"] = nxt["name"]
        rp["nudge_until"] = float(now) + nudge_s
        rp["nudge_tried"] = [nxt["name"]]
        return _cmd("retry", lin=nxt["lin"], ang=nxt["ang"],
                    why="nudge %s (%s)" % (nxt["name"], reason),
                    pop_holds=_nudge_holds(nxt, sensors, caps),
                    fails=n, reason=reason)

    rp["nudge_name"] = nxt["name"]
    rp["nudge_until"] = float(now) + nudge_s
    rp["nudge_tried"] = tried + [nxt["name"]]
    return _cmd("running", lin=nxt["lin"], ang=nxt["ang"],
                why="nudge %s" % nxt["name"],
                pop_holds=_nudge_holds(nxt, sensors, caps),
                fails=fails, reason="nudging")


def _enter_nudge(rp, now, sensors, caps, fails=None):
    """Switch into continuous nudge mode and emit the first (nonzero) nudge now."""
    if fails is not None:
        rp["fails"] = int(fails)
    rp["phase"] = "nudge"
    rp["nudge_tried"] = []
    rp["nudge_name"] = None
    rp["nudge_until"] = 0.0
    odom = sensors.get("odom") or rp.get("start_odom") or (0.0, 0.0, 0.0)
    rp["cycle_odom"] = (float(odom[0]), float(odom[1])) if odom is not None and len(odom) >= 2 else (0.0, 0.0)
    rp["cycle_front"] = sensors.get("front", rp.get("start_front"))
    rp["cycle_sweep"] = sensors.get("sweep", rp.get("start_sweep"))
    return _nudge_tick(rp, now, sensors, caps)


def _rp_finish(rp, now, sensors, caps, progressed, fails_in):
    """Shared end-of-attempt evaluation. Retries enter nudge immediately (no dwell)."""
    hard = bool(caps.get("estop") or caps.get("abort") or caps.get("hard_fault"))
    front_stop = float(caps.get("front_stop", 0.27))
    sweep_need = float(caps.get("sweep_need", REPOSITION_SWEEP_NEED_M))
    if progressed and clear_enough_to_resume(
            sensors.get("front"), sensors.get("sweep"), front_stop, sweep_need, caps.get("avoid_dist")):
        action, reason, n = "resume", "recovered via reposition", 0
    elif progressed:
        # Made progress but not yet clear enough to cruise — keep nudging, reset fails.
        return _enter_nudge(rp, now, sensors, caps, fails=0)
    else:
        action, reason, n = reposition_abort_decision(
            fails_in, int(caps.get("fail_limit", REPOSITION_FAIL_LIMIT)),
            progressed=False, hard_fault=hard)
    rp["fails"] = n
    if action == "resume":
        rp["active"] = False
        rp["phase"] = "done"
        return _cmd("recovered", why="reposition: recovered, resume explore",
                    pop_holds=["fwd", "turn"], fails=0, reason=reason)
    if action == "abort":
        rp["active"] = False
        rp["phase"] = "aborted"
        return _cmd("abort", why="reposition: " + reason, fails=n, reason=reason)
    return _enter_nudge(rp, now, sensors, caps, fails=n)


def reposition_step(rp, now, sensors, caps):
    """One 10 Hz tick of the gated reposition / continuous-nudge maneuver (pure; no I/O).

    sensors: front, rear, corr_rev, sweep, left, right, blind_r, odom,
             turn_ok (bool), rev_ok (bool)
    caps: front_stop, rear_stop, sweep_need, back_v, back_s, turn_s, clear_s,
          margin, blind_frac_stop, progress_m, fail_limit, armed, estop, abort,
          hard_fault, avoid_dist, nudge_lin, nudge_ang, nudge_s
    Returns dict with event, lin, ang (optional rad/s), ang_dir, why, pop_holds,
    fails, reason. The caller must route lin/ang through send() so gb_gate stays
    in the loop. Between nudges this never requests a full stop when a legal
    nudge still exists.
    """
    if not rp:
        return _cmd("stay_stopped", why="reposition: no state", reason="no state")
    if caps.get("estop") or caps.get("abort") or not caps.get("armed", True):
        rp["active"] = False
        return _cmd("stay_stopped", why="reposition: stay stopped (estop/abort/disarm)",
                    fails=int(rp.get("fails") or 0), reason="estop/abort/disarmed")

    front = sensors.get("front")
    rear = sensors.get("rear")
    corr_rev = sensors.get("corr_rev")
    sweep = sensors.get("sweep")
    left = sensors.get("left")
    right = sensors.get("right")
    odom = sensors.get("odom") or rp.get("start_odom") or (0.0, 0.0, 0.0)
    turn_ok = bool(sensors.get("turn_ok", False))
    rear_stop = float(caps.get("rear_stop", 0.28))
    front_stop = float(caps.get("front_stop", 0.27))
    sweep_need = float(caps.get("sweep_need", REPOSITION_SWEEP_NEED_M))
    margin = float(caps.get("margin", BACKOFF_MARGIN))
    blind_frac = float(caps.get("blind_frac_stop", 0.5))
    back_v = float(caps.get("back_v", REPOSITION_BACK_V))
    if back_v > 0:
        back_v = -abs(back_v)
    back_v = max(-0.08, min(-0.06, back_v))
    back_s = float(caps.get("back_s", REPOSITION_BACK_S))
    turn_s = float(caps.get("turn_s", REPOSITION_TURN_S))
    progress_m = float(caps.get("progress_m", REPOSITION_PROGRESS_M))
    avoid_dist = caps.get("avoid_dist")
    fails = int(rp.get("fails") or 0)
    sweep_f = _finite(sweep)

    can_back = rear_safe_to_back(
        rear, corr_rev, rear_stop, margin, sensors.get("blind_r", 0.0), blind_frac,
        estop=caps.get("estop", False), abort=caps.get("abort", False),
        armed=caps.get("armed", True))
    if not can_back and not sensors.get("rev_ok", True):
        can_back = False

    d = rp.get("dir")
    if d not in (1, -1):
        d = open_side_dir(left, right)
        rp["dir"] = d

    phase = rp.get("phase") or "choose"

    if phase == "nudge":
        return _nudge_tick(rp, now, sensors, caps)

    # Deadline only applies to the initial back/turn attempt; nudging is bounded
    # by failed-cycle count instead, so we are not forced to stop mid-nudge.
    if phase not in ("wait_clear", "done", "aborted", "nudge") and float(rp.get("deadline") or 0) and now >= float(rp["deadline"]):
        progressed = reposition_progress(
            rp.get("start_odom"), odom, rp.get("start_front"), front,
            rp.get("start_sweep"), sweep_f, front_stop, sweep_need, progress_m, avoid_dist)
        return _rp_finish(rp, now, sensors, caps, progressed, fails)

    if phase == "choose":
        if can_back and not rp.get("backed"):
            rp["phase"] = "back"
            rp["back_until"] = float(now) + back_s
            phase = "back"
        else:
            # Tight / rear-blocked: go straight into continuous nudging (or a turn
            # if sweep is already open).
            if sweep_f >= sweep_need and turn_ok:
                rp["phase"] = "turn"
                rp["turn_until"] = float(now) + turn_s
                rp["backed"] = True
                phase = "turn"
            else:
                return _enter_nudge(rp, now, sensors, caps, fails=fails)

    if phase == "back":
        gained_sweep = sweep_f >= sweep_need
        rear_tight = False
        try:
            if rear is not None and float(rear) < rear_stop + margin + 0.02:
                rear_tight = True
        except (TypeError, ValueError):
            rear_tight = not can_back
        if (not can_back) or gained_sweep or rear_tight or now >= float(rp.get("back_until") or 0):
            rp["backed"] = True
            # Transition immediately: start the turn or start nudging on this tick
            # (no full-stop gap).
            if gained_sweep and turn_ok:
                rp["phase"] = "turn"
                rp["turn_until"] = float(now) + turn_s
                return _cmd("running", ang_dir=d,
                            why="reposition turn %s" % ("left" if d > 0 else "right"),
                            pop_holds=["turn"], fails=fails, reason="back complete, turning")
            return _enter_nudge(rp, now, sensors, caps, fails=fails)
        return _cmd("running", lin=back_v,
                    why="reposition back (gain sweep room)",
                    pop_holds=["rev"] if can_back else [],
                    fails=fails, reason="backing")

    if phase in ("turn", "alt_turn"):
        pop = []
        if sweep_f >= sweep_need:
            pop.append("turn")
        if (not turn_ok) or (sweep_f < sweep_need and not pop):
            tried = list(rp.get("tried_dirs") or [])
            if d not in tried:
                tried.append(d)
            rp["tried_dirs"] = tried
            if -d not in tried:
                rp["dir"] = -d
                rp["phase"] = "alt_turn"
                rp["turn_until"] = float(now) + turn_s
                # Command the other direction immediately — no zero tick.
                if sweep_f >= sweep_need:
                    return _cmd("running", ang_dir=-d,
                                why="reposition turn %s" % ("left" if -d > 0 else "right"),
                                pop_holds=pop, fails=fails, reason="alt turn")
                return _enter_nudge(rp, now, sensors, caps, fails=fails)
            # Both turn directions blocked: continuous nudge (includes backup if rear is safe).
            return _enter_nudge(rp, now, sensors, caps, fails=fails)
        if now < float(rp.get("turn_until") or 0):
            return _cmd("running", ang_dir=d,
                        why="reposition turn %s" % ("left" if d > 0 else "right"),
                        pop_holds=pop, fails=fails, reason="turning")
        # Turn slice finished: resume if clear, otherwise keep moving via nudges.
        progressed = reposition_progress(
            rp.get("start_odom"), odom, rp.get("start_front"), front,
            rp.get("start_sweep"), sweep_f, front_stop, sweep_need, progress_m, avoid_dist)
        return _rp_finish(rp, now, sensors, caps, progressed, fails)

    if phase == "wait_clear":
        # Legacy phase name: do not dwell. Resume or start nudging immediately.
        progressed = reposition_progress(
            rp.get("start_odom"), odom, rp.get("start_front"), front,
            rp.get("start_sweep"), sweep_f, front_stop, sweep_need, progress_m, avoid_dist)
        return _rp_finish(rp, now, sensors, caps, progressed, fails)

    return _cmd("stay_stopped", why="reposition: idle", fails=fails, reason="idle")


# 2026-11-24 navclr: shared body-edge clearance thresholds (nav_clearance.yaml, also read by gb_rgbguard).
NAV_CLR_FILES = (os.environ.get("GB_NAV_CLEARANCE", ""), "/home/ubuntu/shared/gb_safety/nav_clearance.yaml",
                 "/home/pi/docker/tmp/gb_safety/nav_clearance.yaml")
NAV_CLR_DEFAULT = {"reference": "body_edge", "front_m": 0.20, "side_m": 0.35, "open_dir_min_m": 0.50}

def load_nav_clearance(paths=NAV_CLR_FILES):
    """-> dict: body-edge values + centre-frame equivalents (front_c / side_c / open_c) + source path."""
    cfg = dict(NAV_CLR_DEFAULT); src = "default"
    for p in paths:
        if p and os.path.exists(p):
            try:
                import yaml
                d = yaml.safe_load(open(p)) or {}
                for k in ("front_m", "side_m", "open_dir_min_m"):
                    if k in d: cfg[k] = float(d[k])
                cfg["reference"] = d.get("reference", "body_edge"); src = p; break
            except Exception as ex:
                print("[nav_clearance] bad file %s: %s (using defaults)" % (p, ex), flush=True)
    if cfg["reference"] != "body_edge": raise ValueError("nav_clearance.yaml must use reference: body_edge")
    try:                                               # host side has no module-level OB; same tank.xacro numbers
        import gb_obstacle as _ob; bf, bw = _ob.BODY_FRONT, _ob.BODY_HALF_W
    except Exception:
        bf, bw = 0.13, 0.10
    cfg["front_c"] = round(bf + cfg["front_m"], 3)
    cfg["side_c"] = round(bw + cfg["side_m"], 3)
    cfg["open_c"] = round(bf + cfg["open_dir_min_m"], 3)
    cfg["src"] = src
    return cfg

NAV_CLR = load_nav_clearance()
AVOID_DIST = NAV_CLR["front_c"]          # navclr: wander turns away below 0.20 m from the bumper (0.33 m centre; was 0.45)

# 2026-11-25 learnapply: optional human-approved overlay from applied_params.json (whitelist + clamp).
# Never touches gb_gate stops, nav_clearance hard minimums, e-stop, or the ARMED wake-phrase gate.
# Missing/corrupt file -> keep the defaults already computed above. Re-run at request() / node start
# so a newly approved file is picked up without restarting voice.
SLOWDOWN_DIST = 0.60
LEARN_PARAM_WHITELIST = {
    "explore_cruise_lin": {"min": 0.08, "max": 0.1012},  # conservative default cruise only; pickup speed may exceed this, hard-capped at 1.0 m/s
    "explore_turn_ang":   {"min": 0.30, "max": 0.48},
    "avoid_dist_m":       {"min": 0.40, "max": 0.80},
    "slowdown_dist_m":    {"min": 0.50, "max": 1.00},
}
_LEARN_ENV_LIN = min(HARD_MAX_LIN, abs(float(E("EXPLORE_MAX_LIN", "0.08"))))
_LEARN_ENV_ANG = min(HARD_MAX_ANG, abs(float(E("EXPLORE_MAX_ANG", "0.48"))))

def _learned_param_paths():
    paths, seen = [], set()
    for p in (
        os.path.join(os.environ.get("GB_DATA", "/home/pi/grokbot/data"), "learn", "applied_params.json"),
        "/home/pi/grokbot/data/learn/applied_params.json",
        os.path.expanduser("~/grokbot/data/learn/applied_params.json"),
        os.path.join(HOST_DIR, "applied_params.json"),
        os.path.join(CONT_DIR, "applied_params.json"),
        "/home/ubuntu/shared/grokbot/explore/applied_params.json",
    ):
        if p and p not in seen:
            seen.add(p); paths.append(p)
    return paths

def _clamp_learned(name, value, floor=None):
    spec = LEARN_PARAM_WHITELIST[name]
    v = float(value)
    if v != v:  # NaN
        raise ValueError("nan")
    v = max(float(spec["min"]), min(float(spec["max"]), v))
    if floor is not None:
        v = max(float(floor), v)
    return v

def effective_explore_params():
    """Current MAX_LIN / MAX_ANG / AVOID_DIST / SLOWDOWN_DIST plus source. Read-only."""
    return {
        "explore_cruise_lin": float(MAX_LIN),
        "explore_turn_ang": float(MAX_ANG),
        "avoid_dist_m": float(AVOID_DIST),
        "slowdown_dist_m": float(SLOWDOWN_DIST),
        "hard_max_lin": float(HARD_MAX_LIN),
        "hard_max_ang": float(HARD_MAX_ANG),
        "nav_clearance_front_c": float(NAV_CLR["front_c"]),
        "gate_stops_untouched_m": {"front": 0.27, "rear": 0.28, "sweep": 0.24},
    }

def apply_learned_params(log=False, paths=None):
    """Overlay whitelist keys from applied_params.json; clamp; fall back to defaults on any error."""
    global MAX_LIN, MAX_ANG, AVOID_DIST, SLOWDOWN_DIST
    cruise, turn = _LEARN_ENV_LIN, _LEARN_ENV_ANG
    avoid, slow = float(NAV_CLR["front_c"]), 0.60
    src, err, raw = "defaults", None, {}
    try:
        for p in (paths if paths is not None else _learned_param_paths()):
            if os.path.isfile(p):
                d = json.load(open(p, encoding="utf-8"))
                if not isinstance(d, dict):
                    raise ValueError("applied_params.json is not an object")
                raw = d.get("params", d) if isinstance(d.get("params", d), dict) else {}
                src = p
                break
        if raw:
            if "explore_cruise_lin" in raw:
                cruise = _clamp_learned("explore_cruise_lin", raw["explore_cruise_lin"])
            if "explore_turn_ang" in raw:
                turn = _clamp_learned("explore_turn_ang", raw["explore_turn_ang"])
            if "avoid_dist_m" in raw:
                # never below nav_clearance front (centre) or the whitelist min
                avoid = _clamp_learned("avoid_dist_m", raw["avoid_dist_m"], floor=float(NAV_CLR["front_c"]))
            if "slowdown_dist_m" in raw:
                slow = _clamp_learned("slowdown_dist_m", raw["slowdown_dist_m"])
    except Exception as ex:
        cruise, turn = _LEARN_ENV_LIN, _LEARN_ENV_ANG
        avoid, slow = float(NAV_CLR["front_c"]), 0.60
        err, src = str(ex), "defaults (error: %s)" % ex
    sm = 1.0
    try:
        sm = _speed_mult()
    except Exception:
        sm = 1.0
    MAX_LIN = min(HARD_MAX_LIN, max(0.04, float(cruise) * sm))
    MAX_ANG = min(HARD_MAX_ANG, max(0.30, float(turn) * sm))
    AVOID_DIST = max(float(NAV_CLR["front_c"]), float(avoid))
    SLOWDOWN_DIST = max(AVOID_DIST, float(slow))
    info = effective_explore_params()
    info["source"] = src
    info["speed_mult"] = sm
    info["error"] = err
    if log:
        try:
            _hlog("learned params", json.dumps(info, sort_keys=True))
        except Exception:
            print("[learnapply]", json.dumps(info, sort_keys=True), flush=True)
    return info

apply_learned_params(log=False)

# 2026-11-26 adaptive explore speed (pure helpers; no motion).
# Wide-open: +20% above normal cruise, strictly capped by HARD_MAX_LIN (1.0 m/s).
# Approach: linear ramp from that high speed down to the tight floor as front
# clearance falls from 1.50 m to 0.45 m. Tight/narrow corridors hold the floor.
# Aurora 930 Pro depth is trusted only in [0.30, 3.00] m; LiDAR owns closer range.
# Track breakaway: 0.06 m/s moved 0 cm on the floor (2026-11-25); 0.08 m/s is the
# minimum creep that still drives the tracks. A 0.045-0.050 m/s floor would stall,
# and send()'s lin_floor (0.08) would lift it back whenever the clearance ramp allows.
ADAPT_DEPTH_MIN_M = 0.30
ADAPT_DEPTH_MAX_M = 3.00
ADAPT_OPEN_FRONT_M = 1.50
ADAPT_OPEN_SIDE_M = 1.00
ADAPT_APPROACH_CLOSE_M = 0.45
ADAPT_TIGHT_FRONT_M = 0.55
ADAPT_TIGHT_SIDE_M = 0.55
ADAPT_TIGHT_WIDTH_M = 1.10
ADAPT_TIGHT_LIN = 0.08
ADAPT_OPEN_BOOST = 1.20

def _clear_m(value):
    """Finite non-negative clearance in meters, or None if missing/invalid. +inf kept as inf (wide open)."""
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    if v != v or v < 0.0:
        return None
    return v

def trusted_depth_m(depth_front, depth_fresh=True):
    """Aurora 930 Pro metric depth, or None if stale/invalid/outside the 0.30-3.00 m window."""
    if not depth_fresh:
        return None
    d = _clear_m(depth_front)
    if d is None or d == float("inf"):
        return None
    if d < ADAPT_DEPTH_MIN_M or d > ADAPT_DEPTH_MAX_M:
        return None
    return d

def fuse_explore_front_m(lidar_front, depth_front=None, depth_fresh=True):
    """Front clearance for speed: LiDAR, plus trusted depth only when it is closer in-window."""
    lf = _clear_m(lidar_front)
    df = trusted_depth_m(depth_front, depth_fresh=depth_fresh)
    if lf is None:
        return df
    if df is None:
        return lf
    return min(lf, df)

def adaptive_boost_speed(normal_speed, hard_max_lin=None):
    """+20% boost strictly capped by hard_max_lin.

    With default explore_cruise_lin=0.1012 and HARD_MAX_LIN=1.0:
      min(0.1012 * 1.20, 1.0) = 0.12144  (the +20% open boost now applies)
      pickup may raise normal cruise toward 1.0; the boost stays <= hard cap.
    """
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    normal = min(max(float(normal_speed), 0.0), hard)
    return min(normal * ADAPT_OPEN_BOOST, hard)

def compute_adaptive_explore_speed(lidar_front=None, lidar_left=None, lidar_right=None,
                                   depth_front=None, depth_fresh=True, lidar_fresh=True,
                                   normal_speed=None, hard_max_lin=None, tight_speed=None):
    """Pure adaptive explore cruise. Returns (speed_m_s, regime).

    regime is one of: 'wide_open', 'approach', 'tight', 'normal', 'fail_safe'.
    Stale or missing LiDAR front fails safe to the tight/slow floor.
    """
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    normal = MAX_LIN if normal_speed is None else float(normal_speed)
    normal = min(max(normal, 0.0), hard)
    floor = ADAPT_TIGHT_LIN if tight_speed is None else float(tight_speed)
    floor = min(max(floor, 0.0), normal, hard)
    boost = adaptive_boost_speed(normal, hard)

    if not lidar_fresh:
        return floor, "fail_safe"
    front = fuse_explore_front_m(lidar_front, depth_front, depth_fresh=depth_fresh)
    left = _clear_m(lidar_left)
    right = _clear_m(lidar_right)
    if front is None:
        return floor, "fail_safe"

    sides_known = left is not None and right is not None
    width = (left + right) if sides_known else None
    tight_sides = sides_known and left < ADAPT_TIGHT_SIDE_M and right < ADAPT_TIGHT_SIDE_M
    tight_width = width is not None and width < ADAPT_TIGHT_WIDTH_M
    tight_front = front < ADAPT_TIGHT_FRONT_M
    if tight_sides or tight_width or tight_front:
        return floor, "tight"

    sides_open = sides_known and left >= ADAPT_OPEN_SIDE_M and right >= ADAPT_OPEN_SIDE_M
    if front >= ADAPT_OPEN_FRONT_M and sides_open:
        return boost, "wide_open"

    if front < ADAPT_OPEN_FRONT_M:
        high = boost if sides_open else normal
        span = ADAPT_OPEN_FRONT_M - ADAPT_APPROACH_CLOSE_M
        t = 0.0 if span <= 0 else (front - ADAPT_APPROACH_CLOSE_M) / span
        t = max(0.0, min(1.0, t))
        return floor + t * (high - floor), "approach"

    return normal, "normal"

# 2026-11-26 speedcap (John, GB-48): autonomous hard cap is 1.0 m/s, but entering
# explore always starts at the conservative default cruise (~0.10-0.12 m/s).
# "Pick up speed" raises the cruise target toward HARD_MAX_LIN, then a
# sensor-reaction ceiling (stopping distance vs live LiDAR freshness/clearance)
# stops the ramp before sensors can no longer react in time. Adaptive 20% open
# boost, approach slowdown, and 0.08 m/s tight creep still apply under the cap.
# Stop distances 0.27/0.28/0.24 m are UNCHANGED.
DEFAULT_CRUISE_LIN = 0.1012
PICKUP_STEP_MPS = 0.10
REACTION_DECEL_MPS2 = 0.50
REACTION_LIDAR_PERIOD_S = 0.10
REACTION_PROCESS_S = 0.05
REACTION_STOP_MARGIN_M = 0.27   # equals FRONT_STOP / gate forward stop; do not change the gate

def start_autonomous_cruise(current_cruise=None):
    """Conservative cruise used when entering autonomous explore. Never 1.0 m/s."""
    if current_cruise is None:
        try:
            v = float(MAX_LIN)
        except Exception:
            v = DEFAULT_CRUISE_LIN
    else:
        v = float(current_cruise)
    if v != v or v < 0.0:
        v = DEFAULT_CRUISE_LIN
    return min(HARD_MAX_LIN, DEFAULT_CRUISE_LIN, max(ADAPT_TIGHT_LIN, v))

def pick_up_explore_speed(current, step=None, hard_max_lin=None):
    """Raise the explore cruise target toward the hard cap by one step."""
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    step = PICKUP_STEP_MPS if step is None else float(step)
    try:
        cur = float(current)
    except (TypeError, ValueError):
        cur = start_autonomous_cruise()
    if cur != cur or cur < 0.0:
        cur = start_autonomous_cruise()
    return min(hard, cur + max(0.0, step))

def slow_down_explore_speed(current, step=None, hard_max_lin=None, floor=None):
    """Lower the explore cruise target toward the conservative default (not below it)."""
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    step = PICKUP_STEP_MPS if step is None else float(step)
    lo = start_autonomous_cruise() if floor is None else float(floor)
    try:
        cur = float(current)
    except (TypeError, ValueError):
        cur = lo
    if cur != cur or cur < 0.0:
        cur = lo
    return min(hard, max(lo, cur - max(0.0, step)))

def _explore_speed_paths(path=None):
    if path:
        return [path]
    out, seen = [], set()
    for p in (os.path.join(HOST_DIR, "explore_speed.json"),
              os.path.join(CONT_DIR, "explore_speed.json"),
              "/home/ubuntu/shared/grokbot/explore/explore_speed.json"):
        if p and p not in seen:
            seen.add(p); out.append(p)
    return out

def write_explore_speed_target(cruise, path=None, src="set"):
    v = min(HARD_MAX_LIN, max(0.0, float(cruise)))
    body = json.dumps({"cruise": v, "hard_max_lin": HARD_MAX_LIN, "t": time.time(), "src": src}) + "\n"
    last = None
    for p in _explore_speed_paths(path):
        try:
            os.makedirs(os.path.dirname(p) or ".", exist_ok=True)
            tmp = p + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(body)
            os.replace(tmp, p)
            last = p
            if path:
                break
        except Exception:
            continue
    return v

def read_explore_speed_target(path=None, default=None):
    fallback = start_autonomous_cruise() if default is None else float(default)
    for p in _explore_speed_paths(path):
        try:
            d = json.load(open(p, encoding="utf-8"))
            v = float(d.get("cruise"))
            if v == v and v >= 0.0:
                return min(HARD_MAX_LIN, max(0.0, v))
        except Exception:
            continue
    return fallback

def reset_explore_speed_target(path=None):
    """Mission start / 'normal speed': conservative default cruise."""
    v = start_autonomous_cruise()
    write_explore_speed_target(v, path=path, src="reset")
    return v

def request_pick_up_speed(path=None):
    cur = read_explore_speed_target(path=path)
    nxt = pick_up_explore_speed(cur)
    write_explore_speed_target(nxt, path=path, src="pick_up")
    return nxt

def request_slow_down_speed(path=None):
    cur = read_explore_speed_target(path=path)
    nxt = slow_down_explore_speed(cur)
    write_explore_speed_target(nxt, path=path, src="slow_down")
    return nxt

def sensor_reaction_speed_cap(front_clearance_m, lidar_fresh=True, lidar_age_s=None,
                              hard_max_lin=None, stop_margin_m=None, decel_mps2=None,
                              tight_speed=None):
    """Max safe speed given clearance and LiDAR freshness.

    Stopping model: v * t_react + v^2 / (2 * a) <= max(0, clearance - stop_margin).
    t_react = max(lidar period, scan age) + process margin. Stale or missing
    LiDAR fails safe to the tight floor. Stop margin stays 0.27 m (unchanged).
    """
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    floor = ADAPT_TIGHT_LIN if tight_speed is None else float(tight_speed)
    stop = REACTION_STOP_MARGIN_M if stop_margin_m is None else float(stop_margin_m)
    a = REACTION_DECEL_MPS2 if decel_mps2 is None else float(decel_mps2)
    if not lidar_fresh:
        return min(max(0.0, floor), hard), "stale"
    front = _clear_m(front_clearance_m)
    if front is None:
        return min(max(0.0, floor), hard), "no_range"
    slack = front - stop
    if slack <= 0.0:
        return 0.0, "inside_stop_margin"
    try:
        age = 0.0 if lidar_age_s is None else max(0.0, float(lidar_age_s))
    except (TypeError, ValueError):
        age = REACTION_LIDAR_PERIOD_S
    t = max(REACTION_LIDAR_PERIOD_S, age) + REACTION_PROCESS_S
    if t <= 0.0:
        t = REACTION_LIDAR_PERIOD_S + REACTION_PROCESS_S
    if a <= 0.0:
        v = slack / t
    else:
        v = -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * slack)
    v = max(0.0, min(v, hard))
    return v, "ok"

def apply_explore_speed_limits(speed, front_clearance_m, lidar_fresh=True, lidar_age_s=None,
                               hard_max_lin=None):
    """Clip a requested explore speed by the reaction ceiling and the hard cap."""
    hard = HARD_MAX_LIN if hard_max_lin is None else float(hard_max_lin)
    cap, why = sensor_reaction_speed_cap(
        front_clearance_m, lidar_fresh=lidar_fresh, lidar_age_s=lidar_age_s,
        hard_max_lin=hard,
    )
    try:
        v = float(speed)
    except (TypeError, ValueError):
        v = 0.0
    if v != v:
        v = 0.0
    return min(max(0.0, v), cap, hard), why


def open_dir(s):
    """navstd: which way (+1 left / -1 right) to rotate: toward the clearest KNOWN LiDAR sector (dropout-aware, unknown
    sectors never count as open); falls back to the side scores. Used by every escape / recovery turn."""
    c = s.get("clear_deg"); cm = s.get("clear_m")
    open_ok = cm is None or cm >= NAV_CLR["open_c"]                    # navclr: >= 0.50 m beyond the bumper to count
    if c is not None and open_ok and 15 <= abs(c) <= 165: return 1 if c > 0 else -1
    return 1 if s.get("left_mean", 0.0) >= s.get("right_mean", 0.0) else -1

def fused_view(gst):
    """navstd: the gate's FULL fused picture (LiDAR sectors/sides/dropouts/memories, depth + depth memory, IMU, RGB)."""
    g = (gst or {}).get("sensors") or {}
    return {"lidar": g.get("lidar") or {}, "depth": g.get("depth") or {}, "imu": g.get("imu") or {},
            "rgb": g.get("rgb") or {}, "front_src": g.get("front_src"), "faults": (gst or {}).get("faults") or {}}

# 2026-11-26 research takeaway 1: return-to-start lives in the SLAM map frame.
# Command-odometry breadcrumbs slip on tracks; a map->base_link pose + IMU yaw does not.
# Live LiDAR / 3 s wait-recheck / sidestep (go_to_points) still owns obstacle avoidance —
# there is no unguarded straight-line dash home.
MAP_POSE_FRAMES = ("base_link", "base_footprint")
MAP_POSE_FRESH_S = 0.8
IMU_YAW_FRESH_S = 0.60

def prefer_map_start(map_pose, odom_pose=None):
    """Start pose to remember for return-to-start. Map frame wins; odom is last-resort only."""
    if map_pose is not None and len(map_pose) >= 2:
        try:
            x, y = float(map_pose[0]), float(map_pose[1])
        except (TypeError, ValueError):
            x = y = None
        if x is not None and x == x and y == y:
            yaw = 0.0
            if len(map_pose) > 2 and map_pose[2] is not None:
                try:
                    yaw = float(map_pose[2])
                except (TypeError, ValueError):
                    yaw = 0.0
            return {"frame": "map", "x": x, "y": y, "yaw": yaw}
    if odom_pose is not None and len(odom_pose) >= 2:
        try:
            x, y = float(odom_pose[0]), float(odom_pose[1])
        except (TypeError, ValueError):
            return None
        yaw = 0.0
        if len(odom_pose) > 2 and odom_pose[2] is not None:
            try:
                yaw = float(odom_pose[2])
            except (TypeError, ValueError):
                yaw = 0.0
        return {"frame": "odom", "x": x, "y": y, "yaw": yaw}
    return None

def yaw_with_imu(imu_yaw, map_yaw=None, odom_yaw=None, imu_fresh=False):
    """IMU gyro dominates yaw when fresh; else map TF yaw; else odom."""
    if imu_fresh and imu_yaw is not None:
        try:
            y = float(imu_yaw)
        except (TypeError, ValueError):
            y = None
        if y is not None and y == y:
            return y, "imu"
    if map_yaw is not None:
        try:
            y = float(map_yaw)
        except (TypeError, ValueError):
            y = None
        if y is not None and y == y:
            return y, "map"
    if odom_yaw is not None:
        try:
            y = float(odom_yaw)
        except (TypeError, ValueError):
            y = None
        if y is not None and y == y:
            return y, "odom"
    return None, "none"

def return_target_points(start_xy):
    """Single map-frame home waypoint. Not a reversed odom breadcrumb trail.
    The driver must still run live LiDAR avoidance (no blind straight-line homing)."""
    if not start_xy or len(start_xy) < 2:
        return []
    try:
        return [(float(start_xy[0]), float(start_xy[1]))]
    except (TypeError, ValueError):
        return []

def should_use_map_return(start_frame, have_map_pose):
    """True when return-to-start can run in the SLAM map frame."""
    return (start_frame == "map") and bool(have_map_pose)

def pose_error_m(current, target):
    """Planar distance between two (x, y, ...) poses."""
    if not current or not target or len(current) < 2 or len(target) < 2:
        return None
    try:
        return math.hypot(float(current[0]) - float(target[0]), float(current[1]) - float(target[1]))
    except (TypeError, ValueError):
        return None

def node_main(plan, dry, use_nav2):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry, OccupancyGrid
    import tf2_ros
    from std_msgs.msg import String
    import sys
    sys.path.insert(0, "/home/ubuntu/shared/gb_safety")
    _obd = _gate_code_dir()
    if _obd: sys.path.insert(0, _obd)  # 2026-11-17 allguard: the gate's own gb_obstacle (the gb_safety copy is a Nov 6 one)
    import gb_obstacle as OB          # 2026-11-05: same footprint corridor / ramp as the gate (copied there by gb_safety_run.sh)
    try:                              # 2026-11-27: shared LiDAR + camera fusion stuck-recovery (same file the gate imports)
        import gb_fusion_recovery as FRM
    except Exception as _frx:
        FRM = None
        print(_now(), "[node] fusion recovery unavailable (%s): LiDAR-only recovery" % _frx, flush=True)
    rear_stop = max(REAR_STOP, OB.STOP_REV)   # 2026-11-17 allguard: never below the gate's reverse stop (was 0.20)
    _learn = apply_learned_params(log=False)
    print(_now(), "[node] learned params lin %.3f ang %.3f avoid %.3f slowdown %.3f from %s" % (
        MAX_LIN, MAX_ANG, AVOID_DIST, SLOWDOWN_DIST, _learn.get("source")), flush=True)
    print(_now(), "[node] nav_clearance (body edge) front %.2f side %.2f open %.2f m -> centre %.2f / %.2f / %.2f m from %s" % (
        NAV_CLR["front_m"], NAV_CLR["side_m"], NAV_CLR["open_dir_min_m"], NAV_CLR["front_c"], NAV_CLR["side_c"], NAV_CLR["open_c"], NAV_CLR["src"]), flush=True)
    print(_now(), "[node] obstacle module", OB.__file__, "stops fwd %.2f rev %.2f sweep %.2f blind_stop %.2f" % (
        FRONT_STOP, rear_stop, OB.SWEEP_R, OB.BLIND_FRAC_STOP), flush=True)

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
            self.imu_yaw = None; self.imu_t = 0.0
            self.create_subscription(LaserScan, "/scan_raw", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
            mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, mq)
            self.create_subscription(Twist, NAV_OUT, self.on_nav, 10)
            self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
            self.sect = None; self.gst = None; self.gst_t = 0.0; self.view = None   # view: 2026-11-27 fusion recovery
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
                self.view = v                                # 2026-11-27 fusion recovery: points for the gap check
                self.sect.update(corr=v.fwd, corr_rev=v.rev, sweep=v.sweep, blind=v.blind_f,
                                 blind_r=(getattr(v, "blind_r", 1.0) if len(r) else 1.0), n=len(r))
                # navstd: dropout-aware side scores (the old means treated an all-NaN side as 3 m 'open'), body-side
                # gaps and the clearest known sector - the planner turns toward open space, never into a dark wall
                lo, ro = OB.side_open(v); ch, cm = OB.clearest_heading(v)
                self.sect.update(left_mean=lo, right_mean=ro, left_gap=v.left, right_gap=v.right,
                                 env_f=v.env_f, env_r=v.env_r, imputed=v.imputed,
                                 clear_deg=(math.degrees(ch) if ch is not None else None), clear_m=cm)
            except Exception:
                self.view = None
                self.sect.update(corr=0.0, corr_rev=0.0, sweep=0.0, blind=1.0, blind_r=1.0, n=0)   # fail safe: treat as blocked
            self.scan_t = time.time()
        def on_odom(self, m):
            p = m.pose.pose; q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom = (p.position.x, p.position.y, yaw); self.odom_t = time.time()
        def on_imu(self, m):
            q = m.orientation
            if q.w == q.w and (q.w or q.x or q.y or q.z):
                self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.imu_t = time.time()
        def on_map(self, m):
            self.map = m; self.map_t = time.time()
        def on_nav(self, m):
            self.nav_cmd = (m.linear.x, m.angular.z, time.time())
        def map_pose(self):
            """TF map -> base_link (fallback base_footprint). Used for return-to-start."""
            for frame in MAP_POSE_FRAMES:
                try:
                    t = self.tfb.lookup_transform("map", frame, rclpy.time.Time())
                    q = t.transform.rotation
                    return (t.transform.translation.x, t.transform.translation.y,
                            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                except Exception:
                    continue
            return None
        def imu_fresh(self, now=None):
            now = time.time() if now is None else now
            return self.imu_yaw is not None and (now - self.imu_t) <= IMU_YAW_FRESH_S

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

    def turn_clear(s, now):
        """2026-11-17 allguard: same turn test as the gate - 360 deg sweep (LiDAR, plus the gate's fresh depth) and rear blind."""
        sw = s.get("sweep", 0.0)
        if nd.gst and now - nd.gst_t < 0.5:
            fus = nd.gst.get("fusion") or {}
            df = fus.get("depth_fwd")
            if df is not None and (fus.get("fresh") or {}).get("depth"): sw = min(sw, float(df))
            gs = nd.gst.get("sweep")                                    # 2026-11-23 fix 3: the gate's own sweep
            if gs is not None: sw = min(sw, float(gs))
        if not s.get("n", 1): return False, "SWEEP-BLIND empty scan"
        if sw < OB.SWEEP_R: return False, "SWEEP-BLOCK %.2f" % sw
        if s.get("blind_r", 1.0) >= OB.BLIND_FRAC_STOP: return False, "SWEEP-BLIND rear %.0f%%" % (100 * s.get("blind_r", 1.0))
        return True, ""

    def rev_clear(s):
        g = nd.gst if (nd.gst and time.time() - nd.gst_t < 0.5) else {}
        gr = g.get("rear") if g else None                              # 2026-11-23 fix 3: the gate's own rear distance
        if g and gr is None and "rear" in g: gr = 9.0                  # gate reports None for inf (nothing behind)
        return (s.get("rear", 0.0) >= rear_stop and s.get("corr_rev", 0.0) >= rear_stop
                and (gr is None or float(gr) >= rear_stop)
                and s.get("blind_r", 1.0) < OB.BLIND_FRAC_STOP and s.get("n", 1) > 0)

    def send(lin, ang, why="", fusion=None):
        """THE only path to the motors. Caps, LiDAR stop, staleness stop, logging."""
        checks()
        if fusion is not None:                    # 2026-11-27 fusion stuck-recovery micro-nudge (see _send_fusion)
            return _send_fusion(lin, ang, why, fusion)
        # Adaptive wide-open boost may exceed learned MAX_LIN (cruise) but never HARD_MAX_LIN (1.0).
        # Other call sites still pass min(MAX_LIN, ...); this only lets the boost through when cruise < cap.
        lin = max(-HARD_MAX_LIN, min(HARD_MAX_LIN, float(lin))); ang = max(-MAX_ANG, min(MAX_ANG, float(ang)))
        lin0, ang0 = lin, ang
        now = time.time(); s = nd.sect or {}
        if now - nd.scan_t > 0.6 or now - nd.odom_t > 0.6:
            lin, ang, why = 0.0, 0.0, "STALE " + ("scan" if now - nd.scan_t > 0.6 else "odom")
        elif lin > 0 and s.get("front", 9) < FRONT_STOP:
            lin, why = 0.0, why + " FRONT-BLOCK %.2f" % s.get("front", 9)
        elif lin < 0 and s.get("rear", 0.0) < rear_stop:                 # 2026-11-17 allguard: missing rear = blocked
            lin, why = 0.0, why + " REAR-BLOCK %.2f" % s.get("rear", 0.0)
        elif lin < 0 and (s.get("blind_r", 1.0) >= OB.BLIND_FRAC_STOP or "corr_rev" not in s):
            lin, why = 0.0, why + " REAR-BLIND %.0f%%" % (100 * s.get("blind_r", 1.0))
        hold = st.get("trip_hold") or {}                                 # 2026-11-23 fix 3: never re-command a tripped direction
        if lin > 0 and hold.get("fwd", 0) > now: lin, why = 0.0, why + " TRIP-HOLD fwd"
        elif lin < 0 and hold.get("rev", 0) > now: lin, why = 0.0, why + " TRIP-HOLD rev"
        if ang != 0 and hold.get("turn", 0) > now: ang, why = 0.0, why + " TRIP-HOLD turn"
        if ang != 0:                                                     # 2026-11-17 allguard: turns were never checked here
            ok_t, tw = turn_clear(s, now)
            if not ok_t: ang, why = 0.0, why + " " + tw
        if lin != 0:                                         # corridor + gate's fused distance, speed ramp (0.04 -> max)
            gf = (nd.gst or {}).get("front") if now - nd.gst_t < 0.5 else None
            if lin > 0:
                dd = min(s.get("corr", 0.0), gf if gf is not None else 9.0)
                cap = OB.ramp(dd, HARD_MAX_LIN)
                react, _rw = sensor_reaction_speed_cap(dd, lidar_fresh=(now - nd.scan_t <= 0.6),
                                                       lidar_age_s=(now - nd.scan_t), hard_max_lin=HARD_MAX_LIN)
                cap = min(cap, react); st["lin_cap"] = cap
                if cap <= 0: lin, why = 0.0, why + " CORRIDOR-BLOCK %.2f" % dd
                elif lin > cap: lin, why = cap, why + " ramp %.2f" % dd
            else:
                cap = OB.ramp(s.get("corr_rev", 0.0), MAX_LIN, rear_stop); st["lin_cap"] = OB.ramp(s.get("corr_rev", 0.0), HARD_MAX_LIN, rear_stop)
                if cap <= 0: lin, why = 0.0, why + " REAR-CORRIDOR-BLOCK %.2f" % s.get("corr_rev", 0.0)
                elif -lin > cap: lin = -cap
        if (lin0 or ang0) and not (lin or ang):                         # 2026-11-17 allguard: track refused commands
            if st.get("refused_since") is None: st["refused_since"] = now
            st["refused_last"] = now
        elif lin or ang:
            st["refused_since"] = None
        if plan.get("accel_lin") or plan.get("accel_ang"):          # 2026-11-25 benchtest: gradual acceleration (opt-in)
            pl, pa, pt = st.get("ramp_prev", (0.0, 0.0, now)); dt = max(0.02, min(0.3, now - pt))
            def _slew(v, prev, acc):
                if not acc or v == 0: return v
                if prev == 0 or (v > 0) == (prev > 0):
                    if abs(v) > abs(prev) + acc * dt: return math.copysign(abs(prev) + acc * dt, v)
                    return v
                return math.copysign(min(abs(v), acc * dt), v)   # sign change: restart the ramp from zero
            lin = _slew(lin, pl, float(plan.get("accel_lin") or 0)); ang = _slew(ang, pa, float(plan.get("accel_ang") or 0))
            st["ramp_prev"] = (lin, ang, now)
        # 2026-11-25 linfloor (John 12:54 ET): on the floor the tracks do not break away below ~0.08 m/s (0.06 m/s creep
        # moved 0 cm, 0.08 m/s moved 4 cm). Lift any non-zero linear command to LIN_FLOOR, but ONLY where this cycle's
        # clearance ramp already allows that speed; near obstacles the ramp/gate value (or zero) stands. Never above caps.
        lf = min(float(plan.get("lin_floor", os.environ.get("GB_LIN_FLOOR", "0.08"))), HARD_MAX_LIN)   # may exceed learned cruise 0.07
        if lin != 0 and abs(lin) < lf and st.get("lin_cap", 0.0) >= lf:
            lin = math.copysign(lf, lin); why = why + " floor %.2f" % lf
        st["lin_cap"] = 0.0
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

    # ---------------------------------------------------------------- 2026-11-27 fusion stuck-recovery (John)
    # Shared core layer for EVERY mission (explore / wander / distance / waypoints / return-home / retrace / Nav2 relay).
    # When the normal LiDAR stops say "stuck" but LiDAR + depth camera (+ RGB) agree a real gap wider than the chassis
    # exists, make slow (0.065 m/s), time-bounded, camera-verified micro-nudges in TIGHT-SQUEEZE mode (clearance margins
    # beyond the chassis halved: bumper gap 0.14 -> 0.07 m = 0.20 m stop, side margin 5 -> 2.5 cm, turn margin -> 0.22 m),
    # then hand straight back to the mission. Reduced clearance exists only per command via a 0.6 s token that gb_gate
    # re-verifies with its own sensors, so it auto-restores the instant nudging stops (clear / timeout / abort / end /
    # restart). Camera offline / stale / agreeing -> normal LiDAR-only behaviour, stops 0.27 / 0.28 / 0.24 m untouched.
    FR_TOKEN = FRM.token_path(ESTOP_CONT) if FRM is not None else None
    FR_MAX_PER_GOAL = int(E("GB_FR_MAX_PER_GOAL", "4"))
    FR_RECHECK_S = 2.0
    FR_STUCK_TRIGGER_S = 2.0

    def fr_inputs(now):
        s = nd.sect or {}
        g = nd.gst if (nd.gst and now - nd.gst_t < 0.5) else {}
        lid = FRM.lidar_input(nd.view) if (FRM is not None and nd.view is not None) else None
        cam = (g.get("fr") or {}).get("cam") if g else None
        sn = (g.get("sensors") or {}) if g else {}
        rg = sn.get("rgb") or {}
        rgb = ({"t": now, "hazard": bool(rg.get("hazard")), "label": rg.get("label")}
               if (sn.get("fresh") or {}).get("rgb") else None)
        lm = (sn.get("lidar") or {}).get("mem_fwd"); dm = (sn.get("depth") or {}).get("mem")
        mems = {"lidar_mem": float(lm) if lm is not None else None, "depth_mem": float(dm) if dm is not None else None}
        gf = g.get("front") if g else None; gs = g.get("sweep") if g else None
        normal = {"front": min(s.get("front", 0.0), s.get("corr", 0.0), float(gf) if gf is not None else 9.0),
                  "sweep": min(s.get("sweep", 0.0), float(gs) if gs is not None else 9.0),
                  "rear": s.get("rear", 0.0), "corr_rev": s.get("corr_rev", 0.0), "blind_r": s.get("blind_r", 1.0)}
        return lid, cam, rgb, mems, normal

    def fr_trip_new(tstate):
        g = nd.gst or {}
        n = int(g.get("trips") or 0)
        if "trips" not in tstate:
            tstate["trips"] = n; return None
        if n > tstate["trips"]:
            tstate["trips"] = n; return g.get("last_trip") or "trip"
        return None

    def fusion_verdict(now, goal_bearing=None):
        if FRM is None:
            return {"mode": "lidar_only", "reason": "fusion module not loaded", "allowed": []}
        lid, cam, rgb, mems, normal = fr_inputs(now)
        return FRM.assess(lid, cam, rgb, now, goal_bearing=goal_bearing, mems=mems, normal=normal)

    def _fr_log_verdict(v, label):
        now = time.time(); k = (label, v.get("mode"), (v.get("reason") or "")[:40])
        if st.get("fr_vlog") != k or now - st.get("fr_vlog_t", 0.0) > 5.0:
            st["fr_vlog"] = k; st["fr_vlog_t"] = now
            log("FUSION %s: %s -> %s" % (label, v.get("mode"), (v.get("reason") or "")[:160]))

    def _goal_bearing(tgt, prefer_map=False):
        p = pose_now(prefer_map=prefer_map) or nd.odom
        if not p or tgt is None:
            return None
        return _wrap(math.atan2(tgt[1] - p[1], tgt[0] - p[0]) - p[2])

    def _goal_dist(tgt, prefer_map=False):
        p = pose_now(prefer_map=prefer_map) or nd.odom
        return dist(p, tgt) if (p and tgt is not None) else 9.0

    def _send_fusion(lin, ang, why, fo):
        """Fusion micro-nudge publish path. Normal-rule moves (token False) go through the normal send()."""
        if not fo.get("token"):
            return send(lin, ang, why)
        now = time.time()
        if FRM is None or now - nd.scan_t > 0.6 or now - nd.odom_t > 0.6 or not os.path.exists(ARMED_CONT):
            st["fr_refused"] = now
            return send(0.0, 0.0, "fusion: stale sensors / disarmed -> zero")
        lid, cam, rgb, mems, normal = fr_inputs(now)
        ok, verdict, reason = FRM.check_command(lin, ang, lid, cam, rgb, now, mems)
        if not ok:
            st["fr_refused"] = now
            return send(0.0, 0.0, "fusion refused (%s: %s)" % (verdict, reason[:80]))
        lin, ang = FRM.clamp_cmd(lin, ang)
        assert 0.0 <= lin <= FRM.NUDGE_LIN_MAX + 1e-9 and abs(ang) <= FRM.ANG_MAX + 1e-9
        if not dry:
            try:
                FRM.write_token(FR_TOKEN, lin, ang, fo.get("cand"), plan.get("mission"), now)
            except Exception as ex:
                st["fr_refused"] = now
                return send(0.0, 0.0, "fusion token write failed (%s)" % ex)
        m = Twist(); m.linear.x = lin; m.angular.z = ang
        if not dry:
            try:
                nd.pub.publish(m); res["cmds_published"] += 1
            except Exception as e:
                raise Abort(f"publish failed ({type(e).__name__}: {str(e)[:80]})")
        res["cmds_planned"] += 1
        st["refused_since"] = None
        key = (round(lin, 2), round(ang, 2), st["phase"])
        if key != st["last"] or now - st["last_log"] > 2.0:
            o = nd.odom or (0, 0, 0); s = nd.sect or {}
            rec = {"t": round(now - st["t0"], 2), "phase": st["phase"], "lin": round(lin, 3), "ang": round(ang, 3),
                   "why": ("TIGHT-SQUEEZE " + why).strip(), "odom": [round(o[0], 3), round(o[1], 3), round(o[2], 3)],
                   "front": round(s.get("front", -1), 2), "pub": not dry, "fusion": True}
            cmdlog.write(json.dumps(rec) + "\n"); cmdlog.flush()
            log(("PUB " if not dry else "DRY ") + json.dumps(rec))
            st["last"] = key; st["last_log"] = now
        return lin

    def fusion_step(fe, goal_bearing, tstate, goal_reached=None):
        now = time.time()
        lid, cam, rgb, mems, normal = fr_inputs(now)
        estop_now = os.path.exists(ESTOP_CONT) or bool((nd.gst or {}).get("estop"))
        abort_now = bool(stop_flag["why"]) or os.path.exists(os.path.join(CONT_DIR, "abort"))
        refused = bool(st.pop("fr_refused", None))
        gb = goal_bearing() if callable(goal_bearing) else goal_bearing
        return fe.step(now, lid, cam, rgb, odom=nd.odom, normal=normal, mems=mems, goal_bearing=gb,
                       armed=os.path.exists(ARMED_CONT), estop=estop_now, abort=abort_now,
                       trip=fr_trip_new(tstate), refused=refused,
                       goal_reached=bool(goal_reached()) if callable(goal_reached) else False)

    def fusion_end(fe, out, label):
        if FR_TOKEN:
            FRM.clear_token(FR_TOKEN)                       # tight-squeeze clearance ends with the escape
        res.setdefault("fusion", []).append({"label": label, "event": out.get("event"), "why": (out.get("why") or "")[:160],
                                             "nudges": fe.nudges, "travel_m": round(fe.travel, 3), "fails": fe.fails})
        log("FUSION %s end (%s): %s - normal clearances 0.27/0.28/0.24 restored" % (label, out.get("event"), out.get("why")))

    def fusion_escape_run(label, goal_bearing=None, goal_reached=None):
        """Blocking 10 Hz fusion escape for go-to style missions (distance / waypoints / return / retrace / Nav2).
        -> 'recovered' (continue the mission) | 'failed' | 'not_started' (camera untrusted or obstacle real)."""
        if FRM is None or not FRM.ENABLED:
            return "not_started"
        now = time.time()
        v = fusion_verdict(now, goal_bearing() if callable(goal_bearing) else goal_bearing)
        _fr_log_verdict(v, label)
        if v.get("mode") != "nudge":
            return "not_started"
        fe = FRM.FusionEscape(plan.get("mission"), now, nd.odom)
        tstate = {}; fr_trip_new(tstate)
        prev = st["phase"]; phase(label + "_fusion")
        log("FUSION %s start (tight-squeeze): %s" % (label, (v.get("reason") or "")[:160]))
        out = {"event": "failed", "why": "not started"}
        try:
            while True:
                out = fusion_step(fe, goal_bearing, tstate, goal_reached)
                if out["event"] != "nudge":
                    break
                send(out["lin"], out["ang"], out["why"], fusion=out); time.sleep(0.1)
        finally:
            if FR_TOKEN:
                FRM.clear_token(FR_TOKEN)
        fusion_end(fe, out, label)
        phase(prev)
        if out["event"] != "recovered":
            send(0.0, 0.0, "fusion escape ended (%s)" % out["event"])
        return "recovered" if out["event"] == "recovered" else "failed"

    def fusion_kick(w, now, o, front, s, why):
        """Explore / wander: start the (tick-based) reposition with fusion first, only if fusion can really help."""
        if FRM is None or not FRM.ENABLED or w.get("fr_fallback_until", 0) > now:
            return False
        v = fusion_verdict(now); _fr_log_verdict(v, "explore")
        if v.get("mode") != "nudge":
            w["fr_fallback_until"] = now + FR_RECHECK_S
            return False
        f0 = front if front is not None else min(s.get("front", 0.0), s.get("corr", 0.0))
        rp = start_reposition(now, o, f0, s.get("sweep", 0.0), open_dir(s))
        rp["fails"] = int(w.get("reposition_fails") or 0)
        w["reposition"] = rp; w["reposition_active"] = True
        w.pop("obstacle_wait_start", None); w.pop("obstacle_wait_until", None); w["obstacle_stay_stopped"] = False
        st["refused_since"] = None
        log("FUSION explore: %s -> LiDAR+camera stuck-recovery" % why)
        return True

    def fusion_wander_tick(w, now):
        """Non-blocking fusion escape tick for explore / wander -> 'handled' | 'recovered' | 'fallback'."""
        if FRM is None or not FRM.ENABLED or w.get("fr_fallback_until", 0) > now:
            return "fallback"
        fe = w.get("fusion")
        if fe is None:
            v = fusion_verdict(now); _fr_log_verdict(v, "explore")
            if v.get("mode") != "nudge":
                w["fr_fallback_until"] = now + FR_RECHECK_S
                return "fallback"
            fe = FRM.FusionEscape(plan.get("mission"), now, nd.odom); w["fusion"] = fe
            w["fr_tstate"] = {}; fr_trip_new(w["fr_tstate"])
            log("FUSION explore start (tight-squeeze): %s" % (v.get("reason") or "")[:160])
        out = fusion_step(fe, None, w.setdefault("fr_tstate", {}))
        if out["event"] == "nudge":
            send(out["lin"], out["ang"], out["why"], fusion=out)
            return "handled"
        w.pop("fusion", None); fusion_end(fe, out, "explore")
        if out["event"] == "recovered":
            return "recovered"
        w["fr_fallback_until"] = now + (10.0 if out["event"] == "abort" else FR_RECHECK_S)
        return "fallback"

    def wait_inputs(t=10):
        t0 = time.time()
        hb()  # 2026-11-26 explorefix: heartbeat immediately and each loop while waiting for sensors
        while time.time() - t0 < t:
            checks()
            gst_ok = nd.gst and time.time() - nd.gst_t < 1.0 and (nd.gst.get("fusion") or {}).get("fresh", {}).get("depth")
            if nd.odom and time.time() - nd.odom_t < 0.5 and ((nd.sect and time.time() - nd.scan_t < 0.5) or gst_ok):
                return
            time.sleep(0.1)
            hb()
        raise Abort("no LiDAR/depth scan or odometry")

    def dist(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])

    def pose_now(prefer_map=False):
        """Current (x, y, yaw). Map+IMU when returning home; odom otherwise. IMU dominates yaw when fresh."""
        mp = nd.map_pose()
        o = nd.odom
        imu_fresh = nd.imu_fresh()
        if prefer_map and mp:
            yaw, _src = yaw_with_imu(nd.imu_yaw, mp[2], o[2] if o else None, imu_fresh)
            return (mp[0], mp[1], yaw if yaw is not None else mp[2])
        if not o:
            return None
        map_yaw = mp[2] if mp else None
        yaw, _src = yaw_with_imu(nd.imu_yaw, map_yaw, o[2], imu_fresh)
        return (o[0], o[1], yaw if yaw is not None else o[2])

    # ---------------------------------------------------------------- primitives
    def rotate_to(yaw_target, label, timeout=20.0, tol=0.08, frame="odom"):
        phase(label); t0 = time.time()
        prefer_map = (frame == "map")
        while time.time() - t0 < timeout:
            p = pose_now(prefer_map=prefer_map) or nd.odom
            if not p:
                send(0.0, 0.0, "rotate wait pose"); time.sleep(0.1); continue
            err = _wrap(yaw_target - p[2])
            if abs(err) < tol: break
            w = max(0.25, min(MAX_ANG, 1.2 * abs(err))) * (1 if err > 0 else -1)
            send(0.0, w, f"rotate err={err:.2f}"); time.sleep(0.1)
        p = pose_now(prefer_map=prefer_map) or nd.odom
        zero(5, "rotate done")
        return abs(_wrap(yaw_target - p[2])) if p else 9.0

    def go_to_points(points, label, timeout, final_tol=0.10, frame="odom"):
        """Follow waypoints with universal 3s wait-recheck-sidestep-skip avoidance (return/retrace).
        frame='map' uses TF map->base_link + IMU yaw (return-to-start). Never a blind unguarded dash."""
        t0 = time.time(); i = 0
        ost = {"avoid_retries": 0, "blocked_idx": set()}
        prefer_map = (frame == "map")
        fr_budget = {}                                    # 2026-11-27 fusion escapes per waypoint
        while i < len(points):
            if time.time() - t0 > timeout: return False, "took too long"
            o = pose_now(prefer_map=prefer_map) or nd.odom
            if not o:
                send(0.0, 0.0, "wait pose"); time.sleep(0.1); continue
            tgt = points[i]
            last = i == len(points) - 1
            d = dist(o, tgt)
            if d < (final_tol if last else 0.18):
                i += 1; continue
            s = nd.sect or {}
            now = time.time()
            gf = (nd.gst or {}).get("front") if now - nd.gst_t < 0.5 else None
            front = min(s.get("front", 9), s.get("corr", 9), gf if gf is not None else 9.0)
            estop_now = os.path.exists(ESTOP_CONT) or bool((nd.gst or {}).get("estop"))
            abort_now = bool(stop_flag["why"]) or os.path.exists(os.path.join(CONT_DIR, "abort"))
            armed_now = os.path.exists(ARMED_CONT)
            if ost.get("obstacle_wait_start"):
                waited = now - float(ost.get("obstacle_wait_start") or now)
                uni = universal_obstacle_decision(
                    waited, OBSTACLE_RECHECK_WAIT_S, front, AVOID_DIST, FRONT_STOP,
                    left_clear_m=s.get("left_mean"), right_clear_m=s.get("right_mean"),
                    corridor_m=s.get("corr", front), estop=estop_now, abort=abort_now,
                    armed=armed_now, retries=int(ost.get("avoid_retries") or 0),
                    retry_cap=OBSTACLE_AVOID_RETRY_CAP, mode="retrace",
                    more_breadcrumbs=(i < len(points) - 1))
                if uni == "wait":
                    send(0.0, 0.0, "obstacle wait (retrace recheck in %.1fs)" % max(0.0, OBSTACLE_RECHECK_WAIT_S - waited))
                    time.sleep(0.1); continue
                ost.pop("obstacle_wait_start", None)
                if uni == "resume":
                    pass  # fall through and drive toward the current waypoint
                elif uni == "skip_breadcrumb":
                    ost.setdefault("blocked_idx", set()).add(i)
                    nxt = next_clear_breadcrumb(i, len(points), ost["blocked_idx"])
                    ost["avoid_retries"] = int(ost.get("avoid_retries") or 0) + 1
                    if nxt is None or ost["avoid_retries"] >= OBSTACLE_AVOID_RETRY_CAP:
                        zero(5, "retrace boxed in"); return False, "path blocked by an obstacle"
                    log("RETRACE: skip blocked breadcrumb %s -> %s" % (i, nxt))
                    i = nxt; continue
                elif uni in ("sidestep_left", "sidestep_right"):
                    ost["avoid_retries"] = int(ost.get("avoid_retries") or 0) + 1
                    side = "left" if uni.endswith("left") else "right"
                    log("RETRACE: guarded sidestep %s retry %s" % (side, ost["avoid_retries"]))
                    ang = MAX_ANG if side == "left" else -MAX_ANG
                    t_turn = time.time() + 0.6
                    while time.time() < t_turn:
                        send(0.0, ang, "sidestep turn %s" % side); time.sleep(0.1)
                    travelled = 0.0
                    last_o = nd.odom
                    t_go = time.time() + max(1.0, OBSTACLE_SIDESTEP_M / max(0.04, MAX_LIN))
                    while time.time() < t_go and travelled < OBSTACLE_SIDESTEP_M:
                        s2 = nd.sect or {}; f2 = s2.get("front", 9); c2 = s2.get("corr", f2)
                        if min(f2, c2) < max(AVOID_DIST, FRONT_STOP) or not os.path.exists(ARMED_CONT):
                            send(0.0, 0.0, "sidestep corridor blocked"); break
                        send(min(MAX_LIN, 0.08), 0.0, "sidestep go %s" % side)
                        o2 = nd.odom
                        if last_o and o2:
                            travelled += dist(last_o, o2)
                        last_o = o2
                        time.sleep(0.1)
                    if ost["avoid_retries"] >= OBSTACLE_AVOID_RETRY_CAP:
                        zero(5, "retrace avoid cap"); return False, "path blocked by an obstacle"
                    continue
                else:
                    if fr_budget.get(i, 0) < FR_MAX_PER_GOAL:   # 2026-11-27: LiDAR+camera stuck-recovery before giving up
                        _r = fusion_escape_run(label, goal_bearing=lambda: _goal_bearing(tgt, prefer_map),
                                               goal_reached=lambda: _goal_dist(tgt, prefer_map) < (final_tol if last else 0.18))
                        if _r != "not_started": fr_budget[i] = fr_budget.get(i, 0) + 1
                        if _r == "recovered": continue
                    zero(5, "retrace stay stopped")
                    return False, "path blocked by an obstacle"
            err = _wrap(math.atan2(tgt[1] - o[1], tgt[0] - o[0]) - o[2])
            if abs(err) > 0.45:
                phase("turn_around" if abs(err) > 2.0 and "turn_around" not in res["phases"] else label + "_turn")
                send(0.0, max(0.3, min(MAX_ANG, 1.2 * abs(err))) * (1 if err > 0 else -1), f"align err={err:.2f}")
            else:
                phase(label)
                v = max(0.04, min(MAX_LIN, 0.6 * d))
                got = send(v, max(-MAX_ANG, min(MAX_ANG, 1.5 * err)), f"to wp{i} d={d:.2f}")
                if got == 0.0:
                    if fr_budget.get(i, 0) < FR_MAX_PER_GOAL:   # 2026-11-27: LiDAR+camera stuck-recovery first
                        _r = fusion_escape_run(label, goal_bearing=lambda: _goal_bearing(tgt, prefer_map),
                                               goal_reached=lambda: _goal_dist(tgt, prefer_map) < (final_tol if last else 0.18))
                        if _r != "not_started": fr_budget[i] = fr_budget.get(i, 0) + 1
                        if _r == "recovered": continue
                    why = ((st.get("last") or {}).get("why") if isinstance(st.get("last"), dict) else "") or ""
                    # treat a refused forward (front/corridor) as a universal obstacle event
                    ost = begin_obstacle_wait(ost, time.time())
                    send(0.0, 0.0, "obstacle wait (retrace)")
            time.sleep(0.1)
        zero(5, "arrived")
        return True, "ok"

    def wander_step(w):
        """Camera-first reactive walk (10 Hz). Depth+LiDAR fused via gate status; bump recovery without aborting."""
        import random
        s = nd.sect; now = time.time(); o = nd.odom or (0, 0, 0)
        if s is None:  # 2026-11-17: no LiDAR scan received yet (depth-only start); hold still until one arrives
            send(0.0, 0.0, "waiting for first LiDAR scan"); return
        if w.get("sidestep_turn_until") or w.get("sidestep_go_until"):
            side = w.get("sidestep_side") or "left"
            if now < float(w.get("sidestep_turn_until") or 0):
                send(0.0, MAX_ANG if side == "left" else -MAX_ANG, "sidestep turn %s" % side); return
            w.pop("sidestep_turn_until", None)
            if now < float(w.get("sidestep_go_until") or 0):
                front_now = s.get("front", 9); corr_now = s.get("corr", front_now)
                if min(front_now, corr_now) < max(AVOID_DIST, FRONT_STOP) or not os.path.exists(ARMED_CONT):
                    send(0.0, 0.0, "sidestep blocked; holding")
                    w.pop("sidestep_go_until", None)
                    return
                send(min(MAX_LIN, 0.08), 0.0, "sidestep go %s" % side); return
            w.pop("sidestep_go_until", None)
            w.pop("sidestep_side", None)
        # 2026-11-17 allguard: seed the trip counter from the gate's CURRENT count, so only trips during this mission
        # count as bumps (it started at 0 while the gate already had 98 -> phantom 'bump recover: back' at t=2.7 s).
        if "trips_seen" not in w:
            if not (nd.gst and now - nd.gst_t < 1.0):
                send(0.0, 0.0, "waiting for gate status"); return
            w["trips_seen"] = int(nd.gst.get("trips") or 0)
            log("gate trips at mission start:", w["trips_seen"], "(only newer trips count as bumps)")
        rs = st.get("refused_since")
        if (not w.get("reposition_active")) and rs is not None and now - rs > FR_STUCK_TRIGGER_S and now - st.get("refused_last", 0) < 1.0:
            fusion_kick(w, now, o, None, s, "every move refused %.1fs" % (now - rs))   # 2026-11-27 (only if camera agrees)
            rs = st.get("refused_since")
        if (not w.get("reposition_active")) and rs is not None and now - rs > STUCK_S and now - st.get("refused_last", 0) < 1.0:
            log("STUCK: every move refused for %.1fs -> ending mission" % (now - rs))
            raise Abort("being stuck: every move I tried for %d seconds was blocked by an obstacle. Please move me somewhere open" % int(now - rs))
        gf = (nd.gst or {}).get("front") if now - nd.gst_t < 0.5 else None
        fus = (nd.gst or {}).get("fusion") or {}
        depth_f = fus.get("depth_fwd"); depth_f = float(depth_f) if depth_f is not None else 9.0
        fv = fused_view(nd.gst) if now - nd.gst_t < 0.5 else fused_view(None)   # navstd: full sensor suite
        lm = fv["lidar"].get("mem_fwd"); dm = fv["depth"].get("mem")
        front = min(s.get("front", 9), s.get("corr", 9), gf if gf is not None else 9.0, depth_f,
                    float(lm) if lm is not None else 9.0, float(dm) if dm is not None else 9.0)
        tilt = fv["imu"].get("tilt_deg")
        if tilt is not None and float(tilt) >= OB.TILT_DEG:                   # navstd: IMU says we're climbing something
            send(0.0, 0.0, "IMU tilt %.0f deg: holding" % float(tilt)); return
        # 2026-11-26 John: front obstacle stop -> wait 3 s -> recheck -> resume if clear, else stay stopped.
        if w.get("obstacle_wait_start") or w.get("obstacle_stay_stopped"):
            waited = now - float(w.get("obstacle_wait_start") or now)
            estop_now = os.path.exists(ESTOP_CONT) or bool((nd.gst or {}).get("estop"))
            abort_now = bool(stop_flag["why"]) or os.path.exists(os.path.join(CONT_DIR, "abort"))
            armed_now = os.path.exists(ARMED_CONT)
            decision = obstacle_recheck_decision(
                waited, OBSTACLE_RECHECK_WAIT_S, front, AVOID_DIST, FRONT_STOP,
                estop=estop_now, abort=abort_now, armed=armed_now)
            if decision == "wait":
                left = max(0.0, OBSTACLE_RECHECK_WAIT_S - waited)
                send(0.0, 0.0, "obstacle wait (recheck in %.1fs)" % left); return
            if decision == "resume":
                log("OBSTACLE CLEAR after %.1fs: front %.2f -> resume explore" % (waited, front))
                w.pop("obstacle_wait_start", None)
                w.pop("obstacle_wait_until", None)
                w["obstacle_stay_stopped"] = False
                w["hold_until"] = 0
                w["recover_until"] = 0
                w["escape_until"] = 0
                (st.get("trip_hold") or {}).pop("fwd", None)
                # fall through and continue the current explore/mapping task
            else:
                uni = universal_obstacle_decision(
                    waited, OBSTACLE_RECHECK_WAIT_S, front, AVOID_DIST, FRONT_STOP,
                    left_clear_m=s.get("left_mean"), right_clear_m=s.get("right_mean"),
                    corridor_m=s.get("corr", front), estop=estop_now, abort=abort_now,
                    armed=armed_now, retries=int(w.get("avoid_retries") or 0),
                    retry_cap=OBSTACLE_AVOID_RETRY_CAP, mode="explore")
                if uni in ("sidestep_left", "sidestep_right") and int(w.get("avoid_retries") or 0) < OBSTACLE_AVOID_RETRY_CAP:
                    w["avoid_retries"] = int(w.get("avoid_retries") or 0) + 1
                    w.pop("obstacle_wait_start", None)
                    w["obstacle_stay_stopped"] = False
                    side = "left" if uni.endswith("left") else "right"
                    w["sidestep_side"] = side
                    w["sidestep_turn_until"] = now + 0.6
                    w["sidestep_go_until"] = now + 0.6 + (OBSTACLE_SIDESTEP_M / max(0.04, MAX_LIN))
                    w["sidestep_start_odom"] = o
                    log("OBSTACLE STILL BLOCKED after %.1fs: guarded sidestep %s (retry %s) at turn %.3f rad/s" % (
                        waited, side, w["avoid_retries"], MAX_ANG))
                    # fall through so this 10 Hz tick can begin the turn
                elif uni == "boxed_in":
                    if fusion_kick(w, now, o, front, s, "obstacle boxed in"):   # 2026-11-27
                        return
                    w["obstacle_stay_stopped"] = True
                    send(0.0, 0.0, "obstacle boxed in after retries -> stay stopped (no force)")
                    return
                else:
                    if fusion_kick(w, now, o, front, s, "obstacle still present"):   # 2026-11-27
                        return
                    w["obstacle_stay_stopped"] = True
                    send(0.0, 0.0, "obstacle still present after %.1fs (front %.2f) -> stay stopped" % (waited, front))
                    return
        # 2026-11-17 allguard: boxed in = forward, reverse and turning all refused -> hold zero, then end the mission
        fwd_ok = front >= FRONT_STOP and OB.ramp(min(s.get("corr", 0.0), gf if gf is not None else 9.0), MAX_LIN) > 0
        rev_ok = rev_clear(s); turn_ok = turn_clear(s, now)[0]
        # ---- detect a new gate trip / bump (IMU jolt/tilt, stall, corridor block while we were driving)
        trips = int((nd.gst or {}).get("trips") or 0)
        last = (nd.gst or {}).get("last_trip") or ""
        if trips > w.get("trips_seen", 0):
            w["trips_seen"] = trips
            # 2026-11-23 fix 3: hold the blocked direction(s). 2026-11-27: transient trips reposition instead of aborting.
            dirs = trip_dirs(last); hold = st.setdefault("trip_hold", {})
            for k in dirs: hold[k] = now + TRIP_HOLD_S
            tt = [x for x in w.get("trip_times", []) if now - x <= TRIP_WIN_S] + [now]; w["trip_times"] = tt
            log("TRIP: %s -> hold %s for %.0fs (%d trips in %.0fs, logging only; no TRIP_LIMIT abort)" % (last[:90], sorted(dirs) or "-", TRIP_HOLD_S, len(tt), TRIP_WIN_S))
            send(0.0, 0.0, "trip: stop")
            if "turn" in dirs: turn_ok = False
            if "rev" in dirs: rev_ok = False
            # 2026-11-27: a trip during an in-progress reposition just informs the maneuver (do not abort, do not nest).
            if w.get("reposition_active"):
                w["reposition_last_trip"] = last
                log("REPOSITION: gate trip during maneuver (%s) — adjusting, not aborting" % last[:90])
            else:
                kind = handle_explore_trip(last)
                if kind == "hard_abort":
                    raise Abort("a safety latch stopped me (%s)" % (last[:80] or "hardware fault"))
                if kind == "reposition":
                    # Transient front obstacle / turn-sweep / soft stall: back away + turn + resume.
                    # Do NOT start the 3 s stay-stopped wait, and do NOT abort after 3 trips in 20 s.
                    rp = start_reposition(now, o, front, s.get("sweep", 0.0), open_dir(s))
                    rp["fails"] = int(w.get("reposition_fails") or 0)
                    w["reposition"] = rp
                    w["reposition_active"] = True
                    w.pop("obstacle_wait_start", None)
                    w.pop("obstacle_wait_until", None)
                    w["obstacle_stay_stopped"] = False
                    log("REPOSITION: transient trip (%s) -> back-away/turn and resume (fails so far %d)" % (last[:80], rp["fails"]))
            if (not w.get("reposition_active")) and any(k in last.lower() for k in ("bump", "tilt", "stall", "obstacle", "jolt")):
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
                    if c == "back" and (not rev_ok or s.get("corr_rev", 0) < rear_stop + max(0.08, BACKOFF_MARGIN)): continue
                    if c.startswith("turn") and not turn_ok: continue          # 2026-11-17 allguard
                    if c.startswith("turn") and toward_bump(1 if "opp" not in c else -1): continue
                    pick = c; break
                pick = pick or "turn_opp"
                w["last_recover"] = pick
                w["recover_until"] = now + (3.5 if pick.startswith("turn") else 2.0)
                if pick == "turn_opp": w["dir"] = -(w.get("dir") or open_dir(s))
                elif pick == "turn_same": w["dir"] = open_dir(s)                 # navstd: toward the clearest known sector
                else: w["dir"] = 0                                            # back up
                w["escapes_recent"] = w.get("escapes_recent", 0) + 1
                log("bump recover:", pick, "dir", w["dir"], "front", round(front, 2), "trip", last[:80])

        if not (fwd_ok or rev_ok or turn_ok):
            if not w.get("reposition_active"):
                fusion_kick(w, now, o, front, s, "boxed in")              # 2026-11-27 (only if camera agrees)
            if not w.get("reposition_active"):
                w.setdefault("boxed_since", now)
                if now - w["boxed_since"] >= BOXED_S:
                    log("BOXED IN: front %.2f rear %.2f corr_rev %.2f sweep %.2f blind_r %.2f -> ending mission" % (
                        front, s.get("rear", 0.0), s.get("corr_rev", 0.0), s.get("sweep", 0.0), s.get("blind_r", 1.0)))
                    raise Abort("being boxed in, with front, back and turning all blocked. Please move me somewhere open")
                send(0.0, 0.0, "boxed in: holding zero"); return
            # Reposition already running: do not abort here; the fail counter will hard-abort if nothing frees us.
        else:
            w.pop("boxed_since", None)
        # ---- gated reposition after a transient obstacle / turn-sweep / soft-stall stop
        if w.get("reposition_active"):
            rp = w.get("reposition") or {}
            estop_now = os.path.exists(ESTOP_CONT) or bool((nd.gst or {}).get("estop"))
            abort_now = bool(stop_flag["why"]) or os.path.exists(os.path.join(CONT_DIR, "abort"))
            armed_now = os.path.exists(ARMED_CONT)
            sensors = {
                "front": front, "rear": s.get("rear", 0.0), "corr_rev": s.get("corr_rev", 0.0),
                "sweep": s.get("sweep", 0.0), "left": s.get("left_mean"), "right": s.get("right_mean"),
                "blind_r": s.get("blind_r", 0.0), "odom": o, "turn_ok": turn_ok, "rev_ok": rev_ok,
            }
            caps = {
                "front_stop": FRONT_STOP, "rear_stop": rear_stop, "sweep_need": OB.SWEEP_R,
                "back_v": REPOSITION_BACK_V, "back_s": REPOSITION_BACK_S, "turn_s": REPOSITION_TURN_S,
                "clear_s": REPOSITION_CLEAR_WAIT_S, "margin": BACKOFF_MARGIN,
                "blind_frac_stop": OB.BLIND_FRAC_STOP, "progress_m": REPOSITION_PROGRESS_M,
                "fail_limit": REPOSITION_FAIL_LIMIT, "armed": armed_now, "estop": estop_now,
                "abort": abort_now, "avoid_dist": AVOID_DIST,
                "nudge_lin": NUDGE_LIN, "nudge_ang": NUDGE_ANG, "nudge_s": NUDGE_S,
            }
            fo = fusion_wander_tick(w, now)                # 2026-11-27: LiDAR+camera fusion escape first
            if fo == "handled":
                st["refused_since"] = None
                return
            if fo == "recovered":
                rp["active"] = False; rp["phase"] = "done"
                out = {"event": "recovered", "why": "fusion escape: recovered, resume explore", "fails": 0,
                       "pop_holds": ["fwd", "turn"], "reason": "recovered via fusion escape"}
            else:
                out = reposition_step(rp, now, sensors, caps)
            w["reposition"] = rp
            w["reposition_fails"] = int(out.get("fails") or 0)
            hold = st.setdefault("trip_hold", {})
            for k in (out.get("pop_holds") or []):
                hold.pop(k, None)
            ev = out.get("event")
            if ev == "recovered":
                log("REPOSITION: recovered via reposition")
                w["reposition_active"] = False
                w["reposition_fails"] = 0
                w["reposition"] = rp
                w["hold_until"] = 0
                w["recover_until"] = 0
                w["escape_until"] = 0
                w["obstacle_stay_stopped"] = False
                w.pop("obstacle_wait_start", None)
                w.pop("obstacle_wait_until", None)
                w["trip_times"] = []          # successful recovery resets the transient-stop window
                st["refused_since"] = None
                send(0.0, 0.0, out.get("why") or "reposition: recovered, resume explore")
                return
            if ev == "abort":
                why = out.get("reason") or "truly stuck after failed reposition attempts"
                log("REPOSITION: %s -> ending mission" % why)
                w["reposition_active"] = False
                raise Abort("%s. Please move me somewhere more open" % why)
            if ev == "stay_stopped":
                w["reposition_active"] = False
                send(0.0, 0.0, out.get("why") or "reposition: stay stopped")
                return
            if ev == "retry":
                log("REPOSITION: %s" % (out.get("reason") or "retry"))
                w["reposition_active"] = True
                st["refused_since"] = None
                # fall through and command the next nudge immediately (no full-stop dwell)
            # running / retry: command the planned velocity (still routed through send() -> gb_gate)
            lin = float(out.get("lin") or 0.0)
            if out.get("ang") is not None:
                ang = float(out["ang"])
            else:
                ang = float(out.get("ang_dir") or 0.0) * MAX_ANG * 0.8
            send(lin, ang, out.get("why") or "reposition")
            return
        def side_slide():
            """navstd: the turn sweep is blocked by something BESIDE the tracks (side gap < ENV_SLOW) while the
            straight corridor ahead is clear -> creep straight ahead (gate-capped to V_SIDE / V_MIN) to slide out
            instead of deadlocking. Never turns toward the wall; the gate still checks every command."""
            sg = min(s.get("left_gap", 9.0), s.get("right_gap", 9.0))
            if sg < OB.ENV_SLOW and fwd_ok and hold_ok("fwd"):
                send(OB.V_MIN, 0.0, "slide out along side wall (gap %.2f)" % sg); return True
            return False
        def hold_ok(k): return (st.get("trip_hold") or {}).get(k, 0) <= now
        # ---- held after bump
        if w.get("hold_until", 0) > now:
            send(0.0, 0.0, "hold after bump"); return
        # ---- recovery action
        if w.get("recover_until", 0) > now:
            if w.get("dir") == 0:
                if rev_ok and s.get("corr_rev", 0) > rear_stop + BACKOFF_MARGIN:
                    send(-OB.V_MIN, 0.0, "recover back"); return
                w["dir"] = open_dir(s)
            if not turn_ok:                                                   # 2026-11-17 allguard: never re-command a blocked turn
                if side_slide():                                              # navstd: slide out along a side wall
                    return
                if rev_ok and s.get("corr_rev", 0) > rear_stop + BACKOFF_MARGIN:
                    send(-OB.V_MIN, 0.0, "recover back (turn sweep blocked)"); return
                w["recover_until"] = 0
                send(0.0, 0.0, "recover: turn blocked, holding"); return
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
            elif not turn_ok and side_slide():
                return
            elif not turn_ok and rev_ok and s.get("corr_rev", 0.0) > rear_stop + BACKOFF_MARGIN:
                send(-OB.V_MIN, 0.0, "back out (turn sweep blocked %.2f)" % s.get("sweep", 0.0)); return
            else:
                send(0.0, w.get("dir", 1) * MAX_ANG * 0.8, "escape turn"); return
        if front < AVOID_DIST or s.get("wide_front", 9) < AVOID_DIST * 0.7:
            # prefer the side that does NOT point at a recent bump
            left_ok = not any(dist(o, (b[0], b[1])) < 0.5 and abs(_wrap(b[2] - (o[2] + 1.2))) < 1.0 for b in w.get("bumps", [])[-5:])
            right_ok = not any(dist(o, (b[0], b[1])) < 0.5 and abs(_wrap(b[2] - (o[2] - 1.2))) < 1.0 for b in w.get("bumps", [])[-5:])
            if left_ok and (open_dir(s) > 0 or not right_ok): w["dir"] = 1
            elif right_ok: w["dir"] = -1
            else: w["dir"] = open_dir(s)
            w["escape_until"] = now + 3.5; w["escape_start"] = now
            send(0.0, w["dir"] * MAX_ANG * 0.8, "obstacle %.2f (depth/lidar) -> turn %s" % (front, "left" if w["dir"] > 0 else "right"))
            return
        if now - w.get("drift_t", 0) > 6.0:
            w["drift"] = random.uniform(-0.25, 0.25); w["drift_t"] = now; w["escapes_recent"] = 0
        side = max(-0.4, min(0.4, 0.6 * (min(s["fl"], 1.5) - min(s["fr"], 1.5))))
        lg, rg = s.get("left_gap", 9.0), s.get("right_gap", 9.0)          # navclr: body-edge side gaps (LiDAR)
        side_zone = min(lg, rg) < NAV_CLR["side_m"]
        if side_zone:                                                    # steer away from the closer side, never into it
            side = min(side, -0.15) if lg < rg else max(side, 0.15)
        # Adaptive explore speed from live multi-sector LiDAR + trusted depth (0.30-3.00 m only).
        # Obstacle/avoidance `front` above is unchanged; this only chooses wander cruise.
        # send() / OB.ramp still enforce HARD_MAX_LIN, corridor slowdown, and the 0.08 lin_floor.
        def _side(*vals):
            got = [v for v in (_clear_m(v) for v in vals) if v is not None]
            return min(got) if got else None
        gl = fv["lidar"] or {}
        front_cands = [v for v in (_clear_m(s.get("front")), _clear_m(s.get("corr")),
                                   _clear_m(gf), _clear_m(lm)) if v is not None]
        lidar_front = min(front_cands) if front_cands else None
        left_c = _side(s.get("left_mean"), s.get("left_gap"), gl.get("open_left"))
        right_c = _side(s.get("right_mean"), s.get("right_gap"), gl.get("open_right"))
        depth_raw = fus.get("depth_fwd")
        if depth_raw is None:
            depth_raw = (fv.get("depth") or {}).get("fwd")
        fresh = {}
        if nd.gst and now - nd.gst_t < 0.5:
            fresh = ((nd.gst.get("sensors") or {}).get("fresh")
                     or (nd.gst.get("fusion") or {}).get("fresh") or {})
        lidar_ok = bool(fresh.get("lidar", True)) and (now - nd.scan_t <= 0.6) and lidar_front is not None
        cruise_t = read_explore_speed_target(default=start_autonomous_cruise())
        v, regime = compute_adaptive_explore_speed(
            lidar_front=lidar_front,
            lidar_left=left_c,
            lidar_right=right_c,
            depth_front=depth_raw,
            depth_fresh=bool(fresh.get("depth")),
            lidar_fresh=lidar_ok,
            normal_speed=cruise_t,
            hard_max_lin=HARD_MAX_LIN,
        )
        v, _rwhy = apply_explore_speed_limits(
            v, lidar_front, lidar_fresh=lidar_ok, lidar_age_s=(now - nd.scan_t), hard_max_lin=HARD_MAX_LIN,
        )
        if fv["rgb"].get("hazard"): v = min(v, OB.V_RGB)                       # navstd: RGB advisory (person / furniture)
        if side_zone: v = min(v, OB.V_SIDE)                                      # navclr: slow inside the 0.35 m side zone
        if w.get("chk") is None or now - w["chk"][0] > 4.0:
            if w.get("chk") and dist(o, w["chk"][1]) < 0.03 and w.get("fwd"):
                w["dir"] = open_dir(s)
                w["escape_until"] = now + 3.0; w["escape_start"] = now
                send(0.0, w["dir"] * MAX_ANG * 0.8, "stuck -> turn"); w["chk"] = (now, o); return
            w["chk"] = (now, o)
        w["fwd"] = send(v, w.get("drift", 0) + side, "wander %s" % regime) > 0
        crumbs = w.setdefault("crumbs", [])
        if not crumbs or dist(o, crumbs[-1]) > 0.25: crumbs.append((o[0], o[1]))
        w["path"] = w.get("path", 0.0) + (dist(o, w["prev"]) if w.get("prev") else 0.0); w["prev"] = o

    def known_m2():
        m = nd.map
        if m is None: return 0.0, 0
        g = np.asarray(m.data, dtype=np.int16)
        k = int((g >= 0).sum()); return k * m.info.resolution ** 2, k

    def retrace(crumbs, start, label="return"):
        """LAST-RESORT odom breadcrumb retrace when no map pose is available.
        Preferred return is return_to_start() (map frame + live LiDAR avoidance)."""
        pts = list(reversed(crumbs)) + [(start[0], start[1])]
        # thin: keep points >= 0.3 m apart
        thin = []
        for p in pts:
            if not thin or dist(p, thin[-1]) >= 0.3: thin.append(p)
        if dist(thin[-1], start) > 0.01: thin.append((start[0], start[1]))
        length = sum(dist(thin[i], thin[i + 1]) for i in range(len(thin) - 1)) + dist(nd.odom, thin[0])
        return go_to_points(thin, label, timeout=max(60.0, 3.0 * length / MAX_LIN), frame="odom")

    def return_to_start(start_map, start_odom, crumbs=None, timeout=150.0, try_nav2=False):
        """Return home in the SLAM map frame with live LiDAR avoidance (3 s wait-recheck-sidestep).
        Never a blind straight-line dash. Odom breadcrumb retrace is last resort only."""
        # Refresh map pose briefly
        mp = None
        t_w = time.time()
        while time.time() - t_w < 2.0:
            mp = nd.map_pose()
            if mp:
                break
            time.sleep(0.1)
        if should_use_map_return("map" if start_map else None, mp) and start_map:
            pts = return_target_points(start_map)
            if try_nav2 and use_nav2:
                ok, why = nav_goto(start_map[0], start_map[1], start_map[2], "return", timeout=min(150.0, timeout))
                res["return_via"] = "nav2"
                log("nav2 return:", ok, why)
                if ok:
                    return ok, why
            res["return_via"] = "map-frame"
            log("map-frame return to (%.2f, %.2f) with live LiDAR avoidance" % (pts[0][0], pts[0][1]))
            return go_to_points(pts, "return", timeout=timeout, final_tol=0.12, frame="map")
        res["return_via"] = "odom-retrace"
        log("map pose unavailable - last-resort odom retrace")
        return retrace(crumbs or [], start_odom)

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
        rf = gh.get_result_async(); phase(label); nb = None; nfr = 0
        try:
            while not rf.done():
                if time.time() - t0 > timeout:
                    gh.cancel_goal_async(); return False, "timeout"
                lin, ang, t = nd.nav_cmd
                if time.time() - t > 0.5: lin = ang = 0.0
                got = send(lin, ang, "nav2")
                if lin > 0.01 and got == 0.0:                 # 2026-11-27: Nav2 wants forward but we are stuck
                    nb = nb or time.time()
                    if time.time() - nb > 1.0 and nfr < FR_MAX_PER_GOAL:
                        _r = fusion_escape_run(label + "_nav2", goal_bearing=lambda: _goal_bearing((x, y), True))
                        if _r != "not_started": nfr += 1
                        nb = None
                else:
                    nb = None
                time.sleep(0.1)
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
        # 2026-11-26: remember start in the SLAM map frame whenever TF is up (IMU yaw when fresh).
        start_map = None
        t_ms = time.time()
        while time.time() - t_ms < 3.0:
            start_map = nd.map_pose()
            if start_map:
                break
            time.sleep(0.1)
        if start_map and nd.imu_fresh():
            y, _src = yaw_with_imu(nd.imu_yaw, start_map[2], start[2] if start else None, True)
            if y is not None:
                start_map = (start_map[0], start_map[1], y)
        res["start_map"] = [round(v, 3) for v in start_map] if start_map else None
        start_home = prefer_map_start(start_map, start)
        res["start_frame"] = (start_home or {}).get("frame")
        log("plan", json.dumps(plan), "dry" if dry else "LIVE", "nav2" if use_nav2 else "no-nav2",
            "caps lin", MAX_LIN, "ang", MAX_ANG, "start_frame", res["start_frame"])
        zero(4, "start")
        mis = plan["mission"]
        if mis == "distance":
            D = float(plan["distance_m"]); phase("out"); t0 = time.time(); blocked = 0.0; fr_out = 0
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
                    if blocked >= 0.5 and fr_out < FR_MAX_PER_GOAL:   # 2026-11-27 LiDAR+camera stuck-recovery
                        _r = fusion_escape_run("out", goal_bearing=lambda: _wrap(start[2] - (pose_now() or nd.odom)[2]))
                        if _r != "not_started": fr_out += 1
                        if _r == "recovered": blocked = 0.0; continue
                    if blocked > 5.0: res["blocked_out"] = True; break
                else: blocked = 0.0
                time.sleep(0.1)
            res["dist_out_m"] = round(dist(nd.odom, start), 3)
            phase("stop"); zero(20, "stop at far point"); time.sleep(0.5)
            if plan.get("return"):
                ok, why = return_to_start(start_map, start, crumbs=[], timeout=res["dist_out_m"] / 0.04 + 40)
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    park_yaw = (start_map[2] if start_map else start[2])
                    park_frame = "map" if start_map else "odom"
                    err = rotate_to(park_yaw, "park", frame=park_frame); res["parked"] = err < 0.15
                err_m = pose_error_m(pose_now(prefer_map=bool(start_map)), start_map or start)
                res["return_error_m"] = round(err_m if err_m is not None else dist(nd.odom, start), 3)
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
                ok, why = return_to_start(start_map, start, crumbs=w.get("crumbs", []),
                                          timeout=max(60.0, 3.0 * max(1.0, w.get("path", 1.0)) / max(0.04, MAX_LIN)))
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    park_yaw = (start_map[2] if start_map else start[2])
                    park_frame = "map" if start_map else "odom"
                    err = rotate_to(park_yaw, "park", frame=park_frame); res["parked"] = err < 0.15
                err_m = pose_error_m(pose_now(prefer_map=bool(start_map)), start_map or start)
                res["return_error_m"] = round(err_m if err_m is not None else dist(nd.odom, start), 3)
        elif mis == "explore":
            t_map = time.time()
            while nd.map is None and time.time() - t_map < 15:
                zero(1, "waiting for /map"); time.sleep(0.2)
            if start_map is None:
                for _ in range(30):
                    start_map = nd.map_pose()
                    if start_map: break
                    zero(1, "waiting for map tf"); time.sleep(0.1)
                if start_map and nd.imu_fresh():
                    y, _src = yaw_with_imu(nd.imu_yaw, start_map[2], start[2] if start else None, True)
                    if y is not None:
                        start_map = (start_map[0], start_map[1], y)
                res["start_map"] = [round(v, 3) for v in start_map] if start_map else None
                if start_map:
                    start_home = prefer_map_start(start_map, start)
                    res["start_frame"] = (start_home or {}).get("frame")
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
            ps = max(0.0, min(15.0, float(plan.get("pause_s") or 0)))   # 2026-11-25 benchtest: "sleep" before returning
            if ps:
                phase("sleep"); t_s = time.time()
                while time.time() - t_s < ps: zero(2, "sleep")
                res["sleep_s"] = round(time.time() - t_s, 1)
            if plan.get("return", True):
                ok, why = return_to_start(start_map, start, crumbs=w.get("crumbs", []),
                                          timeout=150.0, try_nav2=True)
                res["returned"] = ok; res["return_reason"] = why
                if ok:
                    park_yaw = (start_map[2] if start_map else start[2])
                    park_frame = "map" if start_map else "odom"
                    err = rotate_to(park_yaw, "park", frame=park_frame); res["parked"] = err < 0.15
                err_m = pose_error_m(pose_now(prefer_map=bool(start_map)), start_map or start)
                res["return_error_m"] = round(err_m if err_m is not None else dist(nd.odom, start), 3)
        res["outcome"] = "done"; phase("done")
    except Abort as a:
        res["outcome"] = "aborted"; res["reason"] = str(a); log("ABORT:", a)
    except Exception as e:
        import traceback; traceback.print_exc()
        res["outcome"] = "error"; res["reason"] = repr(e)
    finally:
        _clear_guard_flag(GUARD_FLAG_CONT)                  # re-arm gb-guard first, before anything else
        try:                                                # 2026-11-27: tight-squeeze clearance can never outlive the mission
            if FRM is not None and FR_TOKEN: FRM.clear_token(FR_TOKEN)
        except Exception:
            pass
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
        st["phase"] = "finishing"                           # 2026-11-25 benchtest: host allows 8 s for teardown
        try:
            with open(os.path.join(CONT_DIR, "hb.tmp"), "w") as f: json.dump({"t": time.time(), "phase": "finishing"}, f)
            os.replace(os.path.join(CONT_DIR, "hb.tmp"), os.path.join(CONT_DIR, "hb"))
        except Exception: pass
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
    ap.add_argument("--dump-params", action="store_true", help="print effective explore params (learned overlay + clamps) and exit")
    a = ap.parse_args(argv)
    if a.dump_params:
        print(json.dumps(apply_learned_params(log=False), indent=2, sort_keys=True)); return 0
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
