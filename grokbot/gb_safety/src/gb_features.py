"""gb_features.py - Hiwonder MentorPi stock features for Grok Bot (driving, line following, colour tracking,
gesture control, LiDAR modes, lights, buzzer, status, navigation check), via rosbridge (ws://127.0.0.1:9090).

Safety rules (do not relax):
 * every drive command is capped at MAX_DRIVE_S (5 s) in total and gated by ALLOW_MOTION;
 * every motion ends with an explicit zero Twist burst on /controller/cmd_vel, sent twice (STOP_REPEAT msgs each);
 * emergency_stop() publishes zero velocity first, cancels any drive in progress, stops every Hiwonder feature
   (whether or not we started it), then publishes zero again;
 * a feature that drives the robot is auto-stopped after FEATURE_MAX_S;
 * dry runs (API robot:false / dry_run:true) never publish a non-zero velocity and never set a feature running.
"""
import json, os, re, threading, time
import gb_robot

E = os.environ.get
CMD_VEL = E("CMD_VEL_TOPIC", "/controller/cmd_vel")
ALLOW_MOTION = E("ALLOW_MOTION", "0") == "1"
MAX_DRIVE_S = min(5.0, float(E("MAX_DRIVE_S", "5")))
MAX_LIN = float(E("MAX_LIN", "0.12")); MAX_ANG = float(E("MAX_ANG", "0.64"))     # -20% caps (was 0.2 / 1.0)
STOP_REPEAT = int(E("STOP_REPEAT", "10"))
FEATURE_MAX_S = float(E("FEATURE_MAX_S", "60"))
SCAN_TOPICS = [t.strip() for t in E("SCAN_TOPICS", "/scan_raw,/scan").split(",") if t.strip()]
COLORS = ["red", "green", "blue", "black", "white", "dark_green"]      # Mono table in lab_config.yaml (aurora camera)
TWIST0 = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

_local = threading.local()
def set_dry(v): _local.dry = bool(v)
def is_dry(): return getattr(_local, "dry", False)

def log(*a): print(time.strftime("%H:%M:%S"), "[features]", *a, flush=True)

# ------------------------------------------------------------------ rosbridge
def call(service, typ, args=None, timeout=6.0):
    """Call a ROS service via rosbridge; returns (ok, values_dict_or_error)."""
    try:
        ws = gb_robot._ws(timeout)
    except Exception as ex:
        return False, f"rosbridge unreachable: {ex}"
    try:
        cid = f"gb{time.time_ns()}"
        ws.send(json.dumps({"op": "call_service", "id": cid, "service": service, "type": typ, "args": args or {}}))
        t0 = time.time(); ws.settimeout(1.0)
        while time.time() - t0 < timeout:
            try: m = json.loads(ws.recv())
            except Exception: continue
            if m.get("op") == "service_response" and m.get("id") == cid:
                return bool(m.get("result", True)), m.get("values")
        return False, "timeout"
    finally:
        try: ws.close()
        except Exception: pass

def zero_burst(n=STOP_REPEAT, hz=20):
    """Publish zero velocity n times. Never raises; returns True if sent."""
    try:
        gb_robot.publish(CMD_VEL, "geometry_msgs/Twist", TWIST0, repeat=n, hz=hz); return True
    except Exception as ex:
        log("zero burst failed:", ex); return False

def safe_stop():
    ok1 = zero_burst(); time.sleep(0.25); ok2 = zero_burst()
    log(f"zero-velocity stop sent ({2 * STOP_REPEAT} msgs) ok={ok1 and ok2}")
    return ok1 or ok2

def topic_alive(topic, typ, timeout=2.0):
    return bool(gb_robot.subscribe_once(topic, typ, timeout=timeout))

def lidar_alive():
    return any(topic_alive(t, "sensor_msgs/LaserScan", 2.0) for t in SCAN_TOPICS)

# ------------------------------------------------------------------ driving (cancelable)
DRIVE = {"cancel": threading.Event(), "lock": threading.Lock(), "busy": False}

def clamp_steps(steps):
    """steps: [(lin, ang, secs)] -> clamped, total <= MAX_DRIVE_S."""
    out, total = [], 0.0
    try:
        import gb_caps; k = gb_caps.speed_mult()            # voice speed setting 50-100 %
    except Exception:
        k = 1.0
    for lin, ang, secs in steps:
        lin = max(-MAX_LIN, min(MAX_LIN, float(lin) * k)); ang = max(-MAX_ANG, min(MAX_ANG, float(ang) * k))
        secs = max(0.0, float(secs))
        if lin == 0 and ang == 0:
            continue
        secs = min(secs, MAX_DRIVE_S - total)
        if secs < 0.1:
            break
        out.append((round(lin, 3), round(ang, 3), round(secs, 2))); total += secs
    return out

def _drive_worker(steps):
    import gb_estop                                   # every step is a gate timed request: the GATE enforces each deadline
    ws = None; rid = None
    try:
        t_end_all = time.time() + MAX_DRIVE_S + 0.05
        for lin, ang, secs in steps:
            if gb_estop.is_set(): break
            secs = min(secs, max(0.0, t_end_all - time.time()))
            if secs < 0.1: break
            rid = gb_estop.gate_request({"op": "timed", "lin": lin, "ang": ang, "dur": secs, "src": "features.drive"})
            if DRIVE["cancel"].wait(secs):
                break
    except Exception as ex:
        log("drive error:", ex)
    finally:
        try:
            if rid: gb_estop.gate_request({"op": "stop", "only_id": rid, "src": "features.drive"})
        except Exception:
            pass
        safe_stop()                                   # second, independent connection
        DRIVE["busy"] = False
        log("drive finished, stopped")

def drive(steps, dry=None):
    """Start a (clamped) drive sequence in the background. Returns (ok, message, steps)."""
    dry = is_dry() if dry is None else dry
    steps = clamp_steps(steps)
    if not steps:
        return False, "I didn't get a movement out of that.", steps
    import gb_caps
    blk = gb_caps.motion_block("drive")
    if blk:
        return False, blk, steps
    if not ALLOW_MOTION:
        return False, "Driving by voice is switched off. Set ALLOW_MOTION to 1 in my settings to allow it.", steps
    total = sum(s[2] for s in steps)
    if dry:
        log(f"DRY RUN drive {steps} total {total:.1f}s (no velocity published)")
        return True, f"Dry run: I would {describe_steps(steps)}.", steps
    DRIVE["cancel"].set()
    with DRIVE["lock"]:
        while DRIVE["busy"]:
            time.sleep(0.05)
        DRIVE["cancel"] = threading.Event(); DRIVE["busy"] = True
        log(f"drive {steps} total {total:.1f}s")
        threading.Thread(target=_drive_worker, args=(steps,), daemon=True).start()
    return True, f"Okay, I'll {describe_steps(steps)}.", steps

def describe_steps(steps):
    parts = []
    for lin, ang, secs in steps:
        if lin and ang: d = ("drive forward" if lin > 0 else "back up") + (" curving left" if ang > 0 else " curving right")
        elif lin: d = "drive forward" if lin > 0 else "back up"
        else: d = "turn left" if ang > 0 else "turn right"
        parts.append(f"{d} for {secs:g} second{'' if secs == 1 else 's'}")
    return ", then ".join(parts)

# ------------------------------------------------------------------ Hiwonder app features
TRIG = "std_srvs/srv/Trigger"; SBOOL = "std_srvs/srv/SetBool"; SSTR = "large_models_msgs/srv/SetString"
ACTIVE = {}                                           # name -> {"t": start, "dry": bool, "timer": Timer}
NAMES = {"line": "line following", "color": "colour tracking", "gesture": "gesture control",
         "lidar": "LiDAR mode"}

def _chk(results):
    bad = [f"{s}: {v}" for s, (ok, v) in results if not ok or (isinstance(v, dict) and v.get("success") is False)]
    return (not bad), bad

def _arm_timeout(name):
    t = threading.Timer(FEATURE_MAX_S, lambda: (log(f"{name} auto-stop after {FEATURE_MAX_S:g}s"), stop_feature(name)))
    t.daemon = True; t.start(); return t

def norm_color(c):
    c = (c or "").lower().strip().replace("dark green", "dark_green")
    c = {"grey": "black", "gray": "black", "purple": "blue", "orange": "red", "pink": "red"}.get(c, c)
    return c if c in COLORS else None

def start_feature(name, color=None, mode=None, dry=None):
    """name: line | color | gesture | lidar. Returns (ok, message)."""
    dry = is_dry() if dry is None else dry
    import gb_caps
    blk = gb_caps.motion_block("lidar_feature" if name == "lidar" else "camera_feature")
    if blk:
        return False, blk
    if not ALLOW_MOTION and not dry:
        return False, f"{NAMES[name].capitalize()} drives the robot, and driving is switched off. Set ALLOW_MOTION to 1."
    if not dry and E("ALLOW_STOCK_DRIVE", "0") != "1":   # 2026-11-05: Hiwonder apps publish /controller/cmd_vel directly
        return False, (f"{NAMES[name].capitalize()} is Hiwonder's own app. It drives the motors directly, outside my obstacle "
                       "sensors, so I keep it switched off. Ask me to explore or wander instead.")
    for other in list(ACTIVE):                       # one driving feature at a time
        stop_feature(other)
    safe_stop() if not dry else zero_burst(3)
    res = []
    if name == "line":
        color = norm_color(color) or "black"
        res.append(("line_following/enter", call("/line_following/enter", TRIG)))
        if not dry:
            res.append(("set_large_model_target_color", call("/line_following/set_large_model_target_color", SSTR, {"data": color})))
            res.append(("set_running", call("/line_following/set_running", SBOOL, {"data": True})))
        what = f"following the {color.replace('_', ' ')} line"
    elif name == "color":
        color = norm_color(color)
        if not color:
            return False, "Which colour should I track? I know red, green, blue, black and white."
        res.append(("object_tracking/enter", call("/object_tracking/enter", TRIG)))
        res.append(("set_large_model_target_color", call("/object_tracking/set_large_model_target_color", SSTR, {"data": color})))
        if not dry:
            res.append(("set_running", call("/object_tracking/set_running", SBOOL, {"data": True})))
        what = f"tracking {color.replace('_', ' ')}"
    elif name == "gesture":
        res.append(("hand_trajectory/enter", call("/hand_trajectory/enter", TRIG)))
        res.append(("hand_gesture/enter", call("/hand_gesture/enter", TRIG)))
        if not dry:
            res.append(("hand_gesture/set_running", call("/hand_gesture/set_running", SBOOL, {"data": True})))
        what = "gesture control. Move one finger in front of the camera: up and down drives, left and right turns"
    elif name == "lidar":
        modes = {"avoid": 1, "follow": 2, "guard": 3}
        if not lidar_alive():
            return False, ("That mode needs the LiDAR, and it isn't publishing scans right now, so I can't start "
                           f"{ {'avoid': 'obstacle avoidance', 'follow': 'LiDAR following', 'guard': 'LiDAR guarding'}.get(mode, 'it') }.")
        res.append(("lidar_app/enter", call("/lidar_app/enter", TRIG)))
        if not dry:
            res.append(("lidar_app/set_running", call("/lidar_app/set_running", "interfaces/srv/SetInt64", {"data": modes.get(mode, 1)})))
        what = {"avoid": "obstacle avoidance", "follow": "LiDAR following", "guard": "LiDAR guarding"}.get(mode, "LiDAR mode")
    else:
        return False, "I don't know that feature."
    ok, bad = _chk(res)
    log(f"start {name} color={color} mode={mode} dry={dry} ok={ok} " + "; ".join(f"{s}={v}" for s, v in res))
    if not ok:
        stop_feature(name, quiet=True)
        return False, f"I couldn't start {NAMES[name]}: {bad[0][:120]}."
    ACTIVE[name] = {"t": time.time(), "dry": dry, "timer": None if dry else _arm_timeout(name)}
    if dry:
        return True, f"Dry run: {NAMES[name] if name != 'lidar' else what} is armed but not driving, so I'm not moving. Tell me to end it."
    return True, f"Okay, {what}. Tell me to end it. I'll end it by myself after {int(FEATURE_MAX_S)} seconds."

def _stop_calls(name):
    if name == "line":
        return [("/line_following/set_running", SBOOL, {"data": False}), ("/line_following/exit", TRIG, None)]
    if name == "color":
        return [("/object_tracking/set_running", SBOOL, {"data": False}), ("/object_tracking/exit", TRIG, None)]
    if name == "gesture":
        return [("/hand_gesture/set_running", SBOOL, {"data": False}), ("/hand_gesture/exit", TRIG, None),
                ("/hand_trajectory/exit", TRIG, None)]
    if name == "lidar":
        return [("/lidar_app/exit", TRIG, None)]
    return []

def stop_feature(name, quiet=False):
    a = ACTIVE.pop(name, None)
    if a and a.get("timer"): a["timer"].cancel()
    zero_burst()
    res = [(s, call(s, t, args, timeout=4)) for s, t, args in _stop_calls(name)]
    safe_stop()
    if not quiet:
        log(f"stop {name}: " + "; ".join(f"{s}={v}" for s, v in res))
    return all(ok for _, (ok, _) in res)

def emergency_stop():
    """Zero velocity immediately, cancel drives, exit every feature we know, zero again."""
    DRIVE["cancel"].set()
    zero_burst()
    started = list(ACTIVE)
    for name in started:
        stop_feature(name, quiet=False)
    # also quietly halt the stock nodes we didn't start (cheap: SetBool false only, no exit)
    def _others():
        for s in ("/line_following/set_running", "/object_tracking/set_running", "/hand_gesture/set_running"):
            if not any(s.startswith("/" + {"line": "line_following", "color": "object_tracking", "gesture": "hand_gesture",
                                           "lidar": "lidar_app"}[n]) for n in started):
                call(s, SBOOL, {"data": False}, timeout=3)
        safe_stop()
    threading.Thread(target=_others, daemon=True).start()
    if not started:
        safe_stop()
    return started

def active_names():
    return [NAMES[n] + (" (dry run)" if ACTIVE[n]["dry"] else "") for n in ACTIVE]

# ------------------------------------------------------------------ lights / buzzer
LIGHTS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255), "white": (255, 255, 255),
          "yellow": (255, 180, 0), "purple": (160, 0, 255), "orange": (255, 80, 0), "pink": (255, 60, 120),
          "cyan": (0, 255, 255), "off": (0, 0, 0)}

def set_lights(color):
    c = (color or "").lower().strip()
    if c not in LIGHTS:
        return False, f"I can make the lights {', '.join(k for k in LIGHTS if k != 'off')}, or turn them off."
    r, g, b = LIGHTS[c]
    msg = {"states": [{"index": 1, "red": r, "green": g, "blue": b}, {"index": 2, "red": r, "green": g, "blue": b}]}
    try:
        gb_robot.publish("/ros_robot_controller/set_rgb", "ros_robot_controller_msgs/RGBStates", msg, repeat=2, hz=10)
    except Exception as ex:
        return False, f"I couldn't reach the light controller: {ex}"
    log(f"lights {c} {r},{g},{b}")
    return True, "Lights off." if c == "off" else f"Lights are {c}."

def beep(times=1):
    times = max(1, min(int(times), 5))
    try:
        gb_robot.publish("/ros_robot_controller/set_buzzer", "ros_robot_controller_msgs/BuzzerState",
                         {"freq": 1900, "on_time": 0.1, "off_time": 0.1, "repeat": times}, repeat=1)
    except Exception as ex:
        return False, f"I couldn't reach the buzzer: {ex}"
    log(f"beep x{times}")
    return True, "Beep!" if times == 1 else f"Beeped {times} times."

# ------------------------------------------------------------------ status / navigation
def status_report(battery_text):
    cam = topic_alive("/ascamera/camera_publisher/rgb0/camera_info", "sensor_msgs/CameraInfo", 2.5)
    odom = topic_alive("/odom", "nav_msgs/Odometry", 2.5)
    lid = lidar_alive()
    parts = [battery_text, f"Camera {'is working' if cam else 'is not publishing'}.",
             f"Wheel odometry {'is working' if odom else 'is not publishing'}.",
             f"The LiDAR {'is working' if lid else 'is not publishing scans, so mapping, navigation and the LiDAR modes are unavailable'}."]
    act = active_names()
    parts.append(("Running now: " + ", ".join(act) + ".") if act else "No driving features are running.")
    parts.append(f"Voice driving is {'on' if ALLOW_MOTION else 'off'}.")
    return " ".join(parts)

def navigate(target, store=None):
    import gb_caps
    blk = gb_caps.motion_block("mission")
    if blk:
        return False, blk
    labs = store.find_labels(target) if store is not None and target else []
    where = ""
    if labs and labs[-1]["map_x"] is not None:
        r = labs[-1]; where = f" I know the {r['name']} is at x {r['map_x']:.1f}, y {r['map_y']:.1f}."
    if not lidar_alive():
        return False, (f"I can't navigate yet.{where} Navigation needs the LiDAR and a saved map, and the LiDAR "
                       "isn't publishing scans right now.")
    ok, v = call("/rosapi/action_servers", "rosapi/srv/GetActionServers", timeout=5)   # nav2 running?
    servers = (v or {}).get("action_servers", []) if ok and isinstance(v, dict) else []
    if not any("navigate_to_pose" in s for s in servers):
        return False, f"I can't drive to {target or 'that place'} yet.{where} The navigation stack isn't running. Start Hiwonder's navigation mode with a saved map first."
    return False, f"Navigation is running, but driving to named places by voice isn't hooked up yet.{where}"
