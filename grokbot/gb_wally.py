"""gb_wally.py - voice-facing Wally behaviours that must not wait on the network.

Handles teaching, pointed missions, the 2 m follow, sensor-health questions,
and the rule that saying "I am John" is not authentication. Emergency stop
and unlock still live in gb_estop; this module only drops goals when asked.
"""
import json, os, re, tempfile, time
import gb_follow
import gb_health
import gb_point
import gb_teach

# Software wake is the exact phrase "wake up Wally" (case and punctuation ignored).
# "Hello Wally", "Hey Wally", and a bare "wally" are not wakes. The WonderEcho
# factory phrase "Hello Hiwonder" is a separate hardware signal and is not this grammar.
EXACT_WAKE = ("wake", "up", "wally")
WALLY_LIKE = {"wally", "wallie", "wali", "walley", "wallys"}
NO_AUTH = "I'll call you John if you like. Saying that does not sign you in, and it doesn't unlock anything."


def accept_hardware_event():
    """The factory WonderEcho event is not a software wake. The chip is not reflashed."""
    return False


def software_wake(text, hardware_event=False):
    """True only for the spoken phrase. A serial wake event does not count."""
    if hardware_event and not accept_hardware_event():
        hardware_event = False
    decided = classify_wake(text)
    return bool(decided and decided[0] is True)

def classify_wake(text):
    """Return (matched, remainder) when the utterance mentions wally, else None.

    The only match is the token sequence wake, up, wally. Fuzzy similarity and a
    bare name are not used, so "hello wally" cannot pass.
    """
    raw = [w for w in re.split(r"[\s,]+", (text or "").replace("-", " ")) if w]
    norms, kept = [], []
    for w in raw:
        n = re.sub(r"[^a-z0-9]+", "", w.lower())
        if n:
            norms.append(n)
            kept.append(w)
    for i in range(0, min(6, max(0, len(norms) - 2))):
        if tuple(norms[i:i + 3]) == EXACT_WAKE:
            return True, " ".join(kept[i + 3:]).strip(" ,.!?")
    if any(n in WALLY_LIKE for n in norms[:8]):
        return False, ""
    return None

_health = gb_health.Health()
_teach = None
_mission = None
_capture = None
_observer = None
_autonomy = os.environ.get("ALLOW_AUTONOMY", "0") == "1"
_shutdown_calls = []


def health():
    return _health


def teacher():
    global _teach
    if _teach is None:
        path = os.environ.get("GB_TEACH_DB") or os.path.join(tempfile.gettempdir(), "wally_teach.db")
        _teach = gb_teach.Teacher(path)
    return _teach


def mission():
    global _mission
    if _mission is None:
        path = os.environ.get("GB_MISSION_STATUS") or os.path.join(tempfile.gettempdir(), "wally_mission_status.json")
        _mission = gb_point.PointMission(path)
        _mission.load()
    return _mission


def set_capture(fn):
    global _capture
    _capture = fn


def set_observer(fn):
    global _observer
    _observer = fn


def set_autonomy(on):
    global _autonomy
    _autonomy = bool(on)


def vision_may_move(_text=None):
    """Image descriptions, including Grok's, are not permission to move."""
    return False


def drop_goals(why):
    try:
        mission().cancel(why)
    except Exception:
        pass
    return True


def note_sensors(lidar=None, depth=None, rgb=None, now=None):
    return _health.update(lidar, depth, rgb, now)


def _default_capture():
    if _capture:
        return _capture()
    return ([[0, 1] * 4 for _ in range(8)], None)


def _obs():
    if _observer:
        return _observer()
    return {"candidates": [], "person": None, "localized": False, "start": {"x": 0, "y": 0, "yaw": 0}}


_runtime = False
_publisher = None
_follow = None
_rt_stop = None
_rt_thread = None


def follow_loop():
    global _follow
    if _follow is None:
        _follow = gb_follow.FollowLoop()
    return _follow


def install_runtime(capture=None, publisher=None, start_thread=False):
    """Called by grokbot_voice.main. Tests leave the thread off so they never touch rosbridge."""
    global _runtime, _publisher
    _runtime = True
    if capture is not None:
        set_capture(capture)
    _publisher = publisher
    if start_thread:
        _start_runtime_thread()


def _start_runtime_thread():
    global _rt_stop, _rt_thread
    if _rt_thread is not None:
        return
    _rt_stop = __import__("threading").Event()

    def _loop():
        while not _rt_stop.wait(0.2):
            try:
                tick_runtime()
            except Exception:
                pass

    _rt_thread = __import__("threading").Thread(target=_loop, daemon=True)
    _rt_thread.start()


def tick_runtime(now=None):
    """One follow step and one mission step. Publishes only through the injected gate publisher."""
    now = time.time() if now is None else float(now)
    protections = {}
    try:
        import gb_estop
        import gb_maint
        if gb_estop.is_set():
            protections["estop"] = True
        maint = gb_maint.is_set()
    except Exception:
        maint = False
    fl = follow_loop()
    if fl.active:
        track = read_person_track(os.environ.get("PERSON_TRACK_FILE", ""), now)
        cmd = fl.step(now, gb_health.read_shared(now=now), track, _autonomy, protections, maint=maint)
        if cmd.get("allow") and (cmd.get("lin") or cmd.get("ang")):
            _publish(cmd)
        else:
            _publish({"op": "stop", "allow": False, "reason": cmd.get("reason")})
    m = mission()
    if m.phase != "idle":
        if maint or protections.get("estop"):
            m.cancel("maintenance" if maint else "estop")
            _publish({"op": "stop", "allow": False, "reason": "maintenance" if maint else "estop"})
        else:
            obs = _obs()
            if obs.get("from_llm") or obs.get("from_vision"):
                m.cancel("ungrounded")
                _publish({"op": "stop", "allow": False, "reason": "ungrounded"})
            else:
                step = m.tick(0.2, obs, _health, protections)
                if step.get("lin") or step.get("ang"):
                    _publish({"allow": True, "op": "timed", "lin": step["lin"], "ang": step.get("ang") or 0.0})
                else:
                    _publish({"op": "stop", "allow": False, "reason": step.get("why") or step.get("phase")})


def read_person_track(path, now=None):
    """A person range from a file the parent points at. No path means no range, not a guess."""
    if not path:
        return None
    now = time.time() if now is None else float(now)
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    p = d.get("person") if isinstance(d, dict) else None
    if not isinstance(p, dict):
        return None
    try:
        age = now - float(p["t"])
        dist = float(p["depth_m"])
    except (KeyError, TypeError, ValueError):
        return None
    if age < 0 or age > 0.5 or dist != dist or dist <= 0:
        return None
    if p.get("unit") not in gb_health.VALIDATED_DEPTH_UNITS and p.get("source") != "lidar":
        return None
    source = "lidar" if p.get("source") == "lidar" else "depth"
    try:
        conf = float(p["conf"]) if "conf" in p else None
    except (TypeError, ValueError):
        conf = None
    return {"depth_m": dist, "source": source, "fresh": True, "bearing": float(p.get("bearing") or 0.0),
            "conf": conf, "ambiguous": bool(p.get("ambiguous")), "unit": p.get("unit"),
            "depth_age": age if source == "depth" else 0.0}


def follow_once(health_snap, track, autonomy, protections=None):
    """One follow decision. Does not invent a distance from a scan median and does not loop."""
    protections = protections or {}
    if protections.get("estop") or protections.get("tip_hard") or protections.get("tip_soft") or protections.get("battery_shutdown") or protections.get("watchdog"):
        return gb_follow.follow_command(None, "none", autonomy=bool(autonomy), protections=protections)
    if not autonomy:
        return gb_follow.follow_command(None, "none", autonomy=False, protections=protections)
    if not gb_health.autonomy_allowed(health_snap):
        return {"lin": 0.0, "ang": 0.0, "speech": "I won't follow until the range check is fresh.",
                "allow": False, "reason": "health"}
    if not track:
        return {"lin": 0.0, "ang": 0.0, "speech": "I don't have a measured distance to a person, so I won't follow.",
                "allow": False, "reason": "no_person_range"}
    return gb_follow.follow_command(track["depth_m"], track["source"], track.get("bearing") or 0.0, True,
                                     autonomy=True, protections=protections)


def _publish(cmd):
    if not _runtime or _publisher is None:
        return None
    return _publisher(cmd)


def handle(text, protections=None):
    """Return a spoken reply, or None to let the rest of the voice stack decide.

    Never calls shutdown. Never treats an identity sentence as a credential.
    """
    raw = (text or "").strip()
    t = re.sub(r"\s+", " ", raw.lower()).strip(" .!?")
    woken = classify_wake(raw)
    if woken == (False, ""):
        return None
    if woken and woken[0]:
        t = re.sub(r"\s+", " ", (woken[1] or "").lower()).strip(" .!?") or t
    t = re.sub(r"^(grok bot|grok|robot)[, ]+", "", t).strip()
    if re.match(r"^(i am|i'm|im) john$", t) or t in ("i am john", "this is john"):
        return NO_AUTH
    if re.search(r"\b(reboot|restart yourself|power off the pi|shut ?down the pi)\b", t):
        return "I don't reboot or shut down from a message. That stays on the Pi, with you there."
    if "what is your load budget" in t or "cpu budget" in t:
        return "Optional vision stands down above a load of %.1f. Stopping, the battery monitor, and the tip cutoff stay on." % gb_health.LOAD_SHED
    taught = teacher().hear(t, _default_capture)
    if taught:
        return taught
    if re.search(r"\b(follow me|follow john|stay with me)\b", t):
        try:
            import gb_maint
            if gb_maint.is_set():
                return "The maintenance lock is on, so I won't follow."
        except Exception:
            return "I couldn't check the maintenance lock, so I won't follow."
        protections = dict(protections or {})
        try:
            import gb_estop
            if gb_estop.is_set():
                protections["estop"] = True
        except Exception:
            protections["estop"] = True
        if _observer and not os.environ.get("PERSON_TRACK_FILE"):
            person = (_obs().get("person") or {})
            track = person if person.get("fresh") else None
            if track is not None and "conf" not in track:
                track = dict(track, conf=track.get("conf"))
        else:
            track = read_person_track(os.environ.get("PERSON_TRACK_FILE", ""))
        try:
            import gb_demand
            gb_demand.write_demand(os.environ.get("GB_DEMAND_FILE", "/home/pi/docker/tmp/gb_safety/demand.json"), "follow")
        except Exception:
            pass
        fl = follow_loop()
        now = time.time()
        fl.start(now)
        cmd = fl.step(now, gb_health.read_shared(now=now), track, _autonomy, protections, maint=False)
        mission().commands.append({"lin": cmd.get("lin") or 0.0, "ang": cmd.get("ang") or 0.0, "why": "follow"})
        if cmd.get("allow") and (cmd.get("lin") or cmd.get("ang")):
            _publish(cmd)
        elif cmd.get("op") == "stop" and cmd.get("reason") not in ("person_lost", "person_gap", "autonomy_off", "confidence"):
            _publish({"op": "stop", "allow": False, "reason": cmd.get("reason")})
        if cmd.get("speech"):
            return cmd["speech"]
        if cmd.get("lin") or cmd.get("ang"):
            return "I'll keep about 2 metres back."
        return "I'm holding about 2 metres."
    m = re.search(r"\b(?:go to|drive to|head to) (?:the |that )?([a-z0-9 ]+?)(?: and wait (\d+) seconds?)?$", t)
    if m and not re.search(r"\b(sleep|bed|left|right|forward|back)\b", m.group(1)):
        dwell = float(m.group(2) or 5)
        res = mission().start(m.group(1).strip(), dwell, _obs(), _health, autonomy=_autonomy, protections=protections)
        return res.get("say")
    if t in ("cancel", "cancel that", "never mind the trip"):
        mission().cancel("cancel")
        return "Cancelled. I'm staying put."
    if re.search(r"\b(what is this|what do you see)\b", t):
        if not gb_health.under_budget(_load()):
            rec = {"kind": "shed"}
        else:
            try:
                gray, _d = _default_capture()
                rec = teacher().recognize(gray)
            except Exception:
                rec = {"kind": "unknown", "say": None}
        if rec.get("say"):
            return rec["say"]
        # Fall through so Grok vision can describe the scene. Motion stays forbidden.
        return None
    if re.search(r"\b(no|nope),? that(?:'s| is) (?:actually )?(?:a |an |the )?([a-z0-9 ]+)$", t):
        return teacher().correct(re.search(r"(?:a |an |the )?([a-z0-9 ]+)$", t).group(1))
    return None


def _load():
    try:
        return os.getloadavg()[0]
    except (AttributeError, OSError):
        return 0.0


def record_shutdown(why):
    """Test hook. Production shutdown stays in gb_lowv, not here."""
    _shutdown_calls.append(why)
