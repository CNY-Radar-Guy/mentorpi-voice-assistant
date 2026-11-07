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


def wake_tokens(text):
    raw = [w for w in re.split(r"[\s,]+", (text or "").replace("-", " ")) if w]
    norms = []
    for w in raw:
        n = re.sub(r"[^a-z0-9]+", "", w.lower())
        if n:
            norms.append(n)
    return norms


def wake_prefix(text):
    """A cut-off start of the exact phrase. Not a wake by itself."""
    norms = wake_tokens(text)
    if not norms or tuple(norms) == EXACT_WAKE:
        return False
    return len(norms) < len(EXACT_WAKE) and tuple(norms) == EXACT_WAKE[:len(norms)]


def join_wake_text(first, second):
    return ((first or "").strip() + " " + (second or "").strip()).strip()


def read_pcm_frame(read, nbytes, max_interruptions=8):
    """One full PCM frame. A short read or an interrupted system call is not the end."""
    buf = bytearray()
    interruptions = 0
    while len(buf) < int(nbytes):
        try:
            chunk = read(int(nbytes) - len(buf))
        except InterruptedError:
            interruptions += 1
            if interruptions > int(max_interruptions):
                raise
            continue
        if not chunk:
            raise IOError("arecord ended")
        buf.extend(chunk)
        interruptions = 0
    return bytes(buf)


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


def autonomy_phrase(text):
    """Explicit on or off. Follow requests do not turn it on."""
    t = re.sub(r"\s+", " ", (text or "").lower()).strip(" .!?")
    if re.search(r"\b(disable autonomy|autonomy off|turn autonomy off|switch autonomy off)\b", t):
        return "off"
    if re.search(r"\b(enable autonomy|autonomy on|turn autonomy on|switch autonomy on)\b", t):
        return "on"
    return None


def controller_heartbeat(path=None, now=None, max_age=2.0):
    """True when the watchdog state file is fresh. That is not wheel motion."""
    path = path or os.environ.get("WD_STATE_HOST") or "/home/pi/docker/tmp/gb_safety/watchdog_state.json"
    now = time.time() if now is None else float(now)
    try:
        with open(path) as f:
            body = json.load(f)
        age = now - float(body["t"])
    except (OSError, ValueError, KeyError, TypeError):
        return False
    return 0 <= age <= float(max_age)


def autonomy_blockers(health_snap, track, controller_ok, now=None):
    """Reasons autonomy must stay off. A person name is not one of the checks."""
    missing = []
    if not gb_health.autonomy_allowed(health_snap, now):
        missing.append("a fresh range check")
    if not isinstance(track, dict) or not track.get("fresh"):
        missing.append("a fresh measured person")
    else:
        if track.get("ambiguous"):
            missing.append("one person, not several")
        try:
            conf = float(track.get("conf"))
        except (TypeError, ValueError):
            conf = None
        if conf is None or conf < gb_follow.CONF_MIN:
            missing.append("a person track at %.2f confidence" % gb_follow.CONF_MIN)
        if track.get("source") not in ("lidar", "depth") or track.get("depth_m") is None:
            missing.append("a measured range to that person")
    if not controller_ok:
        missing.append("a fresh watchdog heartbeat")
    return missing


def _follow_demand(active):
    try:
        import gb_demand
        import gb_paths
        path = gb_paths.demand_path()
        current = gb_demand.read_demand(path)
        if active:
            gb_demand.write_demand(path, "follow")
        elif current in ("follow", "acquire"):
            gb_demand.write_demand(path, "idle")
    except Exception:
        return


def _follow_demand_mode(mode):
    try:
        import gb_demand
        import gb_paths
        gb_demand.write_demand(gb_paths.demand_path(), mode)
    except Exception:
        return


def vision_may_move(_text=None):
    """Image descriptions, including Grok's, are not permission to move."""
    return False


def drop_goals(why):
    try:
        mission().cancel(why)
    except Exception:
        pass
    try:
        bounded_look().cancel(why)
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
_bounded = None
_rt_stop = None
_rt_thread = None


def bounded_look():
    global _bounded
    if _bounded is None:
        import gb_bounded
        _bounded = gb_bounded.BoundedLook()
    return _bounded


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
        track = read_person_track(os.environ.get("PERSON_TRACK_FILE") or __import__("gb_paths").person_track_path(), now)
        cmd = fl.step(now, gb_health.read_shared(now=now), track, _autonomy, protections, maint=maint)
        if cmd.get("allow") and (cmd.get("lin") or cmd.get("ang")):
            _publish(cmd)
        else:
            _publish({"op": "stop", "allow": False, "reason": cmd.get("reason")})
        _follow_demand(bool(fl.active))
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


def read_person_track(path=None, now=None):
    """A person range from the canonical track file. No measured distance is not a range."""
    if path is None:
        import gb_paths
        path = gb_paths.person_track_path()
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
    if p.get("time_base") not in (None, "capture"):
        return None
    try:
        import gb_metric
        age = now - float(p["t"])
        dist = float(p["depth_m"])
    except (KeyError, TypeError, ValueError):
        return None
    if not gb_metric.fresh_capture(p.get("t"), now) or dist != dist or dist <= 0:
        return None
    if p.get("unit") == "documented_mm" and gb_metric.validated_depth_scale("mono16") is None:
        return None
    if p.get("depth_capture_t") is not None:
        if not gb_metric.frames_paired(p.get("t"), p.get("depth_capture_t")):
            return None
        if not gb_metric.fresh_capture(p.get("depth_capture_t"), now):
            return None
    if p.get("source") == "lidar":
        if "clearance_m" in p:
            try:
                clearance = float(p.get("clearance_m"))
            except (TypeError, ValueError):
                return None
            if clearance != clearance or clearance < 0.4:
                return None
    elif p.get("unit") not in gb_health.VALIDATED_DEPTH_UNITS:
        return None
    source = "lidar" if p.get("source") == "lidar" else "depth"
    try:
        conf = float(p["conf"]) if "conf" in p else None
    except (TypeError, ValueError):
        conf = None
    return {"depth_m": dist, "source": source, "fresh": True, "bearing": float(p.get("bearing") or 0.0),
            "conf": conf, "ambiguous": bool(p.get("ambiguous")), "unit": p.get("unit"),
            "depth_age": age if source == "depth" else 0.0,
            "name_ignored": str(p.get("name") or "").lower() == "john"}


def read_sighting(path=None, now=None):
    """A detection that may have no range. Not permission to move."""
    if path is None:
        import gb_paths
        path = gb_paths.person_track_path()
    now = time.time() if now is None else float(now)
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    person = body.get("person") if isinstance(body, dict) else None
    if not isinstance(person, dict) or person.get("ambiguous"):
        return None
    if person.get("time_base") not in (None, "capture"):
        return None
    try:
        import gb_metric
        if not gb_metric.fresh_capture(person.get("t"), now):
            return None
        conf = float(person["conf"])
    except (KeyError, TypeError, ValueError):
        return None
    if conf < gb_follow.CONF_MIN:
        return None
    return person


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


def fresh_john(text):
    """True for an unquoted, non-negated 'I am John' in this utterance. Not a credential."""
    raw = text or ""
    if re.search(r'["“”].{0,60}\b(i am|i\'m|im)\s+john\b.{0,60}["“”]', raw, re.I):
        return False
    low = re.sub(r"\s+", " ", raw.lower())
    if re.search(r"\b(not|never|don'?t|do not|isn'?t)\b.{0,30}\bjohn\b", low):
        return False
    if re.search(r"\b(he|she|they|someone|model|gpt)\s+(said|says|claims)\b", low):
        return False
    return re.search(r"\b(i am|i'm|im|this is)\s+john\b", low) is not None


def maybe_release_bench(text, now=None, source="voice", model=False, paths=None):
    """Clear only an armed bench hold. A clap stop and a hard fault are not touched."""
    import gb_maint
    import gb_paths
    if paths is None:
        paths = [os.path.join(gb_paths.data_dir(), "MAINT"),
                 os.path.join(os.path.dirname(gb_paths.shared_estop()), "MAINT")]
    removed = gb_maint.release_bench(paths, fresh_john(text), now=now, source=source, model=model)
    if removed:
        drop_goals("bench-release")
    return removed


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
    if fresh_john(raw) and not re.search(r"\b(follow|come|drive|toward)\b", t):
        return NO_AUTH
    asked = autonomy_phrase(raw)
    if asked == "off":
        set_autonomy(False)
        follow_loop().cancel("autonomy_off")
        _follow_demand(False)
        drop_goals("autonomy-off")
        return "Autonomy is off. I dropped the trip and I am not moving."
    if asked == "on":
        track = read_person_track()
        missing = autonomy_blockers(gb_health.read_shared(), track, controller_heartbeat(), time.time())
        sight = read_sighting()
        if sight and not track:
            missing = [item for item in missing if "measured person" not in item and "person track" not in item]
            missing.append("a measured range; the depth unit is not confirmed and the camera-to-lidar extrinsics are not validated")
        if missing:
            _follow_demand_mode("acquire")
            return "Autonomy stays off. I still need " + ", ".join(missing) + ". I asked the camera to look, and I am not moving."
        set_autonomy(True)
        return "Autonomy is on for a new follow only. I am not starting the last trip, and I have not moved."
    if re.search(r"\b(reboot|restart yourself|power off the pi|shut ?down the pi)\b", t):
        return "I don't reboot or shut down from a message. That stays on the Pi, with you there."
    if "what is your load budget" in t or "cpu budget" in t:
        return "Optional vision stands down above a load of %.1f. Stopping, the battery monitor, and the tip cutoff stay on." % gb_health.LOAD_SHED
    taught = teacher().hear(t, _default_capture)
    if taught:
        return taught
    if re.search(r"\bmap this room\b", t):
        return ("I have not moved. Mapping is not running, and this is not a mapped explore. "
                "Say look around for a 45 second odometry look through the gate. Nav2 is not running.")
    if re.search(r"\b(come home|go home|return home)\b", t):
        sent = False
        if _runtime and _publisher is not None:
            try:
                _publish({"op": "bounded_return", "src": "voice"})
                sent = True
            except Exception:
                sent = False
        if sent:
            return ("I asked the gate to return only if a look is already running. "
                    "That uses the filtered odometry and stops on its own time and distance limit. "
                    "It is not a surveyed position. Nav2 is not running. I have not seen the wheels move.")
        return ("I have not moved. There is no home pose from this boot unless a look is already running. "
                "Nav2 is not running.")
    if re.search(r"\b(explore|look around|drive around)\b", t):
        import gb_bounded
        import gb_estop
        import gb_intent
        lease = gb_intent.open_lease(raw, time.time(), gb_bounded.CAP_S)
        if lease.get("condition_kind") == "place":
            try:
                gb_intent.write_status({
                    "lease": lease,
                    "motion": False,
                    "reason": "place_not_detectable",
                    "interlock": False,
                    "all_writer_pause": False,
                })
            except Exception:
                pass
            return ("I have not moved. I heard a place to stop, and I cannot detect that place, "
                    "so I will not start a look in its place. Say look around for the 45 second look "
                    "with no place condition, or say stop by itself when you want me to stop.")
        bounded_look().request()
        try:
            gb_intent.write_status({"lease": lease, "compare": gb_intent.compare(lease, lease["t"]), "interlock": False})
        except Exception:
            pass
        sent = False
        if _runtime and _publisher is not None:
            try:
                _publish({"op": "bounded", "cap": gb_bounded.CAP_S, "src": "voice", "intent_id": lease["id"]})
                sent = True
            except Exception:
                sent = False
        later = ""
        if lease.get("condition_kind") == "place":
            later = " I stored the place condition. I cannot detect that place, so it does not extend the look and it is not enforced."
        elif lease.get("condition_kind") == "told":
            later = " I stored the later condition. A bare stop later still ends it. The phrase does not make the look longer than 45 seconds."
        if sent:
            return ("I asked the gate for a 45 second odometry look, not a mapped explore."
                    + later + " "
                    "It uses filtered /odom only, stops at the existing obstacle limits, and the return "
                    "has its own time and distance limit. Nav2 is not running. A restart will not resume it. "
                    "Leave the lidar app, line following, object tracking, and hand gesture idle. I do not switch those off. "
                    "I have not seen the wheels move.")
        return ("I have not moved. Look around is a 45 second odometry look through the gate, not a mapped explore."
                + later + " "
                "It uses filtered /odom only. Nav2 is not running, and a restart will not resume it. "
                "Leave the lidar app, line following, object tracking, and hand gesture idle. I do not switch those off.")
    if re.search(r"\b(look for a person|acquire|find a person)\b", t):
        _follow_demand_mode("acquire")
        return "I am looking for a person. Autonomy is unchanged and I am not moving."
    if re.search(r"\b(follow me|stay with me|drive toward me|come to me|come here)\b", t):
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
            track = read_person_track()
        fl = follow_loop()
        now = time.time()
        if not _autonomy:
            missing = autonomy_blockers(gb_health.read_shared(), track, controller_heartbeat(), time.time())
            if protections.get("estop"):
                missing.append("the emergency stop to be cleared")
            if missing:
                _follow_demand(True)
                return ("I have not moved. I still need " + ", ".join(missing)
                        + ". The camera stays on. Say follow me again when that is true.")
            set_autonomy(True)
        fl.start(now)
        cmd = fl.step(now, gb_health.read_shared(now=now), track, _autonomy, protections, maint=False)
        _follow_demand(bool(fl.active))
        if cmd.get("allow") and (cmd.get("lin") or cmd.get("ang")):
            mission().commands.append({"lin": cmd.get("lin") or 0.0, "ang": cmd.get("ang") or 0.0, "why": "follow"})
            _publish(cmd)
        elif cmd.get("op") == "stop" and cmd.get("reason") not in ("person_lost", "person_gap", "autonomy_off", "confidence"):
            _publish({"op": "stop", "allow": False, "reason": cmd.get("reason")})
        if cmd.get("speech"):
            return cmd["speech"]
        if cmd.get("lin") or cmd.get("ang"):
            who = " The name on the track is not proof that the person is John." if (track or {}).get("name_ignored") else ""
            return "I sent the gate a follow hold to stay about 2 metres from the person I can measure." + who + " I have not seen the wheels move."
        return "I am inside the follow band, so I did not send a drive."
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
