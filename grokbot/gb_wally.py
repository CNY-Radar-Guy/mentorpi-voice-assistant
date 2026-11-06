"""gb_wally.py - voice-facing Wally behaviours that must not wait on the network.

Handles teaching, pointed missions, the 2 m follow, sensor-health questions,
and the rule that saying "I am John" is not authentication. Emergency stop
and unlock still live in gb_estop; this module only drops goals when asked.
"""
import os, re, tempfile
import gb_follow
import gb_health
import gb_point
import gb_teach

# Software wake is the exact phrase "wake up Wally" (case and punctuation ignored).
# "Hello Wally", "Hey Wally", and a bare "wally" are not wakes. The WonderEcho
# factory phrase "Hello Hiwonder" is a separate hardware signal and is not this grammar.
EXACT_WAKE = ("wake", "up", "wally")
NO_AUTH = "I'll call you John if you like. Saying that does not sign you in, and it doesn't unlock anything."

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
    if "wally" in norms[:8]:
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
        obs = _obs()
        person = obs.get("person") or {}
        src = person.get("source") or "none"
        cmd = gb_follow.follow_command(person.get("depth_m"), src, person.get("bearing") or 0.0,
                                        fresh=bool(person.get("fresh")), autonomy=_autonomy, protections=protections)
        mission().commands.append({"lin": cmd["lin"], "ang": cmd["ang"], "why": "follow"})
        if cmd["speech"]:
            return cmd["speech"]
        if cmd["lin"] or cmd["ang"]:
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
