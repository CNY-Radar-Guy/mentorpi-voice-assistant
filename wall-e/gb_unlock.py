"""Fresh user movement can clear a normal stop. It cannot clear anything else.

A normal stop stays until resume/unlock or a new qualifying move. The 15 s
timer is not an unlock. Maintenance, tip, battery, watchdog, obstacle,
unsafe ranging, and a safety-fault latch are never overridden. No old goal
is restored. 'I am John' is not authentication and not a move.
"""
import re, time, uuid

TTL_S = 2.0
SAFETY_SRCS = {"gate-tilt", "tilt", "battery", "watchdog", "physical", "joystick", "hardware"}
_SEEN = set()

_STOP = re.compile(r"\b(stop|halt|freeze|cancel|abort|emergency)\b", re.I)
_NEG = re.compile(r"\b(don't|do not|dont|never|won't|will not|not going to)\b", re.I)
_HYPO = re.compile(r"\b(if i said|what if|suppose|imagine|hypothetically)\b", re.I)
_IDENTITY = re.compile(r"^(i am|i'm|im) john$", re.I)


def reset_seen():
    _SEEN.clear()


def latch_kind(info):
    """'normal', 'safety', or None when there is no latch."""
    if not info:
        return None
    if info.get("kind") == "safety" or info.get("src") in SAFETY_SRCS:
        return "safety"
    if info.get("kind") == "maintenance" or info.get("src") == "maintenance":
        return "maintenance"
    return "normal"


def fresh_provenance(text, source="voice", now=None, origin=None, from_llm=False, epoch=None):
    now = time.time() if now is None else float(now)
    origin = origin or {}
    return {
        "id": uuid.uuid4().hex,
        "t": now,
        "heard_at": now,
        "source": source,
        "text": text or "",
        "from_llm": bool(from_llm or origin.get("from_llm")),
        "origin_ok": bool(origin.get("validated")) if source in ("ui", "web", "web-voice") else source == "voice",
        "local": source == "voice" or bool(origin.get("local")),
        "epoch": epoch,
        "stale": False,
        "replay": False,
    }


def _origin_ok(prov):
    if prov.get("from_llm"):
        return False
    source = prov.get("source") or ""
    if source == "voice":
        return True
    if source in ("ui", "web", "web-voice"):
        return bool(prov.get("origin_ok") and prov.get("local"))
    return False


def _reject_text(text):
    raw = text or ""
    t = " ".join(raw.lower().split())
    if _IDENTITY.match(t.strip(" .!?")):
        return "not_auth"
    if _NEG.search(t):
        return "negated"
    if _HYPO.search(t):
        return "hypothetical"
    if '"' in raw or "“" in raw or "”" in raw:
        return "quoted"
    return None


def decide(text, provenance, latch, gates, motion=None, now=None):
    """Pure decision. Does not delete files or publish velocity.

    motion is a parse_motion dict for a teleop command. Autonomous requests
    pass motion None and gates['autonomous']=True.
    """
    now = time.time() if now is None else float(now)
    gates = gates or {}
    prov = dict(provenance or {})
    say_hold = "I'm stopped. A new drive command will release a normal stop. It will not override a safety fault."
    hard = None
    for key in ("maint", "tip_hard", "tip_soft", "battery_shutdown", "watchdog", "physical", "hardware", "obstacle"):
        if gates.get(key):
            hard = key
            break
    if hard:
        return {"allow": False, "unlock": False, "store": False, "reason": hard,
                "say": "I won't move. %s is still active." % hard.replace("_", " ")}
    kind = latch_kind(latch)
    if kind == "safety":
        return {"allow": False, "unlock": False, "store": False, "reason": "safety_latch", "say": say_hold}
    why = _reject_text(text)
    if why:
        return {"allow": False, "unlock": False, "store": False, "reason": why, "say": say_hold}
    if kind == "normal" and _STOP.search(text or "") and (motion or gates.get("autonomous")):
        return {"allow": False, "unlock": False, "store": False, "reason": "stop_dominates", "say": say_hold}
    if not _origin_ok(prov):
        return {"allow": False, "unlock": False, "store": False, "reason": "origin", "say": say_hold}
    try:
        heard = float(prov.get("heard_at"))
    except (TypeError, ValueError):
        return {"allow": False, "unlock": False, "store": False, "reason": "stale", "say": say_hold}
    if prov.get("stale") or prov.get("replay") or now - heard > TTL_S or now < heard:
        return {"allow": False, "unlock": False, "store": False, "reason": "stale", "say": say_hold}
    if gates.get("epoch") is not None and prov.get("epoch") is not None and gates.get("epoch") != prov.get("epoch"):
        return {"allow": False, "unlock": False, "store": False, "reason": "replay", "say": say_hold}
    pid = prov.get("id")
    if not pid or pid in _SEEN:
        return {"allow": False, "unlock": False, "store": False, "reason": "replay", "say": say_hold}
    if gates.get("autonomous"):
        if not gates.get("range_ok"):
            return {"allow": False, "unlock": False, "store": False, "reason": "sensorless",
                    "say": "I won't drive without a fresh range."}
        if not gates.get("allow_autonomy"):
            return {"allow": False, "unlock": False, "store": False, "reason": "autonomy_off",
                    "say": "Autonomy is off. I will not follow or drive to a target."}
    elif not motion or motion.get("op") not in ("timed", "turn"):
        return {"allow": False, "unlock": False, "store": False, "reason": "not_a_move", "say": say_hold}
    if not gates.get("allow_motion", True):
        return {"allow": False, "unlock": False, "store": False, "reason": "motion_disabled",
                "say": "Driving by voice is switched off."}
    _SEEN.add(pid)
    unlocked = kind == "normal"
    return {
        "allow": True,
        "unlock": unlocked,
        "store": False,
        "reason": "teleop" if not gates.get("autonomous") else "autonomous",
        "say": None,
        "command": motion,
        "provenance": {"id": pid, "t": prov.get("t"), "source": prov.get("source"), "epoch": prov.get("epoch")},
    }
