"""gb_estop.py - Grok Bot kill switch, timed-motion parser and gate client (host side, imported by grokbot_voice).

ESTOP latch: /home/pi/grokbot/data/ESTOP (SSH: `touch ~/grokbot/data/ESTOP`). While it exists the gate refuses all
motion and publishes zero at 10 Hz, the watchdog zeroes the motors, gb-guard does no manoeuvres, missions are killed.
Only 'resume' / 'okay go' (also over the robot's own speech, echo-filtered) or deleting the file releases it, or the auto-clear: after ESTOP_AUTOCLEAR_S (15 s,
0 = off) of being latched, AutoClear releases the latch ONLY (clear('auto')): it never resumes motion or a mission.
Timed motion: parse_motion() -> {"op":"timed"|"turn",...}; gate_request() sends it ONCE; the gate owns the clock.
"""
import json, os, re, threading, time, uuid

E = os.environ.get
GB = os.path.dirname(os.path.abspath(__file__))
ESTOP = os.path.join(GB, "data", "ESTOP")
SHARED_ESTOP = "/home/pi/docker/tmp/gb_safety/ESTOP"
LOG = os.path.join(GB, "data", "estop.log")
CLAP_CFG = os.path.join(GB, "data", "clap.json")
REQ_TOPIC = "/gb/motion_req"
TIMED_DEFAULT = 3.0
TIMED_CAP = 10.0
BASE_LIN = float(E("TIMED_LIN", "0.08")); MAX_LIN = 0.12
BASE_ANG = float(E("TIMED_ANG", "0.48"))
_MOTION = {"until": 0.0}

def log(*a):
    line = time.strftime("%Y-%m-%d %H:%M:%S") + " " + " ".join(str(x) for x in a)
    print(line, flush=True)
    try:
        with open(LOG, "a") as f: f.write(line + "\n")
    except Exception: pass

# ------------------------------------------------------------------ latch
def is_set():
    return os.path.exists(ESTOP)

def info():
    try: return json.load(open(ESTOP))
    except Exception: return {"src": "file", "why": "touched"} if is_set() else None

def _write(path, d):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f: json.dump(d, f)

def estop(src="voice", why="", actions=None):
    """Latch the e-stop NOW (files first, then zero, then kill everything). Safe to call repeatedly."""
    d = {"t": time.time(), "ts": time.strftime("%Y-%m-%d %H:%M:%S"), "src": src, "why": why[:200]}
    for p in (ESTOP, SHARED_ESTOP):
        try: _write(p, d)
        except Exception as ex: log("estop write failed", p, ex)
    log("ESTOP LATCHED src=%s why=%r" % (src, why))
    _MOTION["until"] = 0.0
    threading.Thread(target=zero_now, args=("estop " + src,), daemon=True).start()
    for name, fn in (actions or {}).items():
        def run(name=name, fn=fn):
            try: fn()
            except Exception as ex: log("estop action %s failed: %s" % (name, ex))
        threading.Thread(target=run, daemon=True).start()
    return True

def clear(src="voice"):
    was = is_set()
    for p in (ESTOP, SHARED_ESTOP):
        try: os.remove(p)
        except FileNotFoundError: pass
        except Exception as ex: log("estop clear failed", p, ex)
    if was: log("ESTOP CLEARED by", src)
    return was

# ------------------------------------------------------------------ gate client
_WS = {"ws": None, "adv": set()}; _WSL = threading.Lock()

def _pub(topic, typ, msg, repeat=1, hz=20):
    """Persistent rosbridge publisher (advertise once, so requests are not lost to per-call discovery)."""
    import gb_robot
    for attempt in range(2):
        try:
            with _WSL:
                if _WS["ws"] is None:
                    _WS["ws"] = gb_robot._ws(3); _WS["adv"] = set()
                ws = _WS["ws"]
                if topic not in _WS["adv"]:
                    ws.send(json.dumps({"op": "advertise", "topic": topic, "type": typ})); _WS["adv"].add(topic); time.sleep(0.3)
                for i in range(repeat):
                    ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))
                    if i < repeat - 1: time.sleep(1.0 / hz)
            return True
        except Exception as ex:
            log("rosbridge publish failed (attempt %d): %s" % (attempt + 1, ex))
            with _WSL:
                try: _WS["ws"].close()
                except Exception: pass
                _WS["ws"] = None
    return False

def warm_up():
    _pub(REQ_TOPIC, "std_msgs/String", {"data": json.dumps({"op": "noop", "src": "warmup"})})
    _pub("/controller/cmd_vel", "geometry_msgs/Twist", {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})

def gate_request(d):
    d = dict(d); d.setdefault("id", uuid.uuid4().hex[:8]); d.setdefault("src", "voice")
    if not _pub(REQ_TOPIC, "std_msgs/String", {"data": json.dumps(d)}, repeat=5, hz=20):   # same id: gate runs it once
        raise RuntimeError("could not reach the motion gate (rosbridge)")
    if d.get("op") == "timed": _MOTION["until"] = time.time() + min(TIMED_CAP, float(d.get("dur") or TIMED_DEFAULT)) + 1.0
    elif d.get("op") == "turn": _MOTION["until"] = time.time() + TIMED_CAP + 1.0
    elif d.get("op") == "stop": _MOTION["until"] = 0.0
    return d["id"]

def zero_now(why="stop"):
    """Gate stop request + zero Twists straight to the controller input (zero is always allowed directly)."""
    import gb_robot
    try: gate_request({"op": "stop", "src": why[:30]})
    except Exception as ex: log("gate stop request failed:", ex)
    try: _pub("/controller/cmd_vel", "geometry_msgs/Twist", gb_robot.TWIST0, repeat=10, hz=20)
    except Exception as ex: log("direct zero failed:", ex)

def recently_moving():
    return time.time() < _MOTION["until"]

# ------------------------------------------------------------------ phrases
_WAKE = r"(?:(?:hey|hi|ok|okay|yo)[ ,]+)?(?:(?:g|gee|grok|groc|grog|ji)[ -]?bot|grokbot|robot)?[ ,]*"
_STOPW = r"(?:stop|stopp?ed|stopping|halt|freeze|whoa|woah|abort|emergency)"
_STRONG = re.compile(r"^" + _WAKE + r"(?:emergency stop|e ?stop|stop|halt|freeze|whoa|woah|abort)(?:[ ,]+(?:stop|halt|freeze|now|it|please|"
                     r"moving|right now|robot|bot|g ?bot|gbot|the robot|the motors|motors|everything|all|immediately|that))*$")
_NEG_BEFORE = re.compile(r"\b(then|and|to|don'?t|dont|never|not|without|before|until|bus|non|after|when|will|"
                         r"you|i|we|can|could|should|would)\s+" + _STOPW + r"\b")

def _norm(t):
    t = (t or "").lower().replace("’", "'")
    t = re.sub(r"[^a-z0-9' -]+", " ", t)
    return re.sub(r"\s+", " ", t).strip()

# ------------------------------------------------------------------ natural phrasing (2026-11-05, .bak-natural)
# Conversational lead-ins / tails are stripped before intent + drive parsing: "I want you to drive forward for ten
# seconds please" -> "drive forward for ten seconds". It only ever REMOVES words, so it can never create a command,
# and stop words are never lead-ins (a kill can only get more likely, never less).
_LEAD_WORDS = sorted(["i want you to", "i'd like you to", "i would like you to", "i need you to", "i'd need you to",
                      "can you", "could you", "would you", "will you", "please", "go ahead and", "let's", "lets",
                      "let us", "try to", "i want to", "i'd like to", "hey", "hi", "okay", "ok", "alright", "all right",
                      "so", "and", "now", "just", "well", "um", "uh", "oh"], key=len, reverse=True)
_LEAD_SINGLE = {w for w in _LEAD_WORDS if " " not in w} | {"for", "me", "thanks", "thank", "you"}
_GBOT = r"(?:g|gee|grok|groc|grog|ji)[ -]?bot|grokbot|robot"
_LEADIN = re.compile(r"^(?:" + "|".join(_LEAD_WORDS) + "|" + _GBOT + r")\b[ ,]*")
_TAILOUT = re.compile(r"[ ,]+(?:please|for me|now|thanks|thank you)$")
_LEADIN_RAW = re.compile(r"^[\s,.!?]*(?:" + "|".join(w.replace(" ", r"[\s,]+").replace("'", "['\u2019]") for w in _LEAD_WORDS) +
                         r"|(?:g|gee|grok|groc|grog|ji)[\s-]?bot|grok\s?bot|robot)\b[\s,.!?]*", re.I)
_TAILOUT_RAW = re.compile(r"[\s,]+(?:please|for me|now|thanks|thank you)[\s.!?]*$", re.I)

def strip_leadins(text):
    """'Hey G-Bot, can you please go forward for ten seconds for me' -> 'go forward for ten seconds' (normalized)."""
    t = _norm(text); prev = None
    while t and t != prev:
        prev = t
        t = _LEADIN.sub("", t, count=1).strip(" ,-")
        t = _TAILOUT.sub("", t).strip(" ,-")
    return t

def strip_leadins_raw(text):
    """Same lead-ins / tails on the ORIGINAL text (case, digits and inner punctuation kept) for the other local intents."""
    s = (text or "").strip(); prev = None
    while s and s != prev:
        prev = s
        s = _LEADIN_RAW.sub("", s, count=1).strip()
        s = _TAILOUT_RAW.sub("", s).strip()
    return s

# drive command whose TRAILING stop/halt/pause only ends the drive -> not a kill (the gate's timed deadline stops it)
_DRIVE_VERB = re.compile(r"\b(?:forwards?|backwards?|back|reverse|turn|left|right|drive|go|move)\b")
_TRAIL_STOP = re.compile(r"^(?P<pre>.+?)[ ,]+(?:and then|then|and)[ ,]+(?:stop|halt|pause)(?:[ ,]+(?:after that|there|please))?$")
_PRE_BLOCK = re.compile(r"\b(?:stop|stopp?ed|stopping|halt|freeze|whoa|woah|abort|emergency|e ?stop|pause|don'?t|dont|never|not)\b")
_PRE_FILLER = re.compile(r"\b(?:all right|alright|that'?s right|you'?re right|right now|right there|right away)\b")

def drive_then_stop(text):
    """True only for '<drive verb> ... [for N seconds] then|and stop|halt|pause'. A bare stop/halt never matches."""
    t = _norm(text)
    m = _TRAIL_STOP.match(t)
    if not m: return False
    pre = re.sub(r"^" + _WAKE, "", m.group("pre").strip()).strip()
    pre = strip_leadins(pre)
    pre = _POLITE.sub("", pre).strip()
    if not pre or len(pre.split()) > 12 or _PRE_BLOCK.search(pre): return False
    return bool(_DRIVE_VERB.search(_PRE_FILLER.sub(" ", pre)))

def kill_phrase(text, moving=False):
    """True if this transcript must trigger the e-stop. Checked FIRST on every transcript (wake word or not, mid-TTS)."""
    t = _norm(text)
    if not t: return False
    if drive_then_stop(t):
        return False                        # "forward for 10 seconds then stop/halt/pause": timed drive, ends normally
    s = strip_leadins(t)                    # natural: "I want you to stop" / "can you stop" / "please stop now" -> "stop"
    if s and s != t and _STRONG.match(s):
        return True
    if re.search(r"\bemergency stop\b|\b(?:g ?bot|gbot|grok ?bot|robot) (?:stop|halt|freeze)\b|\bstop stop\b|\bfreeze\b|\bhalt\b", t):
        return True
    if _STRONG.match(t):
        return True
    if not re.search(r"\b" + _STOPW + r"\b", t):
        return False
    if moving:
        return True                         # anything with a stop word while moving wins (false trigger is acceptable)
    if _NEG_BEFORE.search(t):
        return False                        # "go forward 4 seconds then stop" is a command, not a kill
    return len(t.split()) <= 4

_RESUME = re.compile(r"^" + _WAKE + r"(?:resume|resume motion|resume driving|okay go|ok go|all clear|release (?:the )?(?:e ?stop|stop|brake)|"
                     r"clear (?:the )?(?:e ?stop|emergency stop))(?: please| now)?$")

def _resume_core(t):
    return bool(_RESUME.match(t)) or t in ("resume", "okay go", "ok go", "okay, go")

def resume_phrase(text):
    t = re.sub(r"\bresume(?: resume)+\b", "resume", _norm(text))       # "resume resume" (log 08:13:04)
    if _resume_core(t): return True
    s = strip_leadins(t)                                              # "please resume" / "can you resume now"
    return bool(s) and s != t and _resume_core(s)

_RESUME_WORDS = {"resume"}

def strip_echo_counted(text, spoken_texts):
    """Echo filter by COUNT: each word cancels as many heard copies as the robot said in ONE line (max over lines),
    so the robot's single 'resume' is cancelled but John's 'resume resume' leaves one."""
    budget = {}
    for x in spoken_texts or ():
        c = {}
        for w in _norm(x if isinstance(x, str) else x[1]).split(): c[w] = c.get(w, 0) + 1
        for w, n in c.items(): budget[w] = max(budget.get(w, 0), n)
    out = []
    for w in _norm(text).split():
        if budget.get(w, 0) > 0: budget[w] -= 1
        else: out.append(w)
    return " ".join(out)

def resume_over_playback(text, spoken_texts):
    """A resume phrase heard over the robot's own speech (caller checks the latch). Our own 'Say resume when it's safe'
    / 'Say resume first' must never clear it: (1) any context word of a robot line that contains 'resume' ('say',
    'when', 'safe', 'first', ...) means we are hearing ourselves -> no; (2) count-based echo removal, then the
    remainder must be a resume phrase on its own."""
    heard = _norm(text).split()
    if not heard: return False
    ctx = set()
    for x in spoken_texts or ():
        ws = _norm(x if isinstance(x, str) else x[1]).split()
        if _RESUME_WORDS & set(ws): ctx |= set(ws)
    ctx -= _RESUME_WORDS | _LEAD_SINGLE
    if ctx & set(heard): return False
    left = strip_echo_counted(text, spoken_texts)
    return bool(left) and resume_phrase(left)

# ------------------------------------------------------------------ timed motion parser
_UNITS = {"zero": 0, "one": 1, "two": 2, "too": 2, "to": None, "three": 3, "four": 4, "for": None, "five": 5, "six": 6,
          "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "fifteen": 15, "twenty": 20,
          "thirty": 30, "forty": 40, "sixty": 60, "ninety": 90, "forty five": 45, "one hundred eighty": 180,
          "a hundred eighty": 180, "one eighty": 180, "three sixty": 360, "two seventy": 270,
          "thirteen": 13, "fourteen": 14, "sixteen": 16, "seventeen": 17, "eighteen": 18, "nineteen": 19,
          "twenty five": 25}

def _nums(t):
    t = re.sub(r"\b(\d+) and a half\b", lambda m: str(float(m.group(1)) + 0.5), t)
    t = re.sub(r"\bone and a half\b", "1.5", t); t = re.sub(r"\btwo and a half\b", "2.5", t)
    t = re.sub(r"\bhalf (?:a |an )?(?=(?:second|sec|minute)\b)", "0.5 ", t)
    t = re.sub(r"\ba couple(?: of)?\b", "2", t); t = re.sub(r"\ba few\b", "3", t)
    t = re.sub(r"\b(?:a|an|one) (?=(?:second|sec)\b)", "1 ", t)
    for w in sorted((k for k, v in _UNITS.items() if v is not None), key=len, reverse=True):
        t = re.sub(r"\b" + w + r"\b(?=\s*(?:-\s*)?(?:seconds?|secs?|s|minutes?|mins?|degrees?|deg)\b)", str(_UNITS[w]), t)
    t = re.sub(r"\b(?:for|four) (?=(?:seconds?|secs?)\b)", "4 ", t) if re.search(r"\bfor (?:seconds?|secs?)\b", t) and not re.search(r"\d", t) else t
    t = re.sub(r"(\d)\s*-\s*(second|sec|minute|degree)", r"\1 \2", t)
    t = re.sub(r"(\d)(s|sec|secs)\b", r"\1 \2", t)
    return t

_DIST = re.compile(r"\b\d+(?:\.\d+)?\s*(?:feet|foot|ft|meters?|metres?|m|yards?|yds?|inch(?:es)?|cm|centimet(?:er|re)s?)\b|"
                   r"\b(?:a|one|two|three|four|five|half a) (?:feet|foot|meters?|metres?|yards?)\b")
_DUR = re.compile(r"\b(\d+(?:\.\d+)?)\s*(seconds?|secs?|s|minutes?|mins?)\b")
_DEG = re.compile(r"\b(\d+(?:\.\d+)?)\s*(?:degrees?|deg)\b")
_FWD = re.compile(r"\b(forwards?|ahead|straight(?: ahead)?|fwd)\b")
_BWD = re.compile(r"\b(backwards?|backward|back ?up|reverse|in reverse|backing up)\b|\b(?:go|drive|move|run|roll|head) back\b(?! to\b)|^back\b(?! to\b)")
_TURN = re.compile(r"\b(turn|rotate|spin|pivot)\b")
_VERB = re.compile(r"^(?:go|drive|dr|try|move|run|roll|head|creep|inch|scoot|travel|back|backwards?|reverse|forward|forwards|turn|rotate|spin|pivot|"
                   r"keep going|start driving|start moving)\b")
_POLITE = re.compile(r"^(?:(?:can|could|would|will) you |please |i want you to |i'd like you to |go ahead and |now |just |and |so |ok |okay |alright )+")
_DEST = re.compile(r"\b(to (?:the|my|me|where|his|her|that|this)|home|charger|kitchen|room|toward|towards|over to|until|follow|me\b)")

def parse_motion(text, speed_mult=1.0):
    """'go forward for 4 seconds' -> {"op":"timed","lin":+v,"dur":4,...}; 'reverse 90 degrees' -> {"op":"turn","deg":-90}.
    Returns None if this is not a simple timed drive/turn. Duration: default 3 s if none parsed, hard cap 10 s."""
    t = strip_leadins(text)                 # natural lead-ins / tails ("I want you to", "can you", "please", "now")
    t = re.sub(r"^" + _WAKE, "", t).strip()
    t = _POLITE.sub("", t).strip()
    t = re.sub(r"[ ,]*(?:and then |and |then )?(?:stop|halt|pause)(?: after that| there| please)?$", "", t).strip()   # "... then stop/halt/pause"
    if not t or not _VERB.match(t): return None
    if len(t.split()) > 12 or _DEST.search(t) or _DIST.search(t): return None
    if re.search(r"\b(map|explore|wander|patrol|around the|follow|line|colou?r|track|find|look)\b", t): return None
    t = _nums(t)
    k = max(0.5, min(1.0, float(speed_mult or 1.0)))
    deg = _DEG.search(t)
    left = re.search(r"\b(left|counter ?clockwise|anti ?clockwise)\b", t); right = re.search(r"\b(right|clockwise)\b", t)
    if _TURN.search(t) or deg:
        if deg: d = float(deg.group(1))
        elif re.search(r"\b(around|180)\b", t): d = 180.0
        else: d = 90.0
        if right and not re.search(r"counter|anti", right.group(0) if right else ""): sign = -1
        elif left: sign = 1
        elif _BWD.search(t): sign = -1                       # "reverse 90 degrees" -> clockwise
        else: sign = 1
        d = max(5.0, min(360.0, d))
        return {"op": "turn", "deg": sign * d, "ang": round(BASE_ANG * k, 3), "requested": text, "dur_cap": TIMED_CAP}
    fwd = _FWD.search(t); bwd = _BWD.search(t)
    dm = _DUR.search(t)
    if not (fwd or bwd or dm): return None
    if fwd and bwd: return None
    sign = -1 if bwd else 1
    dur = None; clamped = False
    if dm:
        v = float(dm.group(1)); u = dm.group(2)
        dur = v * (60 if u.startswith("m") else 1)
    if dur is None or dur <= 0: dur = TIMED_DEFAULT; parsed = False
    else: parsed = True
    if dur > TIMED_CAP: dur, clamped = TIMED_CAP, True
    lin = round(min(MAX_LIN, BASE_LIN * k) * sign, 3)
    return {"op": "timed", "lin": lin, "ang": 0.0, "dur": round(dur, 2), "parsed_duration": parsed, "clamped": clamped,
            "requested": text}

def describe(p):
    if p["op"] == "turn":
        return "turn %d degrees %s" % (abs(p["deg"]), "left" if p["deg"] > 0 else "right")
    s = "%g second%s" % (p["dur"], "" if p["dur"] == 1 else "s")
    return "drive %s for %s%s" % ("forward" if p["lin"] > 0 else "backward", s,
                                  " (that's my 10 second limit)" if p.get("clamped") else "" if p.get("parsed_duration") else " (my default)")

# ------------------------------------------------------------------ barge-in drive + follow-up window
# Follow-up window (no wake word): measured from the END of the robot's playback. Was 5 s (FOLLOWUP_S default,
# counted from when speak() returned); now 20 s. Env: FOLLOWUP_S.
FOLLOWUP_WINDOW_S = float(E("FOLLOWUP_S", "20"))
# words a drive command needs for its shape/duration; kept even if the robot also said them (only with a drive verb
# John said himself, so a pure echo can never become a command)
_KEEP = {"for", "and", "then", "seconds", "second", "secs", "sec", "s", "degrees", "degree", "deg", "minutes", "minute",
         "half", "a", "an", "couple", "of", "few"}

def _spoken_words(spoken_texts):
    s = set()
    for x in spoken_texts or ():
        s |= set(_norm(x if isinstance(x, str) else x[1]).split())
    return s

def strip_echo(text, spoken_texts, keep_structure=False):
    """Self-speech filter: drop every word the robot itself was saying (the same rule barge_kill used).
    keep_structure=True also keeps numbers and duration/connective words so 'forward for 5 seconds' still parses."""
    sp = _spoken_words(spoken_texts); out = []
    for w in _norm(text).split():
        if w not in sp or (keep_structure and (w in _KEEP or re.match(r"^\d+(?:\.\d+)?$", w) or _UNITS.get(w))):
            out.append(w)
    return " ".join(out)

def followup_deadline(play_end, window=None):
    return float(play_end) + (FOLLOWUP_WINDOW_S if window is None else float(window))

def followup_open(t_listen, play_end, window=None):
    """True if listening started inside the no-wake-word follow-up window [end of playback, end + window)."""
    if not play_end: return False
    return -1.0 <= t_listen - play_end and t_listen < followup_deadline(play_end, window)

def bargein_decision(text, spoken_texts, speed_mult=1.0):
    """A transcript that overlapped the robot's own speech. Words the robot was saying are dropped first (echo).
    -> {"action": "kill"|"drive"|"latched"|"ignore", "left": John's words, "cmd": text to run, "motion": parse}
    kill   : same rule as before (kill_phrase(left, moving=True): stop/halt/freeze/emergency stop always win), except
             '<drive John said> ... then stop|halt|pause', the existing drive-then-stop exemption.
    drive  : John's own words contain a drive verb and parse as a timed drive/turn (3 s default, 10 s cap).
    latched: a drive, but the ESTOP latch is set -> the caller must not move (do_timed refuses as well)."""
    # 2026-11-05 natural: strip lead-ins BEFORE the echo mask. Root cause of log 08:21:13: the robot had just said
    # "What do you need?", so the mask removed John's "you" and "I want you to drive forward for 10 seconds" became
    # "i want to drive forward ..." which parse_motion could not read -> ignored.
    nat = strip_leadins(text) or _norm(text)
    left = strip_echo(nat, spoken_texts)
    keep = strip_echo(nat, spoken_texts, keep_structure=True)
    r = {"action": "ignore", "left": left, "cmd": keep, "motion": None}
    john_drive = bool(left) and bool(_DRIVE_VERB.search(_PRE_FILLER.sub(" ", left)))
    if left or strip_echo(text, spoken_texts):
        raw_left = strip_echo(text, spoken_texts)             # unchanged old rule as well: a kill never gets weaker
        if (kill_phrase(left, moving=True) or kill_phrase(raw_left, moving=True)) and not (john_drive and drive_then_stop(keep)):
            r["action"] = "kill"; return r
    if is_set() and resume_over_playback(text, spoken_texts):  # 'resume' over our speech while latched (echo-filtered)
        r["action"] = "resume"; return r
    if not left: return r
    if not john_drive: return r
    if re.search(r"\b" + _STOPW + r"\b", _norm(text)) and not drive_then_stop(keep):
        return r                            # a stop word inside a multi-step command (even if masked as echo): never drive
    pm = parse_motion(keep, speed_mult)
    if not pm: return r
    r["motion"] = pm; r["action"] = "latched" if is_set() else "drive"
    return r

# Overlap test for the main mic: speech counts as "over our playback" only if it STARTED while we were playing or
# within OVERLAP_TAIL_S (0.5 s self-echo guard) of the TRUE end of playback. Was: any playback end within 0.5 s of when
# the listen slice began (padded by the speaking.json 300 ms tail), which flagged replies John began after we stopped.
OVERLAP_TAIL_S = float(E("OVERLAP_TAIL_S", "0.5"))

def overlapped(t_onset, play_end, playing=False, tail=None):
    if playing: return True
    if not play_end: return False
    return t_onset < float(play_end) + (OVERLAP_TAIL_S if tail is None else float(tail))

# ------------------------------------------------------------------ re-latch self-voice filter (2026-11-05, .bak-relatch)
# ONLY for "stop phrase heard while ALREADY latched" (re-latch = restart the auto-clear timer). The first latch never
# comes through here: it stays instant and unfiltered. Log 08:28:20: the stop listener transcribed the robot's own
# "Emergency stop." from its pipe backlog after the 08:28:13 latch, stamped it with wall-clock read time (so it looked
# like it started after playback) and re-latched, restarting the 15 s timer.
_STEMS = {"stopped": "stop", "stopping": "stop", "stops": "stop", "halted": "halt", "halting": "halt", "woah": "whoa",
          "froze": "freeze", "freezing": "freeze", "estop": "stop", "e-stop": "stop"}
_ECHO_FILLER = {"the", "a", "an", "uh", "um", "oh", "ok", "okay", "so", "and", "it", "is"}

def _stem_words(text):
    return [_STEMS.get(w, w) for w in _norm(text).split()]

def own_line_echo(text, spoken_texts, cover=0.6):
    """True if the heard text is (a fragment of) a run of sentences the robot itself just said: every heard word
    (fillers aside) comes from that run of sentences, and it covers >= `cover` of it ('Emergency stop.' vs our 'Emergency stop.').
    A lone 'stop' against our 'Emergency stop.' covers 50 %, so John's own short stop is NOT treated as echo."""
    heard = [w for w in _stem_words(text) if w not in _ECHO_FILLER]
    if not heard: return False
    for x in spoken_texts or ():
        sents = [z for z in re.split(r"[.!?;]+", x if isinstance(x, str) else x[1]) if z.strip()]
        for sent in (" ".join(sents[i:j]) for i in range(len(sents)) for j in range(i + 1, len(sents) + 1)):   # contiguous spans
            ws = [w for w in _stem_words(sent) if w not in _ECHO_FILLER]
            if not ws: continue
            budget = {}
            for w in ws: budget[w] = budget.get(w, 0) + 1
            ok = True
            for w in heard:
                if budget.get(w, 0) > 0: budget[w] -= 1
                else: ok = False; break
            if ok and len(heard) >= cover * len(ws): return True
    return False

def relatch_decision(text, t_onset, play_end, playing=False, spoken_texts=(), tail=None):
    """Already latched + a stop phrase: re-latch (restart the auto-clear timer) only if it is NOT the robot's own voice.
    -> (ok, why). (1) speech that started during our playback or within the OVERLAP_TAIL_S echo tail -> no;
    (2) text matching a sentence of our own recent lines ('Emergency stop', ...) -> no; (3) must be a stop phrase.
    Never call this for the first latch (not latched): that one is instant and unfiltered."""
    if overlapped(t_onset, play_end, playing, tail): return False, "started during own playback/echo tail"
    if own_line_echo(text, spoken_texts): return False, "matches own recent line"
    if not kill_phrase(text, moving=True): return False, "no stop phrase"
    return True, "john"

# ------------------------------------------------------------------ auto-clear (ESTOP_AUTOCLEAR_S)
def autoclear_s(v=None):
    """Seconds a latch must stand before it auto-clears. ESTOP_AUTOCLEAR_S (default 15); 0 (or negative) disables."""
    v = E("ESTOP_AUTOCLEAR_S", "15") if v is None else v
    try: return max(0.0, float(str(v).strip() or "15"))
    except ValueError: return 15.0

def latch_sig(path=None):
    """Identity of the current latch (mtime_ns, size, t) or None if not latched. Every estop() / clap / touch rewrites
    the file, so a new stop gives a new signature (that is what restarts the auto-clear timer)."""
    p = path or ESTOP
    try: st = os.stat(p)
    except OSError: return None
    t = None
    try:
        with open(p) as f: t = float(json.load(f).get("t"))
    except Exception: pass
    return (st.st_mtime_ns, st.st_size, t)

def latch_time(sig):
    return sig[2] if sig[2] is not None else sig[0] / 1e9      # JSON "t" from estop(), else file mtime (SSH touch)

class AutoClear:
    """Releases the ESTOP latch once it has been set for `timeout` s. Latch ONLY: calls clear('auto') and nothing else -
    no gate request, no Twist, no mission/feature/explore resume (those were cancelled by the stop and stay cancelled).
    A new stop rewrites the file -> new signature -> the timer restarts from that latch. Call step() from a poll thread."""
    def __init__(self, timeout=None, clock=time.time, on_clear=None):
        self.timeout = autoclear_s() if timeout is None else max(0.0, float(timeout))
        self.clock = clock; self.on_clear = on_clear; self.sig = None; self.since = None

    def step(self):
        if self.timeout <= 0: return False
        sig = latch_sig()
        if sig is None:
            self.sig = self.since = None; return False
        now = self.clock()
        if sig != self.sig:                                  # first sight of this latch (or a re-latch): (re)start
            self.sig = sig; lt = latch_time(sig)
            self.since = lt if lt <= now else now            # future stamp / clock jump: count from now
        if now - self.since < self.timeout: return False
        if latch_sig() != sig: return False                  # re-latched this instant: next step restarts the timer
        clear("auto")
        log("ESTOP AUTO-CLEARED after %gs" % self.timeout)
        self.sig = self.since = None
        if self.on_clear:
            try: self.on_clear()
            except Exception as ex: log("auto-clear on_clear failed: %s" % ex)
        return True

# ------------------------------------------------------------------ clap config
def clap_cfg():
    try: return json.load(open(CLAP_CFG))
    except Exception: return {"sensitivity": "high"}

def clap_set(**kw):
    d = clap_cfg(); d.update(kw); _write(CLAP_CFG, d); return d

_LEVELS = ["low", "med", "high", "max"]
def clap_step(direction):
    cur = str(clap_cfg().get("sensitivity", "high"))
    i = _LEVELS.index(cur) if cur in _LEVELS else 2
    i = max(0, min(len(_LEVELS) - 1, i + (1 if direction > 0 else -1)))
    clap_set(sensitivity=_LEVELS[i]); return _LEVELS[i]
