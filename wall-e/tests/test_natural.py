"""Offline natural-phrasing / overlap / resume-over-playback tests: decision functions only, no audio, no motors,
temp ESTOP path (the real data/ESTOP is never touched)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop as g

REAL_ESTOP = g.ESTOP
real_before = os.path.exists(REAL_ESTOP)
tmpd = tempfile.mkdtemp(prefix="gb_natural_")
g.ESTOP = os.path.join(tmpd, "ESTOP"); g.SHARED_ESTOP = os.path.join(tmpd, "ESTOP_shared"); g.LOG = os.path.join(tmpd, "estop.log")
g.zero_now = lambda why="": None          # never publish to rosbridge from a test
g.gate_request = lambda d: (_ for _ in ()).throw(RuntimeError("test must not call the gate"))
g.floor_mode = lambda: False          # tests use a temp latch; do not inherit the live floor/ground mode


rows = []; fails = 0
def check(name, ok, got=""):
    global fails
    fails += not ok
    rows.append((name, str(got), "PASS" if ok else "FAIL"))
def dsc(pm): return g.describe(pm) if pm else "-"
def drive10(pm): return bool(pm) and pm["op"] == "timed" and pm["lin"] > 0 and pm["dur"] == 10

PH = "I want you to drive forward for 10 seconds."         # log 08:21:13
AWAKE = ["I'm awake. What do you need?"]                     # what the robot had just said
LATCH = ["Emergency stop. Motors are latched off. Say resume when it's safe."]
REFUSE = ["I'm emergency-latched, so I won't move. Say resume first."]

# --- (a) root cause, then the fix
old = g.strip_echo(PH, AWAKE, keep_structure=True)
check("root cause: old echo mask drops 'you' -> old _POLITE/_VERB cannot read it", g._POLITE.match(old) is None and not g._VERB.match(old) and "you" not in old.split(), repr(old))
d = g.bargein_decision(PH, AWAKE)
check("08:21:13 phrase DURING playback -> drive 10 s", d["action"] == "drive" and drive10(d["motion"]), "%s %s" % (d["action"], dsc(d["motion"])))
end = 1000.0
on = end + 0.3; ov = g.overlapped(on, end)
d = g.bargein_decision(PH, AWAKE) if ov else None
check("08:21:13 phrase +0.3 s after playback (echo guard) -> barge drive 10 s", ov and d["action"] == "drive" and drive10(d["motion"]),
      "overlap=%s %s" % (ov, dsc(d["motion"]) if d else "-"))
on = end + 1.0; ov = g.overlapped(on, end); pm = g.parse_motion(PH)
ok = (not ov) and g.followup_open(on, end) and not g.kill_phrase(PH) and drive10(pm)
check("08:21:13 phrase +1.0 s after playback -> normal follow-up, drive 10 s", ok, "overlap=%s %s" % (ov, dsc(pm)))
t_listen = end + 0.05; padded = end + 0.3                   # old rule: flag-padded end vs start of the listen slice
check("old window rule flagged it (any_last > t_listen-0.5); new rule does not", padded > t_listen - 0.5 and not g.overlapped(end + 1.0, end), "old=True new=False")
check("overlap while playing -> True; +0.4 s -> True; +0.6 s -> False",
      g.overlapped(end + 5, end, playing=True) and g.overlapped(end + 0.4, end) and not g.overlapped(end + 0.6, end))

# --- (c) natural phrasing
pm = g.parse_motion("can you please go forward for ten seconds")
check("'can you please go forward for ten seconds' -> drive 10 s", drive10(pm), dsc(pm))
d = g.bargein_decision("can you please go forward for ten seconds", AWAKE)
check("same phrase over playback -> drive 10 s", d["action"] == "drive" and drive10(d["motion"]), dsc(d["motion"]))
for ph, want in (("Hey G-Bot, go ahead and drive forward for 5 seconds please", (1, 5)), ("okay let's back up for two seconds now", (-1, 2)),
                 ("I need you to drive forward for thirty seconds for me", (1, 10)), ("G-Bot could you move forward", (1, 3))):
    pm = g.parse_motion(ph)
    ok = bool(pm) and pm["op"] == "timed" and (pm["lin"] > 0) == (want[0] > 0) and pm["dur"] == want[1]
    check("%r -> %s %gs (3 s default / 10 s cap)" % (ph, "fwd" if want[0] > 0 else "back", want[1]), ok, dsc(pm))
pm = g.parse_motion("could you turn left please")
check("'could you turn left please' -> turn left 90", bool(pm) and pm["op"] == "turn" and pm["deg"] == 90, dsc(pm))
pm = g.parse_motion("I want you to tell me a joke")
check("'I want you to tell me a joke' -> no parse, no kill (router LLM fallback)", pm is None and not g.kill_phrase("I want you to tell me a joke"), dsc(pm))
r = g.strip_leadins_raw("Hey G-Bot, can you tell me the battery level please?")
check("raw lead-in strip keeps the rest for local intents", r == "tell me the battery level", repr(r))

# --- kills stay instant
for ph in ("I want you to halt", "please halt", "can you halt", "halt", "freeze", "abort"):
    k = g.kill_phrase(ph, moving=False); d = g.bargein_decision(ph, AWAKE)
    latched = False
    if k and d["action"] == "kill":
        g.estop("voice (natural test)", ph); latched = g.is_set(); g.clear("test")
    check("%r -> kill (idle + over playback)" % ph, k and d["action"] == "kill", "kill=%s barge=%s latched=%s" % (k, d["action"], latched))
for ph in ("I want you to stop", "please stop", "can you stop", "stop", "emergency stop"):
    k = g.kill_phrase(ph, moving=False); d = g.bargein_decision(ph, AWAKE)
    check("%r -> no longer a kill" % ph, (not k) and d["action"] != "kill", "kill=%s barge=%s" % (k, d["action"]))
check("'drive forward for 5 seconds then stop' still no kill", not g.kill_phrase("I want you to drive forward for 5 seconds and then stop"))

# --- (b) resume over playback while latched
for sp, nm in ((REFUSE, "refusal line"), (LATCH, "latch line")):
    g.estop("test", "latched test"); d = g.bargein_decision("resume resume", sp)
    if d["action"] == "resume": g.clear("voice")
    check("'resume resume' over %s while latched -> resume, cleared (temp ESTOP)" % nm, d["action"] == "resume" and not g.is_set(), d["action"])
    g.clear("test")
for echo in ("Say resume when it's safe.", "resume when it's safe", "Say resume", "Resume."):
    g.estop("test", "latched test"); d = g.bargein_decision(echo, LATCH); still = g.is_set(); g.clear("test")
    check("own echo %r while latched -> NOT cleared" % echo, d["action"] != "resume" and still, d["action"])
g.estop("test", "latched test"); d = g.bargein_decision("Say resume first", REFUSE); still = g.is_set(); g.clear("test")
check("own echo 'Say resume first' while latched -> NOT cleared", d["action"] != "resume" and still, d["action"])
d = g.bargein_decision("resume resume", LATCH)
check("'resume resume' over playback when NOT latched -> no resume action", d["action"] != "resume", d["action"])
check("idle path: 'resume resume' / 'please resume' / 'okay go' are resume phrases",
      g.resume_phrase("resume resume") and g.resume_phrase("please resume") and g.resume_phrase("okay go") and not g.resume_phrase("say resume when it's safe"))

g.ESTOP = REAL_ESTOP
check("real data/ESTOP untouched", os.path.exists(REAL_ESTOP) == real_before, "exists=%s" % os.path.exists(REAL_ESTOP))
w = max(len(r[0]) for r in rows)
for r in rows: print("%-*s  %-40s %s" % (w, r[0], r[1][:40], r[2]))
print("RESULT: %d/%d pass" % (len(rows) - fails, len(rows)))
sys.exit(1 if fails else 0)
