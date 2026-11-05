"""Offline barge-in / follow-up window tests: decision functions only, no audio, no motors, temp ESTOP path."""
import os, sys, tempfile
sys.path.insert(0, "/home/pi/grokbot")
import gb_estop as g

REAL_ESTOP = g.ESTOP
real_before = os.path.exists(REAL_ESTOP)
tmpd = tempfile.mkdtemp(prefix="gb_bargein_")
g.ESTOP = os.path.join(tmpd, "ESTOP"); g.SHARED_ESTOP = os.path.join(tmpd, "ESTOP_shared"); g.LOG = os.path.join(tmpd, "estop.log")
g.zero_now = lambda why="": None          # never publish to rosbridge from a test
g.gate_request = lambda d: (_ for _ in ()).throw(RuntimeError("test must not call the gate"))

REPLY = ["Released. Motion is allowed again. I'm not moving; give me a new command."]   # robot's current speech
rows = []; fails = 0
def check(name, ok, got):
    global fails
    fails += not ok
    rows.append((name, got, "PASS" if ok else "FAIL"))

def dec(text, spoken=REPLY):
    return g.bargein_decision(text, spoken)

for ph, sign, dur in (("Drive forward for 4 seconds", 1, 4), ("Move forward for 10 seconds", 1, 10),
                      ("go back for 2 seconds", -1, 2), ("hey G-Bot, drive forward", 1, 3.0), ("reverse for 30 seconds", -1, 10)):
    d = dec(ph); pm = d["motion"]
    ok = d["action"] == "drive" and pm and pm["op"] == "timed" and (pm["lin"] > 0) == (sign > 0) and pm["dur"] == dur
    check("speech: %r -> drive" % ph, ok, "%s %s" % (d["action"], g.describe(pm) if pm else "-"))
d = dec("turn left"); check("speech: 'turn left' -> drive (turn)", d["action"] == "drive" and d["motion"]["op"] == "turn", "%s %s" % (d["action"], g.describe(d["motion"]) if d["motion"] else "-"))

ECHO = ["Okay, I'll drive forward for 3 seconds (my default)."]
for ph in ("I'll drive forward for 3 seconds.", "drive forward for 3 seconds my default", "Okay I'll drive forward"):
    d = dec(ph, ECHO); check("echo of own speech: %r -> ignore" % ph, d["action"] == "ignore", "%s left=%r" % (d["action"], d["left"]))
d = dec("Release. Motion is a lot again.", REPLY); check("echo (log 07:43:28) -> ignore", d["action"] == "ignore", "%s left=%r" % (d["action"], d["left"]))

for ph in ("stop", "Stop!", "halt", "emergency stop", "G-Bot stop"):
    d = dec(ph)
    latched = False
    if d["action"] == "kill":
        g.estop("voice (barge-in test)", ph); latched = g.is_set(); g.clear("test")
    check("speech: %r -> kill + latch (temp ESTOP)" % ph, d["action"] == "kill" and latched, "%s latched=%s" % (d["action"], latched))

for ph in ("forward for 5 seconds then halt", "forward for 5 seconds then stop", "go forward for 5 seconds and pause"):
    d = dec(ph)
    ok = d["action"] == "drive" and d["motion"]["dur"] == 5 and not g.kill_phrase(d["cmd"], moving=False)
    check("speech: %r -> drive, no kill" % ph, ok, "%s %s" % (d["action"], g.describe(d["motion"]) if d["motion"] else "-"))

SP2 = ["Emergency stop. Motors are latched off. Say resume when it's safe."] + REPLY
d = dec("Move forward for 10 seconds. Stop. Turn around. Come back for 10 seconds.", SP2)
check("speech: multi-step with masked 'stop' (log 07:43:38) -> no drive", d["action"] != "drive", d["action"])
g.estop("test", "latched test"); d = dec("Drive forward for 4 seconds"); g.clear("test")
check("speech + ESTOP latched: drive -> 'latched' (no move)", d["action"] == "latched", d["action"])

end = 1000.0
check("follow-up deadline = end of playback + 20 s", g.followup_deadline(end) == end + 20 and g.FOLLOWUP_WINDOW_S == 20, str(g.followup_deadline(end) - end))
ph = "Drive forward for 10 seconds"
for dt, want in ((11, True), (15, True), (19.9, True), (25, False)):
    op = g.followup_open(end + dt, end); pm = g.parse_motion(ph) if op else None
    acc = bool(op and pm and not g.kill_phrase(ph))
    check("no wake word %r at %+gs after playback end -> %s" % (ph, dt, "accept" if want else "reject"), acc == want,
          ("accept: " + g.describe(pm)) if acc else "reject (window closed)")

g.ESTOP = REAL_ESTOP
check("real data/ESTOP untouched", os.path.exists(REAL_ESTOP) == real_before, "exists=%s" % os.path.exists(REAL_ESTOP))
w = max(len(r[0]) for r in rows)
for r in rows: print("%-*s  %-44s %s" % (w, r[0], r[1][:44], r[2]))
print("RESULT: %d/%d pass" % (len(rows) - fails, len(rows)))
sys.exit(1 if fails else 0)
