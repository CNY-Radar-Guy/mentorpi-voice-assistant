"""Offline barge-in / follow-up window tests: decision functions only, no audio, no motors, temp ESTOP path."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop as g

REAL_ESTOP = g.ESTOP
real_before = os.path.exists(REAL_ESTOP)
tmpd = tempfile.mkdtemp(prefix="gb_bargein_")
g.ESTOP = os.path.join(tmpd, "ESTOP"); g.SHARED_ESTOP = os.path.join(tmpd, "ESTOP_shared"); g.LOG = os.path.join(tmpd, "estop.log")
g.zero_now = lambda why="": None          # never publish to rosbridge from a test
g.gate_request = lambda d: (_ for _ in ()).throw(RuntimeError("test must not call the gate"))
g.floor_mode = lambda: False          # tests use a temp latch; do not inherit the live floor/ground mode


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

for ph in ("halt", "freeze", "abort"):
    d = dec(ph)
    check("speech: %r -> kill" % ph, d["action"] == "kill", d["action"])
for ph in ("stop", "Stop!", "emergency stop", "G-Bot stop", "wally stop", "go stop", "we stop"):
    d = dec(ph)
    check("speech: %r -> no longer a kill" % ph, d["action"] != "kill", d["action"])

for ph in ("forward for 5 seconds then halt", "forward for 5 seconds then stop", "go forward for 5 seconds and pause"):
    d = dec(ph)
    ok = d["action"] == "drive" and d["motion"]["dur"] == 5 and not g.kill_phrase(d["cmd"], moving=False)
    check("speech: %r -> drive, no kill" % ph, ok, "%s %s" % (d["action"], g.describe(d["motion"]) if d["motion"] else "-"))

SP2 = ["Emergency stop. Motors are latched off. Say resume when it's safe."] + REPLY
d = dec("Move forward for 10 seconds. Halt. Turn around. Come back for 10 seconds.", SP2)
check("speech: multi-step with halt -> no drive", d["action"] != "drive", d["action"])
d = dec("Move forward for 10 seconds. Stop. Turn around. Come back for 10 seconds.", SP2)
check("speech: multi-step with leftover 'stop' is not a kill", d["action"] != "kill", d["action"])
# Floor mode never writes a latch file, so a drive over playback stays 'drive' unless a latch exists.
g.estop("test", "latched test")
d = dec("Drive forward for 4 seconds")
want = "latched" if g.is_set() else "drive"
g.clear("test")
check("speech + estop() call: drive action is 'latched' only if a latch file exists (floor: drive)", d["action"] == want, d["action"])

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
