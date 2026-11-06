"""Offline re-latch self-voice filter tests (log 08:28:13 / 08:28:20): decision functions + AutoClear with a fake clock.
No audio, no motors; temp ESTOP path only (the real data/ESTOP is never touched)."""
import json, os, sys, tempfile
sys.path.insert(0, "/home/pi/grokbot")
import gb_estop as g

REAL_ESTOP = g.ESTOP
real_before = os.path.exists(REAL_ESTOP)
tmpd = tempfile.mkdtemp(prefix="gb_relatch_")
g.ESTOP = os.path.join(tmpd, "ESTOP"); g.SHARED_ESTOP = os.path.join(tmpd, "ESTOP_shared"); g.LOG = os.path.join(tmpd, "estop.log")
g.zero_now = lambda why="": None
g.gate_request = lambda d: (_ for _ in ()).throw(RuntimeError("test must not call the gate"))

rows = []; fails = 0
def check(name, ok, got=""):
    global fails
    fails += not ok; rows.append((name, str(got), "PASS" if ok else "FAIL"))

def latch(t):
    """A latch written at fake time t (like estop(): JSON t + mtime) -> new identity for AutoClear."""
    for p in (g.ESTOP, g.SHARED_ESTOP):
        with open(p, "w") as f: json.dump({"t": t, "src": "test", "why": "test"}, f)
        os.utime(p, (t, t))

ESTOP_SAY = "Emergency stop."                                    # stop listener's barge-in announcement
LATCH_SAY = "Emergency stop. Motors are latched off. Say resume when it's safe."
CLEAR_SAY = "Brake released. I'm staying put."
REFUSE = "I'm emergency-latched, so I won't move. Say resume first."
clk = [0.0]; said = []
ac = g.AutoClear(15, clock=lambda: clk[0], on_clear=lambda: said.append(clk[0]))

# ---- 08:28 sequence (T = 08:28:13)
T = 50000.0
# 1. John "H-Bot stop." while NOT latched -> first latch, instant, no self-voice filter consulted
k = g.kill_phrase("H-Bot stop.", moving=True)
if k and not g.is_set(): latch(T)
clk[0] = T; ac.step()
check("08:28:13 'H-Bot stop.' not latched -> instant first latch (temp ESTOP)", k and g.is_set(), "kill=%s latched=%s" % (k, g.is_set()))
# 2. robot plays "Emergency stop." 08:28:13.4-08:28:14.4; the stop listener hears it (onset 13.6) but transcribes it at 08:28:20
pe = T + 1.4; spoken = [ESTOP_SAY]
ok, why = g.relatch_decision("Emergency stop.", T + 0.6, pe, False, spoken)
check("08:28:20 own 'Emergency stop.' (audio onset during playback) -> NO re-latch", not ok, why)
ok, why = g.relatch_decision("Emergency stop.", T + 7.0, pe, False, spoken)
check("08:28:20 same echo stamped with laggy wall clock (+5.6 s after end) -> NO re-latch (own-line text)", not ok, why)
ok, why = g.relatch_decision("Emergency stop!", T + 7.0, pe, True, spoken)
check("own 'Emergency stop!' while still playing -> NO re-latch", not ok, why)
# 3. auto-clear: the echo did not restart the timer -> clears 15 s after the ORIGINAL latch (08:28:28), not 08:28:35
clk[0] = T + 7.0; s7 = ac.step(); clk[0] = T + 14.9; s149 = ac.step(); clk[0] = T + 15.0; s15 = ac.step()
check("auto-clear NOT restarted by own echo: still latched at +7/+14.9 s, cleared at +15 s", (s7, s149, s15) == (False, False, True) and not g.is_set(),
      "(%s,%s,%s) said=%s" % (s7, s149, s15, [x - T for x in said]))

# ---- other own lines never re-latch
for line, heard in ((LATCH_SAY, "Emergency stop. Motors are latched off."), (LATCH_SAY, "Say resume when it's safe."),
                    (CLEAR_SAY, "Brake released, I'm staying put."), (REFUSE, "Say resume first.")):
    ok, why = g.relatch_decision(heard, T + 30, T + 20, False, [line])
    check("own line %r at +10 s after playback -> NO re-latch" % heard, not ok, why)
ok, why = g.relatch_decision("Emergency stopped.", T + 3, pe, False, [ESTOP_SAY])
check("misheard own 'Emergency stopped.' -> NO re-latch (stems)", not ok, why)

# ---- John's genuine re-latch still works (not overlapped, not our line)
for heard in ("stop", "stop stop", "halt", "G-Bot stop", "I want you to stop", "freeze"):
    ok, why = g.relatch_decision(heard, pe + 2.0, pe, False, [ESTOP_SAY, LATCH_SAY])
    check("John %r 2 s after playback while latched -> re-latch" % heard, ok, why)
ok, why = g.relatch_decision("stop", pe + 0.3, pe, False, [ESTOP_SAY])
check("'stop' starting 0.3 s after playback end (echo tail) -> NO re-latch", not ok, why)
ok, why = g.relatch_decision("hello there", pe + 2.0, pe, False, [ESTOP_SAY])
check("no stop phrase -> no re-latch", not ok, why)
# genuine re-latch restarts the auto-clear timer
U = T + 100.0; latch(U); clk[0] = U; ac.step()
ok, why = g.relatch_decision("stop stop", U + 5.0, U + 1.4, False, [ESTOP_SAY])
if ok: latch(U + 5.0)
clk[0] = U + 15.0; a = ac.step(); clk[0] = U + 19.9; b = ac.step(); clk[0] = U + 20.0; c = ac.step()
check("John's re-latch at +5 s restarts timer: latched at +15/+19.9, cleared at +20", ok and (a, b, c) == (False, False, True), "(%s,%s,%s)" % (a, b, c))

# ---- CRITICAL: first latch is unfiltered (the filter only runs on the already-latched path)
g.clear("test")
for heard in ("Emergency stop.", "stop", "halt"):
    k = g.kill_phrase(heard, moving=True); filt, _ = g.relatch_decision(heard, T + 0.6, pe, True, [ESTOP_SAY])
    if k and not g.is_set(): g.estop("voice (relatch test)", heard)
    lat = g.is_set(); g.clear("test")
    check("NOT latched %r during own playback -> first latch anyway (filter would have said %s)" % (heard, filt), k and lat, "latched=%s" % lat)

# ---- unchanged neighbours
check("drive-then-stop exemption unchanged", not g.kill_phrase("forward for 10 seconds then stop", moving=True))
pm = g.parse_motion("I want you to drive forward for 30 seconds")
check("10 s cap / natural phrasing unchanged", bool(pm) and pm["dur"] == 10, g.describe(pm) if pm else "-")
check("follow-up 20 s / autoclear default off", g.FOLLOWUP_WINDOW_S == 20 and g.autoclear_s() == 0)

g.clear("test"); g.ESTOP = REAL_ESTOP
check("real data/ESTOP untouched", os.path.exists(REAL_ESTOP) == real_before, "exists=%s" % os.path.exists(REAL_ESTOP))
w = max(len(r[0]) for r in rows)
for r in rows: print("%-*s  %-44s %s" % (w, r[0], r[1][:44], r[2]))
print("RESULT: %d/%d pass" % (len(rows) - fails, len(rows)))
sys.exit(1 if fails else 0)
