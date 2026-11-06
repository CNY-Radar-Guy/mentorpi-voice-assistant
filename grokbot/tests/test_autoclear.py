"""Offline ESTOP auto-clear tests: temp ESTOP paths, mocked clock + one short real timeout. No audio, rosbridge or motors."""
import json, os, sys, tempfile, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop as g

REAL = g.ESTOP; real_sig = g.latch_sig(REAL)
tmpd = tempfile.mkdtemp(prefix="gb_autoclear_")
g.ESTOP = os.path.join(tmpd, "ESTOP"); g.SHARED_ESTOP = os.path.join(tmpd, "ESTOP_shared"); g.LOG = os.path.join(tmpd, "estop.log")
MOTION = []                                   # any motion/zero/publish call lands here; auto-clear must add nothing
g.zero_now = lambda why="": MOTION.append(("zero_now", why))
g.gate_request = lambda d: MOTION.append(("gate_request", d)) or "x"
g._pub = lambda *a, **k: MOTION.append(("_pub", a)) or True
g.warm_up = lambda: MOTION.append(("warm_up",))
until0 = g._MOTION["until"]

rows = []; fails = 0
def check(name, ok, got=""):
    global fails
    fails += not ok; rows.append((name, str(got), "PASS" if ok else "FAIL"))

def latch(t, touch=False):
    """Simulate a latch written at fake time t (both files, like estop()); mtime = t so every latch has a new identity."""
    for p in (g.ESTOP, g.SHARED_ESTOP):
        with open(p, "w") as f:
            if not touch: json.dump({"t": t, "src": "test", "why": "test"}, f)
        os.utime(p, (t, t))

def both_gone(): return not os.path.exists(g.ESTOP) and not os.path.exists(g.SHARED_ESTOP)
clk = [1000.0]; said = []
def mk(timeout=15): return g.AutoClear(timeout, clock=lambda: clk[0], on_clear=lambda: said.append(clk[0]))

# 1 config
check("ESTOP_AUTOCLEAR_S parse: 15 / 0 / -3 / junk", (g.autoclear_s("15"), g.autoclear_s("0"), g.autoclear_s("-3"), g.autoclear_s("x")) == (15, 0, 0, 0),
      (g.autoclear_s("15"), g.autoclear_s("0"), g.autoclear_s("-3"), g.autoclear_s("x")))
check("product default timer is off", g.autoclear_s() == 0)
# 2 does not clear before the timeout, clears at it
ac = mk(); latch(1000.0)
r = []
for t in (1000.0, 1007.0, 1014.9):
    clk[0] = t; r.append(ac.step())
check("not cleared before 15 s (0, 7, 14.9 s)", r == [False] * 3 and g.is_set(), r)
clk[0] = 1015.0; c = ac.step()
check("cleared at 15 s (host + shared file gone)", c and both_gone(), c)
check("on_clear (speak line) fired once", said == [1015.0], said)
lg = open(g.LOG).read()
check("logs 'ESTOP CLEARED by auto' + 'ESTOP AUTO-CLEARED after 15s'", "ESTOP CLEARED by auto" in lg and "ESTOP AUTO-CLEARED after 15s" in lg,
      [l[20:] for l in lg.splitlines()])
clk[0] = 1100.0; check("nothing to do once clear", ac.step() is False and said == [1015.0])
# 3 a re-latch resets the timer
said.clear(); clk[0] = 1000.0; ac = mk(); latch(1000.0); clk[0] = 1010.0; ac.step()
latch(1010.0)                                  # new stop 10 s into the first latch
r = []
for t in (1015.0, 1020.0, 1024.9):
    clk[0] = t; r.append(ac.step())
check("re-latch at 10 s: still latched at 15/20/24.9 s", r == [False] * 3 and g.is_set(), r)
clk[0] = 1025.0; check("re-latch: cleared 15 s after the NEW stop (25 s)", ac.step() and both_gone() and said == [1025.0], said)
# 4 re-latch exactly when the old timer expires (same t, different file identity) still restarts
said.clear(); clk[0] = 1000.0; ac = mk(); latch(1000.0); clk[0] = 1001.0; ac.step(); latch(1014.0)
clk[0] = 1015.0; r1 = ac.step(); clk[0] = 1029.0; r2 = ac.step()
check("re-latch at 14 s: not cleared at 15 s, cleared at 29 s", (r1, r2) == (False, True), (r1, r2))
# 5 0 disables
said.clear(); ac0 = mk(0); latch(1000.0)
r = []
for t in (1015.0, 1100.0, 10 ** 6):
    clk[0] = t; r.append(ac0.step())
check("ESTOP_AUTOCLEAR_S=0: never clears", r == [False] * 3 and g.is_set() and not said, r)
g.clear("test")
# 6 SSH 'touch' latch (no JSON) times from the file mtime
clk[0] = 1000.0; ac = mk(); latch(1000.0, touch=True); clk[0] = 1014.0; r1 = ac.step(); clk[0] = 1015.0; r2 = ac.step()
check("touched (empty) latch: mtime-based, clears at 15 s", (r1, r2) == (False, True) and both_gone(), (r1, r2))
# 7 latch older than this process: a restart must not clear it
clk[0] = 1000.0; ac = mk(); latch(900.0); clk[0] = 1000.0
check("latch that predates this process survives restart", ac.step() is False and g.is_set())
clk[0] = 2000.0
check("preexisting latch is not cleared later by the timer", ac.step() is False and g.is_set())
# 8 future stamp (clock jump) counts from now
ac = mk(); latch(5000.0); clk[0] = 1000.0; r1 = ac.step(); clk[0] = 1014.0; r2 = ac.step(); clk[0] = 1015.0; r3 = ac.step()
check("future-stamped latch: counts from first sight", (r1, r2, r3) == (False, False, True), (r1, r2, r3))
# 9 real clock, short timeout, using the real estop() latch path
said.clear(); ac = g.AutoClear(0.6, on_clear=lambda: said.append("say"))
g.estop("test", "real-clock test")
n_latch = len(MOTION)                          # estop() itself zeroes (stop, not motion): not counted against auto-clear
time.sleep(0.25); r1 = ac.step()
time.sleep(0.5); r2 = ac.step()
check("real clock 0.6 s: not at 0.25 s, cleared at 0.75 s", (r1, r2) == (False, True) and both_gone() and said == ["say"], (r1, r2))
time.sleep(0.2)                                # let estop()'s zero thread finish before counting
# 10 no motion issued by auto-clear
auto_motion = [m for m in MOTION if m[0] != "zero_now" or "estop" not in str(m[1])]
check("auto-clear issued NO motion (no gate_request/_pub/Twist/resume)", auto_motion == [] and g._MOTION["until"] == until0,
      MOTION)
check("real ESTOP untouched by the test", g.latch_sig(REAL) == real_sig)

for n, got, res in rows: print("%-4s %-62s %s" % (res, n, got[:90]))
print("RESULT: %d/%d pass" % (len(rows) - fails, len(rows)))
sys.exit(1 if fails else 0)
