"""Bounded look through the gate. Replay only. A restart does not resume."""
import os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_bounded
import gb_obstacle as OB
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

look = gb_bounded.BoundedLook()
idle = look.step(0, 2, 2, 2, (0, 0, 0), 0.1, 0.1)
check("an unasked look does not move", idle["lin"] == 0.0 and idle["allow"] is False, idle)
look.begin(0, (1.0, 2.0, 0.0))
out = look.step(1, 2.0, 2.0, 2.0, (1.0, 2.0, 0.0), 0.1, 0.1)
check("clear space requests a slow forward", out["allow"] and 0 < out["lin"] <= gb_bounded.MAX_LIN, out)
blocked = look.step(2, OB.STOP_FWD, 2.0, 2.0, (1.2, 2.0, 0.0), 0.1, 0.1)
check("the forward stop blocks closing", blocked["lin"] == 0.0, blocked)
turned = look.step(3, OB.STOP_FWD - 0.01, 1.0, 1.0, (1.2, 2.0, 0.0), 0.1, 0.1)
check("a blocked front turns instead of driving in", turned["lin"] == 0.0 and turned["allow"] and turned["ang"] != 0.0, turned)
stale = look.step(4, 2, 2, 2, (1.2, 2.0, 0.0), 1.0, 0.1)
check("a stale scan stops the look", stale["reason"] == "sensors_stale" and look.active is False, stale)
look.begin(0, (0, 0, 0))
home = look.step(gb_bounded.CAP_S + 1, 2, 2, 2, (1.0, 0, 0), 0.1, 0.1)
check("after the cap it heads back by odometry", home["reason"] == "return_odom" and home["allow"], home)
arrived = look.step(gb_bounded.CAP_S + 2, 2, 2, 2, (0.05, 0, 0), 0.1, 0.1)
check("reaching the start pose stops", arrived["reason"] == "home" and look.active is False, arrived)
path = os.path.join(tempfile.mkdtemp(prefix="gbbound_"), "look.json")
look.save(path)
again = gb_bounded.BoundedLook().load(path)
check("a saved look does not resume", again.active is False and again.requested is False and again.reason == "not_resumed", again.reason)
said = gb_wally.handle("look around")
check("asking does not claim the wheels moved", "not moved" in said and "Nav2" in said, said)
check("obstacle limits stay 0.27, 0.28 and 0.24", OB.STOP_FWD == 0.27 and OB.STOP_REV == 0.28 and abs(OB.SWEEP_R - 0.24) < 1e-9)

print("test_bounded: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
