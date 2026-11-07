import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop as g
# (phrase, expect): drive+/drive- = no kill (idle or moving) and parses as a timed drive/turn with that sign
#                   KILL = latches idle AND moving;  KILL-moving = latches while moving/playing (idle behaviour unchanged)
CASES = [("forward for 10 seconds then stop", "drive+"), ("forward for 10 seconds then stop.", "drive+"),
         ("forward for 10 seconds then halt", "drive+"), ("forward for 10 seconds then pause", "drive+"),
         ("go forward for 5 seconds and stop", "drive+"), ("G-Bot, drive forward for 4 seconds then halt.", "drive+"),
         ("backward for 3 seconds then pause", "drive-"), ("reverse for 2 seconds and then stop", "drive-"),
         ("back for 3 seconds then halt", "drive-"), ("go back for 3 seconds then halt", "drive-"),
         ("move forward then stop", "drive+"), ("forward for 30 seconds then halt", "drive+"),
         ("turn left then halt", "turn"), ("turn right for 2 seconds and pause", "turn"),
         ("stop", "KILL"), ("Stop!", "KILL"), ("G-Bot stop", "KILL"), ("stop stop", "KILL"), ("emergency stop", "KILL"),
         ("stop now", "KILL"), ("halt", "KILL"), ("halt.", "KILL"), ("hey G-Bot, halt", "KILL"), ("freeze", "KILL"),
         ("then halt", "KILL"), ("stop the robot", "KILL"), ("okay stop", "KILL"), ("forward stop", "KILL"),
         ("then stop", "KILL-moving"), ("and stop", "KILL-moving"), ("all right then stop", "KILL-moving"),
         ("stop, go forward then stop", "KILL-moving")]
fails = 0
print("%-46s %-11s %-8s %-8s %-38s %s" % ("phrase", "expect", "idle", "moving", "parse_motion", ""))
for ph, want in CASES:
    ki, km = g.kill_phrase(ph, moving=False), g.kill_phrase(ph, moving=True)
    pm = g.parse_motion(ph)
    if want == "KILL": ok = ki and km
    elif want == "KILL-moving": ok = km
    elif want == "turn": ok = (not ki) and (not km) and bool(pm) and pm["op"] == "turn"
    else:
        ok = (not ki) and (not km) and bool(pm) and pm["op"] == "timed" and (pm["lin"] > 0) == (want == "drive+") and pm["dur"] <= g.TIMED_CAP
    fails += not ok
    print("%-46s %-11s %-8s %-8s %-38s %s" % (ph[:46], want, "KILL" if ki else "no-kill", "KILL" if km else "no-kill",
          g.describe(pm) if pm else "-", "PASS" if ok else "FAIL"))
print("RESULT: %d/%d pass" % (len(CASES) - fails, len(CASES)))
sys.exit(1 if fails else 0)
