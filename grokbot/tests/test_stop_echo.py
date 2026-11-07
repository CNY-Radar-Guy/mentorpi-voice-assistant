"""Own-voice halt echoes and a later stop condition. No audio and no motors."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop as g

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

HALT = "Stopped. I cancelled that motion and I did not latch the emergency lock. I am not continuing the old trip."
HEARD = [
    "Stop. I can't stop that motion and I did not watch the emergency lock. I'm not continuing the road trip.",
    "Stop. I can't solve the motion, and I do not watch the emergency lock.",
    "I can't stop. I can't stop that motion and I did not watch the emergency lock. I'm not continuing the road trip.",
    "Stop. I can't subvert motion and I did not watch the emergency lock. I'm not continuing the road to it.",
]
for line in HEARD:
    check("garbled halt reply is our own voice: " + line[:42], g.announcement_echo(line, [HALT]), line)
    check("garbled halt reply does not halt", g.stop_action(line, moving=True, spoken_texts=[HALT]) is None, line)
ask = "hey drive around the room and explore stop when I tell you"
check("a later stop condition is not a halt", g.future_stop(ask) and g.stop_action(ask, moving=True) is None, ask)
check("a bare stop still halts during the announcement", g.stop_action("stop", moving=True, spoken_texts=[HALT]) == "halt")
check("emergency stop still latches during the announcement",
      g.stop_action("emergency stop", moving=True, spoken_texts=[HALT]) == "latch")
check("stop during other speech is still a kill",
      g.bargein_decision("stop", [HALT])["action"] == "kill")
check("the garbled reply is not a kill during playback",
      g.bargein_decision(HEARD[0], [HALT])["action"] == "ignore")
check("drive then stop is still a drive",
      g.bargein_decision("go forward for 4 seconds then stop", [HALT])["action"] == "drive")
g._STOP_SAID["t"] = 0.0
check("the first halt may be spoken", g.claim_stop_speech(now=100))
check("the same halt is not spoken again inside 8 seconds", g.claim_stop_speech(now=104) is False)
check("a later halt may be spoken", g.claim_stop_speech(now=109))

print("test_stop_echo: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
