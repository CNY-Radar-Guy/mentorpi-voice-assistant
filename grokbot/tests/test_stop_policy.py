"""Voice stop policy. No robot, no microphone, no motor publish."""
import json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gb_clap_logic
import gb_estop
import gb_maint
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

check("emergency stop latches", gb_estop.stop_action("Emergency stop!") == "latch")
check("spaced emergency stop latches", gb_estop.stop_action("emergency   stop") == "latch")
check("quoted emergency stop does not latch", gb_estop.stop_action('"emergency stop"') is None)
check("negated emergency stop does not latch or resume",
      gb_estop.stop_action("do not emergency stop") is None and gb_estop.resume_phrase("don't stop") is False)
check("plain stop halts and does not latch", gb_estop.stop_action("stop") == "halt")
check("halt and hold up halt", gb_estop.stop_action("halt") == "halt" and gb_estop.stop_action("hold up") == "halt")
garble = "Command on. halt. Great call. Sit. Stand. Hold up. The camera is on."
check("the garbled barge-in halts and does not latch", gb_estop.stop_action(garble, moving=True) == "halt")
check("wake is not a stop and creates no lock", gb_estop.stop_action("wake up Wally") is None)
check("sit halts", gb_estop.stop_action("sit") == "halt")

tmp = tempfile.mkdtemp(prefix="gbstop_")
host = os.path.join(tmp, "ESTOP")
shared = os.path.join(tmp, "sharedESTOP")
maint = os.path.join(tmp, "MAINT")
gb_estop.ESTOP = host
gb_estop.SHARED_ESTOP = shared
gb_estop.zero_now = lambda *a, **k: None
check("a clear wake does not create a stop", not os.path.exists(host))
gb_wally.handle("wake up Wally")
check("wake left the stop files absent", not os.path.exists(host) and not os.path.exists(shared))
gb_estop.estop("voice", "emergency stop")
gb_wally.handle("wake up Wally")
gb_wally.handle("I am John")
check("an existing emergency stop survives wake and identity", os.path.isfile(host) and json.load(open(host))["kind"] == "normal")
json.dump({"t": 1, "src": "gate-tilt", "kind": "safety", "why": "tilt"}, open(host, "w"))
check("a hard fault is not a plain halt target", gb_estop.stop_action("stop") == "halt")
check("clearing a normal stop leaves the hard fault", gb_estop.clear("voice") is False and json.load(open(host))["kind"] == "safety")
gb_maint.write_lock([maint], now=1000)
old = json.load(open(maint))
check("an older deploy lock is not this run's hold", gb_maint.release_own_deploy_hold(maint, not_before=old["t"] + 100) is False and os.path.isfile(maint))
os.remove(maint)
gb_maint.write_lock([maint], now=5000)
check("this run's deploy hold can be removed after the check", gb_maint.release_own_deploy_hold(maint, not_before=4999) and not os.path.exists(maint))
check("an expired clap test does not become a live latch", gb_clap_logic.latch_enabled({"sensitivity": "high", "test_until": 10}, now=1000) is False)
check("an explicit latch flag is required to override a stale test", gb_clap_logic.latch_enabled({"test_until": 10, "latch": True}, now=1000) is True)
act = open(os.path.join(os.path.dirname(HERE), "..", "deploy", "wally_activate.sh")).read()
check("activation does not arm a bench hold or enable clap", "arm_bench" not in act and "gb-clap was not enabled" in act)

print("test_stop_policy: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
