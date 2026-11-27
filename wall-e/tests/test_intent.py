"""Partial intent report. No twist, no MAINT removal, no motor patch."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_estop
import gb_intent

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

check("a bare stop no longer halts", gb_estop.stop_action("stop") is None)
check("a bare halt still halts", gb_estop.stop_action("halt") == "halt")
check("emergency stop no longer latches", gb_estop.stop_action("emergency stop") is None)
check("emergency halt still latches", gb_estop.stop_action("emergency halt") == "latch")
check("a quoted emergency stop does not halt", gb_estop.stop_action('"emergency stop"') is None)
check("a negated stop does not halt", gb_estop.stop_action("do not stop") is None)
ask = "hey drive around the room and explore stop when I tell you"
check("a future stop does not halt", gb_estop.future_stop(ask) and gb_estop.stop_action(ask, moving=True) is None)
item = gb_intent.note("look", ask)
check("a note is not approved and is not an interlock", item["approved"] is False and item["interlock"] is False)
naked = gb_intent.observe([{"topic": "/controller/cmd_vel", "node": "lidar_app", "nz": True}], [item])
check("a stock writer without an intent is reported and not called an interlock",
      naked["interlock"] is False and naked["partial"] is True and naked["intent_free"] and naked["wheel_feedback"] is False, naked)
check("the known bypasses stay listed", "hand_gesture" in naked["bypasses"] and "odom_publisher" in naked["bypasses"])
ok = gb_intent.observe([{"topic": "/gb/motion_req", "node": "voice", "nz": True, "intent_id": item["id"]}], [item])
check("a named intent is not intent-free", ok["intent_free"] == [])
decided = gb_intent.approve(item, True)
check("approval has no velocity", decided["approved"] is True and "lin" not in decided and "ang" not in decided and decided["interlock"] is False)
check("rollback does not clear MAINT", "do not remove MAINT" in gb_intent.rollback_steps())
path = gb_intent.write_status(naked)
check("the report is the sandbox file", path == os.environ["GB_INTENT_FILE"] and "interlock" in open(path).read())
unit = open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "deploy", "gb-intent.service")).read()
check("the unit publishes no twist and is not a maint clearer", "Publishes nothing" in open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gb_intent_node.py")).read() and "MAINT" in unit)
act = open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "deploy", "wally_activate.sh")).read()
check("activation leaves MAINT and does not enable the observer or clap",
      "MAINT was left in place" in act and "gb-intent was not enabled" in act and "gb-clap was not enabled" in act
      and "release_own_deploy_hold" not in act)

print("test_intent: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
