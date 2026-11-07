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

check("a bare stop halts", gb_estop.stop_action("stop") == "halt")
check("emergency stop latches", gb_estop.stop_action("emergency stop") == "latch")
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
import gb_wally
import gb_bounded
said = gb_wally.handle("hey drive around the room and explore, stop when I tell you")
check("the whole sentence is a 45 second look and not a halt",
      said and "45 second" in said and "not a mapped explore" in said and "not moved" in said
      and "does not make the look longer" in said and "halt" not in said.lower(), said)
check("a generic later stop is not a lease and not a timed drive",
      gb_intent.is_bounded_lease("stop when I tell you") is False
      and gb_estop.parse_motion("drive forward until I say stop") is None
      and gb_estop.stop_action("drive forward until I say stop", moving=True) is None)
lease = gb_intent.open_lease(ask, 1000, 45)
check("the lease ends at 45 seconds and is not indefinite",
      lease["deadline"] == 1045 and lease["indefinite"] is False and lease["cap_s"] == 45 and lease["all_writer_pause"] is False)
early = gb_intent.compare(lease, 1010, [{"node": "voice", "nz": True, "intent_id": lease["id"],
                                        "odom_vx": 0, "odom_wz": 0, "odom_source": "command"}])
check("inside the lease a zero command-derived odom does not prove a stop",
      early["expected"] == "within_lease" and early["physical_stop_verified"] is False
      and early["stall_unverified"] is True and early["hardware_interlock"] == "unknown")
late = gb_intent.compare(lease, 1050, [{"node": "lidar_app", "topic": "/controller/cmd_vel", "nz": True,
                                       "odom_vx": 0, "odom_wz": 0, "odom_source": "command"}])
check("past the deadline a bypass is reported and not paused",
      late["expected"] == "stop" and late["past_deadline"] is True and "lidar_app" in late["ran_past_deadline"]
      and late["all_writer_pause"] is False and late["bypass_nonzero"] == ["lidar_app"])
repair = gb_intent.repair_record(lease, late, "A stock publisher was still nonzero after the lease.")
check("a repair record is not applied and has no velocity",
      repair["applied"] is False and repair["approved"] is False and "lin" not in repair and repair["physical_stop_verified"] is False)
check("the look cap stays 45", gb_bounded.CAP_S == 45.0)

print("test_intent: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
