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
door = "explore the room and stop at the door"
check("stop at the door is stored and is not a halt",
      gb_estop.future_condition(door) == "stop at the door" and gb_estop.stop_action(door, moving=True) is None)
placed = gb_intent.open_lease(door, 2000, 90)
check("a place clause stays inside 45 seconds and is not enforced",
      placed["condition"] == "stop at the door" and placed["condition_kind"] == "place"
      and placed["condition_enforced"] is False and placed["cap_s"] == 45 and placed["indefinite"] is False)
pending = gb_intent.pending_condition("stop at the door", 5)
check("a place clause with no look does not move",
      pending["motion"] is False and pending["condition"] == "stop at the door" and pending["condition_enforced"] is False)
import gb_motor_cap as cap
check("half rps is half the documented command corner",
      abs(cap.half_rps() - cap.peak_rps() / 2) < 1e-12 and abs(cap.peak_rps() - cap.mps_to_rps(cap.track_mps(0.2, 0.5))) < 1e-12)
clamped = cap.clamp_motors([(1, 5.0), (2, -5.0), (3, 0.1), (4, -0.2)])
check("each track is clamped in both directions",
      clamped and all(abs(r) <= cap.half_rps() + 1e-12 for _i, r in clamped) and clamped[0][1] > 0 and clamped[1][1] < 0)
check("a nonfinite motor command is rejected", cap.clamp_motors([(1, float("nan"))]) is None and cap.clamp_motors([(1, float("inf"))]) is None)
check("a malformed motor command is rejected", cap.clamp_motors([(9, 0.1)]) is None and cap.clamp_motors("fast") is None)
cfg = os.path.join(os.environ["GB_SHARED_DIR"], "motor_cap.json")
cap.write_config(cfg)
saved = cap.read_config(cfg)
check("the saved cap matches the derivation and is not a firmware rating",
      saved and saved["firmware_cap"] is False and saved["serial_bypass"] is True and saved["enforced_on_robot"] is False)
loose = dict(saved); loose["half_rps"] = saved["half_rps"] * 2
check("a looser saved cap is rejected", cap.config_ok(loose) is False)
import gb_actuation
gb_wally.bounded_look().cancel("test")
refused = gb_wally.handle("explore the room and stop at the door")
check("a place condition does not start the 45 second look",
      "will not start" in refused and gb_wally.bounded_look().requested is False, refused)
lease = gb_intent.open_lease("look around", 0, 45)
straight = cap.tank_rps_map(0.05, 0.0)
claim = {"lease_id": lease["id"], "source": "voice", "action": "bounded_look", "lin": 0.05, "ang": 0.0}
ok_speeds, ok_dec = gb_actuation.admit(straight, 10, lease, maint=False, claim=claim)
check("a live lease under the cap is admitted", ok_dec["admitted"] and ok_dec["reason"] == "within_provisional_cap" and ok_dec["serial_bypass"] is True)
borrow, borrow_dec = gb_actuation.admit(cap.tank_rps_map(0.0, 0.4), 10, lease, maint=False, claim=claim)
check("another direction cannot borrow the lease", borrow_dec["reason"] == "direction" and all(r == 0.0 for _i, r in borrow))
corner = cap.tank_rps_map(0.2, 0.5)
over_claim = dict(claim, lin=0.2, ang=0.5)
over, over_dec = gb_actuation.admit(corner, 10, lease, maint=False, claim=over_claim)
check("an over-cap command is clamped and labeled provisional",
      over_dec["admitted"] and over_dec["reason"] == "clamped" and over_dec["hardware_top_speed_verified"] is False
      and all(abs(r) <= cap.half_rps() + 1e-9 for _i, r in over))
late_s, late_dec = gb_actuation.admit(straight, 50, lease, maint=False, claim=claim)
check("motion after the deadline becomes zero", late_dec["reason"] == "deadline" and all(r == 0.0 for _i, r in late_s))
held, held_dec = gb_actuation.admit(straight, 10, lease, maint=True, claim=claim)
check("maintenance replaces motion with zero", held_dec["reason"] == "maintenance" and all(r == 0.0 for _i, r in held))
bare, bare_dec = gb_actuation.admit(straight, 10, None, maint=False, claim=claim)
check("motion without a lease becomes zero", bare_dec["reason"] == "no_matching_lease" and bare_dec["repair"]["applied"] is False)
bad, bad_dec = gb_actuation.admit([(1, float("nan"))], 10, lease, maint=False, claim=claim)
check("a bad motor list becomes zero", bad_dec["reason"] == "malformed" and all(r == 0.0 for _i, r in bad))
place = gb_intent.open_lease("explore and stop at the door", 0, 45)
place_claim = dict(claim, lease_id=place["id"])
blocked, blocked_dec = gb_actuation.admit(straight, 10, place, maint=False, claim=place_claim)
check("a place lease is not admitted", blocked_dec["reason"] == "unenforced_condition" and all(r == 0.0 for _i, r in blocked))
launch = open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "deploy", "ros_robot_controller.remap.launch.py")).read()
install = open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "deploy", "actuation_install.sh")).read()
check("the public remap is refused and is not the boundary",
      "refusing to remap" in launch and "/gb/set_motor_admitted" in launch and "/ros_robot_controller/set_motor" in launch)
check("install does not clear the hold or the watchdog",
      "Do not remove MAINT" in install and "Do not disable" in install and "gb-watchdog" in install)

print("test_intent: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
