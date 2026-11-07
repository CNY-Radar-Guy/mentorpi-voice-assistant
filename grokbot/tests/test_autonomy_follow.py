"""Fail-closed autonomy and follow. No robot and no motor publish."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gb_estop
import gb_follow
import gb_health
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

now = time.time()
gb_wally.set_autonomy(False)
fresh = {"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.9, "user_lidar": True}
track = {"fresh": True, "conf": 0.9, "source": "lidar", "depth_m": 3.0, "clearance_m": 2.0, "bearing": 0.0, "t": now}
check("stale health blocks autonomy", "a fresh range check" in gb_wally.autonomy_blockers(dict(fresh, t=now - 10), track, True, now))
check("missing target blocks autonomy", "a fresh measured person" in gb_wally.autonomy_blockers(fresh, None, True, now))
check("ambiguous target blocks autonomy", "one person" in " ".join(gb_wally.autonomy_blockers(fresh, dict(track, ambiguous=True), True, now)))
check("low confidence blocks autonomy", gb_wally.autonomy_blockers(fresh, dict(track, conf=0.2), True, now))
check("no heartbeat blocks autonomy", "watchdog" in " ".join(gb_wally.autonomy_blockers(fresh, track, False, now)))
check("a ready set has no blocker", gb_wally.autonomy_blockers(fresh, track, True, now) == [])
check("the word john is not a target", gb_wally.autonomy_blockers(fresh, {"fresh": True, "name": "john"}, True, now))

tmp = tempfile.mkdtemp(prefix="gbauto_")
person = os.path.join(tmp, "person.json")
wd = os.path.join(tmp, "wd.json")
health = os.path.join(tmp, "health.json")
demand = os.path.join(tmp, "demand.json")
os.environ["PERSON_TRACK_FILE"] = person
os.environ["WD_STATE_HOST"] = wd
os.environ["HEALTH_FILE"] = health
os.environ["GB_DEMAND_FILE"] = demand
json.dump({"t": now, "maint": False, "zeroing": False, "why": "idle"}, open(wd, "w"))
json.dump(fresh, open(health, "w"))
json.dump({"person": {"t": now, "conf": 0.2, "depth_m": 3.0, "source": "depth", "unit": "crosscheck_mm", "name": "john"}}, open(person, "w"))
said = gb_wally.handle("turn autonomy on")
check("autonomy on is refused without a confident track", "stays off" in said and gb_wally._autonomy is False, said)
json.dump({"person": None, "detect_ok": True}, open(person, "w"))
check("no people is not a follow", gb_wally.read_person_track(person, now) is None)
json.dump({"person": {"t": now, "conf": 0.9, "depth_m": 3.0, "source": "lidar", "clearance_m": 2.2, "name": "john"}}, open(person, "w"))
said = gb_wally.handle("enable autonomy")
check("autonomy on does not start a drive", gb_wally._autonomy and "not moved" in said and "John" not in said, said)
said = gb_wally.handle("drive toward me")
check("follow names the measurement and not John", "not proof that the person is John" in said or "sent the gate" in said, said)
check("follow demand is on while the loop is active", __import__("gb_demand").read_demand(demand) == "follow")
gb_wally.follow_loop().step(now, dict(fresh, t=now - 10), gb_wally.read_person_track(person, now), True)
check("stale health cancels the follow", gb_wally.follow_loop().active is False)
gb_wally._follow_demand(gb_wally.follow_loop().active)
check("follow demand stops when idle", __import__("gb_demand").read_demand(demand) != "follow")
said = gb_wally.handle("autonomy off")
check("autonomy off drops the trip", "not moving" in said and gb_wally._autonomy is False, said)
lost = gb_follow.FollowLoop()
lost.start(now)
step = lost.step(now, fresh, None, True)
check("a missing person does not move", step["allow"] is False and step["lin"] == 0.0, step)
amb = gb_follow.FollowLoop()
amb.start(now)
step = amb.step(now, fresh, {"fresh": True, "ambiguous": True, "conf": 0.9, "source": "lidar", "depth_m": 3}, True)
check("two people do not move", step["reason"] == "ambiguous" and step["lin"] == 0.0, step)
check("a millisecond stamp is still refused when it is old",
      gb_health.read_shared(health, now) is not None)
json.dump({"t": (now - 10) * 1000, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.9}, open(health, "w"))
check("an old millisecond stamp is refused", gb_health.read_shared(health, now) is None)
json.dump({"stamp": {"sec": int(now - 30), "nanosec": 0}, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.9}, open(health, "w"))
check("a stale ROS stamp is refused", gb_health.read_shared(health, now) is None)
json.dump({"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.9, "depth_topic_online": True, "depth_metric_valid": False, "depth_online": True}, open(health, "w"))
snap = gb_health.read_shared(health, now)
check("topic online does not become metric", snap["depth_online"] is False, snap)
report = gb_health.watchdog_report(False, False, "rel", now, "idle")
check("watchdog state is false after the hold is gone", report["maint"] is False and report["zeroing"] is False, report)
check("hardware event is not a wake", gb_wally.software_wake("hello", hardware_event=True) is False)
check("wake does not create a stop", gb_estop.stop_action("wake up Wally") is None)

print("test_autonomy_follow: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
