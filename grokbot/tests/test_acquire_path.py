"""Producer and consumer share the default state directory. No robot and no drive."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gb_demand
import gb_health
import gb_metric
import gb_obstacle as OB
import gb_paths
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

tmp = tempfile.mkdtemp(prefix="gbacq_")
os.environ.pop("PERSON_TRACK_FILE", None)
os.environ.pop("GB_DEMAND_FILE", None)
os.environ["GB_SHARED_DIR"] = tmp
os.environ["GB_SHARED_ESTOP"] = os.path.join(tmp, "ESTOP")
os.environ["HEALTH_FILE"] = os.path.join(tmp, "health.json")
os.environ["WD_STATE_HOST"] = os.path.join(tmp, "watchdog_state.json")
check("voice and tracker use one track path", gb_paths.person_track_path() == os.path.join(tmp, "person_track.json"))
check("voice and tracker use one demand path", gb_paths.demand_path() == os.path.join(tmp, "demand.json"))
gb_demand.write_demand(gb_paths.demand_path(), "acquire", now=time.time())
check("acquire is not a motor command", gb_demand.read_demand(gb_paths.demand_path()) == "acquire" and gb_demand.moves("acquire") is False)
check("acquire asks for the camera while stopped", gb_demand.tracker_wants("acquire", moving=False))
check("idle does not ask for the camera", gb_demand.tracker_wants(None, moving=False) is False)
check("acquire stays on the low duty", gb_demand.tracker_period("acquire", False, 0.7) >= 2.0)
body = gb_metric.sighting_record(time.time(), [{"conf": 0.91, "box": [1, 2, 3, 4]}])
with open(gb_paths.person_track_path(), "w") as f:
    json.dump(body, f)
check("a sighting is not a measured range", gb_wally.read_person_track() is None)
sight = gb_wally.read_sighting()
check("the consumer sees the detection", sight and sight.get("reason") == "depth_unit_unconfirmed" and sight.get("lidar_reason") == "extrinsics_not_validated", sight)
now = time.time()
json.dump({"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.96, "user_lidar": True}, open(os.environ["HEALTH_FILE"], "w"))
json.dump({"t": now, "maint": False, "zeroing": False}, open(os.environ["WD_STATE_HOST"], "w"))
gb_wally.set_autonomy(False)
said = gb_wally.handle("turn autonomy on")
check("autonomy stays off without a range", "stays off" in said and gb_wally._autonomy is False and "not moving" in said, said)
check("the failed check asks the camera to look", gb_demand.read_demand(gb_paths.demand_path()) == "acquire")
front = [(0.0, 0.98), (0.4, 2.4)]
check("an uncalibrated lidar beam is not a person range", OB.lidar_person_range(front, 0.4, None) is None)
check("the forward beam is not borrowed", OB.lidar_person_range(front, 0.4, {"validated": False, "x_m": 0.08, "y_m": 0, "z_m": 0.04, "yaw_rad": 0, "measured_at": "2026-11-06"}) is None)
got = OB.lidar_person_range(front, 0.4, {"validated": True, "x_m": 0.082, "y_m": 0.0, "z_m": 0.045, "yaw_rad": 0.0, "measured_at": "measured-on-robot"})
check("a validated yaw uses the bearing beam, not the forward beam", got == 2.4, got)
gb_demand.write_demand(gb_paths.demand_path(), "acquire", now=time.time() - 30)
check("acquire expires", gb_demand.read_demand(gb_paths.demand_path()) is None)
said = gb_wally.handle("look for a person")
check("look does not enable autonomy", "not moving" in said and gb_wally._autonomy is False, said)

print("test_acquire_path: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
