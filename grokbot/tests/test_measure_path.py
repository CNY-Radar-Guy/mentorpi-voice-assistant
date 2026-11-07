"""Acquire detection to a measured range, then the voice guard. No robot."""
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

tmp = tempfile.mkdtemp(prefix="gbmeas_")
os.environ["GB_SHARED_DIR"] = tmp
os.environ.pop("PERSON_TRACK_FILE", None)
os.environ.pop("GB_DEMAND_FILE", None)
os.environ["HEALTH_FILE"] = os.path.join(tmp, "health.json")
os.environ["WD_STATE_HOST"] = os.path.join(tmp, "watchdog_state.json")
now = time.time()
box = [200, 100, 280, 300]
beams = [(0.0, 0.98), (0.35, 2.4)]
fx, cx = 421.35, 317.62
# box centre u=240, bearing atan2(240-317.62, 421.35) is negative, not 0.35
# build a beam at that exact bearing so the match is the calibration, not the front beam
bearing = gb_metric.bearing_from_box(box, fx, cx)
beams = [(0.0, 0.98), (bearing, 2.4)]
bad = {"validated": False, "x_m": 0.08, "y_m": 0.0, "z_m": 0.05, "yaw_rad": 0.0, "measured_at": "not-measured"}
check("unvalidated calibration stays closed", gb_metric.range_for_detection(box, fx, cx, beams, bad, 0.1) is None)
denied = gb_metric.range_for_detection(box, fx, cx, beams, None, 0.1)
check("missing calibration produces no range", denied is None)
body = gb_metric.sighting_record(now, [{"conf": 0.9, "box": box}], denied)
with open(gb_paths.person_track_path(), "w") as f:
    json.dump(body, f)
check("voice reads the denial and will not follow it", gb_wally.read_person_track() is None)
sight = gb_wally.read_sighting()
check("denial names both missing measurements", sight.get("reason") == "depth_unit_unconfirmed" and sight.get("lidar_reason") == "extrinsics_not_validated", sight)
cal = {"validated": True, "x_m": 0.08, "y_m": 0.0, "z_m": 0.05, "yaw_rad": 0.0, "measured_at": "measured-on-robot"}
got = gb_metric.range_for_detection(box, fx, cx, beams, cal, 0.1)
check("validated calibration calls the bearing beam", got and got["depth_m"] == 2.4 and got["depth_m"] != 0.98, got)
check("a stale receive time is not saved by a fresh header", gb_metric.range_for_detection(box, fx, cx, beams, cal, 2.0) is None)
measured = gb_metric.sighting_record(now, [{"conf": 0.9, "box": box}], got)
with open(gb_paths.person_track_path(), "w") as f:
    json.dump(measured, f)
track = gb_wally.read_person_track()
check("voice accepts the produced lidar range", track and track["source"] == "lidar" and abs(track["depth_m"] - 2.4) < 1e-6, track)
check("the track is not an identity", track.get("name_ignored") is False and track.get("name") in (None, ""))
json.dump({"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.98, "user_lidar": True}, open(os.environ["HEALTH_FILE"], "w"))
json.dump({"t": now, "maint": False, "zeroing": False, "why": "idle"}, open(os.environ["WD_STATE_HOST"], "w"))
gb_wally.set_autonomy(False)
missing = gb_wally.autonomy_blockers(gb_health.read_shared(), track, True, now)
check("produced range passes the guard", missing == [], missing)
said = gb_wally.handle("enable autonomy")
check("autonomy on does not start driving", gb_wally._autonomy and "not moved" in said, said)
ready, why = gb_health.scan_source_ready(["/scan_raw"], 0.2)
check("scan_raw alone is ready after reboot", ready and why == "scan_raw", why)
absent, why = gb_health.scan_source_ready(["/scan"], 0.1)
check("historical /scan is not a substitute", absent is False and why == "scan_raw_absent")
check("receive time stays the freshness clock", gb_health.receive_time_fresh(1.4, 0.0) is False and gb_health.receive_time_fresh(0.4, 50.0) is True)
text = open(os.path.join(os.path.dirname(HERE), "gb_safety", "gb_rgbguard.py")).read()
check("the tracker calls the lidar range path", "range_for_detection" in text)
src = open(os.path.join(os.path.dirname(HERE), "gb_safety", "src", "gb_obstacle.py")).read()
canon = open(os.path.join(os.path.dirname(HERE), "gb_obstacle.py")).read()
check("obstacle source copy matches", src == canon)

print("test_measure_path: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
