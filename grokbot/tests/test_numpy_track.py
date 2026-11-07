"""A real nonempty YOLO row is numpy float32. That must reach the voice file.

An empty detection cannot catch this: json.dump accepts plain floats and
rejects numpy.float32. No camera, no drive, no invented range.
"""
import json, os, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gb_health
import gb_metric
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

raw_box = [np.float32(12.25), np.float32(30.5), np.float32(180.0), np.float32(420.75)]
try:
    json.dumps({"conf": np.float32(0.91), "box": raw_box})
    bare = True
except TypeError:
    bare = False
check("numpy float32 still cannot be dumped on its own", bare is False)

dets = [("person", np.float32(0.91), raw_box[0], raw_box[1], raw_box[2], raw_box[3])]
people = gb_metric.people_from_dets(dets)
check("producer keeps the nonempty person", len(people) == 1 and abs(people[0]["conf"] - 0.91) < 1e-6)
check("producer numbers are plain floats", all(type(v) is float for v in [people[0]["conf"]] + people[0]["box"]))
body = gb_metric.sighting_record(time.time(), people, None)
text = json.dumps(body)
parsed = json.loads(text)
check("writer accepts the produced record", parsed["detect_ok"] is True and parsed["person"]["reason"] == "depth_unit_unconfirmed")
check("no range was invented", "depth_m" not in parsed["person"] and parsed["person"]["lidar_reason"] == "extrinsics_not_validated")
check("the saved box is the detection", parsed["person"]["box"] == [12.25, 30.5, 180.0, 420.75])
check("an empty frame is still no person", gb_metric.sighting_record(time.time(), [])["reason"] == "no_person")

tmp = tempfile.mkdtemp(prefix="gbnp_")
os.environ["GB_SHARED_DIR"] = tmp
os.environ.pop("PERSON_TRACK_FILE", None)
os.environ["HEALTH_FILE"] = os.path.join(tmp, "health.json")
os.environ["WD_STATE_HOST"] = os.path.join(tmp, "watchdog_state.json")
now = time.time()
gb_metric.dump_json(gb_paths.person_track_path(), body)
sight = gb_wally.read_sighting(now=now)
check("voice reads the saved person", sight and abs(sight["conf"] - 0.91) < 1e-6 and sight.get("name") in (None, ""))
check("voice will not follow without a range", gb_wally.read_person_track(now=now) is None)
gb_wally.set_autonomy(False)
said = gb_wally.handle("enable autonomy")
check("autonomy stays off and nothing moves", (not gb_wally._autonomy) and "not moving" in said, said)

pub = gb_health.SCAN_PUBLISHER_QOS
check("health qos can hear the reliable volatile scan", gb_health.qos_compatible(pub, gb_health.HEALTH_SCAN_QOS))
check("a matching one-off qos can hear it", gb_health.qos_compatible(pub, gb_health.ONEOFF_SCAN_QOS))
latched = {"reliability": "reliable", "durability": "transient_local"}
check("a latched one-off is incompatible", gb_health.qos_compatible(pub, latched) is False)
check("that silence is not a dead lidar", gb_health.sampler_silence(pub, latched, None) == "qos_incompatible")
check("a fresh receive survives a foreign header clock", gb_health.receive_time_fresh(0.4, 1.0e9) is True)
check("a stale receive is still stale", gb_health.receive_time_fresh(1.4, 0.0) is False)
guard = open(os.path.join(os.path.dirname(HERE), "gb_safety", "gb_rgbguard.py")).read()
check("the tracker uses the same producer and writer", "people_from_dets" in guard and "dump_json" in guard)

print("test_numpy_track: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
