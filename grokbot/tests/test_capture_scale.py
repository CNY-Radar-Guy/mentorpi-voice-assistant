"""Capture stamps and the logged millimetre record. No robot and no motion."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_follow
import gb_health
import gb_health_node
import gb_metric
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

now = 1_700_000_100.0
cap, why = gb_metric.capture_time(0, now)
check("a zero stamp is refused", cap is None and why == "capture_stamp_zero", why)
cap, why = gb_metric.capture_time(100.0, now)
check("a boot-clock stamp is refused", cap is None and why == "capture_stamp_zero", why)
cap, why = gb_metric.capture_time(now + 1.0, now)
check("a future stamp is refused", cap is None and why == "capture_stamp_future", why)
cap, why = gb_metric.capture_time(now - 120.0, now)
check("a different clock is refused", cap is None and why == "capture_stamp_clock", why)
cap, why = gb_metric.capture_time(now - 0.35, now)
check("a 0.35 s transport delay is still this clock", cap == now - 0.35 and why == "ok", why)
check("receipt time is not a fresh capture", gb_metric.fresh_capture(now - 0.6, now) is False)
check("the 0.5 s limit is unchanged", gb_metric.OBSERVATION_FRESH_S == 0.5 and gb_metric.fresh_capture(now - 0.5, now) is True)
check("a replayed old stamp stays old", gb_metric.fresh_capture(now - 2.0, now + 5) is False)
check("identical captures pair", gb_metric.frames_paired(now, now) is True)
check("a later detection does not pair", gb_metric.frames_paired(now, now + 3.4) is False)
check("the slot keeps only the latest frame", gb_metric.keep_latest("old", "new") == "new")

base = 1_700_000_000.0
raws = (2225, 2230, 2237)
confs = (0.805, 0.808, 0.811)
errors = []
samples = []
for i, (raw, conf) in enumerate(zip(raws, confs)):
    err = (raw * 0.001 - gb_metric.TORSO_REFERENCE_M) / gb_metric.TORSO_REFERENCE_M
    errors.append(err)
    samples.append({"raw_median": raw, "conf": conf, "people": 1, "rgb_capture_t": base + i * 0.24,
                    "depth_capture_t": base + i * 0.24, "error": err})
check("torso errors stay under a few percent and are not zero",
      abs(errors[0] - 0.006878) < 1e-4 and abs(errors[1] - 0.009141) < 1e-4 and abs(errors[2] - 0.012309) < 1e-4
      and all(0 < e < gb_metric.FEW_PERCENT for e in errors), errors)
record = {
    "schema": 1, "scale_m_per_count": 0.001,
    "provenance": {"tape": "7ft_to_feet", "feet_closer_than_torso_m": 0.0762,
                   "reference_m": 2.2098, "landmark": "torso"},
    "camera": {"width": 640, "height": 400, "encoding": "mono16", "align_mode": True,
               "depth_correction": True, "rgb_frame_id": "rgb_camera_link", "depth_frame_id": "depth_camera_link"},
    "samples": samples,
}
found = gb_metric.assess_depth_record(record)
check("the logged torso samples confirm nominal millimetres", found["ok"] and found["scale"] == 0.001 and found["unit"] == "documented_mm", found)
bare = dict(record, provenance={"tape": "7ft_to_feet", "feet_closer_than_torso_m": 0.0, "reference_m": 2.1336, "landmark": "torso"})
check("the uncorrected 7 ft reference does not pass", gb_metric.assess_depth_record(bare)["ok"] is False)
fitted = dict(record, scale_m_per_count=2.2098 / 2230.0, fit_factor=0.99)
check("a fitted multiplier is refused", gb_metric.assess_depth_record(fitted)["reason"] == "fit_factor_not_allowed")
check("absent record does not invent a unit", gb_metric.validated_depth_scale("mono16", path=os.path.join(tempfile.gettempdir(), "missing-scale.json")) is None)

tmp = tempfile.mkdtemp(prefix="gbcap_")
scale_path = os.path.join(tmp, "depth_scale_record.json")
track_path = os.path.join(tmp, "person_track.json")
os.environ["GB_DEPTH_SCALE_FILE"] = scale_path
os.environ["GB_SHARED_DIR"] = tmp
os.environ["PERSON_TRACK_FILE"] = track_path
json.dump(record, open(scale_path, "w"))
check("a present record is the only switch", gb_metric.validated_depth_scale("mono16") == (0.001, "documented_mm"))
live = now
body = gb_metric.track_record(live - 0.1, 2.230, 0.0, 0.808, "documented_mm", 80, "depth")
body["person"]["depth_capture_t"] = live - 0.1
body = gb_metric.stamp_observation(body, live - 0.1, live - 0.05, live, 0.04)
json.dump(body, open(track_path, "w"))
seen = gb_wally.read_person_track(track_path, now=live)
check("voice reads the fresh captured distance", seen and abs(seen["depth_m"] - 2.230) < 1e-6 and seen["conf"] >= 0.75, seen)
check("the age is the capture, not the write", body["person"]["t"] == live - 0.1 and body["write_t"] == live and gb_wally.read_person_track(track_path, now=live + 0.6) is None)
stale = gb_metric.stamp_observation(gb_metric.track_record(live - 2, 2.230, 0, 0.81, "documented_mm", 80, "depth"), live - 2, live - 1.9, live, 0.7)
stale["person"]["depth_capture_t"] = live - 2
json.dump(stale, open(track_path, "w"))
check("a slow inference does not refresh an old capture", gb_wally.read_person_track(track_path, now=live) is None and stale["latency_s"] > 0.5)
json.dump(body, open(track_path, "w"))
body["person"]["time_base"] = "receive"
json.dump(body, open(track_path, "w"))
check("a receipt stamp is not a capture", gb_wally.read_person_track(track_path, now=live) is None)
os.remove(scale_path)
json.dump(body, open(track_path, "w"))
body["person"]["time_base"] = "capture"
json.dump(body, open(track_path, "w"))
check("an injected track without the record is not a range", gb_wally.read_person_track(track_path, now=live) is None)
json.dump(record, open(scale_path, "w"))
gb_wally.set_autonomy(False)
cmd = gb_wally.follow_once({"t": live, "mode": "depth", "depth_online": True, "depth_metric_valid": True, "depth_unit": "documented_mm"}, seen, False)
check("autonomy off sends no follow", gb_wally._autonomy is False and cmd.get("allow") is False and cmd.get("lin") == 0.0, cmd)
health = gb_health_node.record(live, None, "mono16", [2225, 2230, 2237], None, topic_online=True)
check("health uses the record, not a forced flag", health["depth_unit"] == "documented_mm" and health["depth_scale_m"] == 0.001 and health["depth_metric_valid"] is True, health)
os.remove(scale_path)
plain = gb_health_node.record(live, 0.98, "mono16", None, 0.98, topic_online=True, forward_raw=980)
check("without the record the lidar crosscheck is unchanged", plain["depth_unit"] == "crosscheck_mm", plain.get("depth_scale_reason"))
obstacle = open(os.path.join(ROOT, "gb_obstacle.py")).read()
gate = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
check("gate thresholds stay 0.27, 0.28 and 0.24",
      'OBS_STOP_FWD", 0.27)' in obstacle and 'OBS_STOP_REV", 0.28)' in obstacle and 'OBS_SWEEP_R", 0.24)' in obstacle)
check("the cancelled radial rule is not in the gate", "0.508" not in obstacle and "0.508" not in gate)
guard = open(os.path.join(ROOT, "gb_safety", "gb_rgbguard.py")).read()
check("the tracker ages the header and does not read a health-only scale", "capture_time" in guard and "_health_scale" not in guard)
check("follow standoff is not this clearance", gb_follow.STANDOFF_M == 2.0)

print("test_capture_scale: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
