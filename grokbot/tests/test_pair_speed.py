"""Matched depth stamp, recorded millimetre unit, and a real YOLO input size.

A cloud timing number is not a pass on the Pi. The 0.5 s capture limit stays.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import json, os, sys, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_health_node
import gb_metric
import gb_paths

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

stamps = [10.0, 10.2, 10.40]
check("the matching stamp wins over the latest slot", gb_metric.match_depth_stamp(10.2, stamps) == 1)
check("an unmatched stamp is not the latest frame", gb_metric.match_depth_stamp(9.0, stamps) is None)
check("a 0.63 s capture is not fresh", gb_metric.fresh_capture(100.0, 100.63) is False)
check("the isolated 0.503 s total is already over 0.5", gb_metric.fresh_capture(100.0, 100.503) is False)

def dynamic(size):
    return np.zeros((1, 84, gb_metric.yolo_cells(size)), np.float32)

def static640(size):
    return np.zeros((1, 84, gb_metric.yolo_cells(640)), np.float32)

size, why, tried = gb_metric.choose_yolo_size(dynamic)
check("a dynamic net may run at 416", size == 416 and why == "matched_416", (size, why, tried))
size, why, tried = gb_metric.choose_yolo_size(static640)
check("a static 640 output is not relabelled 416", size == 640 and why == "matched_640", (size, why, tried))
check("416 and 320 were rejected for the static net", any("416" in t for t in tried) and any("320" in t for t in tried))

def boom(size):
    raise RuntimeError("bad shape")

size, why, tried = gb_metric.choose_yolo_size(boom)
check("no size is invented when every forward fails", size is None and why == "no_valid_input")

seen = gb_metric.sighting_record(100.0, [{"conf": 0.81, "box": [1, 2, 3, 4]}], {"scale_reason": "frames_not_paired"})
check("the pair failure is the stated reason", seen["person"]["reason"] == "frames_not_paired", seen["person"])

tmp = tempfile.mkdtemp(prefix="gbpair_")
health = os.path.join(tmp, "health.json")
record_path = os.path.join(tmp, "depth_scale_record.json")
os.environ.pop("GB_DEPTH_SCALE_FILE", None)
os.environ["HEALTH_FILE"] = health
os.environ["GB_SHARED_DIR"] = os.path.join(tmp, "empty")
os.makedirs(os.environ["GB_SHARED_DIR"], exist_ok=True)
base = 1_700_000_000.0
samples = []
for i, raw in enumerate((2225, 2230, 2237)):
    err = (raw * 0.001 - 2.2098) / 2.2098
    samples.append({"raw_median": raw, "conf": (0.805, 0.808, 0.811)[i], "people": 1,
                    "rgb_capture_t": base + i * 0.2, "depth_capture_t": base + i * 0.2, "error": err})
json.dump({
    "scale_m_per_count": 0.001,
    "provenance": {"tape": "7ft_to_feet", "feet_closer_than_torso_m": 0.0762, "reference_m": 2.2098, "landmark": "torso"},
    "camera": {"width": 640, "height": 400, "encoding": "mono16", "align_mode": True, "depth_correction": True,
               "rgb_frame_id": "rgb_camera_link", "depth_frame_id": "depth_camera_link"},
    "samples": samples,
}, open(record_path, "w"))
found = gb_paths.depth_scale_record_path()
check("the record beside the health file is the one that is opened", found == record_path, found)
quiet = gb_health_node.record(base, 0.2, "", None, None, topic_online=False)
check("the unit is visible with no live frame and is not called online",
      quiet["depth_scale_record_ok"] is True and quiet["depth_unit"] == "documented_mm"
      and quiet["depth_metric_valid"] is False and quiet["depth_online"] is False, quiet)
live = gb_health_node.record(base, 0.2, "mono16", [2230], None, topic_online=True)
check("a live mono16 sample uses that same unit", live["depth_metric_valid"] is True and live["depth_unit"] == "documented_mm" and live["depth_scale_m"] == 0.001, live)

guard = open(os.path.join(ROOT, "gb_safety", "gb_rgbguard.py")).read()
check("production stays at two OpenCV threads", guard.count("setNumThreads(2)") >= 1 and "setNumThreads(4)" not in guard)
check("depth is queued by stamp", "match_depth_stamp" in guard and "depth_buf" in guard)
obstacle = open(os.path.join(ROOT, "gb_obstacle.py")).read()
gate = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
check("old gate distances are unchanged", 'OBS_STOP_FWD", 0.27)' in obstacle and 'OBS_STOP_REV", 0.28)' in obstacle and 'OBS_SWEEP_R", 0.24)' in obstacle)
check("the cancelled radial distance is absent", "0.508" not in obstacle and "0.508" not in gate and "0.508" not in guard)

print("test_pair_speed: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
