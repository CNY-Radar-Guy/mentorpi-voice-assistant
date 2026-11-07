"""Static 416 YOLOv8n on the three saved pair-2101 frames. OpenCV DNN, raw scores.

Cloud timing is not a Pi measurement. Confidence is not rescaled.
"""
import os, sys
import cv2
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_metric
import gb_yolo

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

MODEL = os.path.join(ROOT, "gb_safety", "yolov8n.onnx")
FRAMES = [os.path.join(HERE, "fixtures", "pair2101", "rgb_%d.png" % i) for i in range(3)]
NAMES = ("person",)
net = cv2.dnn.readNetFromONNX(MODEL)
cv2.setNumThreads(2)

def run(size):
    blob = np.zeros((1, 3, int(size), int(size)), np.float32)
    net.setInput(blob)
    return net.forward()

size, why, tried = gb_metric.choose_yolo_size(run)
check("this file is a real 416 graph", size == 416 and why == "matched_416", (size, why, tried))
try:
    run(320)
    rejected = False
except Exception:
    rejected = True
check("320 is rejected instead of being treated as 416", rejected)

for path in FRAMES:
    img = cv2.imread(path)
    check(os.path.basename(path) + " is a 640x400 BGR frame", img is not None and img.shape == (400, 640, 3), None if img is None else img.shape)
    dets = [d for d in gb_yolo.detect(net, img, 416, NAMES) if d[0] == "person" and d[1] >= 0.75]
    check(os.path.basename(path) + " has one person at or above 0.75", len(dets) == 1, [(round(d[1], 3), [round(v, 1) for v in d[2:]]) for d in dets])
    if not dets:
        continue
    conf, x1, y1, x2, y2 = dets[0][1], dets[0][2], dets[0][3], dets[0][4], dets[0][5]
    check(os.path.basename(path) + " box is the standing person",
          180 < x1 < 340 and 330 < x2 < 480 and y1 < 40 and 220 < y2 < 380 and (y2 - y1) > 180 and (x2 - x1) > 60,
          (round(conf, 3), round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)))

blank = np.zeros((400, 640, 3), np.uint8)
none = [d for d in gb_yolo.detect(net, blank, 416, NAMES) if d[0] == "person" and d[1] >= 0.75]
check("a blank frame is not a person", none == [])
body = __import__("gb_metric").sighting_record(1.0, [])
check("an empty detection record stays no_person", body["reason"] == "no_person" and body["person"] is None)

print("test_yolo416: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
