#!/usr/bin/env python3
"""gb_rgbguard.py - RGB advisory for gb_gate (runs in the MentorPi container at nice 12), 2026-11-05.

YOLOv8n (OpenCV DNN, 2 threads) on the EXISTING rgb0 topic (no second camera pipeline), and ONLY while the gate is
moving: the image subscription is created when /gb/gate_status says mode != idle and destroyed 3 s after it goes idle,
so an idle robot pays nothing. At most RGB_HZ (1 Hz) inferences on the latest frame (older frames are dropped).
Writes rgb_guard.json {t, hazard, label, conf, area, ms}. hazard = person / furniture / pet / object whose box is in
the central half of the image, reaches the lower 45 %, and covers >= 4 % of it. Advisory only: the gate caps the speed
at 0.05 m/s while a fresh hazard is reported; it never raises speed and never blocks on its own (LiDAR/depth do that)."""
import json, os, threading, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

E = os.environ.get
SH = "/home/ubuntu/shared/gb_safety"
OUT = E("GB_RGB_FILE", SH + "/rgb_guard.json")
MODEL = E("GB_RGB_MODEL", SH + "/yolov8n.onnx")
TOPIC = E("GB_RGB_TOPIC", "/ascamera/camera_publisher/rgb0/image")
HZ = float(E("GB_RGB_HZ", "1.0")); CONF = 0.45; SIZE = int(E("GB_RGB_SIZE", "640"))
COCO = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign parking_meter "
        "bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee skis "
        "snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork "
        "knife spoon bowl banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed "
        "dining_table toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink refrigerator book clock "
        "vase scissors teddy_bear hair_drier toothbrush").split()
HAZ = {"person", "chair", "couch", "bed", "dining_table", "potted_plant", "dog", "cat", "suitcase", "backpack", "bench",
       "bicycle", "toilet", "tv", "refrigerator", "oven", "sink", "bottle", "vase", "handbag", "umbrella", "sports_ball",
       "teddy_bear", "laptop", "book", "cup", "bowl", "skateboard"}

def hazard_from(dets, w, h):
    """dets [(label, conf, x1, y1, x2, y2)] -> best hazard dict or None."""
    best = None
    for lab, cf, x1, y1, x2, y2 in dets:
        if lab not in HAZ or cf < CONF: continue
        cx = (x1 + x2) / 2.0; area = (x2 - x1) * (y2 - y1) / float(w * h)
        if 0.25 * w <= cx <= 0.75 * w and y2 >= 0.55 * h and area >= 0.04:
            if best is None or area > best["area"]: best = {"label": lab.replace("_", " "), "conf": round(cf, 2), "area": round(area, 3)}
    return best

class RgbGuard(Node):
    def __init__(self):
        super().__init__("gb_rgbguard")
        cv2.setNumThreads(2)
        self.net = cv2.dnn.readNetFromONNX(MODEL)
        self.moving = False; self.idle_since = time.time(); self.sub = None; self.img = None; self.img_t = 0.0; self.done_t = 0.0
        self.create_subscription(String, "/gb/gate_status", self.on_status, 10)
        self.create_timer(0.5, self.manage)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info("rgb guard up: %s on %s, <= %.1f Hz, only while moving, nice'd" % (os.path.basename(MODEL), TOPIC, HZ))

    def on_status(self, m):
        try: mode = json.loads(m.data).get("mode", "idle")
        except Exception: return
        mv = mode != "idle"
        if mv: self.idle_since = None
        elif self.moving or self.idle_since is None: self.idle_since = time.time()
        self.moving = mv

    def manage(self):
        if self.moving and self.sub is None:
            self.sub = self.create_subscription(Image, TOPIC, self.on_img, qos_profile_sensor_data)
        elif not self.moving and self.sub is not None and self.idle_since and time.time() - self.idle_since > 3.0:
            self.destroy_subscription(self.sub); self.sub = None; self.img = None

    def on_img(self, m):
        self.img = m; self.img_t = time.time()                # keep only the newest frame; inference runs elsewhere

    def detect(self, img):
        h, w = img.shape[:2]
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (SIZE, SIZE), swapRB=True, crop=False)
        self.net.setInput(blob); out = self.net.forward()[0].T          # 8400 x 84
        sc = out[:, 4:]; ci = sc.argmax(1); cf = sc[np.arange(len(sc)), ci]; k = cf > CONF
        b = out[k, :4]; ci = ci[k]; cf = cf[k]; res = []
        if len(b):
            sx, sy = w / float(SIZE), h / float(SIZE)
            xy = np.stack([(b[:, 0] - b[:, 2] / 2) * sx, (b[:, 1] - b[:, 3] / 2) * sy, b[:, 2] * sx, b[:, 3] * sy], 1)
            for i in np.array(cv2.dnn.NMSBoxes(xy.tolist(), cf.tolist(), CONF, 0.45)).flatten():
                x1, y1, bw, bh = xy[i]; res.append((COCO[ci[i]], float(cf[i]), x1, y1, x1 + bw, y1 + bh))
        return res

    def worker(self):
        while rclpy.ok():
            m = self.img
            if not self.moving or m is None or self.img_t <= self.done_t or time.time() - self.done_t < 1.0 / HZ:
                time.sleep(0.05); continue
            self.done_t = time.time()
            try:
                img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == "rgb8": img = img[:, :, ::-1]
                t0 = time.time(); dets = self.detect(img); ms = (time.time() - t0) * 1000
                hz = hazard_from(dets, m.width, m.height)
                d = {"t": time.time(), "hazard": bool(hz), "ms": round(ms), "n": len(dets)}
                if hz: d.update(hz)
                with open(OUT + ".tmp", "w") as f: json.dump(d, f)
                os.replace(OUT + ".tmp", OUT)
            except Exception as ex:
                self.get_logger().warn("rgb guard frame failed: %s" % ex); time.sleep(1.0)

def main():
    rclpy.init(); n = RgbGuard()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, Exception): pass
    finally:
        try: n.destroy_node(); rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
