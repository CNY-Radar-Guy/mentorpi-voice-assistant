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
DEMAND = E("GB_DEMAND_FILE", SH + "/demand.json")
TRACK = E("PERSON_TRACK_FILE", SH + "/person_track.json")
POINT = E("GB_POINT_FILE", SH + "/point_target.json")
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
            if best is None or area > best["area"]:
                best = {"label": lab.replace("_", " "), "conf": round(cf, 2), "area": round(area, 3),
                        "box": [round(float(x1), 1), round(float(y1), 1), round(float(x2), 1), round(float(y2), 1)]}
    return best

class RgbGuard(Node):
    def __init__(self):
        super().__init__("gb_rgbguard")
        cv2.setNumThreads(2)
        self.net = cv2.dnn.readNetFromONNX(MODEL)
        self.moving = False; self.idle_since = time.time(); self.sub = None; self.depth_sub = None; self.info_sub = None
        self.img = None; self.img_t = 0.0; self.depth = None; self.depth_t = 0.0; self.k = None; self.scan = None; self.done_t = 0.0
        self._last_infer = 0.7; self._held_motion = False
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

    def demand(self):
        try:
            import gb_demand
            return gb_demand.read_demand(DEMAND)
        except Exception:
            return None

    def _drop_sub(self, name):
        sub = getattr(self, name, None)
        if sub is not None:
            self.destroy_subscription(sub)
            setattr(self, name, None)

    def manage(self):
        mode = self.demand()
        if self.moving:
            self._held_motion = True
        import gb_demand
        want = gb_demand.tracker_wants(mode, self.moving)
        want_depth = self.moving or mode in ("follow", "point", "acquire")
        if want and self.sub is None:
            self.sub = self.create_subscription(Image, TOPIC, self.on_img, qos_profile_sensor_data)
            from sensor_msgs.msg import LaserScan
            self.scan_sub = self.create_subscription(LaserScan, E("GATE_SCAN", "/scan_raw"), self.on_scan, qos_profile_sensor_data)
        if want_depth and self.depth_sub is None and self.sub is not None:
            self.depth_sub = self.create_subscription(Image, E("GB_DEPTH_TOPIC", "/ascamera/camera_publisher/depth0/image_raw"), self.on_depth, qos_profile_sensor_data)
            from sensor_msgs.msg import CameraInfo
            self.info_sub = self.create_subscription(CameraInfo, E("GB_DEPTH_INFO", "/ascamera/camera_publisher/depth0/camera_info"), self.on_info, qos_profile_sensor_data)
        if self.sub is not None and not want_depth and not self.moving:
            self._drop_sub("depth_sub"); self._drop_sub("info_sub"); self.depth = None
        if not want and self.sub is not None:
            hold = 3.0 if self._held_motion else 0.0
            if self.idle_since and time.time() - self.idle_since >= hold:
                self._drop_sub("sub"); self._drop_sub("depth_sub"); self._drop_sub("info_sub"); self._drop_sub("scan_sub")
                self.img = None; self.depth = None; self.scan = None; self._held_motion = False

    def on_scan(self, m):
        beams = []
        for i, r in enumerate(m.ranges):
            if r == r and 0.05 < float(r) < 30.0:
                beams.append((float(m.angle_min) + i * float(m.angle_increment), float(r)))
        self.scan = (time.time(), beams)

    def on_img(self, m):
        self.img = m; self.img_t = time.time()                # keep only the newest frame; inference runs elsewhere

    def on_depth(self, m):
        self.depth = m; self.depth_t = time.time()

    def on_info(self, m):
        self.k = (float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]))

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
                x1, y1, bw, bh = [float(v) for v in xy[i]]
                res.append((COCO[int(ci[i])], float(cf[i]), x1, y1, x1 + bw, y1 + bh))
        return res

    def worker(self):
        while rclpy.ok():
            m = self.img
            mode = self.demand()
            import gb_demand
            period = gb_demand.tracker_period(mode, self.moving, self._last_infer)
            if (not gb_demand.tracker_wants(mode, self.moving)) or m is None or self.img_t <= self.done_t or time.time() - self.done_t < period:
                time.sleep(0.05); continue
            self.done_t = time.time()
            try:
                img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == "rgb8": img = img[:, :, ::-1]
                frame_t = self.img_t
                t0 = time.time(); dets = self.detect(img); ms = (time.time() - t0) * 1000
                self._last_infer = max(0.05, ms / 1000.0)
                hz = hazard_from(dets, m.width, m.height)
                people = [dt for dt in dets if dt[0] == "person"]
                d = {"t": frame_t, "hazard": bool(hz), "ms": round(ms), "infer_s": round(self._last_infer, 3),
                     "n": len(dets), "detect_ok": True, "wh": [m.width, m.height], "person": None}
                if hz: d.update(hz)
                if people:
                    lab, cf, x1, y1, x2, y2 = max(people, key=lambda r: r[1])
                    d["person"] = {"t": frame_t, "conf": round(float(cf), 2),
                                   "box": [round(float(x1), 1), round(float(y1), 1), round(float(x2), 1), round(float(y2), 1)],
                                   "wh": [m.width, m.height]}
                try:
                    self._write_metric(people, m, frame_t)
                except Exception as ex:
                    self.get_logger().warn("person track write failed: %s" % ex)
                import gb_metric
                gb_metric.dump_json(OUT, d)
            except Exception as ex:
                self.get_logger().warn("rgb guard frame failed: %s" % ex); time.sleep(1.0)

    def _lidar_obs(self, people):
        """Calibrated beam at the box bearing, or None. Does not use the forward minimum."""
        if len(people) != 1 or self.k is None or not self.scan:
            return None
        age = time.time() - self.scan[0]
        try:
            import gb_metric
            import gb_paths
            cal = None
            try:
                with open(gb_paths.extrinsics_path()) as f:
                    cal = json.load(f)
            except (OSError, ValueError):
                cal = None
            box = [people[0][2], people[0][3], people[0][4], people[0][5]]
            return gb_metric.range_for_detection(box, self.k[0], self.k[2], self.scan[1], cal, age)
        except Exception:
            return None

    def _write_metric(self, people, image, frame_t):
        """Demand-driven range. Same YOLO pass. No second model and no invented distance."""
        try:
            import gb_metric, gb_hands
        except Exception:
            return
        mode = self.demand()
        if mode not in ("follow", "point", "acquire"):
            return
        if mode == "acquire" and (self.depth is None or self.k is None or time.time() - self.depth_t > 0.5):
            _atomic(TRACK, gb_metric.sighting_record(frame_t, _people_dicts(people), self._lidar_obs(people)))
            return
        if self.depth is None or self.k is None or time.time() - self.depth_t > 0.5:
            if mode in ("follow", "acquire"):
                _atomic(TRACK, gb_metric.sighting_record(frame_t, _people_dicts(people), self._lidar_obs(people)))
            return
        import numpy as np
        depth = self.depth
        if depth.encoding == "32FC1":
            raw = np.frombuffer(bytes(depth.data), dtype="<f4").reshape(depth.height, depth.width)
            scale, unit = 1.0, "32FC1_metres"
        elif depth.encoding in ("16UC1", "mono16"):
            import gb_obstacle
            raw = gb_obstacle.decode_depth(depth.encoding, bytes(depth.data), depth.height, depth.width, depth.step, getattr(depth, "is_bigendian", 0))
            if raw is None:
                _atomic(TRACK, gb_metric.sighting_record(frame_t, _people_dicts(people), self._lidar_obs(people)))
                return
            scale, unit = _health_scale()
            if unit not in ("crosscheck_mm", "crosscheck_m"):
                _atomic(TRACK, gb_metric.sighting_record(frame_t, _people_dicts(people), self._lidar_obs(people)))
                return
        else:
            _atomic(TRACK, gb_metric.sighting_record(frame_t, _people_dicts(people), self._lidar_obs(people)))
            return
        fx, fy, cx, cy = self.k
        if mode == "follow":
            boxes = gb_metric.people_from_dets(people)
            chosen, ambiguous = gb_metric.choose_person(boxes)
            if ambiguous or chosen is None:
                body = {"t": frame_t, "person": {"t": frame_t, "ambiguous": True}}
            else:
                got = gb_metric.box_range(raw, chosen["box"], scale, fx, fy, cx, cy)
                if not got:
                    return
                body = gb_metric.track_record(frame_t, got["depth_m"], got["bearing"], chosen["conf"], unit, got["valid_pixels"], "depth")
            _atomic(TRACK, body)
        elif mode == "point":
            pts, why = gb_hands.landmarks(np.ascontiguousarray(np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.width, 3)))
            if not pts:
                _atomic(POINT, {"t": frame_t, "available": False, "reason": why, "candidates": []})
                return
            pix = gb_metric.pointing_pixel(pts, image.width, image.height)
            if not pix:
                _atomic(POINT, {"t": frame_t, "available": True, "reason": "no_ray", "candidates": []})
                return
            got = gb_metric.range_at_pixel(raw, pix[0], pix[1], scale, fx, fy, cx, cy)
            if not got:
                _atomic(POINT, {"t": frame_t, "available": True, "reason": "no_depth", "candidates": []})
                return
            _atomic(POINT, {"t": frame_t, "available": True, "candidates": [{
                "name": "pointed", "conf": 0.8, "depth_m": got["depth_m"], "depth_valid": True,
                "depth_t": frame_t, "source": "depth", "unit": unit, "valid_pixels": got["valid_pixels"]}]})


def _people_dicts(people):
    import gb_metric
    return gb_metric.people_from_dets(people)


def _health_scale():
    try:
        body = json.load(open("/home/ubuntu/shared/gb_safety/health.json"))
        if time.time() - float(body["t"]) > 3.0:
            return None, None
        return float(body.get("depth_scale_m") or 0), body.get("depth_unit")
    except (OSError, ValueError, TypeError):
        return None, None


def _atomic(path, body):
    import gb_metric
    gb_metric.dump_json(path, body)

def main():
    rclpy.init(); n = RgbGuard()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, Exception): pass
    finally:
        try: n.destroy_node(); rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
