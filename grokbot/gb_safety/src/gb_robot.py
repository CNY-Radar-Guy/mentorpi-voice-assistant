"""gb_robot.py - robot I/O for Grok Bot via rosbridge (ws://127.0.0.1:9090) and the STM32 board serial port.
Never opens the serial port for long: battery samples are ~1.5 s reads (the ROS battery topic is preferred when it publishes)."""
import json, math, os, struct, sys, time, urllib.request
import numpy as np

E = os.environ.get
ROSBRIDGE = E("ROSBRIDGE_URL", "ws://127.0.0.1:9090")
CMD_VEL = E("CMD_VEL_TOPIC", "/controller/cmd_vel")
ODOM = E("ODOM_TOPIC", "/odom")
BATTERY_TOPIC = E("BATTERY_TOPIC", "/ros_robot_controller/battery")
CAMERA_TOPIC = E("CAMERA_TOPIC", "/ascamera/camera_publisher/rgb0/image")
SNAPSHOT_URL = E("SNAPSHOT_URL", "http://127.0.0.1:8080/snapshot?topic=")
RRC_PORT = E("RRC_PORT", "/dev/ttyACM0")
SDK_DIR = os.path.expanduser(E("HIWONDER_SDK_DIR", "~/hiwonder-toolbox"))
DATA = os.path.expanduser(E("GB_DATA", "~/grokbot/data"))
BATTERY_JSON = os.path.join(DATA, "battery.json")

# ------------------------------------------------------------------ rosbridge helpers (short-lived connections)
def _ws(timeout=5):
    import websocket
    return websocket.create_connection(ROSBRIDGE, timeout=timeout)

def subscribe_once(topic, typ, timeout=4.0, n=1, throttle_ms=0):
    out = []
    try:
        ws = _ws(timeout)
    except Exception:
        return out
    try:
        ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": typ, "throttle_rate": throttle_ms}))
        t0 = time.time(); ws.settimeout(0.5)
        while time.time() - t0 < timeout and len(out) < n:
            try:
                m = json.loads(ws.recv())
            except Exception:
                continue
            if m.get("op") == "publish" and m.get("topic") == topic:
                out.append(m["msg"])
    finally:
        try: ws.close()
        except Exception: pass
    return out

def publish(topic, typ, msg, repeat=1, hz=10):
    ws = _ws(3)
    try:
        ws.send(json.dumps({"op": "advertise", "topic": topic, "type": typ}))
        for _ in range(repeat):
            ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg})); time.sleep(1.0 / hz)
    finally:
        ws.close()

def list_topics(timeout=20):
    ws = _ws(timeout)
    try:
        ws.send(json.dumps({"op": "call_service", "service": "/rosapi/topics", "id": "t"}))
        m = json.loads(ws.recv()); return dict(zip(m["values"]["topics"], m["values"]["types"]))
    finally:
        ws.close()

# ------------------------------------------------------------------ motion
TWIST0 = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
def stop():
    publish(CMD_VEL, "geometry_msgs/Twist", TWIST0, repeat=10, hz=20)
    time.sleep(0.2)
    publish(CMD_VEL, "geometry_msgs/Twist", TWIST0, repeat=5, hz=20)

def move(lin=0.0, ang=0.0, seconds=1.0):
    seconds = max(0.1, min(seconds, 5.0)); lin = max(-0.2, min(lin, 0.2)); ang = max(-1.0, min(ang, 1.0))
    import gb_estop                                   # never publish non-zero cmd_vel directly: the gate owns the clock
    rid = None
    try:
        rid = gb_estop.gate_request({"op": "timed", "lin": lin, "ang": ang, "dur": seconds, "src": "gb_robot.move"})
        time.sleep(seconds)                           # sequencing only; the gate stops at its own deadline
    finally:
        try: gb_estop.gate_request({"op": "stop", "only_id": rid, "src": "gb_robot.move"}) if rid else None
        finally: stop()

# ------------------------------------------------------------------ pose
def _yaw(q):
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))

def get_pose(timeout=3.0):
    """Robot pose. Uses map frame if a map->odom transform is on /tf (SLAM/AMCL running), else odom frame."""
    od = subscribe_once(ODOM, "nav_msgs/Odometry", timeout=timeout)
    if not od:
        return None
    p = od[0]["pose"]["pose"]; x, y, th = p["position"]["x"], p["position"]["y"], _yaw(p["orientation"])
    frame = od[0]["header"].get("frame_id") or "odom"
    for m in subscribe_once("/tf", "tf2_msgs/TFMessage", timeout=1.5, n=30):
        for t in m.get("transforms", []):
            if t["header"]["frame_id"].strip("/") == "map" and t["child_frame_id"].strip("/") == frame.strip("/"):
                tr = t["transform"]; a = _yaw(tr["rotation"]); tx, ty = tr["translation"]["x"], tr["translation"]["y"]
                x, y = tx + math.cos(a) * x - math.sin(a) * y, ty + math.sin(a) * x + math.cos(a) * y
                th = (th + a + math.pi) % (2 * math.pi) - math.pi; frame = "map"
                return {"x": x, "y": y, "theta": th, "frame": frame}
    return {"x": x, "y": y, "theta": th, "frame": frame}

# ------------------------------------------------------------------ camera
def snapshot(timeout=5):
    with urllib.request.urlopen(SNAPSHOT_URL + CAMERA_TOPIC, timeout=timeout) as r:
        data = r.read()
    if not data.startswith(b"\xff\xd8"):
        raise RuntimeError("camera snapshot is not a JPEG (is web_video_server / the camera running?)")
    return data

# ------------------------------------------------------------------ map saving (from /map via rosbridge; no DDS from the host needed)
def save_map(name, maps_dir, timeout=8):
    msgs = subscribe_once("/map", "nav_msgs/OccupancyGrid", timeout=timeout)
    if not msgs:
        raise RuntimeError("no /map topic is publishing, so SLAM or navigation isn't running. Start Hiwonder's mapping "
                           "mode first, drive around, then say 'save the map' again")
    m = msgs[0]; info = m["info"]; w, h, res = info["width"], info["height"], info["resolution"]
    o = info["origin"]["position"]; yaw = _yaw(info["origin"]["orientation"])
    d = np.array(m["data"], dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 205, np.uint8); img[d == 0] = 254; img[d >= 65] = 0
    mid = (d > 0) & (d < 65); img[mid] = (254 - d[mid] * 254 // 100).astype(np.uint8)
    img = np.flipud(img)
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name) or "map"
    stem = os.path.join(maps_dir, f"{time.strftime('%Y%m%d_%H%M%S')}_{safe}")
    from gb_store import atomic_write
    atomic_write(stem + ".pgm", f"P5\n{w} {h}\n255\n".encode() + img.tobytes())
    atomic_write(stem + ".yaml", f"image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {res}\n"
                 f"origin: [{o['x']}, {o['y']}, {yaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return stem, res, w, h, [o["x"], o["y"], yaw]

# ------------------------------------------------------------------ battery (2S Li-ion 7.4 V 2200 mAh: 8.4 V full, ~6.6 V empty)
CELL_CURVE = [(3.30, 0), (3.45, 3), (3.55, 8), (3.62, 15), (3.68, 25), (3.73, 35), (3.77, 45), (3.81, 55),
              (3.86, 65), (3.92, 75), (3.99, 85), (4.08, 93), (4.20, 100)]
CELLS = int(E("BATTERY_CELLS", "2"))

def battery_pct(volts):
    c = volts / CELLS
    if c <= CELL_CURVE[0][0]: return 0
    if c >= CELL_CURVE[-1][0]: return 100
    for (v0, p0), (v1, p1) in zip(CELL_CURVE, CELL_CURVE[1:]):
        if v0 <= c <= v1:
            return int(round(p0 + (p1 - p0) * (c - v0) / (v1 - v0)))

def _crc8_fn():
    sys.path.insert(0, SDK_DIR)
    from ros_robot_controller_sdk import checksum_crc8
    return checksum_crc8

def battery_serial_mV(window=1.6):
    """Briefly read the STM32 board stream and return the first battery report (mV) or None.
    Frame: AA 55 func len data crc8 ; battery = func 0 (SYS), data[0]==0x04, '<H' millivolts."""
    import serial
    crc8 = _crc8_fn()
    s = serial.Serial(None, 1000000, timeout=0.05); s.rts = False; s.dtr = False; s.port = RRC_PORT; s.open()
    try:
        buf = b""; t0 = time.time()
        while time.time() - t0 < window:
            buf += s.read(512)
            while True:
                i = buf.find(b"\xaa\x55")
                if i < 0 or len(buf) < i + 4:
                    buf = buf[-3:]; break
                func, ln = buf[i + 2], buf[i + 3]
                if len(buf) < i + 5 + ln:
                    buf = buf[i:]; break
                frame = buf[i + 2:i + 4 + ln]; ck = buf[i + 4 + ln]; buf = buf[i + 5 + ln:]
                if crc8(bytes(frame)) == ck and func == 0 and ln >= 3 and frame[2] == 0x04:
                    return struct.unpack("<H", frame[3:5])[0]
    finally:
        s.close()
    return None

def battery_topic_mV(timeout=2.5):
    m = subscribe_once(BATTERY_TOPIC, "std_msgs/UInt16", timeout=timeout)
    return m[0]["data"] if m else None

def read_battery():
    """Prefer the fresh smoothed value from the low-voltage monitor; else ROS topic; else a short serial read."""
    try:
        j = json.load(open(BATTERY_JSON))
        if time.time() - j["ts"] < 30:
            return j
    except Exception:
        pass
    mv, src = battery_topic_mV(), "ros_topic"
    if mv is None:
        mv, src = battery_serial_mV(), "serial"
    if mv is None:
        return None
    v = mv / 1000.0
    return {"ts": time.time(), "v_raw": v, "v_avg": v, "pct": battery_pct(v), "source": src, "state": "ok"}
