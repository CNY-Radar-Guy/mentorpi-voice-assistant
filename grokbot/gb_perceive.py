"""gb_perceive.py - local object memory, faces, pointing, people follow, joystick, model updates.
Everything here is LOCAL: no xAI calls, no images/embeddings/maps ever leave the Pi.
Network is used only by update_models() (explicit voice command) to fetch public model files."""
import base64, json, math, os, re, shutil, sqlite3, subprocess, threading, time, urllib.request
import numpy as np
import cv2
import gb_robot, gb_caps

E = os.environ.get
HOME = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HOME, "data"); MODELS = os.path.join(HOME, "models")
OBJ_DIR = os.path.join(DATA, "objects"); FACE_DIR = os.path.join(DATA, "faces"); JOY_DIR = os.path.join(DATA, "joystick")
OBJ_DB = os.path.join(DATA, "objects.db"); FACE_DB = os.path.join(FACE_DIR, "faces.db")
DEPTH_TOPIC = E("DEPTH_TOPIC", "/ascamera/camera_publisher/depth0/image_raw")
FX, FY, CX, CY = 417.9, 418.9, 315.9, 192.1                # rgb0 camera_info (640x400)
CAM_X = float(E("CAM_X_OFFSET", "0.10"))                    # camera ahead of base_footprint (m), rough
OWNER = E("OWNER_NAME", "John")
YOLO_CONF = float(E("YOLO_CONF", "0.35")); FACE_COS = float(E("FACE_COS", "0.40"))
FOLLOW_M = 1.2; COME_M = 1.0; STANDOFF_M = 0.8; LOST_S = 5.0
HOST_SHARED = os.path.expanduser(E("ROS_SHARED_HOST", "/home/pi/docker/tmp"))
CONT_SHARED = E("ROS_SHARED_CONTAINER", "/home/ubuntu/shared")
PERC_HOST = os.path.join(HOST_SHARED, "grokbot", "perc"); PERC_CONT = CONT_SHARED + "/grokbot/perc"
TARGET_FILE = os.path.join(HOST_SHARED, "grokbot", "explore", "target.json")
COCO = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign parking_meter bench "
        "bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee skis snowboard "
        "sports_ball kite baseball_bat baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon "
        "bowl banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed dining_table "
        "toilet tv laptop mouse remote keyboard cell_phone microwave oven toaster sink refrigerator book clock vase scissors "
        "teddy_bear hair_drier toothbrush").replace("_", "~").split()
COCO = [c.replace("~", " ") for c in COCO]
MODEL_URLS = {
    "yolov8n.onnx": "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8n.onnx",
    "face_yunet.onnx": "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx",
    "face_sface.onnx": "https://huggingface.co/opencv/face_recognition_sface/resolve/main/face_recognition_sface_2021dec.onnx",
}
_M = {}; _LOCK = threading.Lock(); LAST = {"label": None, "cls": None, "t": 0}
TIMES = {}

def log(*a): print("[perceive]", *a, flush=True)

class Problem(Exception): pass

# ----------------------------------------------------------------------------------------------- storage
def db():
    os.makedirs(OBJ_DIR, exist_ok=True)
    c = sqlite3.connect(OBJ_DB, timeout=5); c.row_factory = sqlite3.Row
    c.executescript("""
    CREATE TABLE IF NOT EXISTS objects(id INTEGER PRIMARY KEY, label TEXT, yolo_class TEXT, count INTEGER DEFAULT 1,
      first_ts REAL, last_ts REAL, crop TEXT, dist_m REAL, hist TEXT, source TEXT DEFAULT 'taught');
    CREATE TABLE IF NOT EXISTS sightings(id INTEGER PRIMARY KEY, label TEXT, map TEXT, frame TEXT, x REAL, y REAL,
      size_m REAL, dist_m REAL, n INTEGER DEFAULT 1, first_ts REAL, last_ts REAL);
    CREATE TABLE IF NOT EXISTS hits(ts REAL, kind TEXT);
    CREATE TABLE IF NOT EXISTS people_track(ts REAL, name TEXT, frame TEXT, x REAL, y REAL, dist_m REAL);""")
    return c

def fdb():
    os.makedirs(FACE_DIR, exist_ok=True)
    c = sqlite3.connect(FACE_DB, timeout=5); c.row_factory = sqlite3.Row
    c.execute("CREATE TABLE IF NOT EXISTS faces(id INTEGER PRIMARY KEY, name TEXT, emb BLOB, hair TEXT, ts REAL)")
    return c

def hit(kind):
    with db() as c: c.execute("INSERT INTO hits VALUES(?,?)", (time.time(), kind))


# ----------------------------------------------------------------------------------------------- optional LAN offload
import ipaddress, urllib.parse
OFF = {"url": E("VISION_SERVER_URL", "").strip().rstrip("/"), "ok": False, "next": 0.0}

def _lan_only(url):
    try: return ipaddress.ip_address(urllib.parse.urlparse(url).hostname).is_private
    except Exception: return False

def _offload(kind, img):
    """POST a JPEG to the LAN vision server; None = use onboard models. 300 ms timeout, 30 s circuit breaker."""
    if not OFF["url"] or not _lan_only(OFF["url"]): return None
    now = time.time()
    if not OFF["ok"]:
        if now < OFF["next"]: return None
        OFF["next"] = now + 30
        try:
            t = time.time(); urllib.request.urlopen(OFF["url"] + "/health", timeout=0.3).read()
            OFF["ok"] = time.time() - t < 0.15
        except Exception: OFF["ok"] = False
        if not OFF["ok"]: return None
    try:
        buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        req = urllib.request.Request(f"{OFF['url']}/{kind}", data=buf, headers={"Content-Type": "image/jpeg"})
        with urllib.request.urlopen(req, timeout=0.3) as r: return json.loads(r.read())
    except Exception as ex:
        log("offload down:", ex); OFF["ok"] = False; OFF["next"] = time.time() + 30; return None

# ----------------------------------------------------------------------------------------------- models
def yolo():
    if "yolo" not in _M:
        import onnxruntime as ort
        o = ort.SessionOptions(); o.intra_op_num_threads = int(E("YOLO_THREADS", "3"))
        _M["yolo"] = ort.InferenceSession(os.path.join(MODELS, "yolov8n.onnx"), o, providers=["CPUExecutionProvider"])
    return _M["yolo"]

def detect(img):
    """YOLOv8n COCO -> list of (cls, conf, x1,y1,x2,y2) in image px."""
    t = time.time(); rem = _offload("detect", img)
    if rem is not None:
        TIMES["yolo_ms"] = round((time.time() - t) * 1000); TIMES["yolo_via"] = "lan"
        return [tuple(d) for d in rem]
    TIMES["yolo_via"] = "pi"; h, w = img.shape[:2]; s = 640 / max(h, w)
    nh, nw = int(round(h * s)), int(round(w * s)); py, px = (640 - nh) // 2, (640 - nw) // 2
    can = np.full((640, 640, 3), 114, np.uint8); can[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh))
    x = can[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = yolo().run(None, {"images": x})[0][0].T                     # 8400 x 84
    sc = out[:, 4:]; ci = sc.argmax(1); cf = sc[np.arange(len(sc)), ci]; k = cf > YOLO_CONF
    b = out[k, :4]; ci = ci[k]; cf = cf[k]; res = []
    if len(b):
        xy = np.stack([(b[:, 0] - b[:, 2] / 2 - px) / s, (b[:, 1] - b[:, 3] / 2 - py) / s, b[:, 2] / s, b[:, 3] / s], 1)
        for i in np.array(cv2.dnn.NMSBoxes(xy.tolist(), cf.tolist(), YOLO_CONF, 0.45)).flatten():
            x1, y1, bw, bh = xy[i]
            res.append((COCO[ci[i]], float(cf[i]), max(0, int(x1)), max(0, int(y1)), min(w - 1, int(x1 + bw)), min(h - 1, int(y1 + bh))))
    TIMES["yolo_ms"] = round((time.time() - t) * 1000)
    return res

def faces(img):
    """YuNet faces -> list of dict(box, score, emb, hair)."""
    t = time.time(); rem = _offload("faces", img)
    if rem is not None:
        out = []
        for f in rem:
            x, y, x2, y2 = f["box"]; bh = y2 - y; hy0 = max(0, y - int(0.5 * bh))
            out.append({"box": tuple(f["box"]), "score": f["score"], "emb": np.float32(f["emb"]),
                        "hair": hist(img[hy0:max(hy0 + 1, y), max(0, x):x2]) if y > 4 else None})
        TIMES["face_ms"] = round((time.time() - t) * 1000); return out
    if "yunet" not in _M:
        _M["yunet"] = cv2.FaceDetectorYN.create(os.path.join(MODELS, "face_yunet.onnx"), "", (320, 320), 0.8)
        _M["sface"] = cv2.FaceRecognizerSF.create(os.path.join(MODELS, "face_sface.onnx"), "")
    h, w = img.shape[:2]; _M["yunet"].setInputSize((w, h))
    _, f = _M["yunet"].detect(img); out = []
    for row in (f if f is not None else []):
        x, y, bw, bh = [int(v) for v in row[:4]]
        al = _M["sface"].alignCrop(img, row); emb = _M["sface"].feature(al).flatten()
        emb = emb / (np.linalg.norm(emb) + 1e-9)
        hy0 = max(0, y - int(0.5 * bh)); hair = hist(img[hy0:max(hy0 + 1, y), max(0, x):x + bw]) if y > 4 else None
        out.append({"box": (x, y, x + bw, y + bh), "score": float(row[14]), "emb": emb, "hair": hair, "_row": row})
    TIMES["face_ms"] = round((time.time() - t) * 1000)
    return out

def hist(crop):
    if crop is None or crop.size == 0: return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hh = cv2.calcHist([hsv], [0, 1], None, [12, 8], [0, 180, 0, 256]); cv2.normalize(hh, hh)
    return hh.flatten().round(4).tolist()

def hsim(a, b):
    if not a or not b: return 0.0
    return float(cv2.compareHist(np.float32(a), np.float32(b), cv2.HISTCMP_CORREL))

def hands(jpeg):
    """MediaPipe Hands in the MentorPi container (it's installed there). Returns [(x,y) px for lm 5,6,7,8] or None."""
    t = time.time(); os.makedirs(PERC_HOST, exist_ok=True)
    with open(os.path.join(PERC_HOST, "frame.jpg"), "wb") as f: f.write(jpeg)
    try:
        r = subprocess.run(["docker", "exec", "-u", "ubuntu", "MentorPi", "python3", PERC_CONT + "/hands.py",
                            PERC_CONT + "/frame.jpg"], capture_output=True, text=True, timeout=15)
        j = json.loads(r.stdout.strip().splitlines()[-1]) if r.stdout.strip() else None
    except Exception as ex:
        log("hands failed:", ex); j = None
    TIMES["hands_ms"] = round((time.time() - t) * 1000)
    return j

HANDS_PY = r'''
import sys, json, cv2, mediapipe as mp
im = cv2.imread(sys.argv[1]); h, w = im.shape[:2]
with mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=1, min_detection_confidence=0.5) as H:
    r = H.process(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
if not r.multi_hand_landmarks: print("null")
else:
    L = r.multi_hand_landmarks[0].landmark
    print(json.dumps([[L[i].x * w, L[i].y * h] for i in (5, 6, 7, 8)]))
'''

def install_helpers():
    os.makedirs(PERC_HOST, exist_ok=True)
    p = os.path.join(PERC_HOST, "hands.py")
    if not os.path.exists(p) or open(p).read() != HANDS_PY:
        with open(p, "w") as f: f.write(HANDS_PY)

# ----------------------------------------------------------------------------------------------- frames + geometry
def grab(depth=True):
    blk = gb_caps.vision_block()
    if blk: raise PermissionError(blk)
    t = time.time(); dep = {}
    def _d():
        try:
            m = gb_robot.subscribe_once(DEPTH_TOPIC, "sensor_msgs/Image", timeout=4)
            if m:
                a = np.frombuffer(base64.b64decode(m[0]["data"]), "<u2").reshape(m[0]["height"], m[0]["width"])
                dep["d"] = a.astype(np.float32) / 1000.0
        except Exception as ex: log("depth failed:", ex)
    th = threading.Thread(target=_d) if depth else None
    if th: th.start()
    try: jpeg = gb_robot.snapshot()
    except Exception as ex: raise Problem(f"I can't get a camera picture ({ex})")
    if th: th.join(5)
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    TIMES["grab_ms"] = round((time.time() - t) * 1000)
    return jpeg, img, dep.get("d")

def box_depth(d, b):
    if d is None: return None
    x1, y1, x2, y2 = b; sh = (d.shape[1] / 640.0)
    cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * 0.3, (y2 - y1) * 0.3
    roi = d[int((cy - h / 2) * sh):int((cy + h / 2) * sh) + 1, int((cx - w / 2) * sh):int((cx + w / 2) * sh) + 1]
    v = roi[(roi > 0.12) & (roi < 8)]
    return float(np.median(v)) if v.size > 8 else None

def rel_xy(b, z):
    """box center + depth -> (forward, left) meters relative to base."""
    u = (b[0] + b[2]) / 2; X = (u - CX) * z / FX
    return z + CAM_X, -X

def to_world(fwd, left):
    p = None
    try: p = gb_robot.get_pose(timeout=2)
    except Exception: pass
    if not p: return None
    c, s = math.cos(p["theta"]), math.sin(p["theta"])
    return {"frame": p["frame"], "x": p["x"] + c * fwd - s * left, "y": p["y"] + s * fwd + c * left, "pose": p}

def map_name():
    try:
        ms = sorted((os.path.getmtime(os.path.join(DATA, "maps", f)), f) for f in os.listdir(os.path.join(DATA, "maps")) if f.endswith(".yaml"))
        return ms[-1][1][:-5] if ms else None
    except Exception: return None

# ----------------------------------------------------------------------------------------------- object pipeline
def pick_target(jpeg, img, use_hand=True):
    dets = [d for d in detect(img) if d[0] != "person"]
    hand = hands(jpeg) if use_hand else None
    h, w = img.shape[:2]; how = "center"; best = None
    if hand and dets:
        (x0, y0), (x1, y1) = hand[0], hand[3]; vx, vy = x1 - x0, y1 - y0; n = math.hypot(vx, vy) or 1; vx, vy = vx / n, vy / n
        sc = []
        for d in dets:
            cx, cy = (d[2] + d[4]) / 2 - x1, (d[3] + d[5]) / 2 - y1
            along = cx * vx + cy * vy; perp = abs(cx * vy - cy * vx)
            if along > -20: sc.append((perp + 0.15 * max(0, along), d))
        if sc: best = min(sc, key=lambda s: s[0])[1]; how = "pointing"
    if best is None and hand:                                  # crop along the ray
        (x0, y0), (x1, y1) = hand[0], hand[3]; vx, vy = x1 - x0, y1 - y0
        n = math.hypot(vx, vy) or 1; cx, cy = x1 + vx / n * 90, y1 + vy / n * 90
        return None, (int(max(0, cx - 70)), int(max(0, cy - 70)), int(min(w - 1, cx + 70)), int(min(h - 1, cy + 70))), "pointing area"
    if best is None and dets:
        best = min(dets, key=lambda d: math.hypot((d[2] + d[4]) / 2 - w / 2, (d[3] + d[5]) / 2 - h / 2) / (0.5 + d[1]))
    if best is None:
        return None, (int(w * .3), int(h * .25), int(w * .7), int(h * .75)), "center"
    return best, best[2:], how

def _crop(img, b): return img[b[1]:b[3] + 1, b[0]:b[2] + 1]

def look(use_hand=True):
    t = time.time(); jpeg, img, d = grab()
    det, box, how = pick_target(jpeg, img, use_hand)
    z = box_depth(d, box); crop = _crop(img, box)
    r = {"det": det, "box": box, "how": how, "dist": z, "crop": crop, "hist": hist(crop), "img": img}
    TIMES["look_ms"] = round((time.time() - t) * 1000)
    return r

def match_label(r):
    cls = r["det"][0] if r["det"] else None
    with db() as c:
        rows = c.execute("SELECT * FROM objects WHERE yolo_class IS ? ORDER BY last_ts DESC", (cls,)).fetchall() if cls else \
               c.execute("SELECT * FROM objects WHERE yolo_class IS NULL").fetchall()
    best = None
    for row in rows:
        s = hsim(r["hist"], json.loads(row["hist"] or "null"))
        if cls is not None and best is None: best = (s, row)          # newest taught for this class by default
        if best is None or s > best[0] + 0.15: best = (s, row)
    if best and cls is None and best[0] < 0.75: return None
    return best[1] if best else None

def record_sighting(label, r):
    if r["dist"] is None: return None
    fwd, left = rel_xy(r["box"], r["dist"]); wpos = to_world(fwd, left)
    size = (r["box"][2] - r["box"][0]) * r["dist"] / FX
    now = time.time(); mp_ = map_name() if wpos and wpos["frame"] == "map" else None
    with db() as c:
        if wpos:
            for s in c.execute("SELECT * FROM sightings WHERE label=? AND frame=?", (label, wpos["frame"])).fetchall():
                if s["x"] is not None and math.hypot(s["x"] - wpos["x"], s["y"] - wpos["y"]) < 0.5:
                    n = s["n"] + 1
                    c.execute("UPDATE sightings SET x=?,y=?,n=?,last_ts=?,size_m=?,dist_m=?,map=? WHERE id=?",
                              ((s["x"] * s["n"] + wpos["x"]) / n, (s["y"] * s["n"] + wpos["y"]) / n, n, now, size, r["dist"], mp_, s["id"]))
                    break
            else:
                c.execute("INSERT INTO sightings(label,map,frame,x,y,size_m,dist_m,first_ts,last_ts) VALUES(?,?,?,?,?,?,?,?,?)",
                          (label, mp_, wpos["frame"], wpos["x"], wpos["y"], size, r["dist"], now, now))
        else:
            c.execute("INSERT INTO sightings(label,map,frame,size_m,dist_m,first_ts,last_ts) VALUES(?,?,?,?,?,?,?)",
                      (label, None, None, size, r["dist"], now, now))
    write_sidecar()
    return wpos

def write_sidecar():
    try:
        with db() as c:
            rows = [dict(s) for s in c.execute("SELECT label,map,x,y,size_m,last_ts FROM sightings WHERE frame='map'")]
        for name in {s["map"] for s in rows if s["map"]}:
            p = os.path.join(DATA, "maps", name + ".objects.json")
            with open(p + ".tmp", "w") as f: json.dump([s for s in rows if s["map"] == name], f, indent=1)
            os.replace(p + ".tmp", p)
    except Exception as ex: log("sidecar failed:", ex)

def _dist_words(z): return f", about {z:.1f} meters away" if z else ""

def what_is_this():
    t = time.time(); r = look()
    row = match_label(r); cls = r["det"][0] if r["det"] else None
    if row:
        with db() as c: c.execute("UPDATE objects SET last_ts=? WHERE id=?", (time.time(), row["id"]))
        LAST.update(label=row["label"], cls=cls, t=time.time()); hit("local"); record_sighting(row["label"], r)
        out = f"That's your {row['label']}{_dist_words(r['dist'])}."
    elif r["det"]:
        LAST.update(label=cls, cls=cls, t=time.time()); hit("yolo"); record_sighting(cls, r)
        conf = int(r["det"][1] * 100)
        out = (f"That looks like a {cls}" + ("" if conf >= 60 else f", but I'm only {conf} percent sure") +
               f"{_dist_words(r['dist'])}. Say 'this is a' and a name to teach me.")
    else:
        hit("unknown"); LAST.update(label=None, cls=None, t=time.time())
        out = "I don't recognize anything there yet. Point at it and say 'this is a' and its name to teach me."
    TIMES["what_ms"] = round((time.time() - t) * 1000)
    return out

def teach(label, correction=False):
    label = re.sub(r"[^a-z0-9 '\-]", "", label.lower()).strip()[:40]
    if not label: return "I didn't catch the name."
    r = look(); cls = r["det"][0] if r["det"] else None; now = time.time()
    d = os.path.join(OBJ_DIR, label.replace(" ", "_")); os.makedirs(d, exist_ok=True)
    cp = os.path.join(d, time.strftime("%Y%m%d-%H%M%S") + ".jpg"); cv2.imwrite(cp, r["crop"])
    with db() as c:
        row = c.execute("SELECT * FROM objects WHERE label=? AND yolo_class IS ?", (label, cls)).fetchone()
        if row:
            c.execute("UPDATE objects SET count=count+1,last_ts=?,crop=?,dist_m=?,hist=? WHERE id=?",
                      (now, cp, r["dist"], json.dumps(r["hist"]), row["id"]))
        else:
            c.execute("INSERT INTO objects(label,yolo_class,first_ts,last_ts,crop,dist_m,hist) VALUES(?,?,?,?,?,?,?)",
                      (label, cls, now, now, cp, r["dist"], json.dumps(r["hist"])))
    record_sighting(label, r); LAST.update(label=label, cls=cls, t=now)
    via = f" I'll call that {cls} a {label} from now on." if cls and cls != label else ""
    return ("Got it, sorry. " if correction else "Okay. ") + f"That's a {label}.{via}"

def forget_object(label):
    with db() as c:
        n = c.execute("DELETE FROM objects WHERE label=?", (label,)).rowcount
        c.execute("DELETE FROM sightings WHERE label=?", (label,))
    shutil.rmtree(os.path.join(OBJ_DIR, label.replace(" ", "_")), ignore_errors=True)
    if n: write_sidecar()
    return n

def list_objects():
    with db() as c:
        rows = c.execute("SELECT label, yolo_class, SUM(count) n FROM objects GROUP BY label ORDER BY MAX(last_ts) DESC").fetchall()
    if not rows: return "I haven't learned any objects yet."
    names = [r["label"] + (f" (a {r['yolo_class']})" if r["yolo_class"] and r["yolo_class"] != r["label"] else "") for r in rows[:12]]
    return f"I know {len(rows)} object" + ("s" if len(rows) != 1 else "") + ": " + ", ".join(names) + "."

def _ago(ts):
    s = time.time() - ts
    return "just now" if s < 90 else f"{int(s // 60)} minutes ago" if s < 3600 else f"{int(s // 3600)} hours ago" if s < 172800 else f"{int(s // 86400)} days ago"

def _rel_words(dx, dy, p):
    c, s = math.cos(-p["theta"]), math.sin(-p["theta"]); fwd, left = c * dx - s * dy, s * dx + c * dy
    d = math.hypot(fwd, left); b = math.degrees(math.atan2(left, fwd))
    side = ("straight ahead" if abs(b) < 12 else "ahead, a little " + ("left" if b > 0 else "right") if abs(b) < 40 else
            "to my " + ("left" if b > 0 else "right") if abs(b) < 120 else "behind me")
    return f"{d:.1f} meters {side}"

def where_is(label):
    with db() as c:
        s = c.execute("SELECT * FROM sightings WHERE label=? ORDER BY last_ts DESC LIMIT 1", (label,)).fetchone()
        known = c.execute("SELECT 1 FROM objects WHERE label=?", (label,)).fetchone()
    if not s: return f"I don't know where the {label} is." + ("" if known else " I haven't learned it yet.")
    if s["frame"]:
        p = None
        try: p = gb_robot.get_pose(timeout=2)
        except Exception: pass
        if p and p["frame"] == s["frame"]:
            return f"The {label} was last seen {_ago(s['last_ts'])}, about {_rel_words(s['x'] - p['x'], s['y'] - p['y'], p)}."
        return f"The {label} was last seen {_ago(s['last_ts'])} at {s['x']:.1f}, {s['y']:.1f} on the {s['map'] or s['frame']} map."
    return f"I saw the {label} {_ago(s['last_ts'])}, about {s['dist_m']:.1f} meters in front of me, but I had no map position then."

def around_here():
    p = None
    try: p = gb_robot.get_pose(timeout=2)
    except Exception: pass
    with db() as c:
        rows = c.execute("SELECT * FROM sightings ORDER BY last_ts DESC").fetchall()
    near = [(math.hypot(r["x"] - p["x"], r["y"] - p["y"]), r) for r in rows if p and r["frame"] == p["frame"] and r["x"] is not None]
    near = sorted([n for n in near if n[0] < 3.0], key=lambda n: n[0])[:6]
    if near: return "Around here I know about: " + "; ".join(f"the {r['label']}, {_rel_words(r['x'] - p['x'], r['y'] - p['y'], p)}" for _, r in near) + "."
    if rows: return "Nothing I've mapped is close by. Recently I saw: " + ", ".join(dict.fromkeys(r["label"] for r in rows[:6])) + "."
    return "I haven't seen anything around here yet."

def hit_ratio():
    with db() as c:
        d = dict(c.execute("SELECT kind, COUNT(*) FROM hits GROUP BY kind").fetchall())
    tot = sum(d.values()) or 1
    return f"Of {sum(d.values())} 'what is this' answers, {d.get('local', 0)} came from what you taught me, {d.get('yolo', 0)} from my built-in detector and {d.get('unknown', 0)} were unknown. All local."

# ----------------------------------------------------------------------------------------------- faces / people
def known_people():
    with fdb() as c: return [r[0] for r in c.execute("SELECT name FROM faces GROUP BY name ORDER BY MIN(ts)")]

def enroll(name, add=False, say=None):
    """Single-photo enrollment: '3, 2, 1', one frame, exactly one clear face -> one SFace embedding + 112 px thumbnail.
    add=True ('take another picture of me') keeps earlier photos, max 3 per person."""
    name = name.strip().title()[:30]
    with fdb() as c: have = c.execute("SELECT COUNT(*) FROM faces WHERE name=?", (name,)).fetchone()[0]
    if add and have >= 3: return f"I already have 3 pictures of {name}, that's the maximum."
    if say:
        try: say("Hold still. 3, 2, 1."); time.sleep(2.5)
        except Exception: pass
    why = "I couldn't see a face"
    for attempt in range(3):
        _, img, _ = grab(depth=False); fs = faces(img)
        clear = [f for f in fs if f["score"] > 0.85 and f["box"][2] - f["box"][0] > 60]
        if len(fs) > 1: why = "I see more than one face; only the person being enrolled should face me"; continue
        if not clear: why = ("your face is too small or blurry, come a bit closer" if fs else "I couldn't see a face"); continue
        f = clear[0]; thumb = _M["sface"].alignCrop(img, f["_row"]) if "_row" in f else None
        d = os.path.join(FACE_DIR, name.lower()); os.makedirs(d, exist_ok=True)
        with fdb() as c:
            if not add: c.execute("DELETE FROM faces WHERE name=?", (name,)); shutil.rmtree(d, ignore_errors=True); os.makedirs(d, exist_ok=True)
            c.execute("INSERT INTO faces(name,emb,hair,ts) VALUES(?,?,?,?)", (name, f["emb"].astype(np.float32).tobytes(), json.dumps(f["hair"]), time.time()))
        if thumb is not None: cv2.imwrite(os.path.join(d, time.strftime("%Y%m%d-%H%M%S") + ".jpg"), thumb)
        if add: return f"Got another picture of {name}. That's {min(3, have + 1)} of 3."
        return f"Nice to meet you, {name}. I'll remember your face; say forget {name} to delete it."
    return f"{why}, so I didn't save anything. Please try again."

def recognize(fs):
    with fdb() as c: rows = c.execute("SELECT name, emb, hair FROM faces").fetchall()
    out = []
    for f in fs:
        best = {}
        for r in rows:
            s = float(np.dot(f["emb"], np.frombuffer(r["emb"], np.float32)))
            best[r["name"]] = max(best.get(r["name"], (-1, None))[0], s), r["hair"]
        ranked = sorted(best.items(), key=lambda kv: -kv[1][0])
        if len(ranked) > 1 and ranked[0][1][0] - ranked[1][1][0] < 0.05:      # tie -> hair/head colour histogram
            ranked.sort(key=lambda kv: -(kv[1][0] + 0.1 * hsim(f["hair"], json.loads(kv[1][1] or "null"))))
        nm = ranked[0][0] if ranked and ranked[0][1][0] >= FACE_COS else None
        out.append((nm, ranked[0][1][0] if ranked else 0, f))
    return out

def who_is_this(me=False):
    jpeg, img, d = grab(); fs = faces(img)
    if not fs: return "I don't see anyone's face right now."
    rec = recognize(fs); names = [n for n, _, _ in rec if n]
    if me:
        if OWNER in names or names: 
            n = names[0]; z = box_depth(d, rec[[r[0] for r in rec].index(n)][2]["box"])
            return f"Yes, I see you, {n}{_dist_words(z)}."
        return "I see a face, but I don't recognize it. Say 'this is me' to teach me your face."
    unk = len(rec) - len(names)
    if not names: return "I see " + ("someone" if unk == 1 else f"{unk} people") + ", but I don't know them yet."
    return "That's " + " and ".join(names) + (f", plus {unk} person I don't know" if unk else "") + "."

def forget_person(name):
    shutil.rmtree(os.path.join(FACE_DIR, name.lower()), ignore_errors=True)
    with fdb() as c: return c.execute("DELETE FROM faces WHERE lower(name)=lower(?)", (name,)).rowcount

# ---- tracking for follow / come-to-me / go-to-object (writes the target file the gb_explore 'follow' mission reads)
TRACK = {"thread": None, "stop": threading.Event(), "name": None, "lost": False}

def _write_target(seen, fwd=0.0, left=0.0):
    os.makedirs(os.path.dirname(TARGET_FILE), exist_ok=True)
    with open(TARGET_FILE + ".tmp", "w") as f: json.dump({"t": time.time(), "seen": seen, "fwd": fwd, "left": left}, f)
    os.replace(TARGET_FILE + ".tmp", TARGET_FILE)

def find_person(name, img, d, prev=None):
    """face match first; else best person box by IoU with prev box + clothing histogram."""
    people = [x for x in detect(img) if x[0] == "person"]
    box = None
    if name == "__any__" and prev is None and people:          # Pi-only 'follow me': lock the most central/biggest person
        h, w = img.shape[:2]
        p = min(people, key=lambda p: abs((p[2] + p[4]) / 2 - w / 2) / w - 0.5 * (p[4] - p[2]) / w); box = p[2:]
    fs = faces(img) if (name != "__any__" and (prev is None or int(time.time() * 2) % 4 == 0)) else []
    for n, s, f in recognize(fs):
        if n and n.lower() == name.lower():
            fx = (f["box"][0] + f["box"][2]) / 2
            inside = [p for p in people if p[2] <= fx <= p[4]]
            box = inside[0][2:] if inside else f["box"]; break
    if box is None and prev and people:
        def iou(a, b):
            ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
            u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
            return ix * iy / u if u else 0
        sc = [(iou(prev["box"], p[2:]) + 0.5 * hsim(prev["hist"], hist(_crop(img, p[2:]))), p) for p in people]
        s, p = max(sc, key=lambda s: s[0])
        if s > 0.45: box = p[2:]
    if box is None: return None
    return {"box": box, "hist": hist(_crop(img, box)), "dist": box_depth(d, box)}

def _track_loop(name, static=None):
    try: os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 10)   # tracker thread niced below voice/guard
    except Exception: pass
    prev = None; TRACK["lost"] = False
    while not TRACK["stop"].is_set():
        try:
            if static:                                             # go to a mapped object: fixed map point
                p = gb_robot.get_pose(timeout=2)
                if p and p["frame"] == static["frame"]:
                    c, s = math.cos(-p["theta"]), math.sin(-p["theta"]); dx, dy = static["x"] - p["x"], static["y"] - p["y"]
                    _write_target(True, c * dx - s * dy, s * dx + c * dy)
                time.sleep(0.5); continue
            _, img, d = grab()
            r = find_person(name, img, d, prev)
            if r and r["dist"]:
                TRACK["n"] = TRACK.get("n", 0) + 1; prev = r; fwd, left = rel_xy(r["box"], r["dist"]); _write_target(True, fwd, left)
                w = to_world(fwd, left)
                if w:
                    with db() as c: c.execute("INSERT INTO people_track VALUES(?,?,?,?,?,?)", (time.time(), name, w["frame"], w["x"], w["y"], r["dist"]))
            else:
                _write_target(False)
        except PermissionError:
            _write_target(False); break
        except Exception as ex:
            log("track:", ex); _write_target(False); time.sleep(0.5)
    _write_target(False)

def start_track(name, static=None):
    stop_track(); TRACK["stop"].clear(); TRACK["name"] = name
    TRACK["thread"] = threading.Thread(target=_track_loop, args=(name, static), daemon=True, name="gb_track"); TRACK["thread"].start()

def stop_track():
    TRACK["stop"].set()
    if TRACK["thread"]: TRACK["thread"].join(3)
    TRACK["thread"] = None

def people_mission(name, kind, explore_mod, announce, store):
    """kind: follow | come | object. Goes through gb_explore.request (mission flag, AUTO_EXPLORE/ALLOW_MOTION, guard, caps)."""
    static = None
    if kind == "object":
        with db() as c:
            s = c.execute("SELECT * FROM sightings WHERE label=? AND frame IS NOT NULL ORDER BY last_ts DESC LIMIT 1", (name,)).fetchone()
        if not s: return f"I don't have a map position for the {name}, so I can't drive there."
        static = {"frame": s["frame"], "x": s["x"], "y": s["y"]}
    elif name == "__any__": pass
    elif name.lower() not in [n.lower() for n in known_people()]:
        return f"I don't know {name}'s face yet. Say 'this is {name}' while they face me."
    plan = {"mission": "follow", "target": "you" if name in ("__any__", OWNER) else name, "kind": kind,
            "stop_m": COME_M if kind == "come" else 0.5 if kind == "object" else FOLLOW_M,
            "standoff_m": 0.0 if kind == "object" else STANDOFF_M, "once": kind != "follow",
            "search": kind == "come", "lost_s": LOST_S, "cap_s": 300.0 if kind == "follow" else 90.0}
    def ann(text):
        stop_track()
        if announce: announce(text)
    note = ""
    if kind == "follow" and name not in ("__any__", OWNER) and not OFF["ok"]:
        note = f"Without the vision server I only confirm {name}'s face now and then; mostly I track the body box. "
    start_track(name, static)
    r = note + explore_mod.request(plan, store=store, announce=ann)
    if not explore_mod.running(): stop_track()
    return r

# ----------------------------------------------------------------------------------------------- joystick
KNOWN_PADS = {   # Hiwonder joystick_control.py tables (container: ros2_ws/src/peripherals/peripherals/joystick_control.py)
    "SHANWAN Android Gamepad": {"axes": {"lin": 1, "ang": 2}, "buttons": ["cross", "circle", "", "square", "triangle", "", "l1", "r1", "l2", "r2", "select", "start", "mode", "lc", "rc"]},
    "USB WirelessGamepad": {"axes": {"lin": 1, "ang": 2}, "buttons": ["triangle", "circle", "cross", "square", "l1", "r1", "l2", "r2", "select", "start", "lc", "rc", "mode", "", ""]},
}
ESTOP_BUTTONS = ("cross", "select")

def joystick_info():
    devs = []
    try:
        for blk in open("/proc/bus/input/devices").read().split("\n\n"):
            if re.search(r"Handlers=.*\bjs\d", blk):
                nm = re.search(r'N: Name="([^"]*)"', blk); vp = re.search(r"Vendor=(\w+) Product=(\w+)", blk)
                devs.append({"name": nm.group(1) if nm else "?", "vid_pid": f"{vp.group(1)}_{vp.group(2)}" if vp else "unknown",
                             "js": re.search(r"\b(js\d)", blk).group(1)})
    except Exception: pass
    return devs

def joystick_check(setup=False):
    devs = joystick_info()
    if not devs:
        try: hiwonder_joy_off()
        except Exception: pass
        return ("No, I don't see a joystick. There's no /dev/input/js device and nothing gamepad-like on USB. "
                "Plug in the remote's USB receiver and switch the remote on, then ask again.")
    d = devs[0]; os.makedirs(JOY_DIR, exist_ok=True); p = os.path.join(JOY_DIR, d["vid_pid"] + ".json")
    conf = os.path.exists(p)
    if setup and not conf:
        prof = KNOWN_PADS.get(d["name"])
        if not prof: return f"I see {d['name']}, but it's not in my known gamepad table, so I haven't configured it."
        with open(p, "w") as f: json.dump({"name": d["name"], **prof, "estop": list(ESTOP_BUTTONS)}, f, indent=1)
        conf = True
    return f"Yes, I see {d['name']} on {d['js']}. It is " + ("configured." if conf else "not configured yet. Say 'set up the joystick'.")

def hiwonder_joy_off():
    """Hiwonder joystick_control busy-loops (~92% of a core) with no pad and would double-drive cmd_vel. Stop it
    (it comes back on the next container start)."""
    if joystick_info(): return False
    pat = "lib/peripherals/joystick_control"
    subprocess.run(["docker", "exec", "MentorPi", "pkill", "-INT", "-f", pat], capture_output=True, timeout=10); time.sleep(2)
    r = subprocess.run(["docker", "exec", "MentorPi", "pkill", "-KILL", "-f", pat], capture_output=True, timeout=10)
    return True

# ----------------------------------------------------------------------------------------------- model updates (explicit only)
def update_models():
    os.makedirs(MODELS, exist_ok=True); out = []
    for fn, url in MODEL_URLS.items():
        p = os.path.join(MODELS, fn)
        try:
            req = urllib.request.Request(url, method="HEAD")
            with urllib.request.urlopen(req, timeout=15) as r: size = int(r.headers.get("Content-Length") or r.headers.get("x-linked-size") or 0)
            if os.path.exists(p) and (not size or os.path.getsize(p) == size):
                out.append(f"{fn} up to date"); continue
            urllib.request.urlretrieve(url, p + ".part.onnx"); os.replace(p + ".part.onnx", p); _M.clear(); out.append(f"{fn} updated")
        except Exception as ex:
            out.append(f"{fn} check failed"); log("update", fn, ex)
    return "Model check done: " + "; ".join(out) + ". Everything runs offline from local copies."

# ----------------------------------------------------------------------------------------------- voice intents (zero tokens)
HARD_LIMITS = {
    r"\b(guard|obstacle guard)\b": "The obstacle guard stays on outside missions. That's a hard safety limit I can't switch off by voice.",
    r"\b(standoff|distance from people|closer to (me|people))\b": "I always keep at least 0.8 meters from people. That's a hard limit.",
    r"\b(low.?voltage|battery (protection|shutdown|cutoff))\b": "The low battery shutdown protects the battery and can't be disabled by voice.",
    r"\b(speed (cap|limit)s?)\b": "The speed caps are maximums. I can go slower, but voice can't raise them.",
}

_ART = r"(?:a |an |the |my |our |your )"

def intent(text, ctx=None):
    """Return reply str, or None if not ours. ctx: dict(explore=module, announce=fn, store=Store)."""
    t = (text or "").lower().strip().rstrip(".!?")
    t = re.sub(r"^(?:hey |ok |okay )?(?:grok ?bot|grok|robot)[, ]+", "", t)
    t = re.sub(r"^(?:can you|could you|please|would you)\s+", "", t).strip()
    ctx = ctx or {}
    try:
        if re.search(r"\b(disable|turn off|switch off|remove|raise|increase|ignore|bypass)\b", t):
            for rx, msg in HARD_LIMITS.items():
                if re.search(rx, t): return msg
        if re.search(r"\b(reboot|restart yourself|shut ?down|power off)\b", t) and "map" not in t:
            return "I don't reboot or shut down by voice. Do that from the Pi with a confirmation, or by the power switch."
        if re.search(r"\b(update|download|check for new) (your |the )?models?\b", t): return update_models()
        if re.search(r"\b(detect|see|find|set ?up|configure|is there) (the |a |my )?(joystick|gamepad|controller|remote)\b", t):
            return joystick_check(setup=bool(re.search(r"set ?up|configure", t)))
        if re.search(r"\b(hit ratio|how (often|many).*(local|online))\b", t): return hit_ratio()
        # people
        say = ctx.get("announce")
        m = re.match(r"^take (?:another|one more) (?:picture|photo) of ([a-z]+)$", t)
        if m: return enroll(OWNER if m.group(1) == "me" else m.group(1), add=True, say=say)
        m = re.search(r"\b(?:this is me|this is my face|it'?s me|that'?s me|take my (?:picture|photo))\b", t) or re.match(r"^(?:hi,? |hello,? )?(?:i'?m|i am|my name is) ([a-z]+)$", t)
        if m:
            return enroll(m.group(1) if m.groups() and m.group(1) else OWNER, say=say)
        m = re.match(r"^(?:hi,? |hello,? )?this is ([a-z]+)$", t)
        if m and m.group(1) not in ("it", "that", "this", "what", "a", "an", "the"):
            _, img, _ = grab(depth=False)
            if [f for f in faces(img) if f["score"] > 0.85 and f["box"][2] - f["box"][0] > 60]:
                return enroll(m.group(1), say=say)
            return teach(m.group(1))
        if re.search(r"^forget (my face|me)$", t): return "Done, I forgot your face." if forget_person(OWNER) else "I didn't have your face saved."
        if re.search(r"\bwho do you know\b|\bwhose faces\b", t):
            ps = known_people(); return ("I know " + ", ".join(ps) + ".") if ps else "I don't know anyone's face yet."
        if re.search(r"^who am i$|\bdo you see me\b|\bcan you see me\b|^is (that|this) me$", t): return who_is_this(me=True)
        if re.search(r"^who(?:'s| is) (this|that|here|in front of you)\b", t): return who_is_this()
        m = re.match(r"^(?:come (?:here|to me|over here)|come back to me)$", t)
        if m: return _mission(OWNER, "come", ctx)
        m = re.match(r"^follow ([a-z]+)$", t)
        if m:
            if m.group(1) == "me":
                return _mission(OWNER if OWNER.lower() in [n.lower() for n in known_people()] else "__any__", "follow", ctx)
            return _mission(m.group(1).title(), "follow", ctx)
        m = re.match(r"^(?:go|drive|take me) to (?:the |my )?([a-z ]+)$", t)
        if m and _known_obj(m.group(1)): return _mission(m.group(1), "object", ctx)
        # objects
        if re.search(r"^(?:what(?:'s| is) (?:this|that)(?: thing)?|what am i (?:pointing at|holding)|what do you think this is|do you know what this is)$", t):
            return what_is_this()
        m = re.match(r"^(no,? |nope,? |wrong,? )?(?:remember |note )?(?:that )?(?:this|that|it) is " + _ART + "?(.+)$", t) or \
            re.match(r"^(no,? )?(?:it'?s|that'?s) (?:actually )?" + _ART + "(.+)$", t) if not t.startswith("this is me") else None
        if m and len(m.group(2).split()) <= 4:
            return teach(m.group(2), correction=bool(m.group(1)))
        m = re.match(r"^remember (?:this|that) (?:as )?" + _ART + "?(.+)$", t)
        if m: return teach(m.group(1))
        if re.search(r"^(what do you know|list (?:the )?objects|what objects do you know|what have you learned)$", t): return list_objects()
        if re.search(r"\bwhat(?:'s| is) (around here|near me|nearby|around me)\b", t): return around_here()
        m = re.match(r"^where(?:'s| is| are| did i (?:leave|put)) (?:the |my )?([a-z ]+?)(?: at)?$", t)
        if m and _known_obj(m.group(1)): return where_is(m.group(1))
        m = re.match(r"^forget (?:about )?(?:the |my )?([a-z ]+)$", t)
        if m:
            n = m.group(1).strip(); o = forget_object(n); f = forget_person(n)
            if o or f: return f"Done, I forgot {n}." + (" Face deleted." if f else "")
        return None
    except PermissionError as ex:
        return str(ex)
    except Exception as ex:
        log("intent error:", type(ex).__name__, ex)
        return f"I ran into a problem: {str(ex)[:120] or type(ex).__name__}."

def _known_obj(label):
    label = label.strip()
    with db() as c: return c.execute("SELECT 1 FROM objects WHERE label=? UNION SELECT 1 FROM sightings WHERE label=?", (label, label)).fetchone() is not None

def _mission(name, kind, ctx):
    blk = gb_caps.vision_block()
    if blk: return blk
    ex = ctx.get("explore")
    if not ex: return "I can't start that from here."
    return people_mission(name, kind, ex, ctx.get("announce"), ctx.get("store"))

install_helpers() if os.path.isdir(HOST_SHARED) else None
