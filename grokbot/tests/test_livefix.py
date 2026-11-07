"""Measured-defect checks. No robot, no systemd, and no port 8080."""
import json, os, struct, sys, tempfile, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, ROOT)
import cv2
import numpy as np
import gb_demand
import gb_health_node
import gb_obstacle as OB
import gb_stream
import gb_wally
from deploy.migrate_wake_env import migrate_text

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

# --- depth: center ray, not the whole-frame median; no assumed offset ---
scene = [694, 1689, 1845] * 30
whole, why = OB.resolve_depth_scale("mono16", scene, 0.98)
check("a scene grid is not a unit", whole is None and why == "need_forward_ray", (whole, why))
plausible, why = OB.resolve_depth_scale("16UC1", None, 0.98, forward_raw=1689)
check("a plausible millimetre value is not a unit", plausible is None and why == "crosscheck_disagree", why)
ray, why = OB.resolve_depth_scale("16UC1", None, 0.98, forward_raw=980)
check("the forward ray can match the forward lidar", ray == 0.001 and why == "crosscheck_mm", (ray, why))
unshifted, why = OB.resolve_depth_scale("16UC1", None, 0.35, forward_raw=350, camera_ahead_m=0.082, tf_fresh=False)
check("an unverified 8 cm offset is not applied", unshifted == 0.001, (unshifted, why))
shifted, why = OB.resolve_depth_scale("16UC1", None, 0.35, forward_raw=350, camera_ahead_m=0.082, tf_fresh=True)
check("a fresh transform is allowed to change the comparison", shifted is None, why)
check("a filter range is not passed in as a driver scale", OB.resolve_depth_scale("16UC1", None, 0.98, driver_scale=None, forward_raw=1689)[0] is None)

w, h, step = 8, 3, 20
center = 980
raw = bytearray()
for y in range(h):
    row = bytearray()
    for x in range(w):
        val = center if 2 <= x <= 6 else 1689
        row += struct.pack("<H", val)
    row += b"\xab" * (step - w * 2)
    raw += row
img = OB.decode_depth("mono16", bytes(raw), h, w, step, 0)
check("padded little-endian mono16 keeps the center", img is not None and int(OB.center_raw(img, 4, 1)) == center, None if img is None else int(OB.center_raw(img, 4, 1)))
be = bytearray()
for y in range(h):
    row = bytearray()
    for x in range(w):
        val = 700 if 2 <= x <= 6 else 100
        row += struct.pack(">H", val)
    row += b"\x00" * (step - w * 2)
    be += row
img_be = OB.decode_depth("16UC1", bytes(be), h, w, step, 1)
check("big-endian step is not read as little-endian", img_be is not None and int(OB.center_raw(img_be, 4, 1)) == 700)

ranges = [5.0] * 20 + [0.98, 0.97, 0.99] + [4.0] * 20
fwd = OB.lidar_forward_m(ranges, -1.0, 2.0 / 42.0, half_rad=0.06)
check("forward cone is not the scan-wide median", fwd is not None and abs(fwd - 0.98) < 0.02, fwd)
rec = gb_health_node.record(10, 0.9, "mono16", [1689], 0.98, topic_online=True, nonzero_frac=0.67, forward_raw=1689)
check("topic online does not make the metric valid", rec["depth_topic_online"] is True and rec["depth_metric_valid"] is False and rec["depth_online"] is False, rec)
check("the record is still not a motion command", rec["executable"] is False and rec["tf_applied"] is False)
good = gb_health_node.record(10, 0.9, "mono16", [980], 0.98, topic_online=True, nonzero_frac=0.67, forward_raw=980)
check("a matching forward ray is the calibration", good["depth_metric_valid"] is True and good["depth_unit"] == "crosscheck_mm", good)

# --- wake file round-trip ---
live = "\n".join([
    "TRIGGER=wonderecho",
    "WAKE_WORDS=okay wally,ok wally,wake up wally,yo wally,wallie,wali",
    "WAKE_FUZZ=0.78",
    "WAKE_ANYWHERE=wallie,wali",
    "WAKE_ACK=Hello John, are you ready to rock and roll?",
    "STARTUP_GREETING=",
    "SUPPRESS_STARTUP_SPEECH=1",
    "ESTOP_AUTOCLEAR_S=0",
    "",
])
text, changed = migrate_text(live)
check("live alias list is rewritten", changed)
check("only the exact phrase remains", "WAKE_WORDS=wake up wally\n" in text and "okay wally" not in text and "wallie" not in text.split("WAKE_ACK")[0])
check("fuzz is an exact match and anywhere is empty", "WAKE_FUZZ=1.0\n" in text and "WAKE_ANYWHERE=\n" in text)
check("ack and greeting stay", "WAKE_ACK=Hello John, are you ready to rock and roll?" in text and "STARTUP_GREETING=\n" in text)
for phrase in ("okay wally", "ok wally", "yo wally", "wallie", "wali", "hey wally"):
    check("alias does not wake: " + phrase, gb_wally.software_wake(phrase, hardware_event=True) is False)
check("the spoken phrase still wakes", gb_wally.software_wake("wake up Wally") is True)
check("the serial event alone does not", gb_wally.software_wake("", hardware_event=True) is False)
import subprocess
env = os.environ.copy()
env["WAKE_FUZZ"] = ""
env["WAKE_WORDS"] = "wake up wally"
env["WAKE_ANYWHERE"] = ""
env["PYTHONPATH"] = os.path.dirname(HERE)
started = subprocess.run(
    [sys.executable, "-c", "import grokbot_voice; raise SystemExit(0 if grokbot_voice.WAKE_FUZZ == 1.0 else 1)"],
    cwd=os.path.dirname(HERE), env=env, capture_output=True, text=True)
check("blank WAKE_FUZZ survives voice import", started.returncode == 0, (started.returncode, started.stderr[-400:]))

# --- view demand dies with the viewers ---
period = gb_demand.view_period(0.7)
check("view detection stays under about 15 percent of one core", period >= 2.0 and (0.7 / period) <= 0.15, period)
imgj = np.zeros((24, 32, 3), np.uint8)
imgj[:, :] = (30, 10, 20)
_ok, enc = cv2.imencode(".jpg", imgj)
jpeg = enc.tobytes()

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, fmt, *args):
        return
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                part = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg)) + jpeg + b"\r\n"
                self.wfile.write(part)
                self.wfile.flush()
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
threading.Thread(target=httpd.serve_forever, daemon=True).start()
tmp = tempfile.mkdtemp(prefix="gblive_")
demand = os.path.join(tmp, "demand.json")
track = os.path.join(tmp, "rgb.json")
server = gb_stream.StreamServer("127.0.0.1", 0)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.05)
port = server.httpd.server_address[1]
feed = gb_stream.StockMjpeg("http://127.0.0.1:%d/stream" % httpd.server_address[1], server.slot, track, demand, viewers=lambda: server.clients)
feed.start()
time.sleep(0.3)
check("no viewer means no stock read and no view demand", feed.frames == 0 and not os.path.exists(demand), feed.frames)
import socket
clients = []
for _ in range(2):
    sock = socket.create_connection(("127.0.0.1", port), 2)
    sock.sendall(b"GET /stream.mjpg HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n")
    clients.append(sock)
deadline = time.time() + 2
while time.time() < deadline and (feed.frames < 1 or gb_demand.read_demand(demand) != "view"):
    time.sleep(0.05)
check("two viewers request detection", server.clients == 2 and gb_demand.read_demand(demand) == "view" and feed.frames >= 1, (server.clients, feed.frames))
for sock in clients:
    sock.close()
deadline = time.time() + 2
while time.time() < deadline and server.clients != 0:
    time.sleep(0.05)
time.sleep(0.3)
held = feed.frames
time.sleep(0.4)
mode = ""
if os.path.exists(demand):
    mode = json.load(open(demand)).get("mode")
check("disconnect drops demand and the reader", server.clients == 0 and mode == "idle" and feed.frames == held, (server.clients, mode, feed.frames, held))
gb_demand.write_demand(demand, "view", now=time.time() - 3)
check("a view demand expires promptly", gb_demand.read_demand(demand) is None)
now = time.time()
with open(track, "w") as f:
    json.dump({"t": now - 1.2, "detect_ok": True, "infer_s": 0.8, "person": {"t": now - 1.2, "conf": 0.8, "box": [2, 2, 10, 10]}}, f)
seen = gb_stream.read_visual_track(track, now)
motion = gb_stream.read_detector_track(track, now)
check("an aged detection can be labeled", seen and seen.get("aged") and seen.get("for_motion") is False and "0.80" in seen.get("label", ""), seen)
check("that same age is not a motion track", motion is None)
with open(track, "w") as f:
    json.dump({"t": now, "detect_ok": True, "person": None}, f)
empty = gb_stream.read_visual_track(track, now)
check("no person is not a detector failure", empty and empty.get("no_person") is True and empty.get("box") is None, empty)
feed.close()
httpd.shutdown()
server.httpd.shutdown()

print("test_livefix: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
