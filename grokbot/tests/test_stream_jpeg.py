"""Local MJPEG fixture. This does not talk to the robot or to port 8080."""
import json, os, sys, tempfile, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import cv2
import numpy as np
import gb_stream
import gb_demand

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

img = np.zeros((24, 32, 3), np.uint8)
img[:, :] = (40, 20, 10)
ok, enc = cv2.imencode(".jpg", img)
JPEG = enc.tobytes()
check("fixture is a jpeg", ok and JPEG[:2] == b"\xff\xd8" and JPEG[-2:] == b"\xff\xd9")

frames, rest = gb_stream.extract_jpegs(b"junk--" + JPEG + b"\r\nmore" + JPEG)
check("parser finds both fixture frames", len(frames) == 2 and frames[0] == JPEG, len(frames))

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, fmt, *args):
        return
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            for _ in range(6):
                part = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(JPEG)) + JPEG + b"\r\n"
                self.wfile.write(part)
                self.wfile.flush()
                time.sleep(0.02)
            time.sleep(2)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
port = httpd.server_address[1]
threading.Thread(target=httpd.serve_forever, daemon=True).start()

slot = gb_stream.LatestSlot()
tmp = tempfile.mkdtemp(prefix="gbstream_")
demand = os.path.join(tmp, "demand.json")
track = os.path.join(tmp, "rgb_guard.json")
feed = gb_stream.StockMjpeg("http://127.0.0.1:%d/stream?topic=/ascamera/camera_publisher/rgb0/image" % port,
                            slot, track, demand)
feed.start()
deadline = time.time() + 2
while time.time() < deadline and feed.frames < 2:
    time.sleep(0.02)
latest = slot.latest()
check("consumer called offer on fixture jpegs", feed.frames >= 2 and latest and latest[2][:2] == b"\xff\xd8", feed.frames)
decoded = cv2.imdecode(np.frombuffer(latest[2], np.uint8), cv2.IMREAD_COLOR)
check("fixture jpeg decodes", decoded is not None and decoded.shape[0] == 24, None if decoded is None else decoded.shape)
check("view demand is not a motor command", gb_demand.read_demand(demand) == "view" and gb_demand.moves("view") is False)
feed.close()
check("disconnect stops the reader", feed.thread is None)
check("closing twice is safe", feed.close() is None)
httpd.shutdown()

with open(track, "w") as f:
    json.dump({"t": time.time() - 5, "person": {"t": time.time() - 5, "box": [1, 1, 8, 8]}}, f)
check("stale detector box is not used", gb_stream.read_detector_track(track, time.time()) is None)
shown = gb_stream.overlay_jpeg(JPEG, None)
check("no box leaves the jpeg unchanged", shown == JPEG)
fresh = {"t": time.time(), "box": [2, 2, 20, 18]}
drawn = gb_stream.overlay_jpeg(JPEG, fresh["box"])
check("a fresh box is still a jpeg", drawn[:2] == b"\xff\xd8" and drawn[-2:] == b"\xff\xd9")
again = cv2.imdecode(np.frombuffer(drawn, np.uint8), cv2.IMREAD_COLOR)
check("drawn jpeg decodes and is not a copied fixture", again is not None and drawn != JPEG)

server = gb_stream.StreamServer("127.0.0.1", 18083)
threading.Thread(target=server.serve_forever, daemon=True).start()
time.sleep(0.05)
server.slot.offer(time.time(), JPEG)
server.track = {"t": time.time() - 5, "box": [1, 1, 8, 8]}
import socket
sock = socket.create_connection(("127.0.0.1", 18083), 1)
sock.sendall(b"GET /stream.mjpg HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n")
data = b""
deadline = time.time() + 1
while time.time() < deadline and b"\xff\xd8" not in data:
    data += sock.recv(4096)
sock.close()
check("served part is image/jpeg", b"Content-Type: image/jpeg" in data and b"\xff\xd8" in data, data[:80])
time.sleep(0.5)
check("client disconnect releases the slot", server.clients == 0, server.clients)
server.httpd.shutdown()

print("test_stream_jpeg: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
