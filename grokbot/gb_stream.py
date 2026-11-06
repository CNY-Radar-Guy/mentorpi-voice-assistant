#!/usr/bin/env python3
"""gb_stream.py - LAN MJPEG overlay, off the speech and control threads.

Port 8080 is the stock snapshot server (gb_robot.SNAPSHOT_URL, web_video_server
/snapshot). This process does not bind 8080 when something is already listening,
does not kill that server, and exits 0 if its own port is taken so systemd does
not restart-loop. Default overlay port is 8081.

Frames are the latest copy only. A slow client drops frames. Person boxes are
drawn on a copy, and only when the box timestamp matches that frame. No second
detector and no camera device open: the caller pushes frames from the existing
ROS image subscription.
"""
import json, os, socket, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SNAPSHOT_PORT = 8080
DEFAULT_PORT = 8081
MAX_CLIENTS = 2
MATCH_S = 0.35
STALE_S = 0.50

def listener_on(host, port, timeout=0.2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((host, int(port))) == 0
    except OSError:
        return False
    finally:
        s.close()


def lan_host(host):
    """Loopback or an RFC1918 address. 0.0.0.0 would publish the camera on every interface."""
    host = (host or "").strip()
    if host in ("127.0.0.1", "::1"):
        return True
    parts = host.split(".")
    if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
        return False
    a, b = int(parts[0]), int(parts[1])
    return a == 10 or (a == 172 and 16 <= b <= 31) or (a == 192 and b == 168)


def choose_bind(port, host="127.0.0.1"):
    """(addr, status). None addr means do not bind and do not retry in a loop."""
    port = int(port)
    if port == SNAPSHOT_PORT:
        return None, {"state": "blocked", "reason": "port 8080 stays with the stock camera server",
                      "snapshot_port_untouched": SNAPSHOT_PORT, "overlay_port": None}
    if not lan_host(host):
        return None, {"state": "blocked", "reason": "bind %s is not a LAN address" % host, "overlay_port": None}
    if listener_on(host, port) or listener_on("127.0.0.1", port):
        return None, {"state": "blocked", "reason": "port %s is in use" % port, "overlay_port": None}
    return (host, port), {"state": "ready", "overlay_port": port, "snapshot_port_untouched": SNAPSHOT_PORT,
                          "readonly": True}


class LatestSlot(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._item = None
        self.dropped = 0
        self.accepted = 0

    def offer(self, t, frame):
        """Overwrite the one slot. If the consumer holds the lock, drop this frame."""
        if not self._lock.acquire(blocking=False):
            self.dropped += 1
            return False
        try:
            self.accepted += 1
            self._item = (float(t), self.accepted, frame)
            return True
        finally:
            self._lock.release()

    def latest(self):
        if not self._lock.acquire(blocking=False):
            return None
        try:
            return self._item
        finally:
            self._lock.release()


VISUAL_MAX_S = 3.0


def visual_mark(track, now):
    """Display-only box. An age past the motion limit is labeled, and it is not a motion track."""
    if not track:
        return None
    try:
        tt = float(track.get("t"))
        box = list(track.get("box"))
    except (TypeError, ValueError, AttributeError):
        return None
    if len(box) != 4:
        return None
    age = float(now) - tt
    if age < -1.0:
        return None
    try:
        infer = float(track.get("infer_s") or 0.0)
    except (TypeError, ValueError):
        infer = 0.0
    limit = min(VISUAL_MAX_S, max(STALE_S, infer * 2.0 + 0.3))
    if age > limit:
        return None
    label = "person"
    conf = track.get("conf")
    if conf is not None:
        try:
            label += " %.2f" % float(conf)
        except (TypeError, ValueError):
            pass
    aged = age > STALE_S
    if aged:
        label += " aged %.1fs" % age
    return {"box": box, "label": label, "aged": aged, "for_motion": False, "age_s": age}


def matching_box(frame_t, track, now):
    """Box from the same detection instant, or None if it would be a stale track."""
    if not track:
        return None
    try:
        tt = float(track.get("t"))
        box = list(track.get("box"))
    except (TypeError, ValueError, AttributeError):
        return None
    if len(box) != 4:
        return None
    if abs(float(frame_t) - tt) > MATCH_S or (now - tt) > STALE_S:
        return None
    return box


def overlay_jpeg(jpeg, box, label=None):
    """Draw on a copy of a JPEG. No box, or no OpenCV, returns the original bytes unchanged."""
    if not isinstance(jpeg, (bytes, bytearray)) or not box:
        return jpeg
    try:
        import cv2
        import numpy as np
    except Exception:
        return bytes(jpeg)
    arr = np.frombuffer(bytes(jpeg), dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return bytes(jpeg)
    h, w = img.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, min(w - 1, x1)); x2 = max(0, min(w - 1, x2))
    y1 = max(0, min(h - 1, y1)); y2 = max(0, min(h - 1, y2))
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    if label:
        cv2.putText(img, str(label)[:48], (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
    ok, out = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return out.tobytes() if ok else bytes(jpeg)


def draw_box(frame, box):
    """Return a new grid. The caller's frame is not modified."""
    out = [row[:] for row in frame]
    h = len(out)
    w = len(out[0]) if h else 0
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, min(w - 1, x1)); x2 = max(0, min(w, x2))
    y1 = max(0, min(h - 1, y1)); y2 = max(0, min(h, y2))
    for x in range(x1, x2):
        out[y1][x] = 1
        if y2 - 1 > y1:
            out[y2 - 1][x] = 1
    for y in range(y1, y2):
        out[y][x1] = 1
        if x2 - 1 > x1:
            out[y][x2 - 1] = 1
    return out


class Quality(object):
    """Hysteresis on normalized CPU (0-100 across all cores) and control latency."""
    def __init__(self):
        self.fps = 15
        self.edge = 480
        self.paused = False
        self._hot = 0
        self._cool = 0

    def update(self, sys_cpu, proc_cpu_of_machine, control_ms):
        severe = sys_cpu >= 90 or (control_ms is not None and control_ms >= 80)
        hot = sys_cpu >= 75 or proc_cpu_of_machine >= 40 or (control_ms is not None and control_ms >= 40)
        cool = sys_cpu <= 50 and proc_cpu_of_machine <= 20 and (control_ms is None or control_ms <= 20)
        if severe:
            self.paused = True
            self._hot = 0
            self._cool = 0
            return self
        if hot:
            self._hot += 1
            self._cool = 0
            if self._hot >= 3:
                self._step_down()
                self._hot = 0
        elif cool:
            self._cool += 1
            self._hot = 0
            if self._cool >= 5:
                if self.paused:
                    self.paused = False
                else:
                    self._step_up()
                self._cool = 0
        else:
            self._hot = 0
            self._cool = 0
        return self

    def _step_down(self):
        if self.fps > 10:
            self.fps = 10
        elif self.edge > 360:
            self.edge = 360
        else:
            self.paused = True

    def _step_up(self):
        if self.edge < 480:
            self.edge = 480
        elif self.fps < 15:
            self.fps = 15


def normalize_proc_cpu(proc_percent, cpu_count):
    """psutil.Process.cpu_percent is 0-100*ncpu. Return a share of the machine, 0-100."""
    n = max(1, int(cpu_count or 1))
    return float(proc_percent) / n


class StreamServer(object):
    def __init__(self, host, port):
        self.host = host
        self.port = int(port)
        self.slot = LatestSlot()
        self.quality = Quality()
        self.clients = 0
        self._lock = threading.Lock()
        self.track = None
        self.httpd = None

    def metrics(self):
        return {"clients": self.clients, "dropped": self.slot.dropped, "accepted": self.slot.accepted,
                "fps": self.quality.fps, "edge": self.quality.edge, "paused": self.quality.paused,
                "port": self.port}

    def serve_forever(self):
        outer = self
        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            def log_message(self, fmt, *args):
                return
            def do_POST(self):
                self.send_error(405)
            def do_PUT(self):
                self.send_error(405)
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/health":
                    body = json.dumps(outer.metrics()).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path not in ("/", "/stream.mjpg"):
                    self.send_error(404)
                    return
                with outer._lock:
                    if outer.clients >= MAX_CLIENTS:
                        self.send_error(503)
                        return
                    outer.clients += 1
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    seen = -1
                    self.connection.settimeout(0.3)
                    while True:
                        item = outer.slot.latest()
                        if outer.quality.paused or item is None or item[1] == seen:
                            try:
                                peek = self.connection.recv(1, socket.MSG_PEEK)
                            except (BlockingIOError, TimeoutError):
                                peek = b"x"
                            except OSError:
                                break
                            if peek == b"":
                                break
                            time.sleep(0.02)
                            continue
                        seen = item[1]
                        frame = item[2]
                        mark = outer.track if isinstance(outer.track, dict) else None
                        if mark and mark.get("for_motion") is not False and mark.get("box"):
                            mark = visual_mark(mark, time.time())
                        box = mark.get("box") if mark else None
                        label = mark.get("label") if mark else None
                        if isinstance(frame, (bytes, bytearray)):
                            shown = overlay_jpeg(frame, box, label)
                            payload = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(shown)) + shown + b"\r\n"
                        else:
                            shown = draw_box(frame, box) if box else frame
                            payload = ("--frame\r\nContent-Type: text/plain\r\n\r\n%d\r\n" % seen).encode()
                        try:
                            self.wfile.write(payload)
                            self.wfile.flush()
                        except (TimeoutError, OSError, BrokenPipeError):
                            break
                        time.sleep(1.0 / max(1, outer.quality.fps))
                finally:
                    with outer._lock:
                        outer.clients = max(0, outer.clients - 1)
        self.httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self.httpd.serve_forever()


STOCK_MJPEG_URL = (
    "http://127.0.0.1:8080/stream?topic=/ascamera/camera_publisher/rgb0/image"
)
MAX_JPEG_BUFFER = 1_500_000


def venv_preflight():
    """Names of missing host libraries. rclpy is not required."""
    missing = []
    try:
        import cv2  # noqa: F401
    except Exception:
        missing.append("cv2")
    try:
        import numpy  # noqa: F401
    except Exception:
        missing.append("numpy")
    return missing


def extract_jpegs(buf, limit=MAX_JPEG_BUFFER):
    """Pull complete JPEG frames out of an MJPEG byte stream. Returns (frames, remainder)."""
    if len(buf) > limit:
        buf = buf[-limit:]
    frames = []
    while True:
        start = buf.find(b"\xff\xd8")
        if start < 0:
            return frames, buf[-1:]
        end = buf.find(b"\xff\xd9", start + 2)
        if end < 0:
            return frames, buf[start:]
        frames.append(buf[start:end + 2])
        buf = buf[end + 2:]


def read_detector_track(path, now):
    """A person box from the running detector, or None when it is missing or stale."""
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    person = body.get("person") if isinstance(body, dict) else None
    if not isinstance(person, dict):
        return None
    try:
        stamp = float(person.get("t") if person.get("t") is not None else body.get("t"))
        box = list(person.get("box"))
    except (TypeError, ValueError, AttributeError):
        return None
    if len(box) != 4 or now - stamp > STALE_S or stamp - now > 1.0:
        return None
    return {"t": stamp, "box": box, "for_motion": True}


def read_visual_track(path, now):
    """Overlay track. No person is not a detector failure. Motion code must not call this."""
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(body, dict):
        return None
    person = body.get("person")
    if not isinstance(person, dict):
        if body.get("detect_ok"):
            return {"box": None, "no_person": True, "for_motion": False}
        return None
    mark = visual_mark({
        "t": person.get("t", body.get("t")),
        "box": person.get("box"),
        "conf": person.get("conf"),
        "infer_s": body.get("infer_s"),
    }, now)
    if mark is None and body.get("detect_ok"):
        return {"box": None, "no_person": True, "for_motion": False}
    return mark


class StockMjpeg(object):
    """Read the stock web_video_server MJPEG URL. Does not bind port 8080."""

    def __init__(self, url, slot, track_path="", demand_path="", viewers=None):
        self.url = url
        self.slot = slot
        self.track_path = track_path
        self.demand_path = demand_path
        self.viewers = viewers if viewers is not None else (lambda: 0)
        self.stop = threading.Event()
        self.thread = None
        self.frames = 0
        self.errors = 0
        self._sock = None
        self._demand_at = 0.0

    def start(self):
        self.thread = threading.Thread(target=self._run, name="stock-mjpeg", daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.thread = None
        self._resp = None
        self._release_view()

    def _release_view(self):
        if not self.demand_path:
            return
        try:
            import gb_demand
            if gb_demand.read_demand(self.demand_path) == "view":
                gb_demand.write_demand(self.demand_path, "idle")
        except Exception:
            pass

    def _note_demand(self):
        if not self.demand_path or self.viewers() <= 0:
            self._release_view()
            return
        if time.time() - self._demand_at < 1.0:
            return
        try:
            import gb_demand
            if gb_demand.read_demand(self.demand_path) in ("follow", "point"):
                return
            gb_demand.write_demand(self.demand_path, "view")
            self._demand_at = time.time()
        except Exception:
            pass

    def _connect(self):
        import socket
        from urllib.parse import urlsplit
        parts = urlsplit(self.url)
        host = parts.hostname or "127.0.0.1"
        port = parts.port or 80
        path = parts.path or "/"
        if parts.query:
            path += "?" + parts.query
        sock = socket.create_connection((host, port), 5)
        sock.settimeout(1.0)
        sock.sendall(("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\n\r\n" % (path, host)).encode())
        self._sock = sock
        return sock

    def _run(self):
        while not self.stop.is_set():
            if self.viewers() <= 0:
                self._release_view()
                time.sleep(0.1)
                continue
            sock = None
            try:
                sock = self._connect()
                buf = b""
                header = True
                while not self.stop.is_set():
                    if self.viewers() <= 0:
                        self._release_view()
                        break
                    try:
                        chunk = sock.recv(8192)
                    except TimeoutError:
                        continue
                    if not chunk:
                        break
                    buf += chunk
                    if header:
                        cut = buf.find(b"\r\n\r\n")
                        if cut < 0:
                            continue
                        buf = buf[cut + 4:]
                        header = False
                    frames, buf = extract_jpegs(buf)
                    now = time.time()
                    for jpeg in frames:
                        self.slot.offer(now, jpeg)
                        self.frames += 1
                    if frames:
                        self._note_demand()
            except Exception:
                if self.stop.is_set():
                    break
                self.errors += 1
                time.sleep(0.5)
            finally:
                self._sock = None
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass


def main():
    missing = venv_preflight()
    if missing and os.environ.get("STREAM_REQUIRE_CV", "1") == "1":
        print("stream preflight missing: %s" % ", ".join(missing))
        return 1
    host = os.environ.get("STREAM_BIND", "192.168.50.125")
    port = int(os.environ.get("STREAM_PORT", str(DEFAULT_PORT)))
    addr, status = choose_bind(port, host)
    path = os.path.expanduser(os.environ.get("STREAM_STATUS", "/home/pi/grokbot/data/stream_status.json"))
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(status, f)
    except OSError:
        pass
    if addr is None:
        print("stream blocked:", status["reason"])
        return 0
    server = StreamServer(addr[0], addr[1])
    url = os.environ.get("STOCK_MJPEG_URL", STOCK_MJPEG_URL)
    track = os.environ.get("GB_RGB_FILE", "/home/pi/docker/tmp/gb_safety/rgb_guard.json")
    demand = os.environ.get("GB_DEMAND_FILE", "/home/pi/docker/tmp/gb_safety/demand.json")
    feed = StockMjpeg(url, server.slot, track, demand, viewers=lambda: server.clients)

    def _track_loop():
        while not feed.stop.is_set():
            server.track = read_visual_track(track, time.time())
            time.sleep(0.05)

    threading.Thread(target=_track_loop, name="stream-track", daemon=True).start()
    feed.start()
    print("overlay on %s:%s reading %s; port %s left alone" % (addr[0], addr[1], url, SNAPSHOT_PORT))
    try:
        server.serve_forever()
    finally:
        feed.close()
        if server.httpd is not None:
            server.httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
