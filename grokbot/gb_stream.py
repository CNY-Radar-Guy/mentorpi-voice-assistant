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


def choose_bind(port, host="127.0.0.1"):
    """(addr, status). None addr means do not bind and do not retry in a loop."""
    port = int(port)
    if port == SNAPSHOT_PORT and (listener_on("127.0.0.1", SNAPSHOT_PORT) or listener_on(host, SNAPSHOT_PORT)):
        return None, {"state": "blocked", "reason": "port 8080 is in use; left for the snapshot server",
                      "snapshot": "http://127.0.0.1:8080/snapshot", "overlay_port": None}
    if listener_on(host, port):
        return None, {"state": "blocked", "reason": "port %s is in use" % port, "overlay_port": None}
    return (host, port), {"state": "ready", "overlay_port": port, "snapshot_port_untouched": SNAPSHOT_PORT}


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
                            time.sleep(0.02)
                            continue
                        seen = item[1]
                        frame = item[2]
                        box = matching_box(item[0], outer.track, time.time())
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


def main():
    host = os.environ.get("STREAM_BIND", "127.0.0.1")
    port = int(os.environ.get("STREAM_PORT", str(DEFAULT_PORT)))
    addr, status = choose_bind(port, host)
    path = os.path.expanduser(os.environ.get("STREAM_STATUS", "~/grokbot/data/stream_status.json"))
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(status, f)
    except OSError:
        pass
    if addr is None:
        print("stream blocked:", status["reason"])
        return 0
    print("overlay on %s:%s; snapshot port %s left alone" % (addr[0], addr[1], SNAPSHOT_PORT))
    StreamServer(addr[0], addr[1]).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
