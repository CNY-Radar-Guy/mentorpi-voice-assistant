#!/usr/bin/env python3
"""gb_front.py - optional read-only router for phone port 8080.

Stock web_video_server is what gb_robot.snapshot() calls:
  http://127.0.0.1:8080/snapshot?topic=<ros topic>
and it must return a JPEG (bytes start with FFD8). Grok vision and label capture
use that. This process never kills that server.

Phone pages are served by the low-priority overlay on 127.0.0.1:8081.
This router, if and only if preflight says it is safe, binds 8080 and forwards:
  /snapshot, /stream, /stream_viewer  -> the snapshot upstream (default :8082)
  /, /overlay.mjpg, /stream.mjpg, /health -> the overlay on :8081

Preflight fails closed:
  * something on 8080 already answers /snapshot: do not bind. Phone URL is unmet.
  * 8080 is free but the snapshot upstream does not return a JPEG: do not bind,
    or vision would lose its only snapshot port.
The process then writes a status file and exits 0. The unit is Restart=no.
"""
import json, os, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

SNAPSHOT_PORT = 8080
OVERLAY_PORT = 8081
UPSTREAM_PORT = 8082
SNAPSHOT_PREFIXES = ("/snapshot", "/stream_viewer")
OVERLAY_PATHS = {"/", "/overlay.mjpg", "/stream.mjpg", "/health"}


def route(path):
    """Where a request must go. 'snapshot' preserves vision. 'overlay' is the phone page."""
    bare = (path or "/").split("?", 1)[0]
    if bare in OVERLAY_PATHS:
        return "overlay"
    if bare == "/stream" or bare.startswith("/snapshot") or bare.startswith("/stream_viewer"):
        return "snapshot"
    if bare.startswith("/stream"):
        return "snapshot"
    return "reject"


def preflight_decision(port_8080_bound, snapshot_without_topic_empty=False, upstream_snapshot_jpeg=False):
    """Pure decision from a preflight sample. Never kills a process.

    An empty GET /snapshot with no topic query is not evidence that the
    endpoint is missing. gb_robot.snapshot() always sends ?topic=.
    No TCP clients at the sample is not evidence that nothing uses the port.
    """
    del snapshot_without_topic_empty  # recorded by the caller, not a reason to take the port
    if port_8080_bound:
        return {
            "state": "blocked",
            "enable_front": False,
            "enable_stream": False,
            "phone_url_8080": False,
            "unmet_8080": True,
            "reason": "8080 unmet. Stock web_video_server (MentorPi container, host network, 0.0.0.0:8080) was left running. The phone page was not switched onto that port. The overlay stays inactive.",
        }
    if not upstream_snapshot_jpeg:
        return {
            "state": "blocked",
            "enable_front": False,
            "enable_stream": False,
            "phone_url_8080": False,
            "unmet_8080": True,
            "reason": "8080 is not taken, but no JPEG snapshot upstream was confirmed. The proxy stays off so vision still has a real /snapshot?topic= server before anything binds 8080.",
        }
    return {
        "state": "ready_but_inactive",
        "enable_front": False,
        "enable_stream": False,
        "phone_url_8080": False,
        "unmet_8080": False,
        "reason": "A snapshot upstream answered with a JPEG and 8080 was free at preflight. Units still stay stopped until STREAM_FRONT_ENABLE=1. This tree does not move web_video_server.",
    }


def looks_like_jpeg(data):
    return isinstance(data, (bytes, bytearray)) and len(data) >= 2 and data[:2] == b"\xff\xd8"


def probe_snapshot(url, timeout=1.5):
    """Return True only when the URL returns JPEG bytes. Never raises."""
    try:
        req = Request(url, method="GET")
        with urlopen(req, timeout=timeout) as r:
            return looks_like_jpeg(r.read(16))
    except (URLError, HTTPError, TimeoutError, OSError, ValueError):
        return False


def port_bound(host="127.0.0.1", port=SNAPSHOT_PORT):
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.3)
    try:
        return s.connect_ex((host, int(port))) == 0
    except OSError:
        return False
    finally:
        s.close()


def live_preflight(upstream_url=None):
    """Decide from the live port. Never requests /snapshot without a topic query."""
    up = upstream_url or os.environ.get(
        "SNAPSHOT_UPSTREAM",
        "http://127.0.0.1:8082/snapshot?topic=/ascamera/camera_publisher/rgb0/image")
    return preflight_decision(port_bound(), snapshot_without_topic_empty=False,
                              upstream_snapshot_jpeg=probe_snapshot(up))


class Front(object):
    def __init__(self, overlay="http://127.0.0.1:8081", upstream="http://127.0.0.1:8082"):
        self.overlay = overlay.rstrip("/")
        self.upstream = upstream.rstrip("/")

    def target(self, path_qs):
        kind = route(path_qs)
        if kind == "overlay":
            return self.overlay + path_qs
        if kind == "snapshot":
            return self.upstream + path_qs
        return None


def serve(host, port, front):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        def log_message(self, fmt, *args):
            return
        def do_POST(self):
            self.send_error(405)
        def do_GET(self):
            url = front.target(self.path)
            if url is None:
                self.send_error(404)
                return
            try:
                with urlopen(url, timeout=3) as r:
                    body = r.read(2_000_000)
                    status = getattr(r, "status", 200)
                    ctype = r.headers.get("Content-Type", "application/octet-stream")
            except (URLError, HTTPError, TimeoutError, OSError):
                self.send_error(502)
                return
            if route(self.path) == "snapshot" and not looks_like_jpeg(body):
                self.send_error(502)
                return
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    httpd = ThreadingHTTPServer((host, int(port)), Handler)
    httpd.serve_forever()


def write_status(status):
    path = os.path.expanduser(os.environ.get("STREAM_STATUS", "~/grokbot/data/stream_status.json"))
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(status, f)
    except OSError:
        pass


def main():
    decision = live_preflight()
    write_status(decision)
    print(decision["state"] + ": " + decision["reason"])
    if os.environ.get("STREAM_FRONT_ENABLE", "0") != "1" or decision["state"] != "ready_but_inactive":
        return 0
    host = os.environ.get("STREAM_BIND", "192.168.50.125")
    front = Front(os.environ.get("OVERLAY_UPSTREAM", "http://127.0.0.1:8081"),
                  os.environ.get("SNAPSHOT_UPSTREAM_ORIGIN", "http://127.0.0.1:8082"))
    print("front proxy on %s:%s" % (host, SNAPSHOT_PORT))
    serve(host, SNAPSHOT_PORT, front)
    return 0


if __name__ == "__main__":
    sys.exit(main())
