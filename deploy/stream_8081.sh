#!/bin/bash
# Overlay on 8081. Does not stop the stock server on 8080 and does not restart
# the gate, watchdog, or voice. Default is a preflight.
#   deploy/stream_8081.sh
#   STREAM_8081_START=1 deploy/stream_8081.sh start
#   deploy/stream_8081.sh stop
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${WALLY_PYTHON:-/home/pi/grokbot/venv/bin/python}
if [ ! -x "$PY" ]; then PY=python3; fi
cmd=${1:-preflight}
echo "stock camera stays at http://127.0.0.1:8080/stream?topic=/ascamera/camera_publisher/rgb0/image"
echo "overlay, when started, is http://192.168.50.125:8081/stream.mjpg"
"$PY" - << 'PY'
import sys
missing = []
for name in ("cv2", "numpy"):
    try:
        __import__(name)
    except Exception:
        missing.append(name)
if missing:
    print("preflight missing: %s" % ", ".join(missing))
    sys.exit(1)
print("cv2 and numpy import")
PY
rc=$?
if [ "$rc" != 0 ]; then
  echo "do not start the overlay until the venv has opencv and numpy"
  exit "$rc"
fi
if [ "$cmd" = "preflight" ]; then
  echo "preflight only. Port 8080 was not touched. Autonomy was not enabled."
  exit 0
fi
if [ "$cmd" = "stop" ]; then
  if command -v systemctl >/dev/null 2>&1; then
    sudo systemctl stop gb-stream || true
  fi
  echo "gb-stream stop requested. Gate, watchdog, and voice were not restarted."
  exit 0
fi
if [ "$cmd" = "start" ]; then
  if [ "${STREAM_8081_START:-0}" != "1" ]; then
    echo "refusing to start. Re-run with STREAM_8081_START=1 after this preflight."
    exit 2
  fi
  if command -v systemctl >/dev/null 2>&1; then
    sudo systemctl daemon-reload
    sudo systemctl start gb-stream
    systemctl is-active gb-stream || true
  else
    echo "systemctl is not on this machine. The unit was not started."
    exit 1
  fi
  echo "metrics: curl -fsS http://192.168.50.125:8081/health"
  echo "gate, watchdog, voice, and the stock camera were not restarted."
  exit 0
fi
echo "usage: $0 [preflight|start|stop]"
exit 2
