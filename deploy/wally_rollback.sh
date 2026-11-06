#!/bin/bash
# Undo a Wally code copy and stop only the optional camera units.
# Does not stop grokbot, the motion gate, Docker, or web_video_server.
# Usage: wally_rollback.sh /home/pi/grokbot/backup-wally-YYYYMMDD-HHMMSS
set -euo pipefail
BACK=${1:-}
DEST=${WALLY_DEST:-$HOME/grokbot}
if [ -z "$BACK" ] || [ ! -d "$BACK" ]; then
  echo "usage: $0 <backup-wally directory>" >&2
  exit 2
fi
tar -C "$BACK" -cf - . | tar -C "$DEST" -xf -
if [ -f "$DEST/grokbot.env.bak-wake-"* ]; then
  echo "A wake-field backup is next to grokbot.env. Restore that file by hand if you want the previous WAKE_* lines."
fi
if command -v systemctl >/dev/null 2>&1; then
  sudo systemctl disable --now gb-front gb-stream 2>/dev/null || true
fi
echo "code restored from $BACK"
echo "gb-front and gb-stream disabled. web_video_server was not stopped."
