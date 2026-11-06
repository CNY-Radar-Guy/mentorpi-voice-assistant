#!/bin/bash
# Undo a Wally code copy. Camera units are disabled even when the restore fails.
# Does not stop grokbot, the motion gate, Docker, or web_video_server.
# Does not delete data/ESTOP.
# Usage: wally_rollback.sh /home/pi/grokbot/backup-wally-YYYYMMDD-HHMMSS
# errexit is intentionally off. A wake-backup lookup must not skip the camera-unit disable.
set -uo pipefail
BACK=${1:-}
DEST=${WALLY_DEST:-$HOME/grokbot}
ROOT=$(cd "$(dirname "$0")" && pwd)
rc=0

disable_cameras() {
  if command -v systemctl >/dev/null 2>&1; then
    sudo systemctl disable --now gb-front gb-stream >/dev/null 2>&1 || true
  fi
  echo "gb-front and gb-stream disabled. web_video_server was not stopped."
}

# First, so a later restore or backup-lookup error cannot skip it.
disable_cameras

if [ -z "$BACK" ] || [ ! -d "$BACK" ]; then
  echo "usage: $0 <backup-wally directory>" >&2
  exit 2
fi

if ! tar -C "$BACK" -cf - . | tar -C "$DEST" -xf -; then
  echo "restore failed from $BACK" >&2
  rc=1
else
  echo "code restored from $BACK"
fi

# Apply-specific pointer, else newest bak-wake file, else a no-match note.
# Do not test those names with a shell glob. More than one match aborts when errexit is on.
if ! python3 "$ROOT/wake_backup.py" "$DEST"; then
  echo "wake backup lookup failed; camera units were already disabled" >&2
  rc=1
else
  echo "Restore a wake backup by hand if you want the previous WAKE_* lines. This script does not copy it over grokbot.env."
fi

echo "data/ESTOP was not removed."
exit "$rc"
