#!/bin/bash
# Reviewed safety activation for the Pi. Default is a printed plan and no service changes.
# The cloud agent must not run this with WALLY_ACTIVATE=1.
#
# WALLY_ACTIVATE=1, on the Pi, as user pi:
#   * refuses unless live .Mounts show /home/pi/docker/tmp -> /home/ubuntu/shared
#   * writes a maintenance lock and does not create or delete data/ESTOP
#   * publishes zero Twist, then restarts only gb-gate and gb-watchdog
#   * leaves grokbot running so spoken stop and wake stay up
#   * does not move web_video_server and does not clear the maintenance lock
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST=${WALLY_DEST:-$HOME/grokbot}
APPLY=${WALLY_ACTIVATE:-0}
C=MentorPi

python3 "$ROOT/deploy/activate_plan.py" plan
echo "Phone URL http://192.168.50.125:8080/ is unmet. web_video_server is not moved."
echo "Optional overlay on 8081 does not fulfill that phone root."

if [ "$APPLY" != "1" ]; then
  echo "dry plan only. On the Pi, after you have read this script: WALLY_ACTIVATE=1 $0"
  echo "This run did not restart services and did not play audio."
  exit 0
fi

MOUNTS_JSON=$(mktemp)
INFO_FILE=$(mktemp)
cleanup() { rm -f "$MOUNTS_JSON" "$INFO_FILE"; }
trap cleanup EXIT

if ! docker inspect -f '{{json .Mounts}}' "$C" > "$MOUNTS_JSON" 2>/dev/null; then
  echo "[]" > "$MOUNTS_JSON"
fi
if ! docker exec "$C" cat /proc/1/mountinfo > "$INFO_FILE" 2>/dev/null; then
  : > "$INFO_FILE"
fi
HI=$(stat -c %i /home/pi/docker/tmp 2>/dev/null || true)
CI=$(docker exec "$C" stat -c %i /home/ubuntu/shared 2>/dev/null || true)
INODE_FLAGS=""
if [ -n "$HI" ] && [ -n "$CI" ]; then
  INODE_FLAGS="--inodes ${HI},${CI}"
fi
if ! python3 "$ROOT/deploy/mount_check.py" --inspect "$MOUNTS_JSON" --mountinfo "$INFO_FILE" $INODE_FLAGS; then
  echo "activation refused. Live mount does not match /home/pi/docker/tmp -> /home/ubuntu/shared. No service was restarted."
  exit 3
fi

# Zero hold starts before the gate is stopped. It only publishes a zero Twist.
mkdir -p /home/pi/docker/tmp/gb_safety
if ! cp -f "$ROOT/deploy/zero_hold.py" /home/pi/docker/tmp/gb_safety/zero_hold.py; then
  echo "activation refused. Could not stage the zero publisher on the verified mount. No safety unit was restarted."
  exit 4
fi
if ! docker exec -d -u ubuntu "$C" bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /home/ubuntu/shared/gb_safety/zero_hold.py --seconds 4'; then
  echo "activation refused. Zero publisher did not start. No safety unit was restarted."
  exit 4
fi

HOST_ESTOP="$DEST/data/ESTOP"
if [ -e "$HOST_ESTOP" ]; then
  echo "existing ESTOP left untouched. This script does not clear a user stop."
fi
python3 "$ROOT/deploy/activate_plan.py" maint "$DEST/data/MAINT" /home/pi/docker/tmp/gb_safety/MAINT
if [ ! -e "$DEST/data/MAINT" ]; then
  echo "activation refused. Maintenance lock was not written. No unit was restarted."
  exit 4
fi
echo "maintenance lock is present. grokbot is not stopped."

date +%s > "$DEST/data/ACTIVATION"
if command -v systemctl >/dev/null 2>&1; then
  start_ns=$(date +%s%N)
  sudo systemctl restart gb-gate gb-watchdog
  ok=0
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if docker exec "$C" pgrep -f /home/ubuntu/shared/gb_safety/gb_gate.py >/dev/null 2>&1 \
       && docker exec "$C" pgrep -f /home/ubuntu/shared/gb_safety/gb_watchdog.py >/dev/null 2>&1; then
      ok=1
      break
    fi
    now_ns=$(date +%s%N)
    elapsed=$(python3 -c "print((${now_ns} - ${start_ns}) / 1e9)")
    python3 -c "import sys; sys.exit(0 if float('$elapsed') <= 2.0 else 1)" || break
    sleep 0.2
  done
  end_ns=$(date +%s%N)
  elapsed=$(python3 -c "print((${end_ns} - ${start_ns}) / 1e9)")
  if [ "$ok" != 1 ] || ! python3 -c "import sys; sys.exit(0 if float('$elapsed') <= 2.0 else 1)"; then
    echo "gate/watchdog were not both up within 2 seconds (elapsed ${elapsed}s). ESTOP was not cleared. The maintenance lock was not cleared. grokbot was left running."
    exit 5
  fi
  echo "gate and watchdog python processes were visible within ${elapsed}s. grokbot was not stopped."
else
  echo "systemctl is not available. ESTOP was not cleared. The maintenance lock was not cleared."
  exit 5
fi
if [ ! -e "$DEST/data/MAINT" ]; then
  echo "maintenance lock disappeared during restart. Not clearing ESTOP."
  exit 5
fi
echo "Voice reload is not part of this command. Spoken stop stays with the running grokbot service."
echo "A later reload requires SUPPRESS_STARTUP_SPEECH=1 and ESTOP_AUTOCLEAR_S=0. Empty greeting fields are not that switch."
echo "Clear the maintenance lock only after checks, with deploy/maint_clear.sh. This script does not clear it."
echo "No follow, no readiness phrase, no port 8080 change."
exit 0
