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
if ! docker exec -d -u ubuntu "$C" bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /home/ubuntu/shared/gb_safety/zero_hold.py --seconds 25'; then
  echo "activation refused. Zero publisher did not start. No safety unit was restarted."
  exit 4
fi
REL=${WALLY_RELEASE:-}
if [ -z "$REL" ] || [ ! -d "$DEST/releases/$REL" ]; then
  echo "activation refused. Set WALLY_RELEASE to a staged release. No unit was restarted."
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
install_dropin() {
  local unit="$1" text="$2"
  sudo mkdir -p "/etc/systemd/system/${unit}.service.d"
  printf '%s\n' "$text" | sudo tee "/etc/systemd/system/${unit}.service.d/release.conf" >/dev/null
}
if command -v systemctl >/dev/null 2>&1; then
  GATE_DROP=$(python3 -c "import sys; sys.path.insert(0,'$ROOT'); from deploy.release_layout import dropin_text; print(dropin_text('gate','$DEST/releases/$REL'))")
  WD_DROP=$(python3 -c "import sys; sys.path.insert(0,'$ROOT'); from deploy.release_layout import dropin_text; print(dropin_text('watchdog','$DEST/releases/$REL'))")
  install_dropin gb-gate "$GATE_DROP"
  install_dropin gb-watchdog "$WD_DROP"
  sudo systemctl daemon-reload
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
    echo "gate/watchdog were not both up within 2 seconds (elapsed ${elapsed}s)."
    sudo rm -f /etc/systemd/system/gb-gate.service.d/release.conf /etc/systemd/system/gb-watchdog.service.d/release.conf
    sudo systemctl daemon-reload
    sudo systemctl restart gb-gate gb-watchdog
    echo "rolled back the gate and watchdog drop-ins. ESTOP and the maintenance lock were not removed. grokbot was not stopped."
    exit 5
  fi
  echo "gate and watchdog python processes were visible within ${elapsed}s."
else
  echo "systemctl is not available. ESTOP was not cleared. The maintenance lock was not cleared."
  exit 5
fi
if [ ! -e "$DEST/data/MAINT" ]; then
  echo "maintenance lock disappeared during restart. Not clearing ESTOP."
  exit 5
fi
if [ -f "$DEST/grokbot.env" ]; then
  python3 "$ROOT/deploy/migrate_wake_env.py" "$DEST/grokbot.env"
  python3 "$ROOT/deploy/migrate_autoclear.py" "$DEST/grokbot.env"
  if ! grep -q '^SUPPRESS_STARTUP_SPEECH=1$' "$DEST/grokbot.env"; then
    printf '\nSUPPRESS_STARTUP_SPEECH=1\n' >> "$DEST/grokbot.env"
  fi
fi
if ! python3 -c "import sys; sys.path.insert(0,'$ROOT'); from deploy.release_layout import voice_restart_env_ok; import pathlib; sys.exit(0 if voice_restart_env_ok(pathlib.Path('$DEST/grokbot.env').read_text() if pathlib.Path('$DEST/grokbot.env').is_file() else 'SUPPRESS_STARTUP_SPEECH=1\nESTOP_AUTOCLEAR_S=0\nWAKE_WORDS=wake up wally\n') else 1)"; then
  echo "voice restart refused. Stop listener stays on the running process. ESTOP and the maintenance lock were not removed."
  exit 5
fi
VOICE_DROP=$(python3 -c "import sys; sys.path.insert(0,'$ROOT'); from deploy.release_layout import dropin_text; print(dropin_text('voice','$DEST/releases/$REL'))")
install_dropin grokbot "$VOICE_DROP"
sudo systemctl daemon-reload
sudo systemctl restart grokbot
voice_ok=0
for _ in $(seq 1 40); do
  if systemctl is-active --quiet grokbot; then voice_ok=1; break; fi
  sleep 0.5
done
if [ "$voice_ok" != 1 ]; then
  sudo rm -f /etc/systemd/system/grokbot.service.d/release.conf
  sudo systemctl daemon-reload
  sudo systemctl restart grokbot
  echo "voice did not return. The previous unit was restarted. ESTOP and the maintenance lock were not removed."
  exit 5
fi
echo "voice is active again. Startup speech is suppressed. A normal stop is not cleared by this restart."
echo "The maintenance lock remains until deploy/maint_clear.sh. No follow was started. Port 8080 was not changed."
exit 0
