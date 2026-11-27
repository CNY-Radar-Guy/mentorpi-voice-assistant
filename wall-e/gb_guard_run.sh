#!/bin/bash
# gb_guard_run.sh - host-side launcher for the LiDAR obstacle guard (systemd unit gb-guard).
# Copies ~/grokbot/gb_guard.py into the container's shared mount, mirrors the kill-switch
# flag ~/grokbot/guard.disabled into the container, and runs the node as user ubuntu.
set -u
C=MentorPi
GB=/home/pi/grokbot
TREE=${GB_TREE:-$GB}
SH=/home/pi/docker/tmp/gb_guard          # = /home/ubuntu/shared/gb_guard inside the container
# Live .Mounts only. HostConfig.Binds is stale and must not block this copy.
MOUNTS_JSON=$(mktemp)
INFO_FILE=$(mktemp)
FAST=0
if [ -f "$GB/data/ACTIVATION" ]; then
  now=$(date +%s)
  ts=$(tr -cd '0-9' < "$GB/data/ACTIVATION" 2>/dev/null || true)
  if [ -n "$ts" ] && [ "$((now - ts))" -le 30 ]; then FAST=1; fi
fi
if ! docker inspect -f '{{json .Mounts}}' "$C" > "$MOUNTS_JSON" 2>/dev/null; then
  echo '[]' > "$MOUNTS_JSON"
fi
: > "$INFO_FILE"
INODE_FLAGS=""
if [ "$FAST" != 1 ]; then
  docker exec "$C" cat /proc/1/mountinfo > "$INFO_FILE" 2>/dev/null || : > "$INFO_FILE"
  HI=$(stat -c %i /home/pi/docker/tmp 2>/dev/null || true)
  CI=$(docker exec "$C" stat -c %i /home/ubuntu/shared 2>/dev/null || true)
  if [ -n "$HI" ] && [ -n "$CI" ]; then INODE_FLAGS="--inodes ${HI},${CI}"; fi
fi
if ! python3 "$TREE/mount_check.py" --inspect "$MOUNTS_JSON" --mountinfo "$INFO_FILE" $INODE_FLAGS >/tmp/gb-mount-check.json; then
  echo "$(date '+%F %T') [guard-run] safety copy blocked. Live .Mounts were not the expected pair. No files copied."
  cat /tmp/gb-mount-check.json 2>/dev/null || true
  rm -f "$MOUNTS_JSON" "$INFO_FILE"
  exit 1
fi
rm -f "$MOUNTS_JSON" "$INFO_FILE"
mkdir -p "$SH"
cp -f "$TREE/gb_guard.py" "$SH/gb_guard.py"
rm -f "$SH/mission_active"               # never inherit a mission bypass flag across guard restarts

sync_flag() {
  if [ -e "$GB/guard.disabled" ] || [ "${GUARD_DISABLED:-0}" = "1" ]; then
    [ -e "$SH/guard.disabled" ] || touch "$SH/guard.disabled"
  else
    rm -f "$SH/guard.disabled"
  fi
}
sync_flag

if [ "$FAST" = 1 ]; then
  if [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" != "true" ]; then
    echo "$(date '+%F %T') [guard-run] activation: container not running. No motion."
    exit 1
  fi
else
  for i in $(seq 1 120); do
    [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] && break
    sleep 2
  done
fi
docker exec -u ubuntu $C pkill -f /home/ubuntu/shared/gb_guard/gb_guard.py 2>/dev/null

( while true; do sync_flag; sleep 0.5; done ) &
SYNC=$!
trap 'kill $SYNC 2>/dev/null; docker exec -u ubuntu '$C' pkill -INT -f /home/ubuntu/shared/gb_guard/gb_guard.py 2>/dev/null; sleep 0.15; exit 0' TERM INT

ENVS=()
for v in $(compgen -e | grep '^GUARD_'); do ENVS+=(-e "$v=${!v}"); done
docker exec -u ubuntu -e PYTHONUNBUFFERED=1 "${ENVS[@]}" $C bash -lc \
  'source /opt/ros/humble/setup.bash; [ -f /home/ubuntu/ros2_ws/install/setup.bash ] && source /home/ubuntu/ros2_ws/install/setup.bash; exec python3 /home/ubuntu/shared/gb_guard/gb_guard.py' &
CHILD=$!
wait $CHILD
RC=$?
kill $SYNC 2>/dev/null
exit $RC