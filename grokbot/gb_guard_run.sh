#!/bin/bash
# gb_guard_run.sh - host-side launcher for the LiDAR obstacle guard (systemd unit gb-guard).
# Copies ~/grokbot/gb_guard.py into the container's shared mount, mirrors the kill-switch
# flag ~/grokbot/guard.disabled into the container, and runs the node as user ubuntu.
set -u
C=MentorPi
GB=/home/pi/grokbot
SH=/home/pi/docker/tmp/gb_guard          # = /home/ubuntu/shared/gb_guard inside the container
mkdir -p "$SH"
cp -f "$GB/gb_guard.py" "$SH/gb_guard.py"
rm -f "$SH/mission_active"               # never inherit a mission bypass flag across guard restarts

sync_flag() {
  if [ -e "$GB/guard.disabled" ] || [ "${GUARD_DISABLED:-0}" = "1" ]; then
    [ -e "$SH/guard.disabled" ] || touch "$SH/guard.disabled"
  else
    rm -f "$SH/guard.disabled"
  fi
}
sync_flag

# wait for the container and the LiDAR topic
for i in $(seq 1 120); do
  [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] && break
  sleep 2
done
docker exec -u ubuntu $C pkill -f /home/ubuntu/shared/gb_guard/gb_guard.py 2>/dev/null

( while true; do sync_flag; sleep 0.5; done ) &
SYNC=$!
trap 'kill $SYNC 2>/dev/null; docker exec -u ubuntu '$C' pkill -INT -f /home/ubuntu/shared/gb_guard/gb_guard.py 2>/dev/null; sleep 0.5; exit 0' TERM INT

ENVS=()
for v in $(compgen -e | grep '^GUARD_'); do ENVS+=(-e "$v=${!v}"); done
docker exec -u ubuntu -e PYTHONUNBUFFERED=1 "${ENVS[@]}" $C bash -lc \
  'source /opt/ros/humble/setup.bash; [ -f /home/ubuntu/ros2_ws/install/setup.bash ] && source /home/ubuntu/ros2_ws/install/setup.bash; exec python3 /home/ubuntu/shared/gb_guard/gb_guard.py' &
CHILD=$!
wait $CHILD
RC=$?
kill $SYNC 2>/dev/null
exit $RC