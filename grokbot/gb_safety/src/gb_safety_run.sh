#!/bin/bash
# gb_safety_run.sh gate|watchdog - host launcher for the motion gate / watchdog (systemd: gb-gate, gb-watchdog).
# * copies the node into the container shared mount and runs it as user ubuntu with the bringup ROS env
# * mirrors the host ESTOP file (/home/pi/grokbot/data/ESTOP, the one SSH/voice/clap write) into the container
#   (/home/ubuntu/shared/gb_safety/ESTOP) every 50 ms, and on a rising edge kills every mission node in the container
# * gate only: keeps Hiwonder's joystick_control stopped (it double-drives /controller/cmd_vel; the gate reads the pad)
set -u
WHAT=${1:-gate}
C=MentorPi
GB=/home/pi/grokbot
SH=/home/pi/docker/tmp/gb_safety                 # = /home/ubuntu/shared/gb_safety in the container
HOST_ESTOP=$GB/data/ESTOP
# Live .Mounts only. HostConfig.Binds still names docker_ros2/share/tmp and those paths do not exist.
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
if ! python3 "$GB/mount_check.py" --inspect "$MOUNTS_JSON" --mountinfo "$INFO_FILE" $INODE_FLAGS >/tmp/gb-mount-check.json; then
  echo "$(date '+%F %T') [$WHAT-run] safety copy blocked. Live .Mounts do not show /home/pi/docker/tmp -> /home/ubuntu/shared. No files copied. No motion."
  cat /tmp/gb-mount-check.json 2>/dev/null || true
  rm -f "$MOUNTS_JSON" "$INFO_FILE"
  exit 1
fi
rm -f "$MOUNTS_JSON" "$INFO_FILE"
mkdir -p "$SH" "$GB/data"
cp -f "$GB/gb_safety/gb_$WHAT.py" "$SH/gb_$WHAT.py"
cp -f "$GB/gb_obstacle.py" "$SH/gb_obstacle.py"            # 2026-11-05: obstacle fusion shared by gate + explore
cp -f "$GB/gb_health.py" "$SH/gb_health.py"                # ranging mode for autonomous vs teleop
cp -f "$GB/gb_maint.py" "$SH/gb_maint.py"                  # maintenance lock, not the ESTOP timer
NICE=""
if [ "$WHAT" = rgbguard ]; then
  cp -f "$GB/models/yolov8n.onnx" "$SH/yolov8n.onnx"; NICE="nice -n 12"   # advisory only: never starve the safety loop
fi
log() { echo "$(date '+%F %T') [$WHAT-run] $*"; }

mirror() {
  local was=0
  while true; do
    if [ -e "$SH/ESTOP_REQ" ]; then                 # gate hard-tilt latch -> host latch (voice / clap / autoclear see it)
      cp -f "$SH/ESTOP_REQ" "$HOST_ESTOP" 2>/dev/null; rm -f "$SH/ESTOP_REQ"; log "ESTOP requested by gate (tilt) -> host latch"
    fi
    if [ -e "$HOST_ESTOP" ]; then
      [ -e "$SH/ESTOP" ] || cp -f "$HOST_ESTOP" "$SH/ESTOP" 2>/dev/null || touch "$SH/ESTOP"
      if [ $was = 0 ]; then
        was=1; log "ESTOP present -> killing missions in container"
        mkdir -p /home/pi/docker/tmp/grokbot/explore; echo "ESTOP" > /home/pi/docker/tmp/grokbot/explore/abort
        docker exec $C pkill -INT -f "gb_explore.py --node" 2>/dev/null
        docker exec $C pkill -INT -f "gb_explore.py --nav2" 2>/dev/null
      fi
    else
      [ -e "$SH/ESTOP" ] && rm -f "$SH/ESTOP" && log "ESTOP cleared"
      was=0
    fi
    if [ -e "$GB/data/MAINT" ]; then
      cp -f "$GB/data/MAINT" "$SH/MAINT" 2>/dev/null || true
    elif [ -e "$SH/MAINT" ]; then
      rm -f "$SH/MAINT" && log "MAINT cleared on host -> removed container copy"
    fi
    sleep 0.05
  done
}

if [ "$FAST" = 1 ]; then
  if [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" != "true" ]; then
    log "activation: container not running. No motion."
    exit 1
  fi
  if ! docker exec -u ubuntu $C bash -lc 'source /opt/ros/humble/setup.bash; python3 -c "import rclpy"' >/dev/null 2>&1; then
    log "activation: rclpy not importable. No motion."
    exit 1
  fi
  log "activation fast path: container and rclpy already up"
else
  for i in $(seq 1 120); do
    [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] && break
    sleep 2
  done
  # ROS graph must be up (bringup launch running) before we start
  for i in $(seq 1 90); do pgrep -f "ros2 launch bringup" >/dev/null && docker exec $C pgrep -f "ros2 launch bringup" >/dev/null && break; sleep 2; done
  # Historical boot race: python started before rclpy was on the path, systemd then restarted.
  # Wait here and publish nothing until the import works. A miss exits so systemd retries.
  ready=0
  for i in $(seq 1 30); do
    if docker exec -u ubuntu $C bash -lc 'source /opt/ros/humble/setup.bash; python3 -c "import rclpy"' >/dev/null 2>&1; then
      ready=1; log "rclpy import ok (try $i)"; break
    fi
    log "rclpy not importable yet (try $i/30); $WHAT not started, no motion"
    sleep 2
  done
  if [ "$ready" != 1 ]; then
    log "rclpy still missing; exit for systemd retry. No velocity published."
    exit 1
  fi
fi
docker exec -u ubuntu $C pkill -f "/home/ubuntu/shared/gb_safety/gb_$WHAT.py" 2>/dev/null

MIR=""
[ "$WHAT" = rgbguard ] || { mirror & MIR=$!; }      # one ESTOP mirror per safety node is enough
JK=""
if [ "$WHAT" = gate ]; then
  ( while true; do docker exec $C pkill -f "peripherals/joystick_control" >/dev/null 2>&1 && log "stopped Hiwonder joystick_control (gate owns the gamepad)"; sleep 10; done ) & JK=$!
fi
trap 'kill $MIR $JK 2>/dev/null; docker exec -u ubuntu '$C' pkill -INT -f /home/ubuntu/shared/gb_safety/gb_'$WHAT'.py 2>/dev/null; sleep 0.15; exit 0' TERM INT

ENVS=()
for v in $(compgen -e | grep -E '^(GATE_|WD_|GB_|OBS_)'); do ENVS+=(-e "$v=${!v}"); done
docker exec -u ubuntu -e PYTHONUNBUFFERED=1 "${ENVS[@]}" $C $NICE /home/ubuntu/shared/grokbot/rosrun.sh \
  python3 /home/ubuntu/shared/gb_safety/gb_$WHAT.py &
CHILD=$!
wait $CHILD
RC=$?
kill $MIR $JK 2>/dev/null
exit $RC
