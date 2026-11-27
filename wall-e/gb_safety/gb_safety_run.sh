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
TREE=${GB_TREE:-$GB}                            # a release dir when activation pinned this process
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
if ! python3 "$TREE/mount_check.py" --inspect "$MOUNTS_JSON" --mountinfo "$INFO_FILE" $INODE_FLAGS >/tmp/gb-mount-check.json; then
  echo "$(date '+%F %T') [$WHAT-run] safety copy blocked. Live .Mounts do not show /home/pi/docker/tmp -> /home/ubuntu/shared. No files copied. No motion."
  cat /tmp/gb-mount-check.json 2>/dev/null || true
  rm -f "$MOUNTS_JSON" "$INFO_FILE"
  exit 1
fi
rm -f "$MOUNTS_JSON" "$INFO_FILE"
mkdir -p "$SH" "$GB/data"
# State (ESTOP, MAINT, json) stays in $SH. Code for a release goes in its own folder
# so a newer rgbguard or health start cannot overwrite the modules an older gate imports.
if [ -n "${GB_TREE:-}" ]; then
  REL=${GB_RELEASE:-$(basename "$TREE")}
  CODE=/home/pi/docker/tmp/gb_code/$REL
  CONT_CODE=/home/ubuntu/shared/gb_code/$REL
else
  CODE=$SH
  CONT_CODE=/home/ubuntu/shared/gb_safety
fi
mkdir -p "$CODE"
if [ "$WHAT" = "health" ]; then
  cp -f "$TREE/gb_health_node.py" "$CODE/gb_health_node.py"
else
  cp -f "$TREE/gb_safety/gb_$WHAT.py" "$CODE/gb_$WHAT.py"
fi
cp -f "$TREE/gb_obstacle.py" "$CODE/gb_obstacle.py"
cp -f "$TREE/gb_fusion_recovery.py" "$CODE/gb_fusion_recovery.py" 2>/dev/null || true   # 2026-11-27 fusion stuck-recovery (gate re-verifies micro-nudges)
cp -f "$TREE/gb_health.py" "$CODE/gb_health.py"
cp -f "$TREE/gb_maint.py" "$CODE/gb_maint.py"
cp -f "$TREE/gb_floor.py" "$CODE/gb_floor.py"   # 2026-11-25 noestop
cp -f "$TREE/gb_arm.py" "$CODE/gb_arm.py" 2>/dev/null || true   # 2026-11-26: gate revokes MISSION_LEASE on stop
cp -f "$TREE/gb_metric.py" "$CODE/gb_metric.py" 2>/dev/null || true
cp -f "$TREE/gb_demand.py" "$CODE/gb_demand.py" 2>/dev/null || true
cp -f "$TREE/gb_hands.py" "$CODE/gb_hands.py" 2>/dev/null || true
cp -f "$TREE/gb_nav.py" "$CODE/gb_nav.py" 2>/dev/null || true
cp -f "$TREE/gb_paths.py" "$CODE/gb_paths.py"
cp -f "$TREE/gb_speed_caps.py" "$CODE/gb_speed_caps.py" 2>/dev/null || true   # 2026-11-26 dual speed caps
cp -f "$TREE/gb_bounded.py" "$CODE/gb_bounded.py"
cp -f "$TREE/gb_yolo.py" "$CODE/gb_yolo.py" 2>/dev/null || true
NICE=""
if [ "$WHAT" = rgbguard ]; then
  # The release model is the static 416 export. The host models directory still
  # holds the older static 640 file and is not the one this unit runs.
  cp -f "$TREE/gb_safety/yolov8n.onnx" "$CODE/yolov8n.onnx"
  NICE="nice -n 12"   # advisory only: never starve the safety loop
fi
log() { echo "$(date '+%F %T') [$WHAT-run] $*"; }

# 2026-11-25 benchpersist (John 9:17 ET): bench-only guard bypass that survives reboots. Gate launcher only:
# every 1 s, if $GB/data/BENCH_GUARD_OFF exists AND $GB/data/learn/mode.json says "mode": "bench", refresh
# $SH/BENCH_GUARD_OFF (the gate treats it as stale after 5 s). Flag gone -> removed at once; mode not bench for
# 3 checks in a row (tolerates a half-written mode.json) -> removed. Both files live on the SD card (ext4).
BENCH_HOST=$GB/data/BENCH_GUARD_OFF
MODE_FILE=$GB/data/learn/mode.json
bench_was=0; bench_miss=0
bench_mirror() {
  if [ -e "$BENCH_HOST" ] && grep -qE '"mode"[[:space:]]*:[[:space:]]*"bench"' "$MODE_FILE" 2>/dev/null; then
    bench_miss=0
    exp=$(grep -oE '"expires"[[:space:]]*:[[:space:]]*[0-9.]+' "$BENCH_HOST" 2>/dev/null | grep -oE '[0-9.]+$' || true)
    scope=$(grep -oE '"scope"[[:space:]]*:[[:space:]]*"[a-z]+"' "$BENCH_HOST" 2>/dev/null | grep -oE '[a-z]+"$' | tr -d '"' || true)   # navfix
    printf '{"mode": "bench", "t": %s, "expires": %s, "scope": "%s"}\n' "$(date +%s)" "${exp:-0}" "${scope:-all}" > "$SH/BENCH_GUARD_OFF.tmp" \
      && mv -f "$SH/BENCH_GUARD_OFF.tmp" "$SH/BENCH_GUARD_OFF"
    [ "$bench_was" = 1 ] || { bench_was=1; log "bench guard bypass ACTIVE (mode=bench + $BENCH_HOST)"; }
  else
    bench_miss=$((bench_miss + 1))
    if [ ! -e "$BENCH_HOST" ] || [ $bench_miss -ge 3 ]; then
      rm -f "$SH/BENCH_GUARD_OFF"
      [ "$bench_was" = 1 ] && log "bench guard bypass INACTIVE (flag absent or mode not bench)"
      bench_was=0
    fi
  fi
}

# 2026-11-25 noestop (John 14:02 ET): on the FLOOR (mode.json != bench, or unreadable) nothing latches.
# * every 1 s: $SH/FLOOR_MODE.json <- mode from mode.json (the container gate / watchdog read it via gb_floor.py)
# * floor: ESTOP_REQ / host ESTOP / container ESTOP / MAINT that appear -> normal stop (missions killed, ARMED removed,
#   cause logged) and the file is ARCHIVED (moved, never deleted) to data/floor_archive or $SH/floor_archive
# * $SH/FLOOR_STOP (normal-stop event from gate / voice / clap) -> after 0.3 s (gate has seen it): kill missions, disarm,
#   log to data/estop.log + $SH/floor_stop.log, archive the event. Bench: legacy latch behaviour unchanged.
FLOOR_ARCH_HOST=$GB/data/floor_archive
FLOOR_ABORT=${FLOOR_ABORT:-/home/pi/docker/tmp/grokbot/explore/abort}
FLOOR_ARCH_SH=$SH/floor_archive
floor=1
floor_mirror() {
  local m
  m=$(grep -oE '"mode"[[:space:]]*:[[:space:]]*"[A-Za-z_-]+"' "$MODE_FILE" 2>/dev/null | head -1 | grep -oE '"[A-Za-z_-]+"$' | tr -d '"' | tr 'A-Z' 'a-z')
  [ -n "$m" ] || m=unknown
  if [ "$m" = bench ]; then floor=0; else floor=1; fi
  printf '{"mode": "%s", "t": %s, "src": "gb_safety_run %s"}\n' "$m" "$(date +%s)" "$WHAT" > "$SH/FLOOR_MODE.json.tmp.$WHAT" \
    && mv -f "$SH/FLOOR_MODE.json.tmp.$WHAT" "$SH/FLOOR_MODE.json"
}
floor_log() {
  local line="$(date '+%F %T') [floor/$WHAT] $*"
  echo "$line" >> "$SH/floor_stop.log"; echo "$line" >> "$GB/data/estop.log"; log "$*"
}
floor_cause() {  # $1 = file -> one-line cause
  python3 -c 'import json,sys
try:
    d=json.load(open(sys.argv[1])); print(("%s - %s" % (d.get("src","?"), d.get("why","")))[:200])
except Exception:
    print("file (no JSON body)")' "$1" 2>/dev/null || echo unknown
}
floor_halt() {  # normal stop: end missions + disarm (the gate already zeroed / zeroes on the event or the file edge)
  mkdir -p "$(dirname "$FLOOR_ABORT")"; echo "floor stop: $1" > "$FLOOR_ABORT"
  docker exec $C pkill -INT -f "gb_explore.py --node" 2>/dev/null
  docker exec $C pkill -INT -f "gb_explore.py --nav2" 2>/dev/null
  if [ -e "$SH/ARMED" ]; then
    rm -f "$SH/ARMED" && echo "$(date '+%F %T') DISARMED: floor normal stop ($1)" >> "$SH/arm.log"
  fi
}
floor_archive_file() {  # $1 file, $2 archive dir; claim by atomic mv, so two launchers never double-handle it
  local f=$1 d=$2 dst
  mkdir -p "$d"; dst="$d/$(basename "$f").floor-archived-$(date +%Y%m%d-%H%M%S)-$$-$RANDOM"
  mv -f "$f" "$dst" 2>/dev/null && echo "$dst"
}
evt_seen=0
floor_backstop() {
  local f dst cause now
  for f in "$SH/ESTOP_REQ" "$HOST_ESTOP" "$SH/ESTOP" "$GB/data/MAINT" "$SH/MAINT"; do
    [ -e "$f" ] || continue
    cause=$(floor_cause "$f")
    case "$f" in "$GB"/data/*) dst=$(floor_archive_file "$f" "$FLOOR_ARCH_HOST");; *) dst=$(floor_archive_file "$f" "$FLOOR_ARCH_SH");; esac
    [ -n "$dst" ] || continue
    floor_halt "$(basename "$f"): $cause"
    floor_log "ground mode: $(basename "$f") would have latched ($cause) -> NORMAL STOP instead: missions ended, disarmed, latch archived to $dst. Re-arm with 'wake up Wally'."
  done
  if [ -e "$SH/FLOOR_STOP" ]; then
    now=$(date +%s%N)
    if [ "$evt_seen" = 0 ]; then evt_seen=$now
    elif [ $(( (now - evt_seen) / 1000000 )) -ge 300 ]; then
      cause=$(floor_cause "$SH/FLOOR_STOP")
      dst=$(floor_archive_file "$SH/FLOOR_STOP" "$FLOOR_ARCH_SH")
      evt_seen=0
      if [ -n "$dst" ]; then
        floor_halt "$cause"
        floor_log "normal stop (no latch): $cause -> missions ended, disarmed. Re-arm with 'wake up Wally'."
      fi
    fi
  else
    evt_seen=0
  fi
}

mirror() {
  local was=0 bc=0 fc=0
  [ "$WHAT" = gate ] && bench_mirror
  floor_mirror
  while true; do
    fc=$((fc + 1)); [ $fc -ge 20 ] && { fc=0; floor_mirror; }
    [ "$floor" = 1 ] && floor_backstop
    if [ -e "$SH/CLEAR_REQ" ]; then                # 2026-11-17: remote L1+R1 hold -> clear a NORMAL latch only
      rm -f "$SH/CLEAR_REQ"
      if [ -e "$HOST_ESTOP" ] && python3 - "$HOST_ESTOP" <<'PY2'
import json, sys
try:
    b = json.load(open(sys.argv[1]))
except Exception:
    sys.exit(1)
hard = {"gate-tilt", "tilt", "battery", "watchdog", "physical", "joystick", "hardware"}
sys.exit(0 if isinstance(b, dict) and b.get("kind") != "safety" and b.get("src") not in hard else 1)
PY2
      then
        rm -f "$HOST_ESTOP" "$SH/ESTOP"
        echo "{\"t\": $(date +%s.%N), \"src\": \"remote\"}" > "$GB/data/remote_clear.json"
        log "ESTOP cleared by remote (L1+R1 hold)"
      else
        log "remote clear refused: no latch or a safety latch"
      fi
    fi
    if [ -e "$SH/ESTOP_REQ" ]; then                 # gate hard-tilt latch -> host latch (voice / clap / autoclear see it)
      cp -f "$SH/ESTOP_REQ" "$HOST_ESTOP" 2>/dev/null; rm -f "$SH/ESTOP_REQ"; log "ESTOP requested by gate ($(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(d.get("src","?"), "-", d.get("why",""))' "$HOST_ESTOP" 2>/dev/null || echo unknown)) -> host latch"
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
      if [ -e "$SH/ESTOP" ] && python3 - "$SH/ESTOP" <<'PY'
import json, sys
path = sys.argv[1]
try:
    body = json.load(open(path))
except Exception:
    sys.exit(1)
src = body.get("src") if isinstance(body, dict) else ""
hard = src in {"gate-tilt", "tilt", "battery", "watchdog", "physical", "joystick", "hardware"}
sys.exit(0 if isinstance(body, dict) and (body.get("kind") == "safety" or hard) else 1)
PY
      then
        cp -f "$SH/ESTOP" "$HOST_ESTOP" 2>/dev/null || true
        log "left safety latch; host stop file kept"
      else
        [ -e "$SH/ESTOP" ] && rm -f "$SH/ESTOP" && log "ESTOP cleared"
      fi
      was=0
    fi
    if [ -e "$GB/data/MAINT" ]; then
      cp -f "$GB/data/MAINT" "$SH/MAINT" 2>/dev/null || true
    elif [ -e "$SH/MAINT" ]; then
      rm -f "$SH/MAINT" && log "MAINT cleared on host -> removed container copy"
    fi
    if [ "$WHAT" = gate ]; then bc=$((bc + 1)); [ $bc -ge 20 ] && { bc=0; bench_mirror; }; fi   # benchpersist
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
export GB_SHARED_DIR=/home/ubuntu/shared/gb_safety
NODE=gb_$WHAT.py
[ "$WHAT" = "health" ] && NODE=gb_health_node.py
[ "$WHAT" = "rgbguard" ] && export GB_RGB_MODEL=$CONT_CODE/yolov8n.onnx
docker exec -u ubuntu $C pkill -f "$CONT_CODE/$NODE" 2>/dev/null

MIR=""
[ "$WHAT" = rgbguard ] || { mirror & MIR=$!; }      # one ESTOP mirror per safety node is enough
JK=""
if [ "$WHAT" = gate ]; then
  ( while true; do docker exec $C pkill -f "peripherals/joystick_control" >/dev/null 2>&1 && log "stopped Hiwonder joystick_control (gate owns the gamepad)"; sleep 10; done ) & JK=$!
fi
trap 'kill $MIR $JK 2>/dev/null; docker exec -u ubuntu '$C' pkill -INT -f '$CONT_CODE'/'$NODE' 2>/dev/null; sleep 0.15; exit 0' TERM INT

ENVS=()
for v in $(compgen -e | grep -E '^(GATE_|WD_|GB_|OBS_)'); do ENVS+=(-e "$v=${!v}"); done
docker exec -u ubuntu -e PYTHONUNBUFFERED=1 "${ENVS[@]}" $C $NICE /home/ubuntu/shared/grokbot/rosrun.sh \
  python3 $CONT_CODE/$NODE &
CHILD=$!
wait $CHILD
RC=$?
kill $MIR $JK 2>/dev/null
exit $RC
