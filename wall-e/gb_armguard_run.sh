#!/bin/bash
# gb_armguard_run.sh - host launcher for gb-armguard (wake-phrase arming backstop, 2026-11-25)
set -u
C=MentorPi; REL=/home/pi/grokbot/releases/e48d3d8; SH=/home/pi/docker/tmp/gb_safety/armguard
mkdir -p "$SH"; cp -f "$REL/gb_arm.py" "$REL/gb_armguard.py" "$REL/gb_floor.py" "$SH/"   # gb_floor: 2026-11-25 noestop
rm -f /home/pi/docker/tmp/gb_safety/ARMED            # boot / restart default: DISARMED
for i in $(seq 1 120); do [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ] && break; sleep 2; done
docker exec -u ubuntu $C pkill -f /home/ubuntu/shared/gb_safety/armguard/gb_armguard.py 2>/dev/null
exec docker exec -u ubuntu -e PYTHONUNBUFFERED=1 -e GB_ARM_DIR=/home/ubuntu/shared/gb_safety $C bash -lc \
  'source /opt/ros/humble/setup.bash; exec python3 /home/ubuntu/shared/gb_safety/armguard/gb_armguard.py'
