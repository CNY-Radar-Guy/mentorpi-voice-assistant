#!/bin/sh
# Parent-run install notes. This script does not restart the robot and does not move it.
set -e
echo "Do not remove MAINT. Do not disable or stop gb-watchdog. Do not publish a twist."
echo "1. Copy gb_actuation.py, gb_actuation_node.py, and gb_motor_cap.py into /home/pi/grokbot."
echo "2. Install deploy/gb-actuation.service. Leave GB_CONTROLLER_REMAPPED=0."
echo "3. Change the ros_robot_controller node remapping so ~/set_motor is /gb/set_motor_admitted."
echo "4. Only then set GB_CONTROLLER_REMAPPED=1 and enable gb-actuation."
echo "Until step 4, enforced_on_robot stays false. Direct /dev/rrc writes stay outside this path."
echo "Rollback: systemctl disable --now gb-actuation; remove the remap; leave MAINT; leave gb-watchdog running; supervised retest before any motion."
exit 0
