#!/bin/bash
# run_dry_tests.sh - start TEST instances of the gate + watchdog on /gb/test_* topics, run gb_safety_test.py, stop them.
C=MentorPi; SH=/home/ubuntu/shared/gb_safety; R=/home/ubuntu/shared/grokbot/rosrun.sh
cp -f /home/pi/grokbot/gb_safety/gb_gate.py /home/pi/grokbot/gb_safety/gb_watchdog.py /home/pi/grokbot/gb_safety/gb_safety_test.py /home/pi/docker/tmp/gb_safety/
docker exec -u ubuntu $C pkill -f "gb_gate.py --test" ; docker exec -u ubuntu $C pkill -f "gb_watchdog.py --test"; rm -f /home/pi/docker/tmp/gb_safety/ESTOP_TEST
docker exec -u ubuntu -e GATE_OUT=/gb/test_out -e GATE_IN=/gb/test_in -e GATE_REQ=/gb/test_req -e GATE_STATUS=/gb/test_status \
  -e GB_ESTOP_FILE=$SH/ESTOP_TEST -e GATE_JOY=0 -e GATE_ODOM=/gb/test_odom -e GATE_SCAN=/gb/test_scan -e GATE_LOG=$SH/gate_test.log \
  -e PYTHONUNBUFFERED=1 $C $R bash -c "exec -a 'python3 gb_gate.py --test' python3 $SH/gb_gate.py" > /tmp/gate_test.out 2>&1 &
GP=$!
docker exec -u ubuntu -e WD_DRY=1 -e WD_WATCH=/gb/test_wd_in -e WD_MOTOR_TOPIC=/gb/test_wd_motor_in -e WD_GATE_STATUS=/gb/test_status_wd \
  -e GB_ESTOP_FILE=$SH/ESTOP_TEST -e PYTHONUNBUFFERED=1 $C $R bash -c "exec -a 'python3 gb_watchdog.py --test' python3 $SH/gb_watchdog.py" > /tmp/wd_test.out 2>&1 &
WP=$!
sleep 4
docker exec -u ubuntu -e PYTHONUNBUFFERED=1 $C $R python3 $SH/gb_safety_test.py
docker exec -u ubuntu $C pkill -INT -f "gb_gate.py --test"; docker exec -u ubuntu $C pkill -INT -f "gb_watchdog.py --test"
sleep 1; kill $GP $WP 2>/dev/null; rm -f /home/pi/docker/tmp/gb_safety/ESTOP_TEST
echo "--- test gate log tail"; tail -40 /home/pi/docker/tmp/gb_safety/gate_test.log
echo "--- test watchdog log tail"; tail -15 /home/pi/docker/tmp/gb_safety/watchdog_dry.log
