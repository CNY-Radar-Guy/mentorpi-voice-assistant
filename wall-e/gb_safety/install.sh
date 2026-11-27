#!/bin/bash
# install.sh - deploy the gb_safety release (backs up every changed file as *.bak-estop first). Run as pi.
set -e
GB=/home/pi/grokbot; S=$GB/gb_safety; B=.bak-estop
cd $GB
for f in grokbot_voice.py gb_explore.py gb_guard.py gb_robot.py gb_features.py gb_perceive.py guard.env grokbot.env; do
  [ -e "$f$B" ] || cp -p "$f" "$f$B"
done
[ -e /home/pi/hiwonder-toolbox/button_scan.py$B ] || cp -p /home/pi/hiwonder-toolbox/button_scan.py /home/pi/hiwonder-toolbox/button_scan.py$B
[ -e /home/pi/.asoundrc ] && { [ -e /home/pi/.asoundrc$B ] || cp -p /home/pi/.asoundrc /home/pi/.asoundrc$B; }
cp -f $S/src/*.py $GB/
cp -f $S/asoundrc /home/pi/.asoundrc
# guard: always on (no mission bypass), front stop-only backup behind the gate's direction-aware guard
grep -q '^GUARD_MISSION_BYPASS=' guard.env && sed -i 's/^GUARD_MISSION_BYPASS=.*/GUARD_MISSION_BYPASS=0/' guard.env || echo 'GUARD_MISSION_BYPASS=0' >> guard.env
sed -i 's/^GUARD_TURN=.*/GUARD_TURN=0/' guard.env
sed -i 's/^GUARD_STOP_DIST=.*/GUARD_STOP_DIST=0.220/' guard.env
grep -q '^# gb_safety' guard.env || echo '# gb_safety 2026-11-04: gb_gate is the direction-aware guard (0.33 fwd / 0.25 rev / 0.20 sweep; missions 0.25/0.20/0.18). gb_guard = front stop-only backup at 0.22 m, never bypassed.' >> guard.env
# voice: shared mic
sed -i 's/^AUDIO_IN=.*/AUDIO_IN=gbmic/' grokbot.env
grep -q '^STOP_LISTENER=' grokbot.env || echo 'STOP_LISTENER=1' >> grokbot.env
mkdir -p /home/pi/docker/tmp/gb_safety
sudo cp -f $S/gb-gate.service $S/gb-watchdog.service $S/gb-clap.service /etc/systemd/system/
sudo systemctl daemon-reload
echo installed
