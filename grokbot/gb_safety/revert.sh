#!/bin/bash
# revert.sh - undo the gb_safety release: restore *.bak-estop files, remove the three services. Run as pi.
GB=/home/pi/grokbot; B=.bak-estop
sudo systemctl disable --now gb-clap gb-watchdog gb-gate 2>/dev/null
sudo rm -f /etc/systemd/system/gb-gate.service /etc/systemd/system/gb-watchdog.service /etc/systemd/system/gb-clap.service
sudo systemctl daemon-reload
cd $GB
for f in grokbot_voice.py gb_explore.py gb_guard.py gb_robot.py gb_features.py gb_perceive.py guard.env grokbot.env; do
  [ -e "$f$B" ] && cp -p "$f$B" "$f" && echo "restored $f"
done
[ -e /home/pi/hiwonder-toolbox/button_scan.py$B ] && cp -p /home/pi/hiwonder-toolbox/button_scan.py$B /home/pi/hiwonder-toolbox/button_scan.py
if [ -e /home/pi/.asoundrc$B ]; then cp -p /home/pi/.asoundrc$B /home/pi/.asoundrc; else rm -f /home/pi/.asoundrc; fi
rm -f $GB/data/ESTOP /home/pi/docker/tmp/gb_safety/ESTOP
sudo systemctl restart gb-guard grokbot button_scan
echo "reverted (gb_estop.py and gb_safety/ left in place, unused)"
