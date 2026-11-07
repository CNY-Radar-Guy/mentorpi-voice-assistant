#!/bin/sh
# Stop the partial intent observer. Do not clear the maintenance hold and do not move.
set -e
systemctl stop gb-intent 2>/dev/null || true
systemctl disable gb-intent 2>/dev/null || true
echo "gb-intent is stopped. MAINT was not removed. No twist was sent."
if [ -e /home/pi/grokbot/data/MAINT ] || [ -e /home/pi/docker/tmp/gb_safety/MAINT ]; then
  echo "MAINT is still present."
fi
exit 0
