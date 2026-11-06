#!/bin/bash
# Apply this tree onto the Pi as user pi. Default is a dry run.
# Does not restart services, does not start audio, does not enable motion,
# does not copy secrets.env, grokbot.env (except the wake-field migration),
# caps.json, data/, models/, voices/, or anything under ros2_ws.
# WALLY_DEPLOY_APPLY=1 performs the copy. WALLY_SANITY=1 is not a restart.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST=${WALLY_DEST:-$HOME/grokbot}
APPLY=${WALLY_DEPLOY_APPLY:-0}

echo "source $ROOT"
echo "dest   $DEST"
echo "apply  $APPLY (1 to copy)"

python3 - << PY
import os, hashlib, sys
root = "$ROOT"
skip_names = {"secrets.env", "grokbot.env", "guard.env", "caps.json"}
bad = []
for dirpath, dirs, files in os.walk(os.path.join(root, "grokbot")):
    dirs[:] = [d for d in dirs if d not in ("data", "models", "voices", "__pycache__")]
    for name in files:
        if name in skip_names or name.endswith(".wav") or ".bak" in name:
            bad.append(os.path.join(dirpath, name))
if bad:
    print("refusing to package secrets or runtime files:", *bad)
    sys.exit(4)
print("package has no secrets.env, grokbot.env, or caps.json")
PY

if [ "$APPLY" != "1" ]; then
  echo "dry run only. Re-run with WALLY_DEPLOY_APPLY=1 on the Pi."
  exit 0
fi

mkdir -p "$DEST"
STAMP=$(date +%Y%m%d-%H%M%S)
BACK="$DEST/backup-wally-$STAMP"
mkdir -p "$BACK"
# Snapshot the current code before replacing it. Env, secrets, caps, and data stay put.
tar -C "$DEST" \
  --exclude=data --exclude=models --exclude=voices --exclude='backup-wally-*' \
  --exclude=secrets.env --exclude=grokbot.env --exclude=guard.env --exclude=caps.json \
  -cf - . | tar -C "$BACK" -xf -
echo "rollback tree: $BACK"
echo "rollback command: tar -C '$BACK' -cf - . | tar -C '$DEST' -xf -"
# Code only. Never the live env, secrets, or the capability file.
tar -C "$ROOT/grokbot" \
  --exclude=data --exclude=models --exclude=voices --exclude=__pycache__ \
  --exclude=secrets.env --exclude=grokbot.env --exclude=guard.env --exclude=caps.json \
  --exclude='*.wav' --exclude='*.bak*' \
  -cf - . | tar -C "$DEST" -xf -
echo "code snapshot is in $BACK. This script does not delete older backup-wally directories."
# Safety launchers assume /home/pi/docker/tmp -> /home/ubuntu/shared. The 17:44 ET
# preflight bind list did not show that pair. Do not replace those files unless it does.
BINDS='[]'
if command -v docker >/dev/null 2>&1; then
  BINDS=$(docker inspect -f '{{json .HostConfig.Binds}}' MentorPi 2>/dev/null || echo '[]')
fi
set +e
python3 "$ROOT/deploy/mount_check.py" "$BINDS"
MOUNT_RC=$?
set -e
if [ "$MOUNT_RC" != "0" ]; then
  echo "SAFETY MODULES BLOCKED. Restoring launchers and injected nodes from the backup snapshot."
  for rel in gb_safety gb_guard.py gb_guard_run.sh gb_obstacle.py gb_explore.py gb_mapping.py gb-gate.service gb-watchdog.service gb-rgbguard.service gb-guard.service gb-clap.service; do
    if [ -e "$BACK/$rel" ]; then
      mkdir -p "$(dirname "$DEST/$rel")"
      rm -rf "$DEST/$rel"
      cp -a "$BACK/$rel" "$DEST/$rel"
    fi
  done
  echo "Host voice modules, if present in this tree, remain staged. Services were not restarted."
fi

if [ -f "$DEST/grokbot.env" ]; then
  python3 "$ROOT/deploy/migrate_wake_env.py" "$DEST/grokbot.env"
else
  echo "no grokbot.env to migrate; code defaults apply and secrets.env was not touched"
fi

echo "not restarting grokbot, gb-gate, gb-watchdog, gb-guard, gb-rgbguard, or web_video_server."
python3 - << PY
import sys
sys.path.insert(0, "$ROOT/grokbot")
import gb_front
# The 17:44 ET sample: 8080 is bound. Do not probe /snapshot with an empty topic.
print(gb_front.preflight_decision(True, snapshot_without_topic_empty=True, upstream_snapshot_jpeg=False)["reason"])
PY
echo "Phone URL http://192.168.50.125:8080/ is unmet while web_video_server owns 8080. gb-stream and gb-front stay inactive."
echo "Rollback: deploy/wally_rollback.sh $BACK"
