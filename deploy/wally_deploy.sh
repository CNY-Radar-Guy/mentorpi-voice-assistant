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
printf '%s\n' "$BACK" > "$DEST/.wally-last-code-backup"
# Live evidence is docker inspect .Mounts, plus /proc/1/mountinfo when the container answers.
# HostConfig.Binds is stale on this robot and is not consulted.
MOUNTS_JSON=$(mktemp)
INFO_FILE=$(mktemp)
if command -v docker >/dev/null 2>&1; then
  if ! docker inspect -f '{{json .Mounts}}' MentorPi > "$MOUNTS_JSON" 2>/dev/null; then
    echo '[]' > "$MOUNTS_JSON"
  fi
  if ! docker exec MentorPi cat /proc/1/mountinfo > "$INFO_FILE" 2>/dev/null; then
    : > "$INFO_FILE"
  fi
else
  echo '[]' > "$MOUNTS_JSON"
  : > "$INFO_FILE"
fi
HI=$(stat -c %i /home/pi/docker/tmp 2>/dev/null || true)
CI=""
if command -v docker >/dev/null 2>&1; then
  CI=$(docker exec MentorPi stat -c %i /home/ubuntu/shared 2>/dev/null || true)
fi
INODE_FLAGS=""
if [ -n "$HI" ] && [ -n "$CI" ]; then
  INODE_FLAGS="--inodes ${HI},${CI}"
fi
set +e
python3 "$ROOT/deploy/mount_check.py" --inspect "$MOUNTS_JSON" --mountinfo "$INFO_FILE" $INODE_FLAGS
MOUNT_RC=$?
set -e
rm -f "$MOUNTS_JSON" "$INFO_FILE"
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
python3 "$ROOT/deploy/port8080_plan.py"
echo "Phone URL http://192.168.50.125:8080/ is unmet. The stock server was not moved. gb-stream and gb-front stay inactive."
echo "Safety activation is separate and still dry unless you run it on the Pi: deploy/wally_activate.sh"
echo "Rollback: deploy/wally_rollback.sh $BACK"
