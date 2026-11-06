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
echo "code copied. previous files are not deleted from $BACK unless you archived them yourself."
echo "rollback: restore $BACK if you copied it before this apply, or git checkout the previous tree."

if [ -f "$DEST/grokbot.env" ]; then
  python3 "$ROOT/deploy/migrate_wake_env.py" "$DEST/grokbot.env"
else
  echo "no grokbot.env to migrate; code defaults apply and secrets.env was not touched"
fi

echo "not restarting grokbot, gb-gate, or gb-stream."
echo "overlay unit is gb-stream.service (port 8081, Restart=no). Do not point it at 8080 while the snapshot server is healthy."
echo "enable it only after you choose to: sudo cp $DEST/gb-stream.service /etc/systemd/system/ && sudo systemctl daemon-reload"
