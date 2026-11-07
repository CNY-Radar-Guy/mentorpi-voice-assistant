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
REL=${WALLY_RELEASE:-$(date +%Y%m%d-%H%M%S)}
python3 - << PY
import sys
sys.path.insert(0, "$ROOT")
from deploy.release_layout import stage_release
path = stage_release("$ROOT/grokbot", "$DEST", "$REL")
print("staged", path)
print("live modules were not replaced")
PY
echo "release $REL is staged under $DEST/releases/$REL. Live modules were not replaced."
echo "Pins were not changed. A crash of the running services still starts the code they already use."
echo "Env files were not edited. Wake and ESTOP_AUTOCLEAR_S migration happen in activation, with a backup, just before the voice process is restarted."
python3 "$ROOT/deploy/port8080_plan.py"
echo "Phone URL http://192.168.50.125:8080/ is unmet. The stock server was not moved."
echo "Next, on the Pi: WALLY_ACTIVATE=1 WALLY_RELEASE=$REL deploy/wally_activate.sh"
