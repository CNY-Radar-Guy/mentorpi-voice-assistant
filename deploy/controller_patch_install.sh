#!/bin/sh
# Stage the vendor callback patch. Default is a dry check and no write.
# GB_CONTROLLER_PATCH_APPLY=1 writes the file inside the container and does not restart it.
# The robot has one power switch. This script does not ask for a motor-only isolation step.
set -e
echo "Do not remove MAINT. Do not disable or stop gb-watchdog. Do not publish a twist."
echo "Do not restart ros_robot_controller from this script."
echo "The whole robot shares one power switch. There is no separate motor switch to open."
ROOT=$(cd "$(dirname "$0")/.." && pwd)
APPLY=${GB_CONTROLLER_PATCH_APPLY:-0}
python3 - "$ROOT" "$APPLY" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
from deploy.controller_patch import (CANDIDATES, MISSING_FROM_ARCHIVE, ORIGINAL_SHA256,
                                     apply_file, patched_sha256)
print("snapshot", ORIGINAL_SHA256)
print("patched", patched_sha256())
print("missing from the incident archive:")
for item in MISSING_FROM_ARCHIVE:
    print(" ", item)
print("correction factors are read at controller start from the publisher path, not embedded")
apply = sys.argv[2] == "1"
if not apply:
    print("dry run. No file written. No restart.")
    sys.exit(0)
wrote = False
for path in CANDIDATES:
    host = path
    # The parent runs this on the Pi. The files are inside the container.
    result = apply_file(path, apply=False)
    print(path, result.get("reason"), result.get("found", ""))
    if result.get("reason") in ("dry_run_original_matches", "already_patched"):
        result = apply_file(path, apply=True, backup_dir="/home/pi/docker/tmp/gb_safety")
        print(path, result.get("reason"), "changed" if result.get("changed") else "unchanged")
        if not result.get("ok"):
            sys.exit(1)
        wrote = True
if not wrote:
    print("no candidate matched the snapshot or the patched hash. Nothing written.")
    sys.exit(1)
print("patched. ros_robot_controller was not restarted. MAINT was not removed.")
PY
exit 0
