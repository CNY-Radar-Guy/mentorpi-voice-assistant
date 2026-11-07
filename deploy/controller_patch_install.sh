#!/bin/sh
# Patch the controller source the running process already imported.
# Default is a dry check and no write. GB_CONTROLLER_PATCH_APPLY=1 writes
# the regular source file only. It does not restart the controller, does not
# clear MAINT, and does not publish a twist.
#
# The process that is already running keeps its imported module. The next
# start is what loads the written file, and that start imports gb_actuation.
set -e
echo "Do not remove MAINT. Do not disable or stop gb-watchdog. Do not publish a twist."
echo "Do not restart ros_robot_controller from this script."
echo "The whole robot shares one power switch. There is no separate motor switch to open."
ROOT=$(cd "$(dirname "$0")/.." && pwd)
APPLY=${GB_CONTROLLER_PATCH_APPLY:-0}
export GB_ROS_WS="${GB_ROS_WS:-}"
export GB_PATCH_BACKUP="${GB_PATCH_BACKUP:-}"
python3 - "$ROOT" "$APPLY" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
from deploy.controller_patch import ORIGINAL_SHA256, install_layout, patched_sha256
print("snapshot", ORIGINAL_SHA256)
print("patched", patched_sha256())
apply = sys.argv[2] == "1"
ws = os.environ.get("GB_ROS_WS") or ""
if not ws:
    print("dry run. No workspace given. No file written. No restart.")
    sys.exit(0)
backup = os.environ.get("GB_PATCH_BACKUP") or None
plan = install_layout(ws, apply=apply, backup_dir=backup)
print("reason", plan.get("reason"))
for item in plan.get("files") or []:
    print("file", item["kind"], item["realpath"])
    for alias in item.get("aliases") or []:
        print(" alias", alias)
for path in plan.get("absent") or []:
    print("absent", path)
if not plan.get("ok"):
    print("Nothing written." if not plan.get("writes") else "Stopped after a failed write.")
    sys.exit(1)
if not apply:
    print("No file written. No restart.")
    sys.exit(0)
print(plan.get("reason"))
print("ros_robot_controller was not restarted. MAINT was not removed.")
print("The running process keeps the module it already imported.")
print("The next start loads this file and imports gb_actuation. A missing import stops that next start.")
PY
exit 0
