#!/bin/sh
# Stage the hook modules, prove they import, then patch the controller source.
# Default is a dry check and no write. GB_CONTROLLER_PATCH_APPLY=1 writes the
# hook directory and the regular source file. It does not restart the
# controller, does not clear MAINT, and does not publish a twist.
#
# The process that is already running keeps its imported module. The next
# start is what loads the written file.
set -e
echo "Do not remove MAINT. Do not disable or stop gb-watchdog. Do not publish a twist."
echo "Do not restart ros_robot_controller from this script."
echo "The whole robot shares one power switch. There is no separate motor switch to open."
ROOT=$(cd "$(dirname "$0")/.." && pwd)
APPLY=${GB_CONTROLLER_PATCH_APPLY:-0}
export GB_ROS_WS="${GB_ROS_WS:-}"
export GB_PATCH_BACKUP="${GB_PATCH_BACKUP:-}"
export GB_HOOK_DIR="${GB_HOOK_DIR:-}"
python3 - "$ROOT" "$APPLY" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
from deploy.controller_patch import (HOOK_DIR_DEFAULT, HOOK_FILES, ORIGINAL_SHA256,
                                     install_controller, patched_sha256)
print("snapshot", ORIGINAL_SHA256)
print("patched", patched_sha256())
print("hook", " ".join(HOOK_FILES))
apply = sys.argv[2] == "1"
ws = os.environ.get("GB_ROS_WS") or ""
if not ws:
    print("dry run. No workspace given. No file written. No restart.")
    sys.exit(0)
backup = os.environ.get("GB_PATCH_BACKUP") or None
hook = os.environ.get("GB_HOOK_DIR") or HOOK_DIR_DEFAULT
plan = install_controller(ws, hook, sys.argv[1], apply=apply, backup_dir=backup)
print("reason", plan.get("reason"))
print("controller", plan.get("controller") or "")
print("hook_reason", plan.get("hook_reason") or "")
print("hook_dest", plan.get("hook_dest") or "")
for name in plan.get("hook") or []:
    print("hook_file", name)
for item in plan.get("files") or []:
    print("file", item["kind"], item["realpath"])
    for alias in item.get("aliases") or []:
        print(" alias", alias)
for path in plan.get("absent") or []:
    print("absent", path)
if plan.get("missing"):
    print("missing", " ".join(plan["missing"]))
if plan.get("detail"):
    print("detail", plan["detail"])
if not plan.get("ok"):
    print("Nothing written." if not plan.get("writes") else "Hook may be staged. Controller was not patched.")
    sys.exit(1)
if not apply:
    print("No file written. No restart.")
    sys.exit(0)
print("ros_robot_controller was not restarted. MAINT was not removed.")
print("The running process keeps the module it already imported.")
print("The next start loads the patched source and imports the staged hook.")
PY
exit 0
