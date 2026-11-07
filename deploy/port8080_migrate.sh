#!/bin/bash
# Print the 8080 plan. Does not bind 8080, does not kill web_video_server,
# and does not restart ros2 launch bringup.
# WALLY_PORT8080_APPLY=1 is still refused: the bringup argument is not in this repo.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
python3 "$ROOT/deploy/port8080_plan.py"
if [ "${WALLY_PORT8080_APPLY:-0}" = "1" ]; then
  echo "refusing to apply. Remaining requirements are listed above. 8080 is unmet and the server was not moved."
  exit 2
fi
echo "dry plan only. gb-front and gb-stream were not started."
exit 0
