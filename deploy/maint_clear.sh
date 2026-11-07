#!/bin/bash
# Remove the maintenance lock after checks. Does not delete data/ESTOP.
# Requires WALLY_MAINT_CLEAR=1. Default is a no-op.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST=${WALLY_DEST:-$HOME/grokbot}
if [ "${WALLY_MAINT_CLEAR:-0}" != "1" ]; then
  echo "maintenance lock not cleared. Re-run with WALLY_MAINT_CLEAR=1 after the checks."
  exit 0
fi
python3 - << PY
import os, sys
sys.path.insert(0, "$ROOT/grokbot")
import gb_maint
host = os.path.join("$DEST", "data", "MAINT")
shared = "/home/pi/docker/tmp/gb_safety/MAINT"
ok = gb_maint.clear_lock(host, explicit=True)
ok2 = gb_maint.clear_lock(shared, explicit=True)
print("host cleared" if ok else "host had no maintenance lock")
print("shared cleared" if ok2 else "shared had no maintenance lock")
estop = os.path.join("$DEST", "data", "ESTOP")
if os.path.lexists(estop):
    print("ESTOP still present")
PY
echo "ESTOP was not deleted."
exit 0
