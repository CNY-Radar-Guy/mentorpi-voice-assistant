#!/bin/sh
# Dry check of the actuation unit. This does not systemctl, does not restart
# the controller, and does not publish a twist.
set -e
echo "Do not remove MAINT. Do not disable or stop gb-watchdog. Do not publish a twist."
echo "Do not remap ~/set_motor off the watchdog. MAINT zero-racing is not a hardware inhibit."
ROOT=$(cd "$(dirname "$0")/.." && pwd)
python3 - "$ROOT" <<'PY'
import os, sys
sys.path.insert(0, os.path.join(sys.argv[1], "grokbot"))
sys.path.insert(0, sys.argv[1])
from deploy.actuation_layout import files_agree
ok, reason = files_agree(sys.argv[1])
print(reason)
sys.exit(0 if ok else 1)
PY
echo "Files agree. gb-actuation was not started. The controller was not restarted."
echo "Rollback: do not restore raw nonzero writers; leave MAINT; leave gb-watchdog; supervised retest."
exit 0
