"""Safety invariants for a release switch. No robot and no systemd."""
import json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, ROOT)
import gb_estop
import gb_maint
import gb_unlock
from deploy.release_layout import (begin_activation, switch_component, crash_rollback,
                                   code_dir, watchdog_confirmed, watchdog_status, restart_one_at_a_time)

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

tmp = tempfile.mkdtemp(prefix="gbsafe_")
host = os.path.join(tmp, "ESTOP")
shared = os.path.join(tmp, "shared_ESTOP")
maint = os.path.join(tmp, "MAINT")
gb_estop.ESTOP = host
gb_estop.SHARED_ESTOP = shared
gb_estop.zero_now = lambda *a, **k: None
gb_maint.write_lock([maint], now=1)
with open(host, "w") as f:
    json.dump({"t": 1, "src": "gate-tilt", "kind": "safety", "why": "tilt"}, f)
with open(shared, "w") as f:
    json.dump({"t": 1, "src": "gate-tilt", "kind": "safety", "why": "tilt"}, f)
check("clear leaves a hard fault", gb_estop.clear("auto") is False and os.path.isfile(host) and os.path.isfile(shared))
check("resume leaves a hard fault", gb_estop.clear("voice") is False and json.load(open(host))["kind"] == "safety")
check("a fresh-move clear leaves a hard fault", gb_estop.clear("move-intent", only_normal=False) is False and os.path.isfile(shared))
os.remove(host)
gb_estop.estop("voice", "stop")
check("a voice stop does not downgrade the shared hard fault", json.load(open(shared)).get("kind") == "safety")
check("clear removes nothing while one hard fault remains", gb_estop.clear("voice") is False and os.path.isfile(host) and os.path.isfile(shared))
os.remove(host)
check("maintenance file is not a stop file clear can delete", os.path.isfile(maint))
os.remove(shared)
gb_estop.estop("voice", "stop")
normal = json.load(open(host))
check("a normal stop can be cleared", normal["kind"] == "normal" and gb_estop.clear("voice") and not os.path.exists(host) and not os.path.exists(shared))
check("that clear still left maintenance", os.path.isfile(maint) and json.load(open(maint))["kind"] == "maintenance")
with open(host, "w") as f:
    json.dump({"t": 1000.0, "src": "watchdog", "kind": "safety", "why": "stale"}, f)
with open(shared, "w") as f:
    json.dump({"t": 1000.0, "src": "watchdog", "kind": "safety", "why": "stale"}, f)
clk = {"t": 1000.0}
said = []
timer = gb_estop.AutoClear(15, clock=lambda: clk["t"], on_clear=lambda: said.append("spoken"))
timer.step()
clk["t"] = 1020.0
check("the timer does not clear or announce a hard fault", timer.step() is False and os.path.isfile(host) and said == [])
os.remove(host); os.remove(shared)

prov = gb_unlock.fresh_provenance("drive forward", now=10, epoch=1)
prov["heard_at"] = 10
motion = {"op": "timed", "lin": 0.08, "ang": 0, "dur": 2, "requested": "drive forward"}
decision = gb_unlock.decide("drive forward", prov, {"src": "gate-tilt", "kind": "safety"},
                            {"allow_motion": True, "epoch": 1}, motion=motion, now=10)
check("a move does not unlock a hard fault", decision["unlock"] is False and decision["allow"] is False, decision)
check("the hard fault file is still there", os.path.isfile(maint))

state_dir = os.path.join(tmp, "gb_safety")
os.makedirs(state_dir)
code = code_dir(state_dir, "rel-9")
check("release code is not the shared state directory", os.path.basename(os.path.dirname(code)) == "gb_code" and code != state_dir, code)
check("legacy code dir stays the state dir", code_dir(state_dir, "") == state_dir)

now = 100.0
ack = watchdog_status(True, "rel-9", now)
check("fresh watchdog ack is accepted before the gate moves", watchdog_confirmed(ack, "rel-9", now + 0.5))
check("an old ack is not enough", watchdog_confirmed(ack, "rel-9", now + 5) is False)
check("another release cannot borrow the ack", watchdog_confirmed(ack, "other", now) is False)
check("ack without maintenance is rejected", watchdog_confirmed(watchdog_status(False, "rel-9", now), "rel-9", now) is False)

state = begin_activation("rel-9", {"watchdog": "old", "gate": "old"})
switch_component(state, "watchdog")
failed = crash_rollback(state)
check("a failed watchdog switch does not restart the gate", failed["restart"] == ["watchdog"], failed["restart"])
switch_component(state, "gate")
both = crash_rollback(state)
check("rolling both back restarts the gate before the watchdog", both["restart"] == ["gate", "watchdog"], both["restart"])
check("those restarts are single steps", all(" " not in name for name in both["restart"]))
check("the sequence helper never pairs them", restart_one_at_a_time(["gate", "watchdog"]) == ["gate", "watchdog"])

text = open(os.path.join(os.path.dirname(HERE), "gb_safety", "gb_safety_run.sh")).read()
check("a release copies code beside the state directory", "gb_code" in text and "CONT_CODE" in text)
act = open(os.path.join(ROOT, "deploy", "wally_activate.sh")).read()
check("activation does not restart gate and watchdog together", "systemctl restart gb-gate gb-watchdog" not in act)
check("watchdog is restarted before the gate", act.find("systemctl restart gb-watchdog") < act.find("systemctl restart gb-gate"))

print("test_safety_seq: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
