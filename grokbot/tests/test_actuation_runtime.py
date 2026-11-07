"""Deployment, lease, and watchdog path. No robot, no systemctl, no twist.

Pure admission checks live in test_intent.py. This file fails if those pass
while the unit still starts host Python, the lease file is never loaded, or
a remap drops the watchdog zero.
"""
import os, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "grokbot"))

import gb_actuation
import gb_actuation_node
import gb_intent
import gb_motor_cap as cap
import gb_wally
from deploy.actuation_layout import (CONTAINER_MAINT, activation_mentions_supervised,
                                     dropin_ok, files_agree, launcher_text_ok, unit_text_ok)
from deploy.release_layout import ORDER, begin_activation, dropin_text, switch_component

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

unit = open(os.path.join(ROOT, "deploy", "gb-actuation.service")).read()
launcher = open(os.path.join(ROOT, "grokbot", "gb_safety", "gb_safety_run.sh")).read()
activate = open(os.path.join(ROOT, "deploy", "wally_activate.sh")).read()
node_src = open(os.path.join(ROOT, "grokbot", "gb_actuation_node.py")).read()
check("the unit does not start host python", unit_text_ok(unit), unit)
check("the launcher pins the container hold and rosrun", launcher_text_ok(launcher))
check("activation does not start the unit or the controller", activation_mentions_supervised(activate))
ok, reason = files_agree(ROOT)
check("unit, launcher, and activation agree", ok, reason)
check("actuation is outside the ordinary switch order", "actuation" not in ORDER)
try:
    switch_component(begin_activation("rel-1", {}), "actuation")
    check("actuation cannot skip the watchdog", False)
except ValueError:
    check("actuation cannot skip the watchdog", True)
drop = dropin_text("actuation", "/home/pi/grokbot/releases/rel-1")
check("the drop-in is a release tree and the container hold", dropin_ok(drop, "/home/pi/grokbot/releases/rel-1"), drop)
check("a flat tree is not a drop-in", dropin_ok(drop, "/home/pi/grokbot") is False)
try:
    import rclpy  # noqa: F401
    host_rclpy = True
except Exception:
    host_rclpy = False
try:
    import ros_robot_controller_msgs  # noqa: F401
    host_msgs = True
except Exception:
    host_msgs = False
check("this interpreter is not the container ROS runtime", host_rclpy is False and host_msgs is False)
check("the node module imports without rclpy", "import rclpy" not in node_src.split("def main")[0])
check("the boolean remap flag is not consulted", 'GB_CONTROLLER_REMAPPED"' not in node_src and "enforced_on_robot\"] = False" in node_src)

env = dict(os.environ)
env.pop("GB_TREE", None)
refused = subprocess.run(["bash", os.path.join(ROOT, "grokbot", "gb_safety", "gb_safety_run.sh"), "actuation"],
                         env=env, capture_output=True, text=True)
check("the launcher exits before docker when there is no release",
      refused.returncode != 0 and "refused without GB_TREE" in refused.stdout and "rclpy" not in refused.stderr,
      refused.stdout + refused.stderr)

gb_intent._BOOK["lease"] = None
first = gb_intent.issue_lease("look around", 100.0, 45)
second = gb_intent.issue_lease("look around again", 120.0, 45)
check("a second request does not move the deadline",
      first is second and first["mono_deadline"] == 145.0 and first["revoked"] is False)
loaded = gb_intent.read_lease()
slot = {"current": None}
check("the slot starts empty, which is the c535 bug", slot["current"] is None)
gb_actuation_node.bind_lease(slot)
check("bind_lease assigns the file the voice wrote",
      slot["current"] and slot["current"]["id"] == first["id"] == loaded["id"])
motors = cap.tank_rps_map(0.05, 0.0)
gb_intent.write_envelope(first["id"], 0.05, 0.0, 100.2)
speeds, decision = gb_actuation_node.on_motor_pairs(motors, 100.3, {"current": None}, maint=False)
check("a matching gate twist is admitted and the lease was loaded",
      decision["admitted"] and decision["lease_loaded"] and decision["reason"] == "within_provisional_cap"
      and decision["enforced_on_robot"] is False)
borrow, borrow_dec = gb_actuation_node.on_motor_pairs(cap.tank_rps_map(0.0, 0.4), 100.3, {"current": None}, maint=False)
check("a different raw command cannot borrow the explore lease",
      borrow_dec["reason"] == "direction" and all(r == 0.0 for _i, r in borrow))
stale, stale_dec = gb_actuation_node.on_motor_pairs(motors, 101.0, {"current": None}, maint=False)
check("a stale gate twist is not a lease", stale_dec["reason"] == "no_matching_lease" and all(r == 0.0 for _i, r in stale))
naked, naked_dec = gb_actuation_node.on_motor_pairs(motors, 100.3, {"current": None}, maint=False)
gb_intent.clear_envelope(100.3, "test")
naked, naked_dec = gb_actuation_node.on_motor_pairs(motors, 100.3, {"current": None}, maint=False)
check("no envelope means no admission", naked_dec["reason"] == "no_matching_lease")

hook = gb_actuation.ControllerHook()
claim = gb_intent.claim_from(first, gb_intent.read_envelope(100.2) or {"intent_id": first["id"], "lin": 0.05, "ang": 0.0, "source": "gate", "action": "bounded_look", "mono": 100.2}, 100.2)
# envelope was cleared; rebuild the claim the controller would have had
claim = {"lease_id": first["id"], "source": "voice", "action": "bounded_look", "lin": 0.05, "ang": 0.0}
admitted, admitted_dec = hook.set_motor_state(motors, 100.2, first, maint=False, claim=claim)
check("the controller hook can admit once", admitted_dec["admitted"] and any(abs(r) > 0 for _i, r in admitted))
crashed, crash_dec = hook.idle_tick(100.8)
check("a dead actuation process is not required to zero",
      crash_dec["reason"] == "stale_nonzero_zeroed" and crashed and all(r == 0.0 for _i, r in crashed))
raw_zero, raw_dec = gb_actuation.board_command(
    [(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)], None, 100.9, first, False, None, hook)
check("watchdog zeros still reach the raw path", raw_dec["reason"] == "watchdog_raw_zero" and raw_dec["alternate_nonzero"] is False)
rejected, rejected_dec = gb_actuation.board_command(cap.tank_rps_map(0.0, 0.4), None, 101.0, first, False, claim, hook)
check("the watchdog path rejects a nonzero writer", rejected_dec["reason"] == "raw_nonzero_rejected" and all(r == 0.0 for _i, r in rejected))
check("the naive remap loses watchdog zeros", gb_actuation.watchdog_zero_reachable(gb_actuation.naive_remap_subscriptions()) is False)
check("keeping the raw subscription preserves watchdog zeros",
      gb_actuation.watchdog_zero_reachable(gb_actuation.kept_watchdog_subscriptions()) is True)

gb_wally.drop_goals("halt")
revoked = gb_intent.read_lease()
after, after_dec = gb_actuation.admit(motors, 130.0, revoked, maint=False, claim=claim)
check("stop revokes the lease file",
      revoked and revoked["revoked"] is True and after_dec["reason"] == "no_matching_lease" and all(r == 0.0 for _i, r in after))

maint_path = os.environ["GB_MAINT_FILE"]
open(maint_path, "w").write("hold\n")
held, held_dec = gb_actuation_node.on_motor_pairs(motors, 100.3, {"current": None}, maint=gb_actuation.maint_held())
check("the container-visible hold still zeros", held_dec["reason"] == "maintenance" and CONTAINER_MAINT not in maint_path)
os.remove(maint_path)

plan = gb_actuation.install_plan(True, False, True, True, host_python=True)
check("host python is not an install", plan["ok"] is False and plan["start_unit"] is False and plan["clear_maint"] is False)
ready = gb_actuation.install_plan(True, True, True, True)
check("a ready tree is still not started from here", ready["ok"] is True and ready["restart_controller"] is False and ready["start_unit"] is False)
supervised = gb_actuation.supervised_controller_step(False, True, True, True, True)
check("controller restart stays supervised", supervised["restart_controller"] is False and "supervised" in supervised["reason"])
rolled = gb_actuation.rollback_plan(False, False, True, True, naive_remap=True)
check("rollback does not reopen raw writers or call MAINT a hardware inhibit",
      rolled["ok"] is False and rolled["restore_raw_nonzero_writers"] is False and rolled["clear_maint"] is False
      and rolled["maint_is_hardware_inhibit"] is False and rolled["stop_watchdog"] is False)
note = gb_intent.propose_repair("The controller callback is not loaded, so admission is not on the board.")
check("an automatic approval is rejected", gb_intent.accept_repair(note, "auto") is None and note["applied"] is False)
accepted = gb_intent.accept_repair(note, "parent")
check("a named approval still applies nothing",
      accepted and accepted["approved"] is True and accepted["applied"] is False and "lin" not in accepted)
check("retest before approval is refused", gb_intent.record_retest(note, True, "parent") is None)
retest = gb_intent.record_retest(accepted, False, "parent")
check("a supervised retest stays unapplied",
      retest and retest["state"] == "retest_failed" and retest["applied"] is False and retest["retest"] == "failed")

print("test_actuation_runtime: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
