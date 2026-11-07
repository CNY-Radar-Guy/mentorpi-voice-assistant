"""Versioned release and activation rollback. No robot and no systemd."""
import os, sys, tempfile
_iso = tempfile.mkdtemp(prefix="gbrelease_")
os.environ["GB_DEPTH_SCALE_FILE"] = os.path.join(_iso, "missing-depth-scale.json")
os.environ["HEALTH_FILE"] = os.path.join(_iso, "health.json")
os.environ["GB_HEALTH_FILE"] = os.environ["HEALTH_FILE"]
os.environ["GB_SHARED_DIR"] = _iso
os.environ["GB_DATA"] = os.path.join(_iso, "data")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "grokbot"))
from deploy.release_layout import (stage_release, dropin_text, begin_activation, switch_component,
                                   crash_rollback, versions_consistent, voice_restart_env_ok)
import gb_maint
import gb_metric
import gb_demand
import gb_health_node

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

src = tempfile.mkdtemp()
live = tempfile.mkdtemp()
with open(os.path.join(src, "grokbot_voice.py"), "w") as f:
    f.write("print('new')\n")
os.makedirs(os.path.join(src, "gb_safety"))
with open(os.path.join(live, "grokbot_voice.py"), "w") as f:
    f.write("print('old')\n")
stage_release(src, live, "rel-1")
check("live voice was not replaced", open(os.path.join(live, "grokbot_voice.py")).read() == "print('old')\n")
check("release has its own copy", open(os.path.join(live, "releases", "rel-1", "grokbot_voice.py")).read().startswith("print('new')"))
text = dropin_text("voice", os.path.join(live, "releases", "rel-1"))
check("voice drop-in suppresses startup speech", "SUPPRESS_STARTUP_SPEECH=1" in text and "releases/rel-1/grokbot_voice.py" in text)
gate = dropin_text("gate", os.path.join(live, "releases", "rel-1"))
check("gate drop-in pins the release tree", "GB_TREE=" in gate and "gb_safety_run.sh gate" in gate)

state = begin_activation("rel-1", {"gate": "old", "watchdog": "old", "voice": "old"})
switch_component(state, "watchdog")
switch_component(state, "gate")
try:
    switch_component(state, "watchdog")
    check("order is enforced", False)
except ValueError:
    check("order is enforced", True)
rolled = crash_rollback(state)
check("rollback restores the previous gate", rolled["pins"].get("gate") == "old" and rolled["pins"].get("watchdog") == "old", rolled)
check("rollback restarts one safety unit at a time, gate first", rolled["restart"] == ["gate", "watchdog"], rolled["restart"])
check("voice is restarted rather than left down", "voice" not in state["switched"] and rolled["voice_left_down"] is False)
check("rollback does not clear latches", rolled["clear_estop"] is False and rolled["clear_maint"] is False)
state2 = begin_activation("rel-1", {})
switch_component(state2, "watchdog")
switch_component(state2, "gate")
switch_component(state2, "voice")
state2["step"] = "voice"
rolled2 = crash_rollback(state2)
check("a voice switch is rolled back and restarted", "voice" in rolled2["restart"] and rolled2["voice_left_down"] is False)
check("versions must match the pin", versions_consistent({"gate": "rel-1"}, {"gate": "rel-1"}))
check("a mixed process is rejected", versions_consistent({"gate": "rel-1", "voice": "old"}, {"gate": "rel-1", "voice": "rel-1"}) is False)
check("silent reload needs both switches", voice_restart_env_ok("SUPPRESS_STARTUP_SPEECH=1\nESTOP_AUTOCLEAR_S=0\nWAKE_WORDS=wake up wally\n"))
check("empty greeting is not enough", voice_restart_env_ok("STARTUP_GREETING=\nWAKE_ACK=\n") is False)

maint = os.path.join(live, "MAINT")
estop = os.path.join(live, "ESTOP")
with open(estop, "w") as f:
    f.write("user")
gb_maint.write_lock([maint], now=10)
rolled3 = crash_rollback(state2)
check("maintenance file survives the rollback record", os.path.exists(maint) and open(estop).read() == "user" and rolled3["clear_maint"] is False)

depth = [[1000] * 8 for _ in range(8)]
got = gb_metric.box_range(depth, [1, 1, 6, 6], 0.001, 400, 400, 4, 4, min_pixels=8)
check("box range uses the supplied scale", got and abs(got["depth_m"] - 1.0) < 1e-6, got)
people = [{"conf": 0.9, "box": [0, 0, 10, 10]}, {"conf": 0.8, "box": [20, 20, 30, 30]}]
chosen, ambiguous = gb_metric.choose_person(people)
check("two people are ambiguous", chosen is None and ambiguous)
one, ambiguous = gb_metric.choose_person([people[0]])
check("one person is selected without a name", one is people[0] and not ambiguous)
pix = gb_metric.pointing_pixel([{"x": 0.2, "y": 0.5}] + [{}] * 7 + [{"x": 0.4, "y": 0.5}], 100, 100)
check("pointing uses the index tip", pix is not None and pix[0] > 40, pix)
body = gb_metric.track_record(1, 2.5, 0.1, 0.9, "crosscheck_mm", 40, "depth")
check("track record keeps the unit", body["person"]["unit"] == "crosscheck_mm" and body["person"]["valid_pixels"] == 40)
check("an unvalidated unit is not a range", "depth_m" not in gb_metric.track_record(1, 2.5, 0, 0.9, "driver", 40, "depth").get("person", {}))
path = os.path.join(live, "demand.json")
gb_demand.write_demand(path, "follow", now=50)
check("follow demand is readable before motion", gb_demand.read_demand(path, now=51) == "follow")
check("follow demand is still the camera window at 40 seconds", gb_demand.read_demand(path, now=90) == "follow")
check("follow demand expires fail-closed", gb_demand.read_demand(path, now=50 + gb_demand.FOLLOW_TTL + 1) is None)
gb_demand.write_demand(path, "acquire", now=50)
check("acquire demand expires fail-closed", gb_demand.read_demand(path, now=71) is None)
gb_demand.write_demand(path, "point", now=50)
check("point demand expires fail-closed", gb_demand.read_demand(path, now=81) is None)
rec = gb_health_node.record(5, 0.8, "16UC1", [50000] * 20, 1.0)
check("mono16 without a matching lidar range is not depth", rec["mode"] == "lidar" and rec["depth_unit"] is None, rec)
check("health record is not a mission", rec.get("executable") is False)

print("test_release: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
