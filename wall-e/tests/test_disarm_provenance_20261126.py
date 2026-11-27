"""2026-11-26: disarm provenance, mission lease, and session abort fields. No ROS, no motion."""
import json, os, sys, time, tempfile, types

import pytest

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)

@pytest.fixture
def arm(tmp_path, monkeypatch):
    monkeypatch.setenv("GB_ARM_DIR", str(tmp_path))
    # Fresh import per test so path() sees GB_ARM_DIR
    for name in list(sys.modules):
        if name == "gb_arm" or name.startswith("gb_arm."):
            del sys.modules[name]
    import gb_arm
    assert gb_arm._dir() == str(tmp_path)
    return gb_arm

def test_default_disarmed(arm):
    assert not arm.is_armed()
    assert arm.state()["armed"] is False

@pytest.mark.parametrize("actor", ["voice", "jarvis", "remote_script", "other"])
def test_arm_and_disarm_tags_actor(arm, actor):
    assert arm.arm("unit test arm", actor=actor)
    assert arm.is_armed()
    flag = json.load(open(arm.path()))
    assert flag["actor"] == actor
    rec = arm.disarm_result("unit test disarm", actor=actor)
    assert rec["ok"] is True
    assert rec["removed"] is True
    assert rec["actor"] == actor
    assert rec["reason"] == "unit test disarm"
    assert not arm.is_armed()
    ev = arm.last_event()
    assert ev["actor"] == actor
    assert ev["kind"] == "disarm"
    assert os.path.exists(arm.event_path())

def test_lease_blocks_remote_script_during_live_voice_mission(arm):
    assert arm.arm("wake", actor="voice")
    arm.set_mission_lease(owner="voice", pid=os.getpid(), reason="explore", mission="explore")
    assert arm.lease_is_live()
    rec = arm.disarm_result("cleanup finally", actor="remote_script")
    assert rec["blocked"] is True
    assert rec["kind"] == "disarm_blocked"
    assert rec["external_disarm_conflict"] is True
    assert rec["mission_owner"] == "voice"
    assert arm.is_armed()
    assert os.path.exists(arm.lease_path())
    ev = arm.last_event()
    assert ev["blocked"] is True
    assert ev["actor"] == "remote_script"

def test_voice_stop_disarms_immediately_during_voice_mission(arm):
    arm.arm("wake", actor="voice")
    arm.set_mission_lease(owner="voice", pid=os.getpid(), reason="explore", mission="explore")
    rec = arm.disarm_result("stop phrase: stop", actor="voice")
    assert rec["blocked"] is False
    assert rec["removed"] is True
    assert rec["actor"] == "voice"
    assert not arm.is_armed()
    assert arm.read_lease() is None

def test_safety_stop_disarms_immediately_during_voice_mission(arm):
    arm.arm("wake", actor="voice")
    arm.set_mission_lease(owner="voice", pid=os.getpid(), reason="explore", mission="explore")
    rec = arm.disarm_result("ESTOP/MAINT latched", actor="safety", emergency=True)
    assert rec["blocked"] is False
    assert rec["removed"] is True
    assert rec["actor"] == "safety"
    assert not arm.is_armed()
    assert arm.read_lease() is None

def test_stale_dead_pid_lease_does_not_lock_disarm(arm):
    arm.arm("wake", actor="voice")
    # PID 2^31-3 is almost certainly not alive; fall back to a definitely-dead pid.
    dead = 2147483646
    if arm.pid_alive(dead):
        # spawn and reap a child to get a dead pid
        pid = os.fork()
        if pid == 0:
            os._exit(0)
        os.waitpid(pid, 0)
        dead = pid
    arm.set_mission_lease(owner="voice", pid=dead, reason="stale", mission="explore")
    assert not arm.lease_is_live()
    rec = arm.disarm_result("cleanup finally", actor="remote_script")
    assert rec["blocked"] is False
    assert rec["removed"] is True
    assert not arm.is_armed()

def test_session_abort_record_captures_provenance_and_conflict(arm):
    arm.arm("wake", actor="voice")
    arm.set_mission_lease(owner="voice", pid=os.getpid(), reason="explore", mission="explore")
    rec = arm.disarm_result("cleanup finally", actor="remote_script")
    assert rec["blocked"] is True
    fields = arm.session_abort_fields()
    assert fields["disarm_actor"] == "remote_script"
    assert fields["mission_owner"] == "voice"
    assert fields["external_disarm_conflict"] is True
    assert "cleanup finally" in (fields["disarm_reason"] or "")

def test_plain_armed_file_still_gates_is_armed(arm):
    # Back-compat: a bare ARMED file (legacy, no actor) still counts as armed.
    open(arm.path(), "w").write("{}")
    assert arm.is_armed()
    assert arm.disarm("legacy") is True
    assert not arm.is_armed()

def test_infer_actor_does_not_treat_bare_test_as_remote_script(arm):
    assert arm.infer_actor("test") == "other"
    assert arm.infer_actor("t") == "other"
    assert arm.infer_actor("cleanup finally") == "remote_script"
    assert arm.infer_actor("stop phrase: stop") == "voice"
    assert arm.infer_actor("gb-armguard start (boot default = disarmed)") == "safety"

def test_backward_compatible_disarm_return(arm):
    arm.arm("t", actor="other")
    assert arm.disarm("test") is True
    assert not arm.is_armed()
