"""2026-11-27 stay-awake after mission: remain in wake-up mode until bench or power-off.

Pure Python, no ROS, no motors, no live explore. Does not arm the robot.
"""
import os, sys, time, tempfile, json

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
try:
    import sandbox
    sandbox.install()
except Exception:
    pass
sys.path.insert(0, ROOT)

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info if info != "" else cond)

import grokbot_voice as GV
import gb_wakegate as WG
import gb_estop as ESTOP
import gb_wally

# Isolate any mode.json reads the helpers might do.
mode_dir = tempfile.mkdtemp(prefix="gbmode_")
mode_path = os.path.join(mode_dir, "mode.json")
def write_mode(mode):
    with open(mode_path, "w") as f:
        json.dump({"mode": mode, "set_by": "test"}, f)
    os.environ["GB_MODE_FILE"] = mode_path

write_mode("ground")
try:
    import gb_floor
    gb_floor.HOST_MODE_FILE = mode_path
    gb_floor._CACHE.update(t=0.0, mode=None)
except Exception:
    pass

GV.reset_awake_state()

# --------------------------------------------------------------------------- boot starts asleep
check("boot: no stay-awake session", GV.session_awake() is False)
check("boot: not listening without a wake / window / mission",
      GV.stay_listening(session=False, mission_running=False, awake_until=0.0, now=time.time()) is False)
check("_wake_window_open is closed at boot", GV._wake_window_open(now=time.time()) is False)

# --------------------------------------------------------------------------- wake opens a session that outlives a mission and idle time
now = time.time()
GV.enter_awake_session(current_mode="ground")
check("wake opens stay-awake session", GV.session_awake() is True)
check("awake during an active exploration mission",
      GV.stay_listening(mission_running=True, awake_until=now - 30.0, now=now) is True)
check("after mission ends, still listening (expired 20 s window, no mission)",
      GV.stay_listening(mission_running=False, awake_until=now - 30.0, now=now) is True)
check("idle time long after the mission still does not sleep",
      GV.stay_listening(mission_running=False, awake_until=now - 3600.0, now=now + 3600.0) is True)
check("_wake_window_open stays open after the numeric window would have expired",
      GV._wake_window_open(now=now + 3600.0) is True)
check("followup_still_open still expires when idle AND no session flag is passed",
      GV.followup_still_open(now - 30.0, now=now, mission_running=False) is False)
check("followup_still_open still stays open during a mission",
      GV.followup_still_open(now - 30.0, now=now, mission_running=True) is True)

# A new command (including a new mission) is accepted in wake-up mode without a fresh wake word.
# Session-awake is OR'd into the voice-loop motion context (same as a bare-wake first command).
allowed = WG.motion_allowed(False, True, True) or GV.session_awake()
check("post-mission command may start motion without a new wake word (session awake)",
      allowed is True)
check("without a session, follow-up chat still cannot start motion by itself",
      WG.motion_allowed(False, True, True) is False)

# --------------------------------------------------------------------------- bench-mode / plug-in entry returns him to sleep
check("ground -> ground is not a bench entry",
      GV.bench_mode_entered("ground", "ground") is False)
check("bench -> bench is not a fresh plug-in",
      GV.bench_mode_entered("bench", "bench") is False)
check("unset -> bench is not treated as a fresh plug-in (already on the stand)",
      GV.bench_mode_entered(None, "bench") is False)
check("unknown -> bench is not a fresh plug-in",
      GV.bench_mode_entered("unknown", "bench") is False)
check("ground -> bench IS a bench / plug-in entry",
      GV.bench_mode_entered("ground", "bench") is True)

slept = GV.apply_bench_sleep_if_needed(current_mode="bench", previous_mode="ground")
check("entering bench mode ends the stay-awake session", slept is True and GV.session_awake() is False)
check("after bench entry, commands require a fresh wake word",
      GV.stay_listening(mission_running=False, awake_until=0.0, now=time.time()) is False)
check("after bench entry, _wake_window_open is closed",
      GV._wake_window_open(now=time.time()) is False)

# Waking while already in bench (bench tests) must stay awake until a later re-entry or power-off.
GV.reset_awake_state()
GV.enter_awake_session(current_mode="bench")
check("wake while already in bench opens a session", GV.session_awake() is True)
slept = GV.apply_bench_sleep_if_needed(current_mode="bench", previous_mode="bench")
check("already-in-bench does not immediately sleep a just-woken session",
      slept is False and GV.session_awake() is True)
# leave the stand (ground) then plug back in / return to bench
slept = GV.apply_bench_sleep_if_needed(current_mode="ground", previous_mode="bench")
check("leaving bench for ground does not sleep", slept is False and GV.session_awake() is True)
slept = GV.apply_bench_sleep_if_needed(current_mode="bench", previous_mode="ground")
check("plugging back in / re-entering bench returns him to sleep",
      slept is True and GV.session_awake() is False)

# --------------------------------------------------------------------------- existing wake / stop protections still present
src = open(os.path.join(ROOT, "grokbot_voice.py"), encoding="utf-8").read()
check("idle wake-window close is gated so a stay-awake session does not expire",
      "not session_awake()" in src and "wake window closed" in src)
check("voice loop opens a stay-awake session on a matched wake phrase",
      "enter_awake_session()" in src)
check("voice loop sleeps on bench-mode entry",
      "apply_bench_sleep_if_needed" in src and "entered bench mode" in src)
check("intercom hold hook is still present (GB-60)",
      "intercom_hold()" in src and "intercom ended: normal dual listening restored" in src)
check("kill switch still runs first", "KILL SWITCH FIRST" in src)
for phrase in ("halt", "freeze", "abort", "cancel", "disarm", "whoa", "hold up", "stand still"):
    check("mission-free / stop phrase still recognized: %s" % phrase,
          GV.is_mission_free_command(phrase) or ESTOP.kill_phrase(phrase, moving=True)
          or ESTOP.stop_action(phrase, moving=True))
check("'stop' is still not a kill/trigger phrase",
      GV.is_mission_free_command("stop") is False and ESTOP.kill_phrase("stop", moving=True) is False)
check("emergency halt still a free command or kill",
      GV.is_mission_free_command("emergency halt") or ESTOP.kill_phrase("emergency halt", moving=True)
      or ESTOP.stop_action("emergency halt", moving=True))
check("software wake phrase still exact",
      hasattr(gb_wally, "software_wake") and gb_wally.software_wake("wake up Wally") is True)

print("test_stayawake_post_mission_20261127: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
