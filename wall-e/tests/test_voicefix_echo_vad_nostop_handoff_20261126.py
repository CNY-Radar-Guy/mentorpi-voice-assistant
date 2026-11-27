"""2026-11-26 voicefix: self-echo, Silero VAD, no 'stop' trigger, wake-to-mission handoff,
and explore_running process matching. Pure Python: no ROS, no motors, no live explore."""
import os, sys, time, tempfile
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

import gb_estop as g
import gb_wally
import gb_explore
import gb_procmatch
import grokbot_voice as GV

TTS = "Hold still. 3, 2, 1."

# --------------------------------------------------------------------------- 1. self-echo
check("incident echo: 'Go stop. 3-2-1.' is rejected as speaker echo",
      g.speaker_echo("Go stop. 3-2-1.", [TTS]) is True)
check("incident variant without punctuation still echo",
      g.speaker_echo("go stop 3 2 1", [TTS]) is True)
check("exact TTS line is echo", g.speaker_echo(TTS, [TTS]) is True)
check("bare halt during countdown is NOT echo",
      g.speaker_echo("halt", [TTS]) is False)
check("halt mixed with countdown leftover is NOT echo",
      g.speaker_echo("halt 3 2 1", [TTS]) is False)
check("freeze during countdown is NOT echo",
      g.speaker_echo("freeze", [TTS]) is False)
check("unrelated drive is not echo",
      g.speaker_echo("go forward for 4 seconds", [TTS]) is False)
check("bargein ignores the incident echo (does not kill)",
      g.bargein_decision("Go stop. 3-2-1.", [TTS])["action"] == "ignore")
check("bargein still kills on halt over that TTS",
      g.bargein_decision("halt", [TTS])["action"] == "kill")
check("resume resume is not speaker-echo of a 'say resume' line",
      g.speaker_echo("resume resume", ["Say resume when it's safe."]) is False)
check("incident echo still rejected after resume exception",
      g.speaker_echo("Go stop. 3-2-1.", [TTS]) is True)
check("stop_action ignores the incident echo",
      g.stop_action("Go stop. 3-2-1.", moving=True, spoken_texts=[TTS]) is None)
check("hyphenated 3-2-1 normalizes against '3, 2, 1'",
      "3 2 1" in g._echo_norm("Go stop. 3-2-1."))

# --------------------------------------------------------------------------- 2. Silero VAD / mic-gate params
check("STT_VAD_FILTER stays on", GV.STT_VAD_FILTER is True)
check("STT_VAD_THRESHOLD is 0.25 (was 0.40)", abs(GV.STT_VAD_THRESHOLD - 0.25) < 1e-9, GV.STT_VAD_THRESHOLD)
check("STT_MIN_SPEECH_S is 0.15 (was 0.30)", abs(GV.STT_MIN_SPEECH_S - 0.15) < 1e-9, GV.STT_MIN_SPEECH_S)
check("STT_VAD_MIN_SPEECH_MS is 100 (was hardcoded 150)", GV.STT_VAD_MIN_SPEECH_MS == 100, GV.STT_VAD_MIN_SPEECH_MS)
src = open(os.path.join(ROOT, "grokbot_voice.py"), encoding="utf-8").read()
check("stt() still passes vad_filter=STT_VAD_FILTER", "vad_filter=STT_VAD_FILTER" in src)
check("stt() now passes tuned vad_parameters", "vad_parameters=_vad_opts" in src)
check("speech_seconds uses STT_VAD_MIN_SPEECH_MS", "min_speech_duration_ms=STT_VAD_MIN_SPEECH_MS" in src)
check("VAD_NOISE_CAP still 1200", abs(GV.VAD_NOISE_CAP - 1200) < 1e-9)
check("moving VAD ratio still 1.5-2.0", 1.5 <= GV.VAD_START_RATIO_MOVING <= 2.0)

# --------------------------------------------------------------------------- 3. "stop" is no longer a command
for ph in ("stop", "Stop!", "wally stop", "Wally stop", "go stop", "we stop",
           "okay stop", "stop now", "G-Bot stop", "emergency stop", "Go stop. 3-2-1."):
    check("%r is not a kill phrase (idle)" % ph, g.kill_phrase(ph, moving=False) is False)
    check("%r is not a kill phrase (moving)" % ph, g.kill_phrase(ph, moving=True) is False)
    check("%r does not halt/latch via stop_action" % ph,
          g.stop_action(ph, moving=True) is None)

for ph in ("halt", "Halt!", "wally halt", "halt wally", "freeze", "abort",
           "cancel", "cancel that", "disarm", "whoa", "hold up", "stand still"):
    check("%r still triggers halt/kill" % ph,
          g.kill_phrase(ph, moving=True) is True and g.stop_action(ph, moving=True) == "halt")

check("emergency halt still latches (non-stop latch phrase)",
      g.stop_action("emergency halt") == "latch")
check("emergency abort still latches", g.stop_action("emergency abort") == "latch")
check("drive then stop is still not a kill",
      g.kill_phrase("forward for 5 seconds then stop", moving=True) is False)
check("drive then halt is still not a kill",
      g.kill_phrase("forward for 5 seconds then halt", moving=True) is False)
check("is_mission_free_command('stop') is False", GV.is_mission_free_command("stop") is False)
check("is_mission_free_command('halt') is True", GV.is_mission_free_command("halt") is True)
check("is_mission_free_command('cancel') is True", GV.is_mission_free_command("cancel") is True)

REMAINING = (
    "halt", "freeze", "abort", "cancel", "cancel that", "disarm",
    "whoa", "woah", "hold up", "stand still",
    "emergency halt", "emergency abort",
    "wally halt", "halt wally",
)
check("documented remaining halt/cancel set is non-empty", len(REMAINING) >= 8)

# --------------------------------------------------------------------------- 4. wake-to-mission handoff (no live explore)
check("wake up wally wakes", gb_wally.software_wake("Wake up Wally") is True)
check("hey wally wakes", gb_wally.software_wake("Hey Wally") is True)
check("wally wake up wakes", gb_wally.software_wake("Wally wake up") is True)
matched, rest = gb_wally.classify_wake("Hey Wally, explore the room")
check("same utterance: hey wally + explore remainder",
      matched is True and "explore" in (rest or "").lower(), (matched, rest))
plan = gb_explore.parse_plan(rest)
check("same utterance remainder parses as explore mission",
      bool(plan) and plan.get("mission") == "explore", plan)
plan2 = gb_explore.parse_plan("Explore the room")
check("second utterance 'Explore the room' parses as explore",
      bool(plan2) and plan2.get("mission") == "explore", plan2)
plan3 = gb_explore.parse_plan("it's Florida room")
check("Whisper near-miss 'it's Florida room' parses as explore",
      bool(plan3) and plan3.get("mission") == "explore", plan3)
check("ambiguous 'more of the room' does NOT start explore",
      gb_explore.parse_plan("more of the room") is None)
check("joke request is not a mission", gb_explore.parse_plan("tell me a joke") is None)

ack = GV.bare_wake_prompt()
check("bare wake prompt keeps the ack and asks for a command",
      "listening" in ack.lower() and "what should i do" in ack.lower(), ack)
check("handoff: wake-only is classified, not a mission",
      GV.consider_handoff_command("Hey Wally", spoken_texts=["hi"]) == ("wake_only", ""))
dec = GV.consider_handoff_command("Hey Wally, explore the room", spoken_texts=["hello"])
check("handoff: wake+explore is a mission",
      dec and dec[0] == "mission" and dec[1].get("mission") == "explore", dec)
dec2 = GV.consider_handoff_command("Explore the room", spoken_texts=[ack], awake=True)
check("handoff: explore while awake is a mission",
      dec2 and dec2[0] == "mission" and dec2[1].get("mission") == "explore", dec2)
dec3 = GV.consider_handoff_command("Go stop. 3-2-1.", spoken_texts=[TTS], awake=True)
check("handoff: TTS echo is not queued as a command", dec3 is None, dec3)
# bare-wake window: first command allowed when followup is False
import gb_wakegate as WG
check("first command after bare wake is motion-allowed (awake, not followup)",
      WG.motion_allowed(False, True, False) is True)
check("later follow-up without a fresh wake is not motion-allowed",
      WG.motion_allowed(False, True, True) is False)
check("same-utterance wake+command is motion-allowed",
      WG.motion_allowed(True, False, False) is True)
# window helper
end = 1000.0
until = g.followup_deadline(end)
check("bare-wake listen window is FOLLOWUP_S from playback end (20s)",
      abs(until - (end + 20)) < 1e-9, until)

# queue / take does not start motion
GV.PENDING_CMD["item"] = None
queued = GV.queue_pending_command("Hey Wally, explore the room", src="test", spoken_texts=["hello"], awake=False)
item = GV.take_pending_command()
check("queue_pending_command stores a mission and does not move",
      queued is True and item and item["kind"] == "mission" and item["payload"]["mission"] == "explore")
check("queue is empty after take", GV.take_pending_command() is None)

# --------------------------------------------------------------------------- 5. explore_running matcher
fn = gb_procmatch.is_live_explore_cmd
check("python gb_explore.py is a live explore process",
      fn("python3 /home/pi/grokbot/releases/e48d3d8/gb_explore.py") is True)
check("venv python with args is a live explore process",
      fn("/home/pi/grokbot/venv/bin/python /home/pi/grokbot/releases/e48d3d8/gb_explore.py --seconds 60") is True)
check("zsh grep mentioning gb_explore.py is NOT explore_running",
      fn("zsh -c grep -n -E -i collect|twice /home/pi/grokbot/releases/e48d3d8/gb_explore.py") is False)
check("zsh sed mentioning gb_explore.py is NOT explore_running",
      fn("zsh -c sed -n 1,20p;300,330p /home/pi/grokbot/releases/e48d3d8/gb_explore.py") is False)
check("plain grep is not explore_running", fn("grep gb_explore.py") is False)
check("python -c mentioning the file is not explore_running",
      fn("python3 -c open('/home/pi/grokbot/releases/e48d3d8/gb_explore.py')") is False)
check("grokbot_voice.py is not explore_running",
      fn("python3 /home/pi/grokbot/releases/e48d3d8/grokbot_voice.py") is False)

# safety rails this change must not touch
check("HARD_MAX_LIN is 1.0 (GB-48 autonomous cap)", abs(gb_explore.HARD_MAX_LIN - 1.0) < 1e-12, gb_explore.HARD_MAX_LIN)

print("test_voicefix_echo_vad_nostop_handoff_20261126: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
