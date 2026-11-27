"""Tests for the strict wake requirement (gb_wakegate + its hooks in grokbot_voice.py)."""
import os, re, sys, threading
sys.path.insert(0, os.environ.get("GB_REL", "/home/pi/grokbot/releases/e48d3d8"))
import tempfile; os.environ["GB_ARM_DIR"] = tempfile.mkdtemp()   # 2026-11-25: arming is a precondition
import gb_arm; gb_arm.arm("test")
import gb_wakegate as WG
P = F = 0
def check(n, ok):
    global P, F; P += bool(ok); F += (not ok); print(("PASS " if ok else "FAIL ") + n)
check("wake phrase in utterance -> motion allowed", WG.motion_allowed(True, False, False))
check("first command after bare 'wake up wally' -> allowed", WG.motion_allowed(False, True, False))
check("follow-up window after a reply (overheard/chat) -> rejected", not WG.motion_allowed(False, True, True))
check("idle, no wake -> rejected", not WG.motion_allowed(False, False, False))
# replay tonight's transcripts (all were wake=False awake=True in a follow-up window)
for when, txt in [("03:27:35", "while he has no map and no sense of where it is so it can only react to obstacles in real time. The light art of bodylink is missing too."),
                  ("03:46:15", "It's starting to stack on the latest map with speeds held at zero. It also warned the map might be smeared from the elevated run"),
                  ("03:47:12", "Yes, go for the couch.")]:
    WG.set_context(WG.motion_allowed(False, True, True), txt)
    check("replay %s overheard -> mission/navigate rejected" % when, WG.block("mission", txt) == WG.REJECT_LINE and WG.block("intent:navigate", txt) is not None)
WG.set_context(WG.motion_allowed(True, False, False), "wake up wally explore the room")
check("woken 'wake up wally, explore the room' -> mission accepted", WG.block("mission", "explore") is None)
WG.set_context(WG.motion_allowed(False, True, True), "explore the room")
check("overheard 'explore the room' -> rejected", WG.block("mission", "explore") is not None)
res = {}
t = threading.Thread(target=lambda: res.__setitem__("web", WG.block("mission", "web")))
t.start(); t.join()
check("context is thread-local: web/api thread unaffected while voice gate closed", res["web"] is None)
WG.clear_context(); check("clear_context reopens", WG.is_open())
check("local motion regex: follow / go to / explore / turn left", all(WG.looks_like_motion(x) for x in ("follow me", "go to the couch", "explore the room", "turn left")))
check("local motion regex ignores chat / lights / stop", not any(WG.looks_like_motion(x) for x in ("what time is it", "turn on the lights", "how's the battery")))
src = open(os.path.join(os.environ.get("GB_REL", "/home/pi/grokbot/releases/e48d3d8"), "grokbot_voice.py")).read()
check("voice: start_mission is gated", re.search(r"def start_mission\(plan\):\n(?:.*\n){0,3}.*WG\.block\(\"mission\"", src) is not None)
check("voice: do_timed is gated", re.search(r"def do_timed\(p\):\n(?:.*\n){0,4}.*WG\.block\(\"timed\"", src) is not None)
check("voice: dispatch gates MOTION_INTENTS", "if it in WG.MOTION_INTENTS" in src)
check("voice: local follow/go-to gated before gb_wally/gb_perceive", "WG.looks_like_motion(t)" in src)
check("voice: barge-in drive gated", re.search(r"def barge_drive\(dec, src\):\n(?:.*\n){0,6}.*WG\.block\(\"barge-drive\"", src) is not None)
check("voice: main loop sets context from wake/awake/followup", "WG.set_context(WG.motion_allowed(matched, awake, followup), text)" in src)
i_stop = src.find("KILL SWITCH FIRST"); i_ctx = src.find("WG.set_context(WG.motion_allowed")
check("stop / e-stop handling still runs before (and independent of) the wake gate", 0 < i_stop < i_ctx)
print("RESULT: %d passed, %d failed" % (P, F))
