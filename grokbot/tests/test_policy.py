"""Unlock, maintenance, follow loop, and grounded mission. No robot."""
import os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_estop
import gb_follow
import gb_health
import gb_maint
import gb_point
import gb_unlock
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

gb_unlock.reset_seen()
now = 1000.0
gates = {"allow_motion": True, "allow_autonomy": False, "range_ok": False, "epoch": 1}
prov = gb_unlock.fresh_provenance("drive forward for 2 seconds", now=now, epoch=1)
prov["t"] = now
prov["heard_at"] = now
motion = {"op": "timed", "lin": 0.08, "ang": 0.0, "dur": 2.0, "requested": prov["text"]}
latch = {"t": now - 30, "src": "voice", "kind": "normal", "why": "stop"}
d = gb_unlock.decide(prov["text"], prov, latch, gates, motion=motion, now=now)
check("fresh move unlocks a normal stop", d["allow"] and d["unlock"] and d["reason"] == "teleop", d)

gb_unlock.reset_seen()
for key in ("maint", "tip_hard", "tip_soft", "battery_shutdown", "watchdog", "physical", "hardware", "obstacle"):
    gb_unlock.reset_seen()
    blocked = dict(gates)
    blocked[key] = True
    d = gb_unlock.decide(prov["text"], prov, latch, blocked, motion=motion, now=now)
    check("no move while %s" % key, d["allow"] is False and d["unlock"] is False and d["store"] is False, d)

gb_unlock.reset_seen()
safety = {"t": now, "src": "gate-tilt", "kind": "safety"}
d = gb_unlock.decide(prov["text"], prov, safety, gates, motion=motion, now=now)
check("tilt latch is not cleared by a move", d["allow"] is False and d["reason"] in ("safety_latch", "tip_hard"), d)

gb_unlock.reset_seen()
d = gb_unlock.decide("don't drive forward", prov, latch, gates, motion=motion, now=now)
check("negated move does not unlock", d["unlock"] is False and d["reason"] == "negated", d)
d = gb_unlock.decide('please "drive forward"', prov, latch, gates, motion=motion, now=now)
check("quoted move does not unlock", d["reason"] == "quoted", d)
d = gb_unlock.decide("drive forward and then stop", prov, latch, gates, motion=motion, now=now)
check("stop plus move does not unlock", d["reason"] == "stop_dominates" and d["unlock"] is False, d)
old = dict(prov, heard_at=now - 10, id="old-id")
d = gb_unlock.decide(prov["text"], old, latch, gates, motion=motion, now=now)
check("stale command does not unlock", d["reason"] == "stale", d)
replay = dict(prov, epoch=0, id="epoch-id")
d = gb_unlock.decide(prov["text"], replay, latch, gates, motion=motion, now=now)
check("old epoch does not unlock", d["reason"] == "replay", d)
llm = gb_unlock.fresh_provenance(prov["text"], now=now, from_llm=True, epoch=1)
llm["heard_at"] = now
d = gb_unlock.decide(prov["text"], llm, latch, gates, motion=motion, now=now)
check("llm text does not unlock", d["reason"] == "origin", d)
remote = gb_unlock.fresh_provenance(prov["text"], source="web", now=now, origin={"validated": False, "local": False}, epoch=1)
remote["heard_at"] = now
d = gb_unlock.decide(prov["text"], remote, latch, gates, motion=motion, now=now)
check("unauthenticated web does not unlock", d["reason"] == "origin", d)
d = gb_unlock.decide("I am John", prov, latch, gates, motion=None, now=now)
check("i am john is not a move", d["reason"] == "not_auth" and d["unlock"] is False, d)

auto = dict(gates, autonomous=True, allow_autonomy=True, range_ok=False, allow_motion=True)
gb_unlock.reset_seen()
prov = gb_unlock.fresh_provenance("follow me", now=now, epoch=1)
prov["heard_at"] = now
d = gb_unlock.decide("follow me", prov, latch, auto, motion=None, now=now)
check("sensorless autonomous does not unlock", d["reason"] == "sensorless" and d["allow"] is False, d)
tele = dict(gates, range_ok=False)
gb_unlock.reset_seen()
d = gb_unlock.decide(prov["text"], prov, latch, tele, motion=motion, now=now)
check("explicit teleop can proceed without a range", d["allow"] and d["unlock"], d)

# maintenance survives a timer clear of a normal stop
tmp = tempfile.mkdtemp(prefix="wally_maint_")
estop = os.path.join(tmp, "ESTOP")
maint = os.path.join(tmp, "MAINT")
gb_estop.ESTOP = estop
gb_estop.SHARED_ESTOP = os.path.join(tmp, "ESTOP2")
gb_estop.LOG = os.path.join(tmp, "estop.log")
gb_estop.estop("voice", "stop")
gb_maint.write_lock([maint], now=now)
ac = gb_estop.AutoClear(15, clock=lambda: now + 100, on_clear=None)
# born is now+100, latch t is ~time.time(), so this latch may or may not predate.
# The maintenance file must remain either way. Force a clear of only the normal file.
gb_estop.clear("test")
check("clearing a normal stop leaves maintenance", os.path.exists(maint) and not os.path.exists(estop))
check("maintenance clear requires the explicit flag", gb_maint.clear_lock(maint, explicit=False) is False and os.path.exists(maint))
check("explicit clear removes only MAINT", gb_maint.clear_lock(maint, explicit=True) and not os.path.exists(maint))
check("maintenance clear refuses an ESTOP path", gb_maint.clear_lock(estop, explicit=True) is False)

# follow loop
health = {"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.9, "user_lidar": True}
loop = gb_follow.FollowLoop()
loop.start(now)
track = {"depth_m": 3.2, "source": "lidar", "fresh": True, "conf": 0.9, "bearing": 0.0, "ambiguous": False}
a = loop.step(now, health, track, True)
b = loop.step(now + 0.2, health, track, True)
check("follow keeps commanding", a["allow"] and b["allow"] and loop.active, (a, b))
low = dict(track, conf=0.2)
c = loop.step(now + 0.4, health, low, True)
check("low confidence cancels follow", c["reason"] == "confidence" and not loop.active, c)
loop.start(now)
d = loop.step(now, health, dict(track, ambiguous=True), True)
check("ambiguous person cancels follow", d["reason"] == "ambiguous", d)
loop.start(now)
gap = loop.step(now, health, None, True)
held = loop.step(now + 0.2, health, None, True)
lost = loop.step(now + gb_follow.LOST_S, health, None, True)
check("person lost after the gap", gap["reason"] == "person_lost" and not loop.active, (gap, held, lost))
loop.start(now)
loop.step(now, health, track, True)
gap = loop.step(now + 0.2, health, None, True)
lost = loop.step(now + 0.2 + gb_follow.LOST_S, health, None, True)
check("a track that disappears cancels after the gap", gap["reason"] == "person_gap" and lost["reason"] == "person_lost_later", (gap, lost))
loop.start(now)
stale = loop.step(now, health, {"depth_m": 3, "source": "depth", "fresh": True, "conf": 0.9,
                                "depth_age": 1.0, "unit": "crosscheck_mm"}, True)
check("stale depth cancels follow", stale["reason"] == "depth_stale", stale)
loop.start(now)
stopped = loop.step(now, health, track, True, {"estop": True})
check("estop cancels follow", stopped["reason"] == "estop" and stopped["op"] == "stop", stopped)

# grounded mission: llm and stale depth produce no target
raw = {"from_llm": True, "candidates": [{"name": "table", "conf": 0.9, "depth_m": 1.2, "depth_valid": True}]}
g = gb_point.ground_observation(raw, now)
check("llm observation has no target", g["candidates"] == [] and g["from_llm"], g)
raw = {"camera_ok": True, "candidates": [
    {"name": "table", "conf": 0.2, "depth_m": 1.2, "depth_valid": True, "depth_t": now},
    {"name": "table", "conf": 0.9, "depth_m": 1.2, "depth_valid": True, "depth_t": now - 5, "x": 1, "y": 2},
]}
g = gb_point.ground_observation(raw, now)
check("low confidence and stale depth are dropped", g["candidates"] == [], g)
raw = {"camera_ok": True, "pose": {"fresh": True, "frame": "map", "x": 0.0, "y": 0.0, "yaw": 0.0},
       "candidates": [{"name": "cup", "conf": 0.8, "depth_m": 1.5, "depth_valid": True, "depth_t": now,
                       "source": "lidar", "x": 1.2, "y": -0.2}]}
g = gb_point.ground_observation(raw, now)
check("measured target keeps its pose", len(g["candidates"]) == 1 and g["candidates"][0]["x"] == 1.2 and g["tf_ok"], g)
goal = gb_point.navigate_to_pose_goal(1.2, -0.4, 0.0)
check("nav goal yaw is a unit quaternion", abs(goal["pose"]["pose"]["orientation"]["w"] - 1.0) < 1e-9, goal)

# wake still exact
check("wake phrase", gb_wally.classify_wake("wake up Wally") == (True, ""))
check("stop is not a wake", gb_wally.classify_wake("stop") is None)

# startup suppress is an explicit switch
voice = open(os.path.join(ROOT, "grokbot_voice.py")).read()
check("suppress switch gates the greeting", 'SUPPRESS_STARTUP_SPEECH' in voice and 'greet.strip() and E("SUPPRESS_STARTUP_SPEECH"' in voice)
check("autoclear migration edits one key", 'KEY = "ESTOP_AUTOCLEAR_S"' in open(os.path.join(os.path.dirname(ROOT), "deploy", "migrate_autoclear.py")).read())

print("test_policy: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
