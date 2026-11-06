"""Synthetic tests for Wally sensor health, teaching, follow, and pointed missions.

No motors, no microphone, no tracks. Sensor samples are dicts.
"""
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import gb_cadence
import gb_estop
import gb_follow
import gb_health
import gb_point
import gb_teach
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL:", name, info)

GOOD_L = {"fresh": True, "valid_frac": 0.96, "driver_ok": True}
BAD_L = {"fresh": False, "valid_frac": 0.0, "driver_ok": False, "fault": "stale"}
GOOD_D = {"fresh": True, "valid_frac": 0.5, "unit_ok": True, "unit_how": "crosscheck_mm"}
BAD_D = {"fresh": False, "valid_frac": 0.0, "unit_ok": False}

def boot_lidar():
    h = gb_health.Health()
    h.grace_s = 0
    said = []
    for _ in range(2):
        said += h.update(lidar=GOOD_L, depth=GOOD_D)
    return h, said

# --- health transitions ---
h, said = boot_lidar()
check("lidar online phrase", gb_health.LIDAR_ONLINE in said, said)
check("rgbd online phrase", gb_health.DEPTH_ONLINE in said, said)
check("primary mode lidar", h.mode == "lidar", h.mode)
said = []
for _ in range(2):
    said += h.update(lidar=BAD_L)
check("fallback phrase", gb_health.LIDAR_FALLBACK in said, said)
check("depth mode despite old fault", h.mode == "depth" and h.last_lidar_fault, h.snapshot())
dec = gb_health.motion_policy("autonomous", h.mode, {})
check("old fault does not block depth", dec.allow and dec.reason == "depth", dec.as_tuple())
said = []
epoch = h.goal_epoch
for _ in range(2):
    said += h.update(lidar=BAD_L, depth=BAD_D)
check("exact no-range phrase", gb_health.NO_RANGE in said, said)
check("goals cancelled", h.goal_epoch == epoch + 1)
check("no auto resume", h.resume_goal() is False)
for _ in range(2):
    h.update(depth=GOOD_D, lidar=BAD_L)
check("recovery keeps epoch", h.mode == "depth" and h.goal_epoch == epoch + 1 and h.resume_goal() is False, h.goal_epoch)

h_rgb = gb_health.Health()
h_rgb.grace_s = 0
h_rgb.ever_online = True
h_rgb.t0 = 0
for _ in range(2):
    h_rgb.update(rgb={"fresh": True}, lidar=BAD_L, depth=BAD_D)
check("rgb alone is not depth", h_rgb.mode == "none", h_rgb.mode)

check("blind empty", gb_health.sector_is_clear(0, 12, None, 0.3) is False)
check("blind sparse", gb_health.sector_is_clear(1, 12, 4.0, 0.3) is False)
check("near contact not clear", gb_health.sector_is_clear(10, 12, 0.2, 0.3) is False)
check("measured clear", gb_health.sector_is_clear(10, 12, 1.5, 0.3) is True)

check("teleop sensorless allowed", gb_health.motion_policy("teleop", "none", {}).allow)
check("teleop still loses to estop", not gb_health.motion_policy("teleop", "none", {"estop": True}).allow)
check("teleop loses to watchdog", not gb_health.motion_policy("teleop", "lidar", {"watchdog": True}).allow)
check("teleop loses to battery", not gb_health.motion_policy("teleop", "depth", {"battery_shutdown": True}).allow)
tip = gb_health.motion_policy("teleop", "lidar", {"tip_soft": True})
check("15-25 tip stops teleop", (not tip.allow) and tip.reason == "tip_soft" and tip.scale == 0.0)
check("hard tip stops", not gb_health.motion_policy("autonomous", "lidar", {"tip_hard": True}).allow)
auto = gb_health.motion_policy("autonomous", "none", {})
check("autonomous no-range phrase", (not auto.allow) and auto.speech == gb_health.NO_RANGE)
check("observed load 2.2 stays in budget", gb_health.under_budget(2.2))
check("load shed", (not gb_health.under_budget(4.0)) and gb_health.under_budget(1.0))
check("depth fraction ignores zeros once unit is mm", abs(gb_health.depth_valid_fraction([0, 0, 400, 800], 0.001) - 0.5) < 1e-9)
check("unconfirmed depth unit is not ranging", gb_health.depth_valid_fraction([400, 800], None) == 0.0)
import gb_obstacle as OB
mm, why = OB.resolve_depth_scale("16UC1", [1500] * 20, 1.5)
check("mono16 matches lidar as millimetres", mm == 0.001 and why == "crosscheck_mm", (mm, why))
no, why = OB.resolve_depth_scale("16UC1", [1500] * 20, None)
check("mono16 without a range stays unconfirmed", no is None, why)
metres, why = OB.resolve_depth_scale("32FC1")
check("32FC1 is metres", metres == 1.0, why)
bad, why = OB.resolve_depth_scale("16UC1", [50] * 20, 1.5)
check("disagreeing mono16 is not a range", bad is None, why)
ranges = [float("nan")] * 10 + [2.0] * 390
st = gb_health.lidar_status(ranges, "lidar_frame")
check("healthy scan is online without opening a port", st["fresh"] and st["driver_ok"] and st["fault"] is None, st)
badf = gb_health.lidar_status(ranges, "wrong_frame")
check("wrong lidar frame is not online", not badf["fresh"] and badf["fault"] == "unexpected frame")
raw = __import__("struct").pack("<20H", *([1500] * 20))
dep = gb_health.depth_status({"encoding": "16UC1", "data": raw}, 1.5)
check("cross-checked depth frame is a unit", dep["unit_ok"] and dep["scale_m"] == 0.001, dep)
nounit = gb_health.depth_status({"encoding": "16UC1", "data": raw}, None)
check("depth bytes alone are not ranging", nounit["unit_ok"] is False and nounit["fresh"] is False, nounit)

t0 = time.time()
hh = gb_health.Health()
for i in range(2000):
    hh.update(lidar=GOOD_L if i % 5 else BAD_L, depth=GOOD_D)
check("health budget 2000 updates", time.time() - t0 < 1.0, time.time() - t0)

# --- cadence ---
fast = gb_cadence.length_scale(2.0, "one two three four five six seven eight nine ten")
slow = gb_cadence.length_scale(8.0, "one two three four five six seven eight nine ten")
check("faster speech shorter scale", fast < slow, (fast, slow))
check("scale clamped", 0.85 <= fast <= 1.25 and 0.85 <= slow <= 1.25, (fast, slow))

# --- teaching ---
td = tempfile.mkdtemp(prefix="wally_teach_")
teacher = gb_teach.Teacher(os.path.join(td, "t.db"))

def picture(seed):
    return [[((x * 13 + seed * (y + 3)) % 80) + (30 if x < 4 else 0) for x in range(9)] for y in range(8)]

gb_wally._teach = teacher
gb_wally.set_capture(lambda: (picture(3), [200, 0, 800]))
gb_wally.set_autonomy(False)
r1 = gb_wally.handle("wake up Wally, I'm going to tell you what this object is")
check("teach prompt", r1 and "Tell me the name" in r1, r1)
r2 = gb_wally.handle("a coffee mug")
check("teach confirm", r2 == "Got it, that's a coffee mug.", r2)
rec = teacher.recognize(picture(3))
check("recognize taught", rec["kind"] == "match" and rec["name"] == "coffee mug", rec)
r3 = gb_wally.handle("no, that's a cup")
check("correction confirm", r3 == "Got it, that's a cup.", r3)
rec2 = teacher.recognize(picture(3))
check("corrected name", rec2["name"] == "cup", rec2)
far = teacher.recognize(picture(9))
check("different view not confident match", far["kind"] != "match", far)
check("vision cannot move", gb_wally.vision_may_move("drive forward now") is False)
# retention
for i in range(50):
    teacher.teach("thing %d" % i, picture(i + 20))
with teacher._conn() as c:
    n = c.execute("SELECT count(*) FROM objects").fetchone()[0]
check("retention cap", n <= gb_teach.MAX_OBJECTS, n)
check("stored crop bounded", teacher.stored_bytes() <= gb_teach.MAX_OBJECTS * gb_teach.MAX_BYTES, teacher.stored_bytes())

# --- follow ---
cmd = gb_follow.follow_command(3.2, "lidar", 0.1, True, autonomy=True)
check("follow closes slowly", 0 < cmd["lin"] <= gb_follow.MAX_LIN, cmd)
cmd = gb_follow.follow_command(2.0, "lidar", 0.0, True, autonomy=True)
check("follow holds 2m", cmd["lin"] == 0.0 and cmd["allow"], cmd)
cmd = gb_follow.follow_command(1.1, "depth", 0.0, True, autonomy=True)
check("follow does not close inside 1.5m", cmd["lin"] == 0.0, cmd)
cmd = gb_follow.follow_command(None, "none", 0.0, False, autonomy=True)
check("follow no sensors phrase", cmd["lin"] == 0.0 and cmd["speech"] == gb_health.NO_RANGE, cmd)
cmd = gb_follow.follow_command(3.0, "rgb", 0.0, True, autonomy=True)
check("rgb is not a follow range", cmd["speech"] == gb_health.NO_RANGE, cmd)
cmd = gb_follow.follow_command(3.0, "lidar", 0.0, True, autonomy=True, protections={"estop": True})
check("follow estop", cmd["allow"] is False and cmd["lin"] == 0.0, cmd)
cmd = gb_follow.follow_command(3.0, "lidar", 0.0, True, autonomy=False)
check("follow autonomy default off", cmd["allow"] is False, cmd)

# --- pointed mission ---
md = tempfile.mkdtemp(prefix="wally_mission_")
mission = gb_point.PointMission(os.path.join(md, "status.json"))
# unsafe reboot file
with open(mission.path, "w") as f:
    f.write('{"goal": {"lin": 0.2}, "phase": "approach"}')
check("reboot drops goal", mission.load() is None and mission.goal is None and not os.path.exists(mission.path))

h = gb_health.Health()
h.grace_s = 0
h.update(lidar=GOOD_L, depth=GOOD_D)
h.update(lidar=GOOD_L, depth=GOOD_D)
obs_amb = {"candidates": [
    {"name": "table", "conf": 0.9, "depth_m": 1.2, "depth_valid": True, "bearing": 0.0, "sector_clear": True},
    {"name": "table", "conf": 0.8, "depth_m": 1.4, "depth_valid": True, "bearing": 0.4, "sector_clear": True},
], "person": {"conf": 0.9, "depth_m": 2.4, "fresh": True, "track_id": "p1", "source": "lidar"},
   "localized": True, "start": {"x": 0, "y": 0, "yaw": 0}}
res = mission.start("table", 15, obs_amb, h, autonomy=True)
check("ambiguous does not drive", res["ok"] is False and mission.commands == [], res)
check("vision caption cannot start", mission.start("table", 15, dict(obs_amb, from_vision=True, candidates=obs_amb["candidates"][:1]), h, autonomy=True)["ok"] is False)

import gb_nav
one = {"candidates": [dict(obs_amb["candidates"][0], depth_m=1.2, x=1.5, y=0.2)],
       "person": dict(obs_amb["person"], x=0.2, y=0.0), "localized": True, "tf_ok": True,
       "start": {"x": 1, "y": 0, "yaw": 0}}
res = mission.start("table", 15, one, h, autonomy=True)
check("no navigation does not drive", res["ok"] is False and mission.phase == "idle", res)
nav = gb_nav.ScriptedNav(["active", "succeeded", "succeeded"])
mission.transport = nav
res = mission.start("table", 1, one, h, autonomy=True)
check("measured pose is sent to navigation", res["ok"] is True and len(nav.goals) == 1, res)
step = mission.tick(0.2, one, h)
check("approach does not open-loop", step["lin"] == 0.0 and step["ang"] == 0.0 and step["phase"] == "approach", step)
step = mission.tick(0.2, one, h)
check("navigation arrival is zero", step["lin"] == 0.0 and step["phase"] == "dwell", step)
step = mission.tick(1.0, one, h)
check("dwell stays zero", step["lin"] == 0.0 and step.get("why") == "dwell_done", step)
step = mission.tick(0.2, one, h)
check("return is a second navigation goal", step["lin"] == 0.0 and (step.get("why") == "back_with_person" or step["phase"] == "done"), step)

# cancel
mission.transport = gb_nav.ScriptedNav(["active"])
mission.start("table", 15, one, h, autonomy=True)
mission.cancel("cancel")
check("cancel zeros", mission.phase == "idle" and mission.commands[-1]["lin"] == 0.0)

# sensor dropout mid goal
mission.transport = gb_nav.ScriptedNav(["active"])
mission.start("table", 15, one, h, autonomy=True)
h.update(lidar=BAD_L, depth=BAD_D)
h.update(lidar=BAD_L, depth=BAD_D)
step = mission.tick(0.2, one, h)
check("dropout stops with phrase", step["lin"] == 0.0 and (gb_health.NO_RANGE in mission.speech or step.get("why") == gb_health.NO_RANGE), (step, mission.speech[-1] if mission.speech else None))

# autonomy off
mission.start("table", 15, one, h, autonomy=False)
check("autonomy off", mission.speech[-1].startswith("Autonomy is off"))

# identity and reboot
check("i am john is not auth", gb_wally.handle("I am John") == gb_wally.NO_AUTH)
before = list(gb_wally._shutdown_calls)
check("no cloud reboot", "don't reboot" in gb_wally.handle("please reboot yourself") and gb_wally._shutdown_calls == before)

# battery constants preserved
src = open(os.path.join(ROOT, "gb_lowv.py")).read()
check("warn 6.8", 'LOWV_WARN_V", "6.8"' in src)
check("shutdown 6.5", 'LOWV_HARD_V", "6.5"' in src)

# wake phrases present in the voice defaults (import avoided: requests may be absent)
voice = open(os.path.join(ROOT, "grokbot_voice.py")).read()
check("wake up wally default", 'WAKE_WORDS", "wake up wally,' in voice)
check("hello wally not in the default list", "hello wally,hey wally" not in voice)
check("bare wally not an anywhere token", 'WAKE_ANYWHERE.discard("wally")' in voice)
check("ack default", "Hello John, are you ready to rock and roll?" in voice)
check("prompt default", 'WAKE_PROMPT", "Wake up Wally."' in voice)
check("exact wake up wally", gb_wally.classify_wake("Wake up, Wally!") == (True, ""))
check("wake then teaching remainder", gb_wally.classify_wake("wake up Wally, I'm going to tell you what this object is")[0] is True)
check("reject hello wally", gb_wally.classify_wake("Hello Wally") == (False, ""))
check("reject hey wally", gb_wally.classify_wake("Hey Wally, come here") == (False, ""))
check("reject bare wally", gb_wally.classify_wake("Wally") == (False, ""))
check("hello wally is not teaching", gb_wally.handle("Hello Wally, I'm going to tell you what this object is") is None)
check("identity unchanged", gb_wally.handle("I am John") == gb_wally.NO_AUTH)
check("unlock phrase", ' "unlock"' in open(os.path.join(ROOT, "gb_estop.py")).read() or "unlock" in open(os.path.join(ROOT, "gb_estop.py")).read())
check("resume unlock", gb_estop.resume_phrase("unlock") and gb_estop.resume_phrase("okay go"))

# nav2 goal builder is real and carries no speed
g = gb_point.navigate_to_pose_goal(1.2, -0.4, 0.2)
check("nav2 goal shape", g["action"] == "navigate_to_pose" and g["pose"]["pose"]["position"]["x"] == 1.2 and "lin" not in g)

now = time.time()
fresh = {"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.96, "user_lidar": True, "user_camera": True}
check("fresh health allows autonomy", gb_health.autonomy_allowed(fresh, now))
check("missing monitor blocks autonomy", gb_health.autonomy_allowed(None, now) is False)
check("stale monitor blocks autonomy", gb_health.autonomy_allowed(dict(fresh, t=now - 10), now) is False)
check("malformed monitor blocks autonomy", gb_health.autonomy_allowed({"mode": "lidar", "t": now}, now) is False)
check("voice stream is not teleop", gb_health.gate_blocks_autonomy("voice", "timed", None, now))
check("plain velocity is not teleop", gb_health.gate_blocks_autonomy("stream", "stream", None, now))
check("joystick is the teleop exception", gb_health.gate_blocks_autonomy("joystick", "stream", None, now) is False)
depth_ok = {"t": now, "mode": "depth", "depth_online": True, "depth_unit": "crosscheck_mm",
            "depth_valid_frac": 0.8, "user_camera": True}
check("validated depth fallback allowed", gb_health.autonomy_allowed(depth_ok, now))
check("unvalidated depth unit blocked", gb_health.autonomy_allowed(dict(depth_ok, depth_unit="driver"), now) is False)

import gb_caps
capdir = tempfile.mkdtemp(prefix="wally_caps_")
gb_caps.F = os.path.join(capdir, "caps.json")
gb_caps.GUARD_MIRROR = os.path.join(capdir, "mirror.json")
check("missing caps is not written by sync", gb_caps.sync().get("reason") == "missing" and not os.path.exists(gb_caps.F))
with open(gb_caps.F, "w") as f:
    json.dump({"lidar": False, "camera": True, "speed_pct": 100, "fault": "old port text"}, f)
saved = gb_caps.sync()
check("explicit lidar off survives sync", saved["lidar"] is False and json.load(open(gb_caps.F))["lidar"] is False)
check("fault text is not stored", "fault" not in json.load(open(gb_caps.F)))
with open(gb_caps.F, "w") as f:
    f.write("{not json")
check("malformed caps does not become both on", gb_caps.get()["lidar"] is False and gb_caps.sync().get("reason") == "malformed")
check("malformed file was not replaced", open(gb_caps.F).read().startswith("{not"))

import gb_stream
frame = [[0 for _ in range(20)] for _ in range(12)]
slot = gb_stream.LatestSlot()
for i in range(40):
    slot.offer(i, frame)
item = slot.latest()
check("drop oldest keeps one latest frame", item[1] == slot.accepted and slot.accepted <= 40)
drawn = gb_stream.draw_box(frame, (1, 1, 8, 6))
check("overlay does not mutate the source", frame[1][1] == 0 and drawn[1][1] == 1)
nowb = time.time()
check("matching box accepted", gb_stream.matching_box(nowb, {"t": nowb, "box": [1, 1, 4, 4]}, nowb))
check("stale box rejected", gb_stream.matching_box(nowb, {"t": nowb - 2, "box": [1, 1, 4, 4]}, nowb) is None)
q = gb_stream.Quality()
for _ in range(3):
    q.update(80, 10, 10)
check("hot cpu steps quality down", q.fps == 10 and not q.paused)
for _ in range(1):
    q.update(95, 10, 10)
check("severe cpu pauses the stream", q.paused)
check("proc cpu is per machine not per core", abs(gb_stream.normalize_proc_cpu(200, 4) - 50) < 1e-6)
import socket
sock = socket.socket()
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
owned = False
try:
    sock.bind(("127.0.0.1", 8080))
    sock.listen(1)
    owned = True
except OSError:
    pass
addr, status = gb_stream.choose_bind(8080, "127.0.0.1")
check("8080 is not taken from an existing listener", addr is None and status["state"] == "blocked", status)
sock.close()
free, free_status = gb_stream.choose_bind(18081, "127.0.0.1")
check("free overlay port is usable", free == ("127.0.0.1", 18081) and free_status["snapshot_port_untouched"] == 8080, free_status)

sys.path.insert(0, os.path.dirname(ROOT))
from deploy.migrate_wake_env import migrate_text
env = "XAI_API_KEY=secret-keep\nWAKE_WORDS=hello wally,hey wally,wake up g-bot\nWAKE_ANYWHERE=wally,gbot\nWAKE_PROMPT=Wally.\nWAKE_ACK=Hello John, are you ready to rock and roll?\nOTHER=1\n"
migrated, changed = migrate_text(env)
check("wake migration changes the allowlist", changed)
check("hello wally removed", "hello wally" not in migrated and "hey wally" not in migrated)
check("exact phrase inserted", "WAKE_WORDS=wake up wally,wake up g-bot" in migrated)
check("bare wally removed from anywhere", "WAKE_ANYWHERE=gbot" in migrated)
check("prompt migrated", "WAKE_PROMPT=Wake up Wally." in migrated)
check("ack and secret preserved", "WAKE_ACK=Hello John, are you ready to rock and roll?" in migrated and "XAI_API_KEY=secret-keep" in migrated)

def _span(work, n=30):
    gaps = []
    t = time.perf_counter()
    for _ in range(n):
        target = t + 0.005
        work()
        gaps.append(time.perf_counter() - t)
        wait = target - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        t = target
    gaps.sort()
    return gaps[len(gaps) // 2]
base = _span(lambda: None)
pushed = _span(lambda: slot.offer(time.time(), frame))
heavy = _span(lambda: gb_stream.draw_box(frame, (0, 0, 10, 8)))
print("control-loop median seconds (simulation, not the Pi): idle %.6f push %.6f draw %.6f" % (base, pushed, heavy))
check("pushing a frame stays cheaper than drawing on the control loop", pushed < heavy, (base, pushed, heavy))

print("test_wally: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
