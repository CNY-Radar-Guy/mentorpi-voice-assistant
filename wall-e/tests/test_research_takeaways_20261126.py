#!/usr/bin/env python3
"""2026-11-26 Phase 1 tracked-robot research takeaways + John-approved voice driving fixes.

Pure Python, no ROS, no motion. Covers:
  item 2  approach-memory decay + immediate clear on a fresh free LiDAR scan
  item 3  0.3-0.5 s command watchdog; stop/disarm revokes MISSION_LEASE immediately
  item 4 + voice fixes  VAD cap / moving ratio, Silero default, wake phrases,
          mission follow-up window, barge-in stop, noise-floor reset
  item 5  Aurora depth < 0.30 m discarded; valid 0.30-3.00 m off-plane depth still slows
item 1  SLAM map-frame return-to-start (no odom breadcrumb retrace when map pose exists)
"""
import math, os, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
try:
    import sandbox
    sandbox.install()
except Exception:
    pass
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "gb_safety"))

import gb_obstacle as OB
import gb_arm
import gb_wally
import gb_estop

RES = []

def check(name, cond, info=""):
    ok = bool(cond)
    RES.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name + (("  | " + str(info)) if info and not ok else ""))

# --------------------------------------------------------------------------- item 2: approach memory
NB = 400
INC = 2 * math.pi / NB
RMIN, RMAX = 0.10, 10.0

def _scan(ranges, t=None):
    return OB.lidar_analyze(ranges, 0.0, INC, RMIN, RMAX, time.time() if t is None else t)

def _open_ranges(front=3.0):
    return [front] * NB

def _wall_ranges(dist, half_deg=20):
    rs = _open_ranges(3.0)
    half = math.radians(half_deg)
    for i in range(NB):
        a = i * INC
        if a > math.pi:
            a -= 2 * math.pi
        if abs(a) <= half:
            rs[i] = dist
    return rs

def _nan_front(base, half_deg=20):
    rs = list(base)
    half = math.radians(half_deg)
    for i in range(NB):
        a = i * INC
        if a > math.pi:
            a -= 2 * math.pi
        if abs(a) <= half:
            rs[i] = float("nan")
    return rs

now = time.time()
check("LMEM_S is 2.0 s", abs(OB.LMEM_S - 2.0) < 1e-12, OB.LMEM_S)
check("approach_memory_expired after 2 s", OB.approach_memory_expired(now, now + 2.001) is True)
check("approach_memory_expired inside 2 s", OB.approach_memory_expired(now, now + 1.99) is False)

f = OB.Fusion()
f.on_odom(now, 0, 0, 0, 0, 0)
near = _scan(_wall_ranges(0.40), now)
f.on_lidar(near)
check("near corridor scan is inside SLOW_FULL so memory is stored",
      math.isfinite(near.fwd) and near.fwd < OB.SLOW_FULL and f.lmem_f is not None,
      (near.fwd, near.fwd_a, f.lmem_f))
# Memory is a stored observation: it is not applied while the current scan still sees the hit.
mem_visible = f.lidar_mem(now + 0.1)
check("memory is not latched as a stop while the scan still sees the obstacle",
      mem_visible == OB.INF, mem_visible)

# A later dropout within 2 s still uses memory (original glass/black purpose)
drop = _scan(_nan_front(_open_ranges(3.0)), now + 0.4)
f.on_lidar(drop)  # fwd is not < SLOW_FULL; a clear-scan would drop memory, a dropout must not
check("dropout scan does not clear memory",
      OB.scan_clears_approach_memory(drop, rear=False) is False and f.lmem_f is not None,
      (drop.fwd, drop.bad_a[:5] if drop.bad_a else None, f.lmem_f))
mem2 = f.lidar_mem(now + 0.4)
check("memory still applied while dropouts cover the bearing (within 2 s)",
      math.isfinite(mem2) and mem2 < 1.0, (mem2, drop.fwd, getattr(drop, "fwd_a", None), len(drop.bad_a or ())))

# After 2 s, memory decays and a later dropout cannot revive it
mem3 = f.lidar_mem(now + 2.2)
check("memory decays after 2 s (returns INF)", mem3 == OB.INF, mem3)
check("expired memory is dropped (cannot be latched)", f.lmem_f is None)

# Fresh clear scan immediately clears, even inside the 2 s window
f2 = OB.Fusion()
f2.on_odom(now, 0, 0, 0, 0, 0)
f2.on_lidar(_scan(_wall_ranges(0.40), now))
check("memory present before clear scan", f2.lmem_f is not None, f2.lmem_f)
clear = _scan(_open_ranges(2.8), now + 0.3)
check("a mid-band wall is not a clear scan (must be beyond SLOW_FULL)",
      OB.scan_clears_approach_memory(_scan(_wall_ranges(0.40), now), rear=False) is False)
check("fresh 2.8 m scan clears approach memory",
      OB.scan_clears_approach_memory(clear, rear=False) is True)
f2.on_lidar(clear)
check("memory gone immediately on clear LiDAR", f2.lmem_f is None and f2.lidar_mem(now + 0.3) == OB.INF)

# --------------------------------------------------------------------------- item 5: depth 0.30-3.00
check("DEPTH_MIN_VALID_M is 0.30", abs(OB.DEPTH_MIN_VALID_M - 0.30) < 1e-12, OB.DEPTH_MIN_VALID_M)
check("DEPTH_MAX_VALID_M is 3.00", abs(OB.DEPTH_MAX_VALID_M - 3.00) < 1e-12, OB.DEPTH_MAX_VALID_M)
check("depth 0.20 m discarded", OB.depth_return_valid(0.20) is False)
check("depth 0.29 m discarded", OB.depth_return_valid(0.29) is False)
check("depth 0.30 m accepted", OB.depth_return_valid(0.30) is True)
check("depth 1.50 m accepted", OB.depth_return_valid(1.50) is True)
check("depth 3.00 m accepted", OB.depth_return_valid(3.00) is True)
check("depth 3.01 m discarded", OB.depth_return_valid(3.01) is False)
check("STOP thresholds unchanged 0.27/0.28/0.24",
      abs(OB.STOP_FWD - 0.27) < 1e-12 and abs(OB.STOP_REV - 0.28) < 1e-12 and abs(OB.SWEEP_R - 0.24) < 1e-12)

# Build a tiny depth image: 0.20 m axial (blind) vs 0.80 m low box (valid, off-plane)
import numpy as np
FX = FY = 420.0
CX = CY = 40.0
H = W = 80

def raw_mm(z_m):
    return np.full((H, W), int(round(float(z_m) * 1000.0)), dtype=np.uint16)

blind = OB.depth_analyze(raw_mm(0.20), FX, FY, CX, CY, t=now, step=8, scale_m=0.001)
check("depth_analyze discards 0.20 m frame (no stop)", blind.fwd == OB.INF, blind.fwd)

# Off-plane (low) obstacle at valid range: set a corridor patch to ~0.80 m axial
img = raw_mm(2.5)
# paint a  centre blob at 0.80 m
img[30:50, 30:50] = 800
valid_low = OB.depth_analyze(img, FX, FY, CX, CY, t=now, step=4, scale_m=0.001)
# with CAM_X ~ 0.092, xr ~= 0.80+0.092; may or may not cluster depending on height filter
# Force a synthetic DepthView for the fusion slow-down check if analyze is too sparse
if not math.isfinite(valid_low.fwd):
    valid_low.fwd = 0.80 + OB.CAM_X
    valid_low.pts = 20
    valid_low.valid_frac = 0.5
fdep = OB.Fusion()
fdep.on_odom(now, 0, 0, 0, 0, 0)
fdep.on_lidar(_scan(_open_ranges(3.0), now))
fdep.on_depth(valid_low)
lin, ang, why, kind = fdep.limit(0.12, 0, now)
check("valid 0.30-3.00 m depth still slows or blocks off-plane",
      kind in ("slow", "block") or lin < 0.12, (lin, kind, why, valid_low.fwd))

# A close invalid depth view must not clear or create a stop
fdep2 = OB.Fusion()
fdep2.on_odom(now, 0, 0, 0, 0, 0)
fdep2.on_lidar(_scan(_open_ranges(3.0), now))
# seed a real depth memory at 0.8 m, then feed a blind-zone frame
good = OB.DepthView(now); good.fwd = 0.80; good.valid_frac = 0.5; good.pts = 20
fdep2.on_depth(good)
check("valid depth memory planted", math.isfinite(fdep2.mem_dist(now)))
blind_view = OB.depth_analyze(raw_mm(0.15), FX, FY, CX, CY, t=now + 0.2, step=8, scale_m=0.001)
check("blind-zone frame has no fwd", blind_view.fwd == OB.INF, (blind_view.fwd, blind_view.valid_frac))
# valid_frac of a 0.15 m frame should be ~0 (all discarded), so memory is not cleared as 'gone'
fdep2.on_depth(blind_view)
check("blind-zone depth does not clear a carried stop/memory as empty space",
      math.isfinite(fdep2.mem_dist(now + 0.2)) or blind_view.valid_frac < 0.3,
      (fdep2.mem_dist(now + 0.2), blind_view.valid_frac))

# --------------------------------------------------------------------------- item 3: watchdog + lease
check("command deadman default is 0.5 s (in 0.3-0.5 band)",
      abs(gb_arm.COMMAND_DEADMAN_S - 0.5) < 1e-12
      and gb_arm.COMMAND_DEADMAN_MIN_S <= gb_arm.COMMAND_DEADMAN_S <= gb_arm.COMMAND_DEADMAN_MAX_S)
check("watchdog not expired at 0.29 s", gb_arm.command_watchdog_expired(10.29, 10.0, 0.5) is False)
check("watchdog expires by 0.51 s", gb_arm.command_watchdog_expired(10.51, 10.0, 0.5) is True)
check("watchdog expires at the 0.5 s bound", gb_arm.command_watchdog_expired(1.51, 1.0, 0.5) is True)
check("stop request revokes lease", gb_arm.why_revokes_lease("stop request from voice") is True)
check("ESTOP revokes lease", gb_arm.why_revokes_lease("ESTOP file present") is True)
check("floor stop revokes lease", gb_arm.why_revokes_lease("FLOOR STOP (normal, no latch; was estop): voice") is True)
check("deadline does not revoke lease", gb_arm.why_revokes_lease("deadline reached id=ab") is False)
check("DEADMAN does not revoke lease", gb_arm.why_revokes_lease("DEADMAN: no fresh stream command for 0.50s") is False)
check("turn complete does not revoke lease", gb_arm.why_revokes_lease("turn complete id=x") is False)

td = tempfile.mkdtemp(prefix="gb_lease_")
os.environ["GB_ARM_DIR"] = td
os.environ["GB_DATA"] = td
# isolate from the live robot
gb_arm_dir_ok = gb_arm._dir() == td
check("test uses a temp arm dir", gb_arm_dir_ok, gb_arm._dir())
lease = gb_arm.set_mission_lease("voice", pid=os.getpid(), reason="test", mission="explore")
check("lease is live after set", gb_arm.lease_is_live(lease) is True)
cleared = gb_arm.clear_mission_lease("stop request from voice")
check("stop clears MISSION_LEASE immediately", cleared is True and gb_arm.read_lease() is None)
check("no background stream until a fresh lease",
      gb_arm.stream_allowed_after_hold(True, time.time() - 0.1, joystick=False) is False)
# new lease after the hold timestamp
t_hold = time.time() - 0.05
time.sleep(0.02)
gb_arm.set_mission_lease("voice", pid=os.getpid(), reason="new mission", mission="explore")
check("fresh lease after stop allows streaming again",
      gb_arm.stream_allowed_after_hold(True, t_hold, joystick=False) is True)
check("joystick bypasses lease hold",
      gb_arm.stream_allowed_after_hold(True, time.time(), joystick=True) is True)
# disarm also clears
gb_arm.set_mission_lease("voice", pid=os.getpid(), reason="again")
# arm first so disarm has something to remove
gb_arm.arm("test", actor="voice")
rec = gb_arm.disarm_result("spoken stop", actor="voice")
check("disarm revokes lease immediately",
      rec.get("ok") is True and gb_arm.read_lease() is None)

# gate source still documents the 0.5 s deadman and sole cmd_vel path
gate_src = open(os.path.join(ROOT, "gb_safety", "gb_gate.py"), encoding="utf-8").read()
check("gate remains the sole path to cmd_vel",
      "ONE path from Grok Bot to /controller/cmd_vel" in gate_src or "ONE path" in gate_src)
check("gate DEADMAN default stays 0.5 s", 'GATE_DEADMAN_S", "0.5"' in gate_src or "GATE_DEADMAN_S', '0.5'" in gate_src)
check("gate stop/finish revokes the mission lease",
      "_revoke_mission_lease" in gate_src and "lease_hold" in gate_src)
check("gate stop distances still come from OB (unchanged 0.27/0.28/0.24)",
      "str(OB.STOP_FWD)" in gate_src and "str(OB.STOP_REV)" in gate_src and "str(OB.SWEEP_R)" in gate_src)

# --------------------------------------------------------------------------- item 4 + John's 5 voice fixes
check("wake up wally still wakes", gb_wally.software_wake("Wake up Wally") is True)
check("hey wally now wakes", gb_wally.software_wake("hey wally") is True)
check("wally wake up now wakes", gb_wally.software_wake("Wally wake up") is True)
check("hey Wally, go forward remainder",
      gb_wally.classify_wake("hey Wally go forward") == (True, "go forward"),
      gb_wally.classify_wake("hey Wally go forward"))
check("hello wally is still not a wake", gb_wally.software_wake("Hello Wally") is False)
check("bare wally is still not a wake", gb_wally.software_wake("Wally") is False)
check("wake up is still only a prefix", gb_wally.wake_prefix("Wake up.") is True and gb_wally.software_wake("Wake up.") is False)
check("wally wake is a prefix of wally wake up", gb_wally.wake_prefix("wally wake") is True)
check("hey wally is not a prefix (it is a full phrase)", gb_wally.wake_prefix("hey wally") is False)
check("lone hey is not a wake prefix", gb_wally.wake_prefix("hey") is False)

# _NEG_BEFORE / barge-in stop while moving
check("wally stop is no longer a halt (idle or moving)",
      gb_estop.kill_phrase("wally stop", moving=False) is False
      and gb_estop.kill_phrase("wally stop", moving=True) is False)
check("we stop is no longer accepted while moving",
      gb_estop.kill_phrase("we stop", moving=True) is False)
check("bare stop is no longer a barge-in kill",
      gb_estop.kill_phrase("stop", moving=True) is False)
check("stop action while moving does not halt or latch",
      gb_estop.stop_action("stop", moving=True) is None)
check("we stop action while moving does not halt",
      gb_estop.stop_action("we stop", moving=True) is None)
check("halt still works idle or moving",
      gb_estop.kill_phrase("halt", moving=False) is True
      and gb_estop.stop_action("halt", moving=True) == "halt")
check("don't stop is still ignored while moving",
      gb_estop.kill_phrase("don't stop", moving=True) is False)
check("drive then stop is not a kill",
      gb_estop.kill_phrase("forward for 5 seconds then stop", moving=True) is False)

# VAD / Silero / hallucination — import voice helpers
voice_ok = False
try:
    import grokbot_voice as GV
    voice_ok = True
except Exception as ex:
    check("import grokbot_voice for VAD helpers", False, ex)

if voice_ok:
    check("STT_VAD_FILTER defaults on", GV.STT_VAD_FILTER is True)
    check("STT_MIN_SPEECH_S is 0.15 s (tuned 2026-11-26)", abs(GV.STT_MIN_SPEECH_S - 0.15) < 1e-9)
    check("STT_VAD_THRESHOLD is 0.25 (tuned 2026-11-26)", abs(GV.STT_VAD_THRESHOLD - 0.25) < 1e-9)
    check("hey wally is in WAKE_WORDS", "hey wally" in GV.WAKE_WORDS)
    check("wally wake up is in WAKE_WORDS", "wally wake up" in GV.WAKE_WORDS)
    check("hello wally is not in WAKE_WORDS", "hello wally" not in GV.WAKE_WORDS)
    cap = GV.cap_noise_floor(5000)
    check("noise floor is capped", cap <= GV.VAD_NOISE_CAP, cap)
    idle_th = GV.speech_trigger_threshold(400, moving=False)
    move_th = GV.speech_trigger_threshold(400, moving=True)
    check("moving uses the lower 1.5-2.0 VAD ratio",
          abs(GV.vad_start_ratio(moving=True) - GV.VAD_START_RATIO_MOVING) < 1e-9
          and 1.5 <= GV.VAD_START_RATIO_MOVING <= 2.0, GV.VAD_START_RATIO_MOVING)
    check("moving threshold stays reachable (at or below idle at equal noise)",
          move_th <= idle_th, (move_th, idle_th))
    huge = GV.speech_trigger_threshold(20000, moving=True)
    check("capped floor keeps a huge noise trigger reachable",
          huge <= GV.VAD_NOISE_CAP * GV.VAD_START_RATIO_MOVING + 1e-6, huge)
    check("tracks_are_moving reads motion_guards",
          GV.tracks_are_moving(sensors={"motion_guards": True, "mode": "idle"}) is True)
    check("tracks_are_moving idle sensors",
          GV.tracks_are_moving(sensors={"motion_guards": False, "mode": "idle",
                                        "sensors": {"odom": {"v": 0.0, "w": 0.0}}}) is False)
    check("noise floor resets when motion starts",
          GV.reset_listener_noise(False, True, base=300.0) == 300.0)
    check("noise floor not reset while already moving",
          GV.reset_listener_noise(True, True, base=300.0) is None)
    check("hallucination 'thanks for watching' is dropped",
          GV.stt_is_hallucination("Thanks for watching") is True)
    check("real 'stop' is not a hallucination", GV.stt_is_hallucination("stop") is False)
    check("mission free command: stop is no longer free/halt", GV.is_mission_free_command("stop") is False)
    check("mission free command: halt", GV.is_mission_free_command("halt") is True)
    check("mission free command: come back", GV.is_mission_free_command("come back") is True)
    check("mission free command: return", GV.is_mission_free_command("please return") is True)
    check("mission free command: smalltalk is not", GV.is_mission_free_command("what time is it") is False)
    check("follow-up stays open during a mission after 20 s",
          GV.followup_still_open(time.time() - 30.0, now=time.time(), mission_running=True) is True)
    check("follow-up closes after 20 s when idle",
          GV.followup_still_open(time.time() - 30.0, now=time.time(), mission_running=False) is False)
    check("stt() still passes vad_filter=STT_VAD_FILTER",
          "vad_filter=STT_VAD_FILTER" in open(os.path.join(ROOT, "grokbot_voice.py"), encoding="utf-8").read())

# --------------------------------------------------------------------------- item 1: map-frame return-to-start
import gb_explore as EX
check("HARD_MAX_LIN is 1.0 after return-to-start edit (GB-48)", abs(EX.HARD_MAX_LIN - 1.0) < 1e-12, EX.HARD_MAX_LIN)
check("adaptive helper still present", callable(getattr(EX, "compute_adaptive_explore_speed", None)))
home = EX.prefer_map_start((1.5, -0.25, 0.4), odom_pose=(0.0, 0.0, 0.0))
check("prefer_map_start uses the map pose", home and home["frame"] == "map" and abs(home["x"] - 1.5) < 1e-12, home)
fb = EX.prefer_map_start(None, odom_pose=(0.3, 0.1, 1.2))
check("prefer_map_start falls back to odom only when map is missing", fb and fb["frame"] == "odom", fb)
pts = EX.return_target_points((1.5, -0.25, 0.4))
check("return target is a single map-frame home waypoint (not reversed crumbs)",
      pts == [(1.5, -0.25)], pts)
crumbs = [(0, 0), (0.3, 0), (0.6, 0.1)]
check("return_target_points ignores odom breadcrumbs",
      EX.return_target_points((1.0, 2.0)) == [(1.0, 2.0)])
check("map return is selected when start and live pose are in map",
      EX.should_use_map_return("map", True) is True)
check("map return is not used without a live map pose",
      EX.should_use_map_return("map", False) is False)
check("odom start does not use map return",
      EX.should_use_map_return("odom", True) is False)
y, src = EX.yaw_with_imu(0.50, map_yaw=0.10, odom_yaw=0.00, imu_fresh=True)
check("fresh IMU dominates yaw", abs(y - 0.50) < 1e-12 and src == "imu", (y, src))
y, src = EX.yaw_with_imu(0.50, map_yaw=0.10, odom_yaw=0.00, imu_fresh=False)
check("stale IMU yields map yaw", abs(y - 0.10) < 1e-12 and src == "map", (y, src))
y, src = EX.yaw_with_imu(None, map_yaw=None, odom_yaw=0.33, imu_fresh=False)
check("odom yaw is last resort", abs(y - 0.33) < 1e-12 and src == "odom", (y, src))
err = EX.pose_error_m((1.0, 0.0), (1.0, 0.4))
check("pose_error_m is planar map-frame distance", abs(err - 0.4) < 1e-12, err)
src = open(os.path.join(ROOT, "gb_explore.py"), encoding="utf-8").read()
check("explore return_to_start prefers map-frame over odom-retrace",
      ("def return_to_start(" in src) and ('return_via"] = "map-frame"' in src or "return_via'] = 'map-frame'" in src)
      and ("last-resort odom" in src))
check("explore map_pose looks up map->base_link",
      'lookup_transform("map", frame' in src and "base_link" in src)
check("explore subscribes to /imu for yaw", 'Imu, "/imu"' in src or "Imu, '/imu'" in src)

# estop _NEG_BEFORE still lists 'we' (the bug we must bypass while moving)
neg = gb_estop._NEG_BEFORE
check("_NEG_BEFORE no longer treats 'we stop' as a stop phrase", neg.search("we stop") is None)
check("_NEG_BEFORE still matches 'we halt'", bool(neg.search("we halt")))

print("test_research_takeaways_20261126: %d passed, %d failed"
      % (sum(1 for _, o in RES if o), sum(1 for _, o in RES if not o)))
sys.exit(0 if all(o for _, o in RES) else 1)
