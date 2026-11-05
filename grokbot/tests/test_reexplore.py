#!/usr/bin/env python3
"""test_reexplore.py - camera-primary fusion + explore recovery wiring (no motors)."""
import math, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE) if os.path.basename(HERE) == "tests" else HERE
# Prefer installed tree when present
for cand in ("/home/pi/grokbot", ROOT, os.path.join(ROOT, "sim")):
    if os.path.exists(os.path.join(cand, "gb_obstacle.py")): ROOT = cand; break
sys.path.insert(0, ROOT)
import gb_obstacle as OB
OK = BAD = 0
def check(n, c, info=""):
    global OK, BAD
    if c: OK += 1
    else: BAD += 1; print("FAIL:", n, info)
INF = float("inf")
now = 1000.0
def LV(fwd=INF, rev=INF, sweep=INF, blind=0.0, t=None):
    v = OB.LidarView(t or now); v.fwd, v.rev, v.sweep, v.blind_f, v.blind_run = fwd, rev, sweep, blind, 0; return v
def DV(fwd=INF, vf=0.5, t=None):
    v = OB.DepthView(t or now); v.fwd, v.valid_frac, v.pts = fwd, vf, 20; return v
def F(lidar=None, depth=None, odom=True, imu=True):
    f = OB.Fusion()
    if odom: f.on_odom(now, 0, 0, 0, 0, 0)          # pose first so depth memory can latch
    if lidar is not None: f.on_lidar(lidar)
    if depth is not None: f.on_depth(depth)
    if imu: f.imu_t = now
    return f

# ---- depth height range raised
check("DEPTH_HMAX >= 0.40", OB.DEPTH_HMAX >= 0.40 - 1e-9, OB.DEPTH_HMAX)
check("STOP_FWD ~0.27", abs(OB.STOP_FWD - 0.27) < 1e-6, OB.STOP_FWD)
check("SLOW_FULL ~0.70", abs(OB.SLOW_FULL - 0.70) < 1e-6, OB.SLOW_FULL)
check("JOLT >= 7", OB.JOLT >= 6.9, OB.JOLT)
check("soft tilt >= 20", OB.TILT_DEG >= 19.9, OB.TILT_DEG)
check("hard tilt 25", abs(OB.TILT_HARD - 25) < 1e-6, OB.TILT_HARD)
check("SWEEP_R >= 0.24", OB.SWEEP_R >= 0.24 - 1e-9, OB.SWEEP_R)

# ---- camera-primary: depth alone stops / slows with LiDAR missing
f = F(lidar=None, depth=DV(0.30))
l, a, why, kind = f.limit(0.12, 0, now)
check("depth-only 0.30 blocks forward", l == 0 and kind == "block", (l, kind, why))
f = F(lidar=None, depth=DV(0.80))
l, a, why, kind = f.limit(0.12, 0, now)
check("depth-only 0.80 creeps <= V_DEGRADED", 0 < l <= OB.V_DEGRADED + 1e-9, (l, why))
f = F(lidar=None, depth=DV(0.80))
l, a, why, kind = f.limit(-0.1, 0, now)
check("depth-only refuses reverse", l == 0 and kind == "stale", (l, kind))
f = F(lidar=None, depth=None)
check("no lidar no depth stops", f.limit(0.12, 0, now)[3] == "stale")

# ---- depth wins when closer than lidar
f = F(lidar=LV(1.5, 1.0, 1.0), depth=DV(0.40))
d, src = f.front(now)
check("closer depth wins", abs(d - 0.40) < 1e-9 and src == "depth", (d, src))
check("depth 0.40 slows", 0.04 <= f.limit(0.12, 0, now)[0] <= 0.06)

# ---- lidar still works when depth empty room
f = F(lidar=LV(0.30), depth=DV(INF, 0.5))
check("lidar stop still works", f.limit(0.12, 0, now)[3] == "block")

# ---- memory used on turn sweep
f = F(lidar=LV(2.0, 2.0, 2.0), depth=DV(0.20))
f.on_odom(now, 0, 0, 0, 0, 0)
# drive forward 0.05 then turn — memory of 0.20 should block turn
f2_t = now + 1.0; f.on_odom(f2_t, 0.1, 0, 0.05, 0, 0); f.on_lidar(LV(2.0, 2.0, 2.0, t=f2_t)); f.imu_t = f2_t
# depth gone below FOV (stale): memory of the 0.20 m low obstacle must still block the turn
f.depth = OB.DepthView(f2_t - 2.0); f.depth.fwd = INF; f.depth.valid_frac = 0.5
l, a, why, kind = f.limit(0.0, 0.6, f2_t)
check("depth memory blocks turn", a == 0 and kind == "block", (a, kind, why, f.mem_dist(f2_t)))

# ---- explore wiring
ex = open(os.path.join(ROOT, "gb_explore.py")).read()
check("refusal removed", "EXPLORE_VOICE_OK" not in ex)
check("nav2 default off", 'EXPLORE_NAV2", "0"' in ex)
check("bump recover present", "bump recover" in ex and "hold after bump" in ex and "recover turn" in ex)
check("no permanent disable after bumps", "stuck in a small area" in ex and "EXPLORE_VOICE_OK" not in ex)
check("avoid 0.45", 'EXPLORE_AVOID_DIST", "0.45"' in ex)
check("no Abort on boxed-in", "raise Abort" not in ex.split("def wander_step")[1].split("def known_m2")[0] or "wide turn around clutter" in ex)
check("uses depth_fwd from gate", "depth_fwd" in ex)

# ---- voice latency / style wiring
voice = open(os.path.join(ROOT, "grokbot_voice.py")).read()
check("reasoning_effort none chat", '"reasoning_effort": "none"' in voice)
check("vision effort none", '"effort": "none"' in voice)
check("http session keepalive", "_http = requests.Session()" in voice)
check("whisper default 2 threads", 'WHISPER_THREADS", "2"' in voice)
check("split_sentences min 12", "min_len=12" in voice)
check("style: lead with the answer", "Lead with the answer" in voice)
check("style: short clauses", "short clauses" in voice)
check("style: not human", "You are not human" in voice)
check("style: safety precise", "never playful about motors" in voice.lower() or "never playful about motors, ESTOP" in voice)

# split_sentences behaviour
sys.path.insert(0, ROOT)
# Importing grokbot_voice pulls heavy deps; exec just the function
import re, ast
tree=ast.parse(voice); fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="split_sentences")
ns={"re":re}; exec(compile(ast.Module(body=[fn], type_ignores=[]), "ss", "exec"), ns)
parts = ns["split_sentences"]("Okay, I'll drive around slowly and map this room for up to 4 minutes.")
check("first clause short", parts and len(parts[0].split()) <= 12, parts)
check("rest kept", len(parts) >= 2, parts)

print("test_reexplore: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
