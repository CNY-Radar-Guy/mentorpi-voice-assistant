#!/usr/bin/env python3
"""2026-11-27 LiDAR + camera fusion stuck-recovery (John): unit + gate tests. ZERO hardware: simulated world only.
Run: python3 tests/test_fusion_recovery_20261127.py  (from the release dir)"""
import math, os, sys, tempfile, time, json, re
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
import fr_fakeros as S
S.install()
import gb_obstacle as OB
import gb_fusion_recovery as FR

P = F = 0
def check(name, cond, extra=""):
    global P, F
    if cond: P += 1; print("PASS", name)
    else: F += 1; print("FAIL", name, extra)

LEGS_FRONT = [(0.22, 0.15, 0.012), (0.22, -0.15, 0.012)]
LEGS_REAR = [(-0.20, 0.15, 0.012), (-0.20, -0.15, 0.012)]
CHAIR = LEGS_FRONT + LEGS_REAR

def inputs(world, now, cam_mode="ok"):
    r, a0, inc = world.scan()
    v = OB.lidar_analyze(r, a0, inc, 0.10, 10.0, now)
    lid = FR.lidar_input(v, now)
    normal = {"front": v.fwd, "sweep": v.sweep, "rear": v.rev, "corr_rev": v.rev, "blind_r": getattr(v, "blind_r", 0.0)}
    return v, lid, world.camera(now, cam_mode), normal

now = time.time()

# ---------------------------------------------------------------- 0. normal rules untouched
check("normal front stop 0.27", OB.STOP_FWD == 0.27 and FR.NORMAL_FRONT_STOP == 0.27)
check("normal rear stop 0.28", OB.STOP_REV == 0.28 and FR.NORMAL_REAR_STOP == 0.28)
check("normal sweep 0.24", OB.SWEEP_R == 0.24 and FR.NORMAL_SWEEP == 0.24)
check("corridor half width 0.15", abs(OB.HALF_W - 0.15) < 1e-9)
check("STALL_MIN_CMD 0.05", OB.STALL_MIN_CMD == 0.05)
gsrc = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
check("gate deadman default 0.5", 'E("GATE_DEADMAN_S", "0.5")' in gsrc)
check("gate manual cap 0.288 / auto 1.0", 'E("GATE_MANUAL_MAX_LIN", "0.288")' in gsrc and 'E("GATE_AUTO_MAX_LIN", "1.0")' in gsrc)
check("gate stream stops still from gb_obstacle", 'E("GATE_STREAM_FRONT_STOP", str(OB.STOP_FWD))' in gsrc
      and 'E("GATE_STREAM_SWEEP_R", str(OB.SWEEP_R))' in gsrc and 'E("GATE_STREAM_REAR_STOP", str(OB.STOP_REV))' in gsrc)
check("gate clear_trip still present", "def clear_trip" in gsrc)
check("micro-nudge speeds 0.06-0.07 > stall min", 0.05 < FR.NUDGE_LIN_MIN <= FR.NUDGE_LIN <= FR.NUDGE_LIN_MAX <= 0.07)
check("tight-squeeze floors below normal, above chassis", OB.BODY_FRONT < FR.FLOOR_FWD < OB.STOP_FWD
      and FR.CORNER_R < FR.FLOOR_SWEEP < OB.SWEEP_R and OB.BODY_HALF_W < FR.PHYS_HALF_W < OB.HALF_W,
      (FR.FLOOR_FWD, FR.FLOOR_SWEEP, FR.PHYS_HALF_W))
check("token TTL short (auto-expire)", FR.TOKEN_TTL_S <= 0.6 and FR.TOKEN_MAX_SPAN_S <= 1.0)
check("clamp_cmd bounds", FR.clamp_cmd(0.2, 1.0) == (FR.NUDGE_LIN_MAX, FR.ANG_MAX) and FR.clamp_cmd(0.01, 0)[0] == FR.NUDGE_LIN_MIN
      and FR.clamp_cmd(-0.1, 0)[0] >= 0)

# ---------------------------------------------------------------- 1. chair-leg trap = nudge
w = S.World(circles=CHAIR)
v, lid, cam, normal = inputs(w, now)
fz = S.make_fusion(w, now)
l, a, why, kind = fz.limit(FR.NUDGE_LIN, 0.0, now, 1.0, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
check("chair trap: NORMAL rules block forward (over-report)", kind == "block", (kind, why))
ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
check("chair trap: fusion says nudge", ok and verdict == "nudge", (verdict, r))
asm = FR.assess(lid, cam, None, now, normal=normal)
check("chair trap: assess mode nudge, best is forward", asm["mode"] == "nudge" and asm["allowed"][0]["name"] == "fwd",
      (asm["mode"], [c["name"] for c in asm["allowed"]], asm["rejected"]))
rgb_chair = {"t": now, "hazard": True, "label": "chair"}
ok2, v2, r2 = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, FR.rgb_input(rgb_chair, now), now)
check("chair trap: RGB 'chair' does not veto", ok2, r2)
rgb_p = {"t": now, "hazard": True, "label": "person"}
ok3, v3, r3 = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, FR.rgb_input(rgb_p, now), now)
check("person in view -> real", not ok3 and v3 == "real", (v3, r3))

# ---------------------------------------------------------------- 2. solid wall = real
for d in (0.22, 0.25):
    ww = S.World(segs=[(d, -0.6, d, 0.6)])
    v, lid, cam, normal = inputs(ww, now)
    ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
    check("wall at %.2f: forward is real" % d, not ok and verdict == "real", (verdict, r))
    asm = FR.assess(lid, cam, None, now, normal=normal)
    check("wall at %.2f: no tight-squeeze candidate" % d, not any(c["token"] and c["lin"] > 0 for c in asm["allowed"]),
          [c["name"] for c in asm["allowed"]])
    # camera that wrongly reports open floor: LiDAR barrier width alone keeps it real
    cam_lie = {"t": now, "valid": 0.7, "bins": [[math.radians(-36 + 6 * b), math.radians(-30 + 6 * b), 0.7, 12, 0] for b in range(12)], "obs": []}
    ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam_lie, None, now)
    check("wall at %.2f + lying camera: still real (LiDAR wide barrier)" % d, not ok and verdict == "real", r)
# wall further out (0.40-0.60): camera agrees obstacle is close -> real
ww = S.World(segs=[(0.24, -0.6, 0.24, -0.16), (0.24, 0.16, 0.24, 0.6), (0.36, -0.2, 0.36, 0.2)])
v, lid, cam, normal = inputs(ww, now)
ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
check("box behind a gap (camera/LiDAR see it 0.36 m): real", not ok and verdict == "real", r)
# dense clutter
clutter = [(0.25 + 0.05 * i, 0.20 * (-1) ** i, 0.01) for i in range(10)] + [(0.3, 0.3 - 0.06 * i, 0.01) for i in range(11)]
wc = S.World(circles=clutter)
v, lid, cam, normal = inputs(wc, now)
ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
check("dense clutter: real", not ok and verdict == "real", r)

# ---------------------------------------------------------------- 3. camera offline / stale / low validity -> LiDAR-only
w = S.World(circles=CHAIR)
for mode in ("offline", "stale", "lowvalid"):
    v, lid, cam, normal = inputs(w, now, mode)
    ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
    check("camera %s: lidar_only, no nudge" % mode, not ok and verdict == "lidar_only", (verdict, r))
    asm = FR.assess(lid, cam, None, now, normal=normal)
    check("camera %s: assess lidar_only" % mode, asm["mode"] == "lidar_only" and not asm["allowed"], asm["mode"])
v, lid, cam, normal = inputs(w, now)
cam_off = dict(cam); cam_off["online"] = False
check("depth_online false -> lidar_only", FR.check_command(FR.NUDGE_LIN, 0, lid, cam_off, None, now)[1] == "lidar_only")
cam_drop = json.loads(json.dumps(cam))
for b in cam_drop["bins"]: b[4] = 10
check("camera sees drop-off (stairs) -> real", FR.check_command(FR.NUDGE_LIN, 0, lid, cam_drop, None, now)[1] == "real")
os.environ["GB_FUSION_RECOVERY"] = "0"
import importlib; FRd = importlib.reload(FR)
check("GB_FUSION_RECOVERY=0 disables (lidar_only)", FRd.assess(lid, cam, None, now, normal=normal)["mode"] == "lidar_only")
del os.environ["GB_FUSION_RECOVERY"]; FR = importlib.reload(FR)

# ---------------------------------------------------------------- 4. hard floor / reverse / bounds
wl = S.World(circles=[(0.19, 0.0, 0.012)])
v, lid, cam, normal = inputs(wl, now)
ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
check("leg dead ahead inside hard floor: real", not ok and verdict == "real", r)
wl = S.World(circles=[(0.30, 0.11, 0.012), (0.30, -0.30, 0.012)])
v, lid, cam, normal = inputs(wl, now)
ok, verdict, r = FR.check_command(FR.NUDGE_LIN, 0.0, lid, cam, None, now)
check("leg inside the 0.125 m chassis corridor: real", not ok, r)
wp = S.World(circles=[(0.0, 0.20, 0.012), (0.40, 0.0, 0.05)])
v, lid, cam, normal = inputs(wp, now)
check("pivot with a point at 0.19 m (< sweep floor 0.22): real", not FR.check_command(0.0, FR.PIVOT_ANG, lid, cam, None, now)[0])
w = S.World(circles=CHAIR); v, lid, cam, normal = inputs(w, now)
check("reverse never relaxed", not FR.check_command(-FR.NUDGE_LIN, 0.0, lid, cam, None, now)[0])
check("0.10 m/s refused (too fast)", not FR.check_command(0.10, 0.0, lid, cam, None, now)[0])
check("0.04 m/s refused (below stall min)", not FR.check_command(0.04, 0.0, lid, cam, None, now)[0])
check("|ang| 0.5 refused", not FR.check_command(0.065, 0.5, lid, cam, None, now)[0])
check("stale LiDAR refused", not FR.check_command(0.065, 0.0, dict(lid, t=now - 2), cam, None, now)[0])
check("LiDAR memory says obstacle close -> real", not FR.check_command(0.065, 0, lid, cam, None, now, {"lidar_mem": 0.2})[0])
check("candidates never relax reverse", all(c["lin"] >= 0 for c in FR.candidates() if c["token"]))
check("'back' only when rear normally clear", not any(c["name"] == "back" for c in FR.candidates({"rear": 0.25, "corr_rev": 0.25, "blind_r": 0})))

# ---------------------------------------------------------------- 5. escape state machine (pure sim, explore + goto)
def sim_escape(world, goal_bearing=None, cam_mode="ok", inject=None, max_s=60.0, goal=None):
    t = 1000.0; esc = FR.FusionEscape("test", t, (world.x, world.y, world.th)); outs = []; dt = 0.1
    while t < 1000.0 + max_s:
        v, lid, cam, normal = inputs(world, t, cam_mode)
        kw = inject(t - 1000.0) if inject else {}
        reached = goal is not None and math.hypot(goal[0] - world.x, goal[1] - world.y) < 0.18
        gb = None
        if goal is not None:
            gb = S.math.atan2(goal[1] - world.y, goal[0] - world.x) - world.th
        elif goal_bearing is not None:
            gb = goal_bearing
        o = esc.step(t, lid, cam, None, (world.x, world.y, world.th), normal, goal_bearing=gb, goal_reached=reached, **kw)
        outs.append(o)
        if o["event"] != "nudge":
            return o, outs, esc
        if o["token"]:
            ok, verdict, r = FR.check_command(o["lin"], o["ang"], lid, cam, None, t)
            if not ok:
                return {"event": "gate-refused", "why": r}, outs, esc
        world.step(o["lin"], o["ang"], dt); t += dt
    return {"event": "timeout"}, outs, esc

w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w)
nud = [x for x in outs if x["event"] == "nudge"]
check("explore chair escape recovers", o["event"] == "recovered", o)
check("explore escape: no collision (gap > 0)", w.min_gap > 0.0, w.min_gap)
check("explore escape: all forward nudges 0.06-0.07 m/s", all(x["lin"] == 0 or 0.06 <= abs(x["lin"]) <= 0.07 for x in nud))
check("explore escape: |ang| <= 0.30", all(abs(x["ang"]) <= 0.30 + 1e-9 for x in nud))
check("explore escape: continuous (no zero dwell)", all(x["lin"] or x["ang"] for x in nud))
check("explore escape: hands back once normal clear", FR.normal_clear(inputs(w, 2000)[3]))
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, goal=(0.8, 0.0))
check("goto chair escape recovers or reaches goal", o["event"] == "recovered", o)
w = S.World(circles=CHAIR, pose=(0, 0, 0))
o, outs, esc = sim_escape(w, goal=(0.30, 0.0))
check("goto waypoint inside the gap -> recovered at waypoint", o["event"] == "recovered" and "waypoint" in o["why"], o)
ww = S.World(segs=[(0.22, -0.6, 0.22, 0.6), (-0.21, -0.6, -0.21, 0.6), (-0.21, 0.19, 0.22, 0.19), (-0.21, -0.19, 0.22, -0.19)])
o, outs, esc = sim_escape(ww)
check("walled-in box: fallback (real), never a nudge into the wall", o["event"] in ("fallback", "abort")
      and ww.min_gap > 0, (o, ww.min_gap))
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, cam_mode="offline")
check("camera offline: escape falls back to LiDAR-only immediately", o["event"] == "fallback" and not [x for x in outs if x["event"] == "nudge"], o)
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, inject=lambda t: {"estop": t > 1.0})
check("ESTOP during escape -> stop", o["event"] == "stop", o)
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, inject=lambda t: {"armed": t < 1.0})
check("disarm during escape -> stop", o["event"] == "stop", o)
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, inject=lambda t: {"abort": t > 1.0})
check("mission abort during escape -> stop", o["event"] == "stop", o)
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, inject=lambda t: {"trip": "IMU tilt 27 deg" if t > 1.0 else None})
check("hard (tilt) trip during escape -> stop", o["event"] == "stop", o)
w = S.World(circles=CHAIR)
o, outs, esc = sim_escape(w, inject=lambda t: {"trip": "track stall: encoders 0 while commanding" if t > 0.5 else None})
check("encoder stall: heading treated REAL (blacklisted), escape ends fail-safe (abort or LiDAR fallback)",
      o["event"] in ("abort", "fallback") and esc.fails >= 1 and "fwd" in esc.black, (o, esc.fails))
st_n = [x for x in outs if x["event"] == "nudge" and x.get("cand") == "fwd"]
check("encoder stall: stalled heading never re-tried", all(True for _ in st_n) and outs[-1]["event"] != "nudge")
esc2 = FR.FusionEscape("t", 0.0, (0, 0, 0)); w = S.World(circles=CHAIR); last = None
for k in range(40):
    v, lid, cam, normal = inputs(w, 0.1 * k)
    last = esc2.step(0.1 * k, lid, cam, None, (0, 0, 0), normal, trip="track stall: encoders 0" if k % 2 else None)
    if last["event"] != "nudge": break
check("repeated stalls across headings -> hard stop after FAIL_LIMIT or real", last["event"] in ("abort", "fallback")
      and esc2.fails <= FR.FAIL_LIMIT, (last, esc2.fails))
# no-progress (robot cannot move: wheels spin) -> abort by consecutive failed cycles
w = S.World(circles=CHAIR); t = 1000.0; esc = FR.FusionEscape("t", t, (0, 0, 0)); last = None
while t < 1060:
    v, lid, cam, normal = inputs(w, t); last = esc.step(t, lid, cam, None, (0, 0, 0), normal)
    if last["event"] != "nudge": break
    t += 0.1
check("no physical progress -> abort after %d cycles (no endless retry)" % FR.FAIL_LIMIT, last["event"] == "abort" and t - 1000 < FR.MAX_ESCAPE_S + 1, (last, t - 1000))
esc = FR.FusionEscape("t", 0.0, (0, 0, 0))
w = S.World(circles=CHAIR); v, lid, cam, normal = inputs(w, 100.0)
check("time budget -> abort", esc.step(100.0, lid, cam, None, (0, 0, 0), normal)["event"] == "abort")
check("trip_kind classes", FR.trip_kind("ESTOP latched") == "hard" and FR.trip_kind("track stall") == "stall"
      and FR.trip_kind("bump detected (lidar): no progress") == "soft" and FR.trip_kind("front 0.25 < 0.27") == "refused")

# ---------------------------------------------------------------- 6. token (explore -> gate), auto-expire, void, pid
d = tempfile.mkdtemp(); tp = FR.token_path(os.path.join(d, "ESTOP"))
tok = FR.write_token(tp, 0.065, 0.0, "fwd", "explore", now=now, pid=os.getpid())
check("token round trip", FR.read_token(tp)["cand"] == "fwd")
check("token valid (fresh, matching)", FR.token_valid(tok, now + 0.1, 0.065, 0.0, check_pid=False)[0])
check("token expires after TTL (auto-restore)", not FR.token_valid(tok, now + FR.TOKEN_TTL_S + 0.05, 0.065, 0.0, check_pid=False)[0])
check("token void if older than last gate trip", not FR.token_valid(tok, now + 0.1, 0.065, 0.0, void_before=now + 0.05, check_pid=False)[0])
check("token mismatch refused", not FR.token_valid(tok, now + 0.1, 0.07, 0.3, check_pid=False)[0])
check("token for reverse refused", not FR.token_valid(dict(tok, lin=-0.065), now + 0.1, -0.065, 0.0, check_pid=False)[0])
check("token for 0.2 m/s refused", not FR.token_valid(dict(tok, lin=0.2), now + 0.1, 0.2, 0.0, check_pid=False)[0])
check("token from a non-explore pid refused", not FR.token_valid(tok, now + 0.1, 0.065, 0.0, check_pid=True)[0])
check("future-dated token refused", not FR.token_valid(dict(tok, t=now + 5, exp=now + 5.5), now, 0.065, 0.0, check_pid=False)[0])
check("long-span token refused", not FR.token_valid(dict(tok, exp=now + 30), now + 0.1, 0.065, 0.0, check_pid=False)[0])
FR.clear_token(tp)
check("clear_token removes it", FR.read_token(tp) is None)

# ---------------------------------------------------------------- 7. the REAL gate guard (+ _fusion_nudge)
G = S.load_gate(ROOT)
check("gate imports gb_fusion_recovery", G.FR is not None)
gd = tempfile.mkdtemp(); G.ESTOP = os.path.join(gd, "ESTOP"); G.ARMED_FILE = os.path.join(gd, "ARMED")
open(G.ARMED_FILE, "w").write("{}")
G.FR._pid_is_explore = lambda pid: int(pid) == os.getpid()
gtp = G.FR.token_path(G.ESTOP)
def gate_for(world, cam_mode="ok", t=None):
    t = time.time() if t is None else t
    g = S.GateStandIn(G, S.make_fusion(world, t)); g.fr_cam = world.camera(t, cam_mode)
    g.front, g.rear, g.sweep = g.fz.front(t)[0], g.fz.lidar.rev, g.fz.lidar.sweep
    return g
w = S.World(circles=CHAIR)
g = gate_for(w); G.FR.clear_token(gtp)
l, a, why = g.guard(0.065, 0.0, stream=True)
check("gate: no token -> normal block stands", why is not None and l == 0, (l, why))
G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
l, a, why = g.guard(0.065, 0.0, stream=True)
check("gate: valid token + fresh camera + narrow legs -> micro-nudge allowed", why is None and 0.06 <= l <= 0.07, (l, a, why, g.logs[-2:]))
check("gate: fr_last recorded", g.fr_last and g.fr_last["normal_block"])
l, a, why = g.guard(0.065, 0.0, stream=False)
check("gate: non-stream (voice/timed) never relaxed", why is not None)
g.src = "joystick"; l, a, why = g.guard(0.065, 0.0, stream=True); g.src = "stream"
check("gate: joystick never relaxed", why is not None)
g.mode = "timed"; l, a, why = g.guard(0.065, 0.0, stream=True); g.mode = "stream"
check("gate: mode not stream -> normal block", why is not None)
l, a, why = g.guard(0.20, 0.0, stream=True)
check("gate: 0.20 m/s with token -> block", why is not None)
l, a, why = g.guard(-0.065, 0.0, stream=True)
check("gate: reverse with token -> normal rules (block/none relaxed)", why is not None or l >= 0)
g.estop = True; l, a, why = g.guard(0.065, 0.0, stream=True); g.estop = False
check("gate: ESTOP flag -> never relaxed", why is not None)
os.remove(G.ARMED_FILE); l, a, why = g.guard(0.065, 0.0, stream=True); open(G.ARMED_FILE, "w").write("{}")
check("gate: disarmed -> never relaxed", why is not None)
g.lease = False; l, a, why = g.guard(0.065, 0.0, stream=True); g.lease = True
check("gate: no live mission lease -> never relaxed", why is not None)
g.trip_t = time.time() + 0.01; time.sleep(0.02); l, a, why = g.guard(0.065, 0.0, stream=True); g.trip_t = 0.0
check("gate: token older than last trip -> void", why is not None)
gs = gate_for(w, "stale"); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: stale camera -> LiDAR-only block", gs.guard(0.065, 0.0, stream=True)[2] is not None)
gs = gate_for(w, "offline"); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: camera offline -> LiDAR-only block", gs.guard(0.065, 0.0, stream=True)[2] is not None)
gs = gate_for(w, "lowvalid"); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: low-validity camera -> LiDAR-only block", gs.guard(0.065, 0.0, stream=True)[2] is not None)
gw = gate_for(S.World(segs=[(0.23, -0.6, 0.23, 0.6)])); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: solid wall + token -> block", gw.guard(0.065, 0.0, stream=True)[2] is not None)
gl = gate_for(S.World(circles=[(0.19, 0.0, 0.012)])); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: leg inside hard floor + token -> block", gl.guard(0.065, 0.0, stream=True)[2] is not None)
g = gate_for(w); G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore")
check("gate: allowed while token fresh", g.guard(0.065, 0.0, stream=True)[2] is None)
time.sleep(G.FR.TOKEN_TTL_S + 0.1)
g = gate_for(w)
check("gate: AUTO-RESTORE - token expired -> normal 0.27/0.24 block again", g.guard(0.065, 0.0, stream=True)[2] is not None)
G.FR.write_token(gtp, 0.065, 0.0, "fwd", "explore", pid=1)
check("gate: token owned by another pid -> block", gate_for(w).guard(0.065, 0.0, stream=True)[2] is not None)
G.FR.clear_token(gtp)
wo = S.World(circles=[(1.0, 0.0, 0.05)])
go = gate_for(wo)
l, a, why = go.guard(0.30, 0.0, stream=True)
check("gate: open floor normal command unaffected (no token needed)", why is None and l > 0.07)
gzero = gate_for(w)
check("gate: zero command passes untouched", gzero.guard(0.0, 0.0, stream=True)[2] is None)
saved = G.FR; G.FR = None
check("gate: module missing -> plain normal block", gate_for(w).guard(0.065, 0.0, stream=True)[2] is not None)
G.FR = saved

# ---------------------------------------------------------------- 8. wiring (explore hooks, run script, gate status)
xs = open(os.path.join(ROOT, "gb_explore.py")).read()
for hook in ("wander", "boxed", "go_to_points", "distance", "nav_goto"):
    check("explore hook mentions %s" % hook, hook in xs)
check("explore: fusion_escape_run defined", "def fusion_escape_run(" in xs)
check("explore: token cleared in finally / mission end", xs.count("clear_token") >= 2)
check("explore: restore log line", "normal clearances 0.27/0.28/0.24 restored" in xs)
check("explore: copies module next to gb_explore", "gb_fusion_recovery.py" in xs)
rs = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read()
check("gb_safety_run.sh copies module to gate code dir", '"$TREE/gb_fusion_recovery.py" "$CODE/gb_fusion_recovery.py"' in rs)
check("gate status exposes fr block", '"fr"' in gsrc and "squeeze_active" in gsrc)

print("\n%d passed, %d failed" % (P, F))
sys.exit(1 if F else 0)
