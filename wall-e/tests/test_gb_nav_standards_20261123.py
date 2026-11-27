"""Unit tests for the 2026-11-23 'navstd' phase (nav standards): stalled-progress (LiDAR + depth + IMU gyro), all-sides
body-envelope zones with escape, gated recovery + clearest-sector turn, fault -> ESTOP escalation, and full-sensor-suite
consumption by the gate status / explore planner / voice AI. Pure Python + numpy, no ROS. Scans mimic live /scan_raw:
400 rays, inc 2*pi/400, range_min 0.10, range_max 10.0, invalid = NaN."""
import ast, json, math, os, re, sys, tempfile, time, types
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_obstacle as OB
N = 400; A0 = 0.0; INC = 2 * math.pi / N; RMIN = 0.10; RMAX = 10.0; NAN = float("nan"); INF = float("inf")
RES = []
def check(name, cond, info=""):
    RES.append((name, bool(cond))); print(("PASS " if cond else "FAIL ") + name + ("  | " + info if info else ""))
def ang(i): return OB._wrap(A0 + i * INC)
def ray(a, walls):
    best = INF; c, s = math.cos(a), math.sin(a)
    for w in walls:
        if w[0] == "x" and abs(c) > 1e-9:
            r = w[1] / c
            if r > 0 and w[2] <= r * s <= w[3]: best = min(best, r)
        if w[0] == "y" and abs(s) > 1e-9:
            r = w[1] / s
            if r > 0 and w[2] <= r * c <= w[3]: best = min(best, r)
    return best
ROOM = [("x", 3.0, -9, 9), ("x", -2.5, -9, 9), ("y", 2.6, -9, 9), ("y", -2.4, -9, 9)]
def scan(walls=(), nan_deg=(), base=ROOM):
    r = []
    for i in range(N):
        d = ray(ang(i), list(walls) + list(base)); r.append(d if RMIN <= d <= RMAX else NAN)
    for lo, hi in nan_deg:
        for i in range(N):
            if lo <= math.degrees(ang(i)) <= hi: r[i] = NAN
    return r
def view(r, t=100.0): return OB.lidar_analyze(r, A0, INC, RMIN, RMAX, t)
def fusion(r, t=100.0, pose=(0.0, 0.0, 0.0)):
    f = OB.Fusion(); f.on_odom(t, 0.0, 0.0, *pose); f.imu_t = t
    D = OB.DepthView(t); D.fwd = INF; D.valid_frac = 0.9; f.on_depth(D)
    f.on_lidar(view(r, t)); return f
def lim(f, l, a, t=100.0): return f.limit(l, a, t, 0.12)
def box(x0, x1, y0, y1):   # small solid box (4 walls)
    return [("x", x0, y0, y1), ("x", x1, y0, y1), ("y", y0, x0, x1), ("y", y1, x0, x1)]

# ============ standing rules
check("thresholds unchanged in gb_obstacle 0.27/0.28/0.24", (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24))
gsrc = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
check("gate takes stop distances from gb_obstacle (no override literals)",
      not re.search(r"^(FRONT_STOP|REAR_STOP|SWEEP_R)\s*=\s*[0-9.]+\s*$", gsrc, re.M), "")
check("no 0.50 m 360-degree clearance rule (envelope zones sized to the body)",
      OB.ENV_SLOW <= 0.10 and abs(OB.HALF_W - 0.15) < 1e-9 and not re.search(r"0\.50?\b.*360|360.*0\.50?\b", open(os.path.join(ROOT, "gb_obstacle.py")).read()))
check("body footprint constants 0.13 / 0.155 / 0.10 / 0.05", (OB.BODY_FRONT, OB.BODY_REAR, OB.BODY_HALF_W, OB.SIDE_MARGIN) == (0.13, 0.155, 0.10, 0.05))

# ============ CATEGORY 2: all-sides body envelope
r = scan(); f = fusion(r); l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 open room: no envelope cap (forward 0.10 allowed)", l == 0.10 and kind is None, "l=%.3f why=%s" % (l, why))
wallL = [("y", 0.13, -0.15, 0.12)]                  # object 3 cm left of the left track, alongside the body only
v = view(scan(walls=wallL)); f = fusion(scan(walls=wallL))
check("C2 left wall alongside: left gap ~0.03 m measured", abs(v.left - 0.03) < 0.006 and v.right > 0.5, "left=%.3f right=%.3f" % (v.left, v.right))
check("C2 left wall alongside: NOT counted as 'ahead'/'behind' (no deadlock)", v.fwd > 0.27 and v.rev > 0.28, "fwd=%.2f rev=%.2f" % (v.fwd, v.rev))
l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 left wall alongside: straight forward escape allowed, capped to V_SIDE", abs(l - OB.V_SIDE) < 1e-9 and kind == "slow", "l=%.3f why=%s" % (l, why))
l, a, why, kind = lim(f, -0.10, 0.0)
check("C2 left wall alongside: straight reverse escape allowed, capped", 0 < -l <= OB.V_SIDE + 1e-9, "l=%.3f why=%s" % (l, why))
l, a, why, kind = lim(f, 0.0, 0.6)
check("C2 left wall alongside: rotate in place refused (sweep 0.24)", l == 0 and a == 0 and kind == "block", "why=%s" % why)
l, a, why, kind = lim(f, 0.0, -0.6)
check("C2 left wall alongside: rotate the other way also refused (rear corner swings into it)", a == 0 and kind == "block")
l, a, why, kind = lim(f, 0.08, 0.6)
check("C2 left wall alongside: arc INTO wall -> turn removed, straight creep only", a == 0.0 and l <= OB.V_SIDE + 1e-9, "l=%.3f a=%.2f why=%s" % (l, a, why))
wallR = [("y", -0.13, -0.15, 0.12)]; v = view(scan(walls=wallR)); f = fusion(scan(walls=wallR))
l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 right wall alongside (mirror): right gap ~0.03, forward capped not stopped", abs(v.right - 0.03) < 0.006 and abs(l - OB.V_SIDE) < 1e-9, "right=%.3f l=%.3f" % (v.right, l))
touch = [("y", 0.105, -0.15, 0.12)]; f = fusion(scan(walls=touch)); l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 object touching the side (gap 0.005): creep only V_MIN", abs(l - OB.V_MIN) < 1e-9 and "touching" in (why or ""), "l=%.3f why=%s" % (l, why))
post_fc = box(0.19, 0.22, 0.165, 0.195)               # post near the front-left corner, outside the corridor
v = view(scan(walls=post_fc)); f = fusion(scan(walls=post_fc)); l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 front-left corner post (gap < 0.10): forward slowed to V_SIDE", abs(l - OB.V_SIDE) < 1e-9 and v.env_f < OB.ENV_SLOW, "env_f=%.3f l=%.3f why=%s" % (v.env_f, l, why))
l, a, why, kind = lim(f, -0.10, 0.0)
check("C2 front corner post does not slow reverse", -l > OB.V_SIDE, "l=%.3f why=%s" % (l, why))
post_rc = box(-0.25, -0.22, -0.20, -0.17); v = view(scan(walls=post_rc)); f = fusion(scan(walls=post_rc))
l, a, why, kind = lim(f, -0.10, 0.0)
check("C2 rear-right corner post: reverse slowed to V_SIDE", abs(-l - OB.V_SIDE) < 1e-9 and v.env_r < OB.ENV_SLOW, "env_r=%.3f l=%.3f" % (v.env_r, l))
l, a, why, kind = lim(f, 0.10, 0.0)
check("C2 rear corner post does not slow forward", l > OB.V_SIDE, "l=%.3f why=%s" % (l, why))
f = fusion(scan(walls=[("x", 0.24, -9, 9)])); l, a, why, kind = lim(f, 0.08, 0.0)
check("C2 front stop 0.27 still enforced (wall 0.24)", kind == "block" and l == 0)
f = fusion(scan(walls=[("x", -0.27, -9, 9)])); l, a, why, kind = lim(f, -0.08, 0.0)
check("C2 rear stop 0.28 still enforced (wall 0.27 behind)", kind == "block" and l == 0)
f = fusion(scan(walls=box(0.115, 0.125, -0.02, 0.02))); l, a, why, kind = lim(f, 0.08, 0.0)
check("C2 point inside the body width at the front edge: forward STOP", kind == "block" and l == 0, "why=%s" % why)
f = fusion(scan(walls=[("y", -0.26, -0.3, 0.6)])); l, a, why, kind = lim(f, 0.08, 0.6)
check("C2 arc AWAY (left) from a right side wall 16 cm off the track: allowed (corners clear)", l > 0 and a > 0 and kind != "block", "l=%.3f a=%.2f why=%s" % (l, a, why))

f = fusion(scan(walls=[("y", 0.13, -0.15, 0.60)])); l, a, why, kind = lim(f, 0.08, 0.0)
check("C2 wall that CONTINUES ahead inside the 5 cm margin still blocks forward (corridor unchanged)", kind == "block" and l == 0, "why=%s" % why)

# ============ CATEGORY 1: stalled progress
P = OB.Progress(); ev = None; t = 0.0
while t < 5.0: ev = P.update(t, 0.08, 0.50, None) or ev; t += 0.05
check("C1 forward 0.08 m/s, LiDAR obstacle stays 0.50 m for 3.5 s+ -> no-progress (genuine stall still caught)", ev and "no progress forward" in ev and "ahead" in ev, str(ev))
P = OB.Progress(); ev = None; t = 0.0; d = 0.70
while t < 3.0: ev = P.update(t, 0.06, d, d + 0.08) or ev; d -= 0.06 * 0.05; t += 0.05
check("C1 forward with LiDAR+depth range closing at the commanded rate: no event", ev is None, str(ev))
P = OB.Progress(); ev = None; t = 0.0
while t < 3.0: ev = P.update(t, 0.04, 2.5, None) or ev; t += 0.05
check("C1 slow creep in an open room (nothing within 0.8 m): never judged", ev is None)
P = OB.Progress(); ev = None; t = 0.0
while t < 5.0: ev = P.update(t, 0.08, INF, 0.45) or ev; t += 0.05
check("C1 depth-only evidence (low obstacle, LiDAR clear) not shrinking -> no-progress", ev and "depth closed" in ev, str(ev))
P = OB.Progress(); ev = None; t = 0.0; dz = 0.6
while t < 5.0: ev = P.update(t, 0.08, 0.5, dz) or ev; dz -= 0.08 * 0.05; t += 0.05
check("C1 LiDAR flat but depth closing (moving) -> no false event", ev is None, str(ev))
P = OB.Progress(); ev = None; t = 0.0
while t < 5.0: ev = P.update(t, -0.08, 0.45, None) or ev; t += 0.05
check("C1 reversing, rear obstacle 0.45 m not closing -> 'no progress reversing ... behind'", ev and "reversing" in ev and "behind" in ev, str(ev))
P = OB.Progress(); ev = None; t = 0.0
while t < 3.0: ev = P.update(t, 0.0, 0.40, 0.40) or ev; t += 0.05
check("C1 stationary (lin 0, e.g. in-place turn): progress never judged", ev is None)
P = OB.Progress(); ev = None; t = 0.0
while t < OB.PROG_WIN - 0.2: ev = P.update(t, 0.08, 0.5, None) or ev; t += 0.05
check("C1 judged only after the full %.1fs window" % OB.PROG_WIN, ev is None)
f = fusion(scan(walls=[("x", 0.50, -9, 9)])); evs = []; t = 100.0
while t < 104.0:
    f.lidar.t = t; f.on_cmd(t, 0.08, 0.0); t += 0.05
e = f.pop_event(t - 0.05)
check("C1 wired into Fusion.on_cmd (gate path) -> event 'lidar+depth progress'", e and e[1] == "lidar+depth progress", str(e))
# 2026-11-25 noprog (John 12:59/13:03 ET): phantom no-progress at slow speed must not trip; genuine stalls still do
import random as _r; _r.seed(7)
P = OB.Progress(); ev = None; t = 0.0; d = 0.70
while t < 6.0: ev = P.update(t, 0.08, d + _r.uniform(-0.015, 0.015), None) or ev; d -= 0.08 * 0.05; t += 0.05
check("C1 noprog: 0.08 m/s really moving, LiDAR noise +/-1.5 cm -> no phantom no-progress", ev is None, str(ev))
SL = OB.Slip(); ev = None; t = 0.0; d = 1.45; od = 0.0
while t < 6.0: ev = SL.update(t, True, od, d + _r.uniform(-0.015, 0.015)) or ev; d -= 0.08 * 0.05; od += 0.08 * 0.05; t += 0.05
check("C1 noprog: 12:57 case (front 1.45 m, 0.08 m/s, noise) -> Slip no false trip", ev is None, str(ev))
P = OB.Progress(); ev = None; t = 0.0
while t < 6.0: ev = P.update(t, 0.03, 0.50, None) or ev; t += 0.05
check("C1 noprog: speed-scaled - 0.03 m/s never reaches 0.15 m commanded in %.1fs -> not judged" % OB.PROG_WIN, ev is None, str(ev))
SL = OB.Slip(); ev = None; t = 0.0
while t < 6.0: ev = SL.update(t, True, t * 0.08, 0.80) or ev; t += 0.05
check("C1 noprog: genuine slip (odom +0.08 m/s, wall fixed 0.80 m) -> still 'no progress'", ev and "no progress" in ev, str(ev))
for dist, cmd in ((0.20, 0.08), (0.24, 0.08), (0.10, 0.04)):
    f = fusion(scan(walls=[("x", dist, -9, 9)])); l, a, why, kind = lim(f, cmd, 0.0)
    check("C1 noprog: close object %.2f m ahead (simulated scan) -> forward STOP" % dist, kind == "block" and l == 0, "l=%s kind=%s why=%s" % (l, kind, why))
check("C1 noprog: stop distances unchanged (front 0.27 / rear 0.28)", abs(OB.STOP_FWD - 0.27) < 1e-9 and abs(OB.STOP_REV - 0.28) < 1e-9, str((OB.STOP_FWD, OB.STOP_REV)))
TP = OB.TurnProgress(); ev = None; t = 0.0
while t < 3.0: ev = TP.update(t, 0.0, 0.8, 0.01) or ev; t += 0.02
check("C1 gyro turn-stall NOT armed until the gyro has proven itself (no false trips on a dead gyro)", ev is None and not TP.proven)
TP.update(3.0, 0.0, 0.0, 0.5); ev = None; t = 4.0
while t < 7.0: ev = TP.update(t, 0.0, 0.8, 0.02) or ev; t += 0.02
check("C1 proven gyro: commanded 0.8 rad/s, gyro 0.02 rad/s -> 'turn stall (imu gyro)'", ev and "turn stall (imu gyro)" in ev, str(ev))
TP = OB.TurnProgress(); TP.proven = True; ev = None; t = 0.0
while t < 3.0: ev = TP.update(t, 0.0, 0.8, 0.45) or ev; t += 0.02
check("C1 skid-steer turn slipping but turning (0.45 of 0.8 rad/s) -> no event", ev is None)
TP = OB.TurnProgress(); TP.proven = True; ev = None; t = 0.0
while t < 3.0: ev = TP.update(t, 0.06, 0.8, 0.0) or ev; t += 0.02
check("C1 arc (lin != 0) is not judged by the in-place turn check", ev is None)
f = fusion(scan()); f.tprog.proven = True; f.cmd = (0.0, 0.8); t = 100.0
while t < 103.0: f.on_imu(t, 0.0, 0.0, 9.8, 0.01); t += 0.02
e = next((x for x in f.events if x[1] == "imu-gyro"), None)
check("C1 wired into Fusion.on_imu(wz) -> event 'imu-gyro'", e and e[1] == "imu-gyro", str(e))
check("C1 Fusion.on_imu stays backward compatible (no wz)", (OB.Fusion().on_imu(1.0, 0.0, 0.0, 9.8) is None))

# ============ explore planner (static extraction, no ROS)
esrc = open(os.path.join(ROOT, "gb_explore.py")).read(); et = ast.parse(esrc); ens = {"math": math}
for node in et.body:
    if isinstance(node, ast.FunctionDef) and node.name in ("trip_dirs", "open_dir", "fused_view"):
        exec(compile(ast.Module([node], []), node.name, "exec"), ens)
td = ens["trip_dirs"]; od = ens["open_dir"]; fv = ens["fused_view"]
check("C1->planner: forward no-progress holds fwd only", td("bump detected (lidar+depth progress): no progress forward: ... obstacle 0.50m ahead") == {"fwd"})
check("C1->planner: reverse no-progress holds rev only", td("bump detected (lidar+depth progress): no progress reversing: ... obstacle 0.45m behind") == {"rev"})
check("C1->planner: gyro turn stall holds turn", td("bump detected (imu-gyro): turn stall (imu gyro): commanded 0.80 rad/s") == {"turn"})

# ============ CATEGORY 3: recovery + escalation
dark_left = scan(walls=[("y", 0.5, -9, 9)], nan_deg=[(15, 165)])   # dark wall: the whole left half NaN (> 60-ray run)
v = view(dark_left); lo, ro = OB.side_open(v); ch, cm = OB.clearest_heading(v)
check("C3 all-NaN left side scores 0 (unknown is not open; old mean() called it 3 m)", lo == 0.0 and ro > 2.0, "left=%.2f right=%.2f" % (lo, ro))
check("C3 clearest heading never picks an unknown (NaN) sector", ch is not None and not (15 <= math.degrees(ch) <= 165), "clear=%.0f deg" % math.degrees(ch))
v = view(scan(walls=[("x", 0.6, -9, 9), ("y", 0.5, -9, 9)])); ch, cm = OB.clearest_heading(v, exclude_rear=True)
check("C3 wall ahead + wall left -> clearest sector is to the right", ch is not None and math.degrees(ch) < -15, "clear=%.0f deg %.2f m" % (math.degrees(ch), cm))
check("C3 planner open_dir: clearest -90 deg -> rotate right", od({"clear_deg": -90.0, "left_mean": 3.0, "right_mean": 0.0}) == -1)
check("C3 planner open_dir: no known sector -> side-score fallback", od({"clear_deg": None, "left_mean": 1.2, "right_mean": 0.4}) == 1)
check("C3 explore back-off requires rear >= rear stop + BACKOFF_MARGIN (0.05)", "BACKOFF_MARGIN = 0.05" in esrc and esrc.count("rear_stop + BACKOFF_MARGIN") >= 3)
check("C3 explore: recovery turns only via turn_clear (360 sweep >= SWEEP_R, rear not blind)",
      "if sw < OB.SWEEP_R" in esrc and "if not turn_ok:" in esrc and "OB.BLIND_FRAC_STOP" in esrc)
check("C3 explore: side-wall slide-out escape (straight, gate-capped) when the sweep is blocked", "def side_slide" in esrc and esrc.count("side_slide()") >= 2)
# gate collision()/fault_latch() exercised with a stub (no ROS)
gt = ast.parse(gsrc); cls = next(n for n in gt.body if isinstance(n, ast.ClassDef) and n.name == "Gate")
gns = {"time": time, "os": os, "OB": OB, "math": math, "json": json}
for node in gt.body:
    if isinstance(node, ast.Assign) and any(getattr(t_, "id", "") in ("FAULT_LIMIT", "FAULT_WIN", "BACKOFF_MARGIN", "BACKOFF_V", "BACKOFF_S", "SOFT_SENSORS", "SOFT_LIMIT", "SOFT_WIN") for t_ in node.targets):
        exec(compile(ast.Module([node], []), "c", "exec"), {"E": lambda k, d: d, **gns}, gns)
tmpd = tempfile.mkdtemp(); gns.update(ESTOP=os.path.join(tmpd, "ESTOP"), REAR_STOP=0.28)
meth = {}
for n in cls.body:
    if isinstance(n, ast.FunctionDef) and n.name in ("collision", "fault_latch"):
        exec(compile(ast.Module([n], []), n.name, "exec"), gns); meth[n.name] = gns[n.name]
class FZ:
    def __init__(s_, rev, blind_r): s_.lidar = types.SimpleNamespace(rev=rev, blind_r=blind_r); s_.q = []
    def pop_event(s_, now): return s_.q.pop(0) if s_.q else None
    def reset_detectors(s_): pass
class G:
    collision = meth["collision"]; fault_latch = meth["fault_latch"]
    def __init__(s_, rev=0.6, blind_r=0.0):
        s_.fz = FZ(rev, blind_r); s_.faults = []; s_.estop = False; s_.backoff = False; s_.lin = 0.06; s_.mode = "stream"
        s_.trips = []; s_.latches = []; s_.logs = []
    def trip(s_, why, src): s_.trips.append(why)
    def finish(s_, why): s_.mode = "idle"
    def log(s_, m): s_.logs.append(m)
    def _write_latch(s_, f, d): s_.latches.append((os.path.basename(f), d))
g = G()
for k in range(2):
    g.fz.q.append((time.time(), "encoders", "track stall")); g.lin = 0.06; g.backoff = False; g.collision("explore")
check("C3 two collision faults: no ESTOP latch yet", not g.estop and not g.latches, "faults=%d" % len(g.faults))
check("C3 back-off started after a forward bump with rear 0.60 m clear and rear not blind", any("backoff" in m for m in g.logs))
g.fz.q.append((time.time(), "lidar+depth progress", "no progress forward")); g.lin = 0.06; g.collision("explore")
check("C3 noprog: 3 stall-type events (12:57 pattern) -> soft pauses, NO ESTOP", not g.estop and not g.latches and len(g.trips) == 3, "trips=%d latches=%s" % (len(g.trips), g.latches))
check("C3 noprog: stall events do not count toward FAULT_LIMIT", len(g.faults) == 0, len(g.faults))
for k in range(gns["SOFT_LIMIT"] - 3):
    g.fz.q.append((time.time(), "encoders", "track stall")); g.lin = 0.06; g.backoff = False; g.collision("explore")
check("C3 noprog: %d repeated stall events in %ds -> latch (real stuck robot still escalates)" % (gns["SOFT_LIMIT"], gns["SOFT_WIN"]), g.estop and any(x[0] == "ESTOP" for x in g.latches))
g = G()
for k in range(3):
    g.fz.q.append((time.time(), "imu", "jolt 2.1 g")); g.lin = 0.06; g.collision("explore")
names = sorted(x[0] for x in g.latches)
check("C3 third HARD collision (IMU jolt) within %ds -> ESTOP latched (ESTOP_REQ + ESTOP, kind=safety)" % gns["FAULT_WIN"],
      g.estop and names == ["ESTOP", "ESTOP_REQ"] and all(d["kind"] == "safety" for _n, d in g.latches), str(names))
fl = "\n".join(l_ for l_ in gsrc.splitlines())
body = gsrc[gsrc.index("def fault_latch"):gsrc.index("def tilt_latch")]
check("C3 fault latch never clears ESTOP (no remove/unlink/CLEAR_REQ in the latch path)", not re.search(r"remove|unlink|CLEAR_REQ", body))
g = G(rev=0.31); g.fz.q.append((time.time(), "encoders", "track stall")); g.collision("explore")
check("C3 no back-off when rear < STOP_REV + 0.05 (rear 0.31)", not any("backoff" in m for m in g.logs))
g = G(rev=0.9, blind_r=0.7); g.fz.q.append((time.time(), "encoders", "track stall")); g.collision("explore")
check("C3 no back-off when rear LiDAR is blind (70% dropouts)", not any("backoff" in m for m in g.logs))
g = G(); g.fz.q.append((time.time(), "imu-gyro", "turn stall (imu gyro)")); g.lin = 0.0; g.collision("explore")
check("C3 gyro turn stall: stop, no back-off", g.trips and not any("backoff" in m for m in g.logs))

# ============ full sensor suite: gate snapshot -> planner -> voice AI
f = fusion(scan(walls=wallL + [("x", 0.6, -0.05, 0.05)])); f.on_imu(100.0, 0.0, 0.0, 9.8, 0.3)
f.on_rgb({"t": 100.0, "hazard": True, "label": "person"})
D = OB.DepthView(100.0); D.fwd = 0.45; D.valid_frac = 0.8; f.on_depth(D)
snap = f.snapshot(100.0)
need = [("lidar", "fwd"), ("lidar", "rev"), ("lidar", "sweep"), ("lidar", "left_gap"), ("lidar", "right_gap"), ("lidar", "env_front"),
        ("lidar", "env_rear"), ("lidar", "blind_f"), ("lidar", "blind_r"), ("lidar", "imputed"), ("lidar", "clearest_deg"),
        ("lidar", "mem_fwd"), ("lidar", "mem_rev"), ("depth", "fwd"), ("depth", "mem"), ("depth", "valid"), ("imu", "tilt_deg"),
        ("imu", "wz"), ("rgb", "hazard"), ("odom", "v")]
missing = [a + "." + b for a, b in need if b not in (snap.get(a) or {})]
check("SUITE gate snapshot carries LiDAR sectors/sides/dropouts/memories + depth + depth memory + IMU + RGB + odom", not missing, "missing=%s" % missing)
check("SUITE snapshot values: left gap 0.03, depth 0.45 fused front, RGB person, gyro 0.3",
      abs(snap["lidar"]["left_gap"] - 0.03) < 0.006 and snap["front_fused"] == 0.45 and snap["front_src"] == "depth"
      and snap["rgb"]["hazard"] and abs(snap["imu"]["wz"] - 0.3) < 1e-6, json.dumps(snap)[:160])
check("SUITE gate publishes the snapshot in /gb/gate_status and writes sensors.json (2 Hz) for the AI",
      'd["sensors"] = snap' in gsrc and "SENSORS_FILE" in gsrc and "os.replace(SENSORS_FILE" in gsrc)
fvv = fv({"sensors": snap, "faults": {"recent": 1}})
check("SUITE explore fused_view exposes lidar/depth/imu/rgb/faults", fvv["depth"]["fwd"] == 0.45 and fvv["rgb"]["hazard"] and fvv["faults"]["recent"] == 1)
check("SUITE explore front = min(LiDAR cone, corridor, gate fused, depth, LiDAR memory, depth memory)",
      re.search(r"front = min\(s.get\(\"front\", 9\), s.get\(\"corr\", 9\), gf .*depth_f,\s*float\(lm\).*float\(dm\)", esrc, re.S) is not None)
check("SUITE explore uses IMU tilt hold + RGB advisory slow + dropout-aware side scores",
      "IMU tilt %.0f deg: holding" in esrc and "OB.V_RGB" in esrc and "OB.side_open(v)" in esrc)
vsrc = open(os.path.join(ROOT, "grokbot_voice.py")).read(); vt = ast.parse(vsrc); vns = {"json": json, "time": time, "os": os, "re": re}
for node in vt.body:
    if (isinstance(node, ast.FunctionDef) and node.name in ("_cm", "sensor_suite", "sensor_suite_note")) or \
       (isinstance(node, ast.Assign) and any(getattr(t_, "id", "") == "SENSORS_STALE_S" for t_ in node.targets)):
        exec(compile(ast.Module([node], []), "v", "exec"), vns)
tf_ = os.path.join(tmpd, "sensors.json")
json.dump({"t": time.time(), "estop": True, "front": 0.45, "rear": None, "sweep": 0.2, "faults": {"recent": 2, "limit": 3, "win_s": 60},
           "sensors": snap}, open(tf_, "w"))
d_ = vns["sensor_suite"](tf_); note = vns["sensor_suite_note"](d_)
for word in ("front 45 cm via depth", "left gap 3 cm", "dropouts ahead", "depth camera ahead 45 cm", "LiDAR memory",
             "IMU tilt", "camera sees person", "2 collision faults", "EMERGENCY STOP"):
    check("SUITE voice AI note mentions %r" % word, word in note, note[:200] if word not in note else "")
json.dump({"t": time.time() - 10, "sensors": snap}, open(tf_, "w"))
check("SUITE voice: stale telemetry -> None and an explicit 'do not guess' note", vns["sensor_suite"](tf_) is None and "do not guess" in vns["sensor_suite_note"](None))
check("SUITE voice: AI context + status answer include the fused sensor note",
      "parts.append(sensor_suite_note(sensor_suite()))" in vsrc and vsrc.count("return status_with_sensors(), \"local\"") == 2)

# ============ performance (per scan)
r = scan(walls=wallL + post_fc, nan_deg=[(23, 30), (-20, -10)]); n_ = 300; t0 = time.perf_counter()
for _ in range(n_): OB.lidar_analyze(r, A0, INC, RMIN, RMAX, 100.0)
ms = (time.perf_counter() - t0) / n_ * 1000
check("PERF lidar_analyze per 400-ray scan < 2 ms on this host", ms < 2.0, "%.3f ms" % ms)
print("\nRESULT: %d passed, %d failed" % (sum(1 for _n, ok in RES if ok), sum(1 for _n, ok in RES if not ok)))
sys.exit(0 if all(ok for _n, ok in RES) else 1)
