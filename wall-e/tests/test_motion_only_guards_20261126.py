#!/usr/bin/env python3
"""2026-11-26 motion-only safety guards (John): while motionless, safety guards must not
trip faults or blocks (close obstacle, stall, tilt, stale sensor). While moving (nonzero
commanded linear or angular velocity), the LiDAR collision/obstacle guards stay active
and still enforce the unchanged 0.27 m front / 0.28 m rear / 0.24 m turn-sweep stops.

Pure Python, no ROS, no motion. AST-extracts Gate.guard / Gate.collision like the
existing floor-noestop tests so we exercise the real gate methods.
"""
import ast, math, os, re, sys, tempfile, time, types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "gb_safety"))

import gb_obstacle as OB

RES = []

def check(name, cond, info=""):
    RES.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name + (("  | " + str(info)) if info and not cond else ""))

# --------------------------------------------------------------------------- stops must stay exactly these values
check("STOP_FWD is 0.27 m", abs(OB.STOP_FWD - 0.27) < 1e-12, OB.STOP_FWD)
check("STOP_REV is 0.28 m", abs(OB.STOP_REV - 0.28) < 1e-12, OB.STOP_REV)
check("SWEEP_R is 0.24 m", abs(OB.SWEEP_R - 0.24) < 1e-12, OB.SWEEP_R)

GATE_PATH = os.path.join(ROOT, "gb_safety", "gb_gate.py")
gsrc = open(GATE_PATH, encoding="utf-8").read()
check("gate FRONT/REAR/SWEEP aliases still come from OB.STOP_* (no literal override)",
      'str(OB.STOP_FWD)' in gsrc and 'str(OB.STOP_REV)' in gsrc and 'str(OB.SWEEP_R)' in gsrc
      and not re.search(r"^(FRONT_STOP|REAR_STOP|SWEEP_R|S_FRONT_STOP|S_REAR_STOP|S_SWEEP_R)\s*=\s*[0-9.]+\s*$", gsrc, re.M))
check("gate implements motion-only guard short-circuit",
      "commanded_motion" in gsrc and "GUARD OFF (motionless)" in gsrc and "2026-11-26 motiononly" in gsrc)

# --------------------------------------------------------------------------- AST-extract helpers + guard/collision (no ROS)
gt = ast.parse(gsrc)
gns = {"time": time, "os": os, "OB": OB, "math": math, "json": __import__("json"), "E": (lambda k, d: d)}
KEEP_ASSIGN = {
    "FRONT_STOP", "REAR_STOP", "SWEEP_R", "S_FRONT_STOP", "S_REAR_STOP", "S_SWEEP_R",
    "MAX_LIN", "MAX_ANG", "MANUAL_MAX_LIN", "AUTO_MAX_LIN", "MOTION_EPS_LIN", "MOTION_EPS_ANG", "MOTION_MODES",
    "FAULT_LIMIT", "FAULT_WIN", "SOFT_SENSORS", "SOFT_LIMIT", "SOFT_WIN",
    "BACKOFF_MARGIN", "BACKOFF_V", "BACKOFF_S", "RGB_FILE",
}
for node in gt.body:
    if isinstance(node, ast.Assign) and any(getattr(t, "id", "") in KEEP_ASSIGN for t in node.targets):
        exec(compile(ast.Module([node], []), "c", "exec"), gns, gns)
    if isinstance(node, ast.FunctionDef) and node.name in ("_clamp", "commanded_motion", "lin_cap_for"):
        exec(compile(ast.Module([node], []), node.name, "exec"), gns, gns)
cls = next(n for n in gt.body if isinstance(n, ast.ClassDef) and n.name == "Gate")
meth = {}
for n in cls.body:
    if isinstance(n, ast.FunctionDef) and n.name in ("guard", "collision", "trip", "finish"):
        exec(compile(ast.Module([n], []), n.name, "exec"), gns, gns)
        meth[n.name] = gns[n.name]
check("extracted commanded_motion + guard + collision",
      callable(gns.get("commanded_motion")) and "guard" in meth and "collision" in meth)

check("FRONT_STOP/REAR_STOP/SWEEP_R aliases equal 0.27/0.28/0.24",
      (gns["FRONT_STOP"], gns["REAR_STOP"], gns["SWEEP_R"]) == (0.27, 0.28, 0.24),
      (gns.get("FRONT_STOP"), gns.get("REAR_STOP"), gns.get("SWEEP_R")))
check("stream aliases also equal 0.27/0.28/0.24",
      (gns["S_FRONT_STOP"], gns["S_REAR_STOP"], gns["S_SWEEP_R"]) == (0.27, 0.28, 0.24))

# --------------------------------------------------------------------------- lidar wall scans (same convention as test_floor_noestop)
N = 400
INC = 2 * math.pi / N
NAN = float("nan")
INF = float("inf")

def scan_wall(x):
    """Flat wall perpendicular to heading at +x (front) or -x (rear)."""
    r = []
    for i in range(N):
        a = OB._wrap(i * INC)
        c = math.cos(a)
        d = (x / c) if abs(c) > 1e-9 and (x / c) > 0 else INF
        d = min(d, 3.0 / abs(math.sin(a)) if abs(math.sin(a)) > 1e-9 else INF)
        r.append(d if 0.10 <= d <= 10.0 else NAN)
    return r

def fusion(r, t=None):
    if t is None:
        t = time.time()
    f = OB.Fusion()
    f.on_odom(t, 0.0, 0.0, 0.0, 0.0, 0.0)
    f.imu_t = t
    D = OB.DepthView(t)
    D.fwd = INF
    D.valid_frac = 0.9
    f.on_depth(D)
    f.on_lidar(OB.lidar_analyze(r, 0.0, INC, 0.10, 10.0, t))
    return f

class GuardGate:
    """Minimal stand-in that runs the real Gate.guard method."""
    guard = meth["guard"]

    def __init__(self, fz):
        self.fz = fz
        self.slow_why = ""
        self._thr = {}
        self.front = getattr(fz.lidar, "fwd", float("inf")) if fz.lidar else float("inf")
        self.rear = getattr(fz.lidar, "rev", float("inf")) if fz.lidar else float("inf")
        self.sweep = getattr(fz.lidar, "sweep", float("inf")) if fz.lidar else float("inf")
        self.rgb_chk = 1e18  # skip RGB file reads
        self.logs = []

    def log(self, m):
        self.logs.append(m)

    def bench_off(self):
        return False

    def _lease_live(self):
        return False

    def _lin_cap(self, src=None, joystick=False):
        return 0.288

class FZ:
    def __init__(self):
        self.lidar = types.SimpleNamespace(rev=0.6, blind_r=0.0)
        self.q = []
        self.reset_n = 0

    def pop_event(self, now):
        return self.q.pop(0) if self.q else None

    def reset_detectors(self):
        self.reset_n += 1

class CollGate:
    """Minimal stand-in that runs the real Gate.collision method."""
    collision = meth["collision"]

    def __init__(self, lin=0.0, ang=0.0, mode="idle"):
        self.fz = FZ()
        self.faults = []
        self.soft = []
        self.estop = False
        self.maint = False
        self.backoff = False
        self.lin = lin
        self.ang = ang
        self.mode = mode
        self.trips = []
        self.finishes = []
        self.logs = []
        self._thr = {}
        self.src = "test"
        self.rid = "t"

    def trip(self, why, src):
        self.trips.append(why)

    def finish(self, why):
        self.mode = "idle"
        self.lin = 0.0
        self.ang = 0.0
        self.finishes.append(why)

    def log(self, m):
        self.logs.append(m)

    def tilt_latch(self, why, src):
        self.estop = True
        self.finishes.append("TILT ESTOP: " + why)

    def fault_latch(self, why, src):
        self.estop = True
        self.finishes.append("FAULT LATCH: " + why)

    def bench_off(self):
        return False

# --------------------------------------------------------------------------- commanded_motion helper
cm = gns["commanded_motion"]
check("commanded_motion(0,0) is False (motionless)", cm(0.0, 0.0) is False)
check("commanded_motion(None, None) is False", cm(None, None) is False)
check("commanded_motion(0.08, 0) is True (forward)", cm(0.08, 0.0) is True)
check("commanded_motion(-0.08, 0) is True (reverse)", cm(-0.08, 0.0) is True)
check("commanded_motion(0, 0.4) is True (turn)", cm(0.0, 0.4) is True)
check("commanded_motion tiny noise 1e-5 is motionless", cm(1e-5, -1e-5) is False)

# --------------------------------------------------------------------------- motionless: guard must not block, even with a close wall or stale sensors
g = GuardGate(fusion(scan_wall(0.12)))
l, a, why = g.guard(0.0, 0.0)
check("MOTIONLESS close front wall 0.12 m: guard(0,0) does not block", why is None and abs(l) < 1e-9 and abs(a) < 1e-9, (l, a, why))

g = GuardGate(fusion(scan_wall(-0.12)))
l, a, why = g.guard(0.0, 0.0)
check("MOTIONLESS close rear wall 0.12 m: guard(0,0) does not block", why is None and abs(l) < 1e-9, (l, a, why))

f_stale = OB.Fusion()  # no lidar, no depth, no odom -> would be stale if moving
g = GuardGate(f_stale)
l, a, why = g.guard(0.0, 0.0)
check("MOTIONLESS stale/missing sensors: guard(0,0) does not block", why is None, (l, a, why))

# start-of-move (mode may still be idle) WITH a nonzero command MUST still stop at 0.27
g = GuardGate(fusion(scan_wall(0.12)))
l, a, why = g.guard(0.08, 0.0)
check("IDLE mode but commanded 0.08 m/s into 0.12 m wall: LiDAR guard BLOCKS (0.27 m)",
      l == 0 and why and "obstacle" in why and "0.27" in why, (l, a, why))

# --------------------------------------------------------------------------- motionless: collision events must not trip / latch / finish
for sensor, evwhy, label in (
    ("encoders", "track stall: commanded 0.10 m/s, encoders 0.00 m/s", "stall"),
    ("imu-tilt", "tilted pitch 18 / roll 2 deg (climbing / stuck on something)", "tilt"),
    ("imu-tilt-hard", "tipping: pitch 31 / roll 0 deg (over 25 deg)", "hard-tilt"),
    ("imu", "jolt 9.1 m/s2 (bump / sudden stop)", "jolt"),
    ("lidar+depth progress", "no progress toward a closer reading", "no-progress"),
):
    g = CollGate(lin=0.0, ang=0.0, mode="idle")
    g.fz.q.append((time.time(), sensor, evwhy))
    fired = g.collision("test")
    check("MOTIONLESS %s event: no trip/latch/block" % label,
          fired is False and not g.trips and not g.estop and not g.finishes
          and any("GUARD OFF (motionless)" in m for m in g.logs) and g.fz.reset_n >= 1,
          (fired, g.trips, g.finishes, g.logs, g.estop))

# mode=turn with lin=0 is still "moving" — stall must still be processed (existing floor tests rely on this)
g = CollGate(lin=0.0, ang=0.0, mode="turn")
g.fz.q.append((time.time(), "imu-gyro", "turn stall (imu gyro)"))
fired = g.collision("explore")
check("MOVING (mode=turn, lin=0): stall event is still processed",
      fired is True and g.trips and g.finishes, (fired, g.trips, g.finishes))

# moving + jolt still trips
g = CollGate(lin=0.08, ang=0.0, mode="stream")
g.fz.q.append((time.time(), "imu", "jolt 9.1 m/s2 (bump / sudden stop)"))
fired = g.collision("explore")
check("MOVING forward: jolt event still trips a stop",
      fired is True and g.trips and g.finishes, (fired, g.trips, g.finishes))

# --------------------------------------------------------------------------- moving: LiDAR stops at the unchanged thresholds (via Fusion AND via gate.guard)
for d in (0.10, 0.20, 0.24, 0.26):
    f = fusion(scan_wall(d))
    l, a, why, kind = f.limit(0.08, 0.0, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
    check("MOVING fwd: front wall %.2f m -> Fusion BLOCKS (stop 0.27)" % d,
          l == 0 and kind == "block", "l=%s kind=%s why=%s" % (l, kind, why))
    g = GuardGate(f)
    gl, ga, gwhy = g.guard(0.08, 0.0)
    check("MOVING fwd: front wall %.2f m -> gate.guard BLOCKS (stop 0.27)" % d,
          gl == 0 and gwhy, "l=%s why=%s" % (gl, gwhy))

for d in (0.28, 0.40, 1.50):
    f = fusion(scan_wall(d))
    l, a, why, kind = f.limit(0.08, 0.0, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
    check("MOVING fwd: front wall %.2f m -> Fusion ALLOWS (beyond 0.27)" % d,
          l > 0 and kind != "block", "l=%s kind=%s why=%s" % (l, kind, why))
    g = GuardGate(f)
    gl, ga, gwhy = g.guard(0.08, 0.0)
    check("MOVING fwd: front wall %.2f m -> gate.guard ALLOWS (beyond 0.27)" % d,
          gl > 0 and gwhy is None, "l=%s why=%s" % (gl, gwhy))

for d in (0.10, 0.20, 0.27):
    f = fusion(scan_wall(-d))
    l, a, why, kind = f.limit(-0.08, 0.0, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
    check("MOVING rev: rear wall %.2f m -> Fusion BLOCKS (stop 0.28)" % d,
          l == 0 and kind == "block", "l=%s kind=%s why=%s" % (l, kind, why))
    g = GuardGate(f)
    gl, ga, gwhy = g.guard(-0.08, 0.0)
    check("MOVING rev: rear wall %.2f m -> gate.guard BLOCKS (stop 0.28)" % d,
          gl == 0 and gwhy, "l=%s why=%s" % (gl, gwhy))

for d in (0.30, 0.50, 1.50):
    f = fusion(scan_wall(-d))
    l, a, why, kind = f.limit(-0.08, 0.0, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
    check("MOVING rev: rear wall %.2f m -> Fusion ALLOWS (beyond 0.28)" % d,
          l < 0 and kind != "block", "l=%s kind=%s why=%s" % (l, kind, why))
    g = GuardGate(f)
    gl, ga, gwhy = g.guard(-0.08, 0.0)
    check("MOVING rev: rear wall %.2f m -> gate.guard ALLOWS (beyond 0.28)" % d,
          gl < 0 and gwhy is None, "l=%s why=%s" % (gl, gwhy))

# Turn sweep 0.24 m: a tight frontal wall at 0.20 m is inside the 0.24 m sweep; 1.5 m is clear.
# (Same geometry the floor/obstacle tests use: front clearance also feeds the sweep.)
f = fusion(scan_wall(0.20))
l, a, why, kind = f.limit(0.0, 0.6, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
check("MOVING turn: obstacle 0.20 m inside 0.24 m sweep -> Fusion BLOCKS",
      a == 0 and kind == "block", "l=%s a=%s kind=%s why=%s" % (l, a, kind, why))
g = GuardGate(f)
gl, ga, gwhy = g.guard(0.0, 0.6)
check("MOVING turn: obstacle 0.20 m inside 0.24 m sweep -> gate.guard BLOCKS",
      ga == 0 and gwhy, "l=%s a=%s why=%s" % (gl, ga, gwhy))

f = fusion(scan_wall(1.50))
l, a, why, kind = f.limit(0.0, 0.6, time.time(), 0.12, OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)
check("MOVING turn: clear 1.50 m -> Fusion ALLOWS turn (sweep 0.24)",
      abs(a - 0.6) < 1e-9 and kind != "block", "l=%s a=%s kind=%s why=%s" % (l, a, kind, why))
g = GuardGate(f)
gl, ga, gwhy = g.guard(0.0, 0.6)
check("MOVING turn: clear 1.50 m -> gate.guard ALLOWS turn (sweep 0.24)",
      abs(ga - 0.6) < 1e-9 and gwhy is None, "l=%s a=%s why=%s" % (gl, ga, gwhy))

# --------------------------------------------------------------------------- summary
failed = [(n, ok) for n, ok in RES if not ok]
print("----")
print("%d passed, %d failed, %d total" % (len(RES) - len(failed), len(failed), len(RES)))
if failed:
    print("FAILED:")
    for n, _ in failed:
        print("  -", n)
    sys.exit(1)
sys.exit(0)
