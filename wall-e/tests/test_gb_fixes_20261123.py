"""Unit tests for the 2026-11-23 John-approved fixes 1, 2, 3, 6. Pure Python + numpy, no ROS.
Scans mimic the live sclidar /scan_raw: 400 rays, angle_min 0, inc 2*pi/400, range_min 0.10, range_max 10.0,
every invalid return = NaN; open-room NaN runs of 6-8 rays between returns 2.3-3.0 m (benign)."""
import ast, math, os, re, sys, types
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_obstacle as OB
N = 400; A0 = 0.0; INC = 2 * math.pi / N; RMIN = 0.10; RMAX = 10.0; NAN = float("nan")
RES = []
def check(name, cond, info=""):
    RES.append((name, bool(cond))); print(("PASS " if cond else "FAIL ") + name + ("  | " + info if info else ""))

def ang(i): return OB._wrap(A0 + i * INC)
def ray_to_box(a, walls):
    """range along bearing a to the nearest of axis-aligned walls: ('x', x0, ylo, yhi) or ('y', y0, xlo, xhi)."""
    best = float("inf"); c, s = math.cos(a), math.sin(a)
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
        a = ang(i); d = ray_to_box(a, list(walls) + list(base))
        r.append(d if RMIN <= d <= RMAX else NAN)
    for lo, hi in nan_deg:
        for i in range(N):
            if lo <= math.degrees(ang(i)) <= hi: r[i] = NAN
    return r
def open_room():   # live-like benign far NaN runs (7 rays at 23..30 deg, singles at -20, 1)
    return scan(nan_deg=[(23, 30), (-20.5, -19.5), (0.5, 1.5)])
def fusion(ranges, t=100.0, pose=(0.0, 0.0, 0.0)):
    f = OB.Fusion(); f.on_odom(t, 0.0, 0.0, *pose); f.imu_t = t
    D = OB.DepthView(t); D.fwd = float("inf"); D.valid_frac = 0.9; f.on_depth(D)      # fresh depth, nothing seen
    f.on_lidar(OB.lidar_analyze(ranges, A0, INC, RMIN, RMAX, t)); return f
def lim(f, lin, ang_, t=100.0): return f.limit(lin, ang_, t, 0.12)

# ---- thresholds untouched
check("thresholds unchanged 0.27/0.28/0.24", (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24),
      str((OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R)))
# ================= FIX 1: dropouts
r = open_room(); v = OB.lidar_analyze(r, A0, INC, RMIN, RMAX, 100.0)
f = fusion(r); l, a, why, kind = lim(f, 0.10, 0.0)
check("F1 open room with benign far NaN runs: forward NOT stopped", kind != "block" and kind != "blind" and l > 0,
      "lin=%.3f kind=%s why=%s imputed=%d fwd=%.2f" % (l, kind, why, v.imputed, v.fwd))
check("F1 open room: far-flanked NaN runs are not imputed", v.imputed == 0, "imputed=%d" % v.imputed)
# black wall 0.25 m ahead: centre +-20 deg NaN, edges return (flanks ~0.26 m)
r = scan(walls=[("x", 0.25, -9, 9)], nan_deg=[(-20, 20)]); f = fusion(r); l, a, why, kind = lim(f, 0.10, 0.0)
check("F1 dark wall 0.25 m ahead (centre NaN, near flanks): STOP", kind == "block" and l == 0.0, "why=%s" % why)
# same wall at 0.40 m: should not block but slow by ramp (lidar distance from imputed points)
r = scan(walls=[("x", 0.40, -9, 9)], nan_deg=[(-20, 20)]); f = fusion(r); l, a, why, kind = lim(f, 0.10, 0.0)
check("F1 dark wall 0.40 m ahead: not stopped, slowed", kind == "slow" and 0 < l < 0.10, "lin=%.3f why=%s" % (l, why))
# 60%+ of the front cone NaN with depth fresh & OK -> must STOP now (depth-OK bypass removed)
r = scan(nan_deg=[(-40, 40)])
f = fusion(r); D = OB.DepthView(100.0); D.fwd = float("inf"); D.valid_frac = 0.9; f.on_depth(D)
l, a, why, kind = lim(f, 0.10, 0.0)
check("F1 front blind >=60%% with depth OK: STOP (bypass removed)", l == 0.0 and kind in ("blind", "block"), "why=%s kind=%s" % (why, kind))
l, a, why, kind = lim(f, 0.0, 0.5)
check("F1 front blind >=60%% with depth OK: turn refused", l == 0.0 and a == 0.0, "why=%s" % why)
# approach memory: obstacle seen at 0.45 m, then same bearing goes NaN (far flanks) after travelling 0.22 m
r1 = scan(walls=[("x", 0.45, -0.08, 0.08)]); f = fusion(r1, 100.0, (0.0, 0.0, 0.0))
f.on_odom(100.5, 0.06, 0.0, 0.22, 0.0, 0.0)
r2 = scan(nan_deg=[(-10, 10)]); f.on_lidar(OB.lidar_analyze(r2, A0, INC, RMIN, RMAX, 100.5)); f.imu_t = 100.5
l, a, why, kind = lim(f, 0.08, 0.0, 100.5)
check("F1 obstacle vanishes into NaN up close (approach memory): STOP", kind == "block" and "memory" in (why or ""), "why=%s" % why)
f2 = fusion(r2, 100.0); l, a, why, kind = lim(f2, 0.08, 0.0)
check("F1 same NaN centre with no prior near obstacle: not stopped", kind not in ("block", "blind"), "kind=%s why=%s" % (kind, why))
# reverse: dark wall 0.25 m behind, centre NaN with near flanks
r = scan(walls=[("x", -0.25, -9, 9)], nan_deg=[(160, 180), (-180, -160)]); f = fusion(r); l, a, why, kind = lim(f, -0.06, 0.0)
check("F1 dark wall 0.25 m behind (rear NaN, near flanks): reverse STOP", kind == "block" and l == 0.0, "why=%s" % why)
# ================= FIX 2: arc sweep
r = open_room(); f = fusion(r); l, a, why, kind = lim(f, 0.08, 0.0)
check("F2 straight clear: allowed", l > 0 and kind not in ("block", "blind"), "lin=%.3f why=%s" % (l, why))
r = scan(walls=[("x", 0.24, -9, 9)]); f = fusion(r); l, a, why, kind = lim(f, 0.08, 0.0)
check("F2 straight blocked at 0.24 m: STOP", kind == "block" and l == 0.0, "why=%s" % why)
side = [("y", -0.30, -0.5, 1.5)]   # wall on the RIGHT, 0.30 m: outside the 0.15 m strip and the 0.24 m sweep
r = scan(walls=side); vv = OB.lidar_analyze(r, A0, INC, RMIN, RMAX, 100.0)
check("F2 setup: side wall outside straight strip and sweep", vv.fwd > 1.0 and vv.sweep >= 0.24, "fwd=%.2f sweep=%.2f" % (vv.fwd, vv.sweep))
f = fusion(r); l, a, why, kind = lim(f, 0.08, -0.45)
check("F2 arcing INTO side wall (right turn): capped by the arc (lidar-arc)", kind == "block" or (kind == "slow" and l < 0.08 and "lidar-arc" in why),
      "lin=%.3f ang=%.2f kind=%s why=%s" % (l, a, kind, why))
post = [("x", 0.18, -0.24, -0.17), ("y", -0.17, 0.18, 0.24)]     # post front-right: |y|>=0.18 (outside strip), range>=0.31
r = scan(walls=post); vp = OB.lidar_analyze(r, A0, INC, RMIN, RMAX, 100.0)
lp, ap, whyp, kindp = lim(fusion(r), 0.08, 0.0)
check("F2 setup: post outside straight strip, sweep >= 0.24 -> straight drive allowed", kindp != "block" and vp.sweep >= 0.24, "fwd=%.2f sweep=%.2f kind=%s" % (vp.fwd, vp.sweep, kindp))
lp, ap, whyp, kindp = lim(fusion(r), 0.08, -0.45)
check("F2 arcing INTO a post outside the strip: STOP", kindp == "block" and lp == 0.0, "why=%s" % whyp)
lp, ap, whyp, kindp = lim(fusion(r), 0.08, 0.45)
check("F2 arcing AWAY from that post: allowed", lp > 0 and kindp != "block", "lin=%.3f why=%s" % (lp, whyp))
d_into = OB.arc_clearance(vv.px, vv.py, 0.08, -0.45, 0.57)
l2, a2, why2, kind2 = lim(fusion(r), 0.08, 0.45)
d_away = OB.arc_clearance(vv.px, vv.py, 0.08, 0.45, 0.57)
check("F2 arcing AWAY from side wall (left turn): allowed", l2 > 0 and kind2 not in ("block", "blind"),
      "lin=%.3f kind=%s why=%s arc_into=%.2f arc_away=%s" % (l2, kind2, why2, d_into, d_away))
check("F2 straight arc == corridor distance", abs(OB.arc_clearance(*(lambda q: (q.px, q.py))(OB.lidar_analyze(scan(walls=[("x", 0.5, -9, 9)]), A0, INC, RMIN, RMAX, 1.0)), 0.08, 0.0, 0.57) - 0.5) < 0.011)
# reverse arc into a wall behind-right
r = scan(walls=[("y", -0.30, -1.5, 0.5)]); f = fusion(r); l, a, why, kind = lim(f, -0.08, 0.45)
check("F2 reverse arc swinging into side wall: stopped or capped by arc", kind == "block" or (kind == "slow" and abs(l) < 0.08 and "arc" in why), "lin=%.3f kind=%s why=%s" % (l, kind, why))
# ================= FIX 3: explore trip handling + honest startup message (static extraction, no ROS)
src = open(os.path.join(ROOT, "gb_explore.py")).read(); tree = ast.parse(src); ns = {}
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name == "trip_dirs":
        exec(compile(ast.Module([node], []), "trip_dirs", "exec"), ns)
td = ns["trip_dirs"]
check("F3 trip 'turn sweep' -> hold turn", td("obstacle 0.17m inside the turn sweep (r 0.24m)") == {"turn"})
check("F3 trip 'behind' -> hold reverse", td("obstacle 0.07m behind (stop 0.28m)") == {"rev"})
check("F3 trip 'track stall' -> hold forward", "fwd" in td("bump detected (encoders): track stall: commanded 0.04 m/s"))
check("F3 trip 'ahead' -> hold forward", td("obstacle 0.24m ahead (lidar, stop 0.27m)") == {"fwd"})
check("F3 explore: trip-hold in send() + reposition/nudge instead of TRIP_LIMIT abort",
      "TRIP-HOLD rev" in src and 'TRIP_LIMIT = int(E("EXPLORE_TRIP_LIMIT", "3"))' in src
      and "def reposition_step" in src and "truly stuck after" in src
      and "if len(tt) >= TRIP_LIMIT" not in src)
check("F3 explore asks gb_mapping for the autonomous message", "gb_mapping.start(autonomous=True)" in src)
# gb_mapping.start with stubs
stub = types.ModuleType("gb_store"); stub.DATA = "/tmp/gbtest_data"; stub.atomic_write = lambda p, t: open(p, "w").write(t)
class _S:
    def event(self, *a): pass
stub.Store = _S; sys.modules["gb_store"] = stub
os.environ["ROS_SHARED_HOST"] = "/tmp/gbtest_shared"
import gb_mapping as GM
class R:  # fake subprocess results
    def __init__(self, rc): self.returncode = rc; self.stdout = ""
GM.running = lambda: None; GM._dx = lambda args, t=20, user="1000": R(0)
GM.subprocess = types.SimpleNamespace(Popen=lambda *a, **k: None, DEVNULL=None, run=None)
import time as _time
GM.time = types.SimpleNamespace(sleep=lambda s: None, time=lambda: 0, strftime=_time.strftime)
ok, m_auto = GM.start(autonomous=True); ok2, m_man = GM.start()
check("F3 autonomous explore message does NOT say 'I won't drive myself'", ok and "won't drive myself" not in m_auto and "drive myself slowly" in m_auto, m_auto)
check("F3 manual mapping message unchanged", ok2 and "won't drive myself" in m_man, m_man)
# ================= FIX 6: chat motion honesty
vsrc = open(os.path.join(ROOT, "grokbot_voice.py")).read(); vt = ast.parse(vsrc); vns = {"re": re, "log": lambda *a: None}
for node in vt.body:
    if (isinstance(node, ast.Assign) and any(getattr(t, "id", "") in ("_MOTION_CLAIM", "MOTION_HONEST_REPLY") for t in node.targets)) or \
       (isinstance(node, ast.FunctionDef) and node.name == "honest_motion"):
        exec(compile(ast.Module([node], []), "hm", "exec"), vns)
hm = vns["honest_motion"]; H = vns["MOTION_HONEST_REPLY"]
for s_ in ["I'm driving slowly toward you now, watching the distance. Let me know if you want me to stop.",
           "I'm following you now, staying close and slow. Lead the way.",
           "I'm still following you, nice and slow. Keep leading.",
           "I'm heading back to the bench now. Staying close.",
           "On my way!", "I'll follow you.", "I'm coming to you."]:
    check("F6 chat motion claim replaced: %r" % s_[:45], hm(s_) == H)
for s_ in ["It's 3:05 in the afternoon.", "My battery is at 7.9 volts.", "I'm here and listening.",
           "Happy Thanksgiving, John!", "I'm not moving right now.", "The couch is to my left.",
           "I'm going to need a clearer view of that."]:
    check("F6 normal chat untouched: %r" % s_[:45], hm(s_) == s_)
check("F6 router prompt has MOTION HONESTY rule", "MOTION HONESTY (hard rule)" in vsrc)
check("F6 guard applied to chat + converse replies only", vsrc.count("= honest_motion(reply)") == 2)
n_fail = sum(1 for _n, ok in RES if not ok)
print("\nRESULT: %d passed, %d failed" % (len(RES) - n_fail, n_fail)); sys.exit(1 if n_fail else 0)
