#!/usr/bin/env python3
"""test_obstacle.py - obstacle fusion (gb_obstacle) + wiring into gate / explore / features / rgbguard, 2026-11-05.
Simulated LiDAR scans (ray-cast segments / circles), synthetic depth frames, encoder / IMU / RGB streams, a closed-loop
approach simulator with scan + tick + controller latency. No ROS, no motors.  Run: python3 tests/test_obstacle.py"""
import ast, math, os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_obstacle as OB

N_OK = N_BAD = 0
def check(name, cond, info=""):
    global N_OK, N_BAD
    if cond: N_OK += 1
    else: N_BAD += 1; print("FAIL:", name, info)
INF = float("inf")
# robot body in the LiDAR frame (base_footprint x -0.152..0.126, y +/-0.098, LiDAR at x=-0.002)
FRONT_EDGE, REAR_EDGE, HALF = 0.128, 0.150, 0.098

# ------------------------------------------------------------------ LiDAR simulator (400 beams, 0..2pi, like /scan_raw)
NB = 400; INC = 2 * math.pi / NB; RMIN, RMAX = 0.10, 10.0
def _ray_seg(ox, oy, dx, dy, s):
    x1, y1, x2, y2 = s; ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12: return INF
    t = ((x1 - ox) * ey - (y1 - oy) * ex) / den; u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    return t if t > 1e-6 and 0 <= u <= 1 else INF
def _ray_circ(ox, oy, dx, dy, c):
    cx, cy, r = c; fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy; cc = fx * fx + fy * fy - r * r; disc = b * b - cc
    if disc < 0: return INF
    t = -b - math.sqrt(disc)
    return t if t > 1e-6 else INF
ROOM = [(-4, -4, 4, -4), (4, -4, 4, 4), (4, 4, -4, 4), (-4, 4, -4, -4)]   # 8 x 8 m room walls (a real room always returns)
def scan(segs=(), circs=(), pose=(0, 0, 0), dark=(), near_blind=0.0, noise=0.0, rng=None, room=True):
    """pose of the LiDAR in world; dark = list of (a_lo, a_hi) robot-frame angle ranges that return inf (glass / black)."""
    px, py, pth = pose; out = []; segs = list(segs) + (ROOM if room else [])
    for i in range(NB):
        a = i * INC; wa = pth + a; dx, dy = math.cos(wa), math.sin(wa)
        t = min([_ray_seg(px, py, dx, dy, s) for s in segs] + [_ray_circ(px, py, dx, dy, c) for c in circs] + [INF])
        aw = math.atan2(math.sin(a), math.cos(a))
        if any(lo <= aw <= hi for lo, hi in dark): t = INF
        if t < near_blind: t = 0.0
        if noise and t < INF: t += rng.normal(0, noise)
        out.append(t if t <= RMAX else INF)
    return out
def LV(ranges, t=None): return OB.lidar_analyze(ranges, 0.0, INC, RMIN, RMAX, time.time() if t is None else t)
def wall(d, ang_deg=0.0, length=6.0):
    """Wall whose closest point is d ahead on the x axis, rotated by ang_deg (0 = square to the path)."""
    a = math.radians(ang_deg); ux, uy = -math.sin(a), math.cos(a)    # along-wall direction
    return (d - ux * length / 2, -uy * length / 2, d + ux * length / 2, uy * length / 2)

# ------------------------------------------------------------------ depth simulator (640x400 mono16 like depth0)
FX, FY, CX, CY = 421.4, 422.4, 317.6, 200.2; W, H = 640, 400
def depth_img(boxes=(), floor=True, zmax=5.0, noise=0.0, speckle=0.0, rng=None, drop_rows=None):
    """boxes: (x0, x1, y0, y1, z0, z1) in the LiDAR-centred robot frame (z from the floor). Returns uint16 mm."""
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    dx = np.ones_like(u); dy = -(u - CX) / FX; dz = -(v - CY) / FY          # per unit optical Z
    ox, oy, oz = OB.CAM_X, 0.0, OB.CAM_Z
    Z = np.full(u.shape, np.inf)
    if floor:
        tf = np.where(dz < -1e-9, -oz / np.where(dz < -1e-9, dz, -1), np.inf); Z = np.minimum(Z, tf)
    for x0, x1, y0, y1, z0, z1 in boxes:
        tmin = np.full(u.shape, -np.inf); tmax = np.full(u.shape, np.inf)
        for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1), (oz, dz, z0, z1)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t1 = (lo - o) / d; t2 = (hi - o) / d
            par = np.abs(d) < 1e-12
            t1 = np.where(par, np.where((o >= lo) & (o <= hi), -np.inf, np.inf), t1)
            t2 = np.where(par, np.where((o >= lo) & (o <= hi), np.inf, -np.inf), t2)
            tmin = np.maximum(tmin, np.minimum(t1, t2)); tmax = np.minimum(tmax, np.maximum(t1, t2))
        hit = (tmax >= tmin) & (tmin > 0)
        Z = np.where(hit, np.minimum(Z, tmin), Z)
    if noise: Z = Z * (1 + rng.normal(0, noise, Z.shape))
    Z = np.where(np.isfinite(Z) & (Z <= zmax) & (Z > 0), Z, 0)
    img = (Z * 1000).astype(np.uint16)
    if speckle:
        m = rng.random(img.shape) < speckle; img[m] = rng.integers(150, 1500, int(m.sum()))
    if drop_rows is not None: img[drop_rows[0]:drop_rows[1]] = 0
    return img
def DV(img, t=None):
    # Fixture images are built as millimetres (Z * 1000). Live frames do not get this default.
    return OB.depth_analyze(img, FX, FY, CX, CY, time.time() if t is None else t, scale_m=0.001)

def fusion(now, lidar=None, depth=None, odom=True, imu=True, rgb=None):
    f = OB.Fusion()
    f.on_lidar(lidar if lidar is not None else LV(scan(), now))
    if depth is not None: f.on_depth(depth)
    if odom: f.on_odom(now, 0.0, 0.0, 0.0, 0.0, 0.0)
    if imu: f.imu_t = now
    if rgb is not None: f.on_rgb(rgb)
    return f

def main():
    rng = np.random.default_rng(7); now = 1000.0
    empty_depth = DV(depth_img(), now)

    # ===== 1. speed ramp: values, monotonic, never above max, 0 below stop
    tab = [(d, OB.ramp(d, 0.12)) for d in np.arange(0.0, 3.01, 0.01)]
    check("ramp 0 below stop", all(v == 0 for d, v in tab if d < OB.STOP_FWD - 1e-9))
    check("ramp vmin at stop", abs(OB.ramp(OB.STOP_FWD, 0.12) - 0.04) < 1e-9)
    check("ramp full at SLOW_FULL", abs(OB.ramp(OB.SLOW_FULL, 0.12) - 0.12) < 1e-9 and OB.ramp(3.0, 0.12) == 0.12)
    check("ramp never above max", all(v <= 0.12 + 1e-12 for _d, v in tab))
    check("ramp monotonic", all(tab[i + 1][1] >= tab[i][1] for i in range(len(tab) - 1)))
    check("ramp mid", OB.ramp((OB.STOP_FWD+OB.SLOW_FULL)/2, 0.12) > 0.04)
    check("ramp lower vmax", OB.ramp(2.0, 0.08) == 0.08 and OB.ramp(0.35, 0.03) == 0.03)
    check("ramp NaN/inf", OB.ramp(float("nan"), 0.12) == 0 and OB.ramp(INF, 0.12) == 0.12)
    print("ramp table (m -> m/s):", " ".join("%.2f:%.3f" % (d, OB.ramp(d, 0.12)) for d in (0.2, 0.27, 0.35, 0.5, 0.7, 1.0, 2.0)))

    # ===== 2. LiDAR geometry: wall ahead at many distances, corridor vs cone
    for d in (0.20, 0.26, 0.28, 0.5, 0.8, 1.2, 3.0):
        v = LV(scan([wall(d)]), now)
        check("wall %.2f fwd" % d, abs(v.fwd - d) < 0.01, v.fwd)
        f = fusion(now, v, empty_depth)
        l, a, why, kind = f.limit(0.12, 0.0, now)
        if d < 0.27: check("wall %.2f blocks" % d, l == 0 and kind == "block", (l, kind))
        else: check("wall %.2f ramps" % d, abs(l - OB.ramp(d, 0.12)) < 0.003, (l, OB.ramp(d, 0.12)))
    # object beside the front of the track (old +/-30 deg cone missed it): post at x=0.20, y=0.13
    v = LV(scan(circs=[(0.18, 0.14, 0.015)]), now)
    old_cone = min(r for i, r in enumerate(scan(circs=[(0.18, 0.14, 0.015)])) if abs(math.atan2(math.sin(i * INC), math.cos(i * INC))) <= math.radians(30))
    check("side post outside old cone", old_cone > 3.0, old_cone)
    check("side post in corridor", v.fwd < 0.2, v.fwd)
    check("side post blocks", fusion(now, v, empty_depth).limit(0.12, 0, now)[3] == "block")
    # object just outside the corridor (y=0.20) -> no block, no slow
    v = LV(scan(circs=[(0.30, 0.21, 0.02)]), now)
    check("outside corridor ignored", v.fwd > 3.0 and fusion(now, v, empty_depth).limit(0.12, 0, now)[0] == 0.12, v.fwd)
    # chair legs (2 cm) at 0.3 and 0.6 m inside the corridor
    for x in (0.3, 0.6):
        v = LV(scan(circs=[(x, 0.05, 0.012), (x, -0.30, 0.012)]), now)
        check("chair leg %.1f seen" % x, abs(v.fwd - (x - 0.012)) < 0.02, v.fwd)
    check("chair leg inside stop blocks", fusion(now, LV(scan(circs=[(0.22, 0.05, 0.012)]), now), empty_depth).limit(0.12, 0, now)[3] == "block")
    check("chair leg 0.6 slows", 0.04 < fusion(now, LV(scan(circs=[(0.6, 0.05, 0.012)]), now), empty_depth).limit(0.12, 0, now)[0] < 0.12)
    # angular resolution check: a 1.2 cm leg at 1.0 m spans 0.012 rad < inc 0.0157 -> can fall between beams far away
    hits = sum(LV(scan(circs=[(1.0, y, 0.006)]), now).fwd < 2 for y in np.linspace(-0.1, 0.1, 21))
    print("thin 12 mm pole at 1.0 m: seen in %d/21 lateral positions (beam spacing %.1f cm at 1 m)" % (hits, 100 * INC))
    hits05 = sum(LV(scan(circs=[(0.5, y, 0.006)]), now).fwd < 2 for y in np.linspace(-0.1, 0.1, 21))
    check("12 mm pole at 0.5 m always seen", hits05 == 21, hits05)

    # ===== 3. invalid returns: <rmin, >rmax, 0, NaN, inf; glass / black dropouts
    r = scan([wall(1.0)])
    for bad in (0.0, 0.05, float("nan"), INF, 11.0, -1.0):
        rr = list(r); rr[0:3] = [bad] * 3; v = LV(rr, now)
        check("invalid %r ignored" % bad, abs(v.fwd - 1.0) < 0.01, v.fwd)
    v = LV(scan([wall(1.0)], dark=[(-0.6, 0.6)]), now)      # glass wall ahead: whole +/-34 deg returns nothing
    check("glass blind frac", v.blind_f > 0.6, v.blind_f)
    l, a, why, kind = fusion(now, v, None).limit(0.12, 0, now)
    check("glass + no depth -> stop", l == 0 and kind == "blind", (l, kind))
    l, a, why, kind = fusion(now, v, empty_depth).limit(0.12, 0, now)
    check("glass + depth ok -> creep", 0 < l <= OB.V_BLIND + 1e-9 and kind == "slow", (l, kind))
    v = LV(scan([wall(1.0)], dark=[(-0.08, 0.0)]), now)      # small black patch dead ahead (5 beams)
    check("black patch run", v.blind_run >= 4, v.blind_run)
    check("black patch -> creep", fusion(now, v, empty_depth).limit(0.12, 0, now)[0] <= OB.V_BLIND + 1e-9)
    v = LV(scan([wall(0.2)], near_blind=0.25), now)          # LiDAR returns 0 for anything closer than 25 cm
    l, a, why, kind = fusion(now, v, None).limit(0.12, 0, now)
    check("too-close wall (zeros) -> stop", l == 0, (l, kind, v.blind_f))
    check("8 random inf beams harmless", fusion(now, LV([INF if i in (5, 70, 133, 190, 250, 300, 333, 390) else x for i, x in enumerate(scan([wall(2.0)]))], now), empty_depth).limit(0.12, 0, now)[0] > 0.11)
    v = LV(scan([wall(0.6)], noise=0.01, rng=rng), now)
    check("1 cm noise", abs(v.fwd - 0.6) < 0.04, v.fwd)

    # ===== 4. turning: sweep radius vs footprint, corners, dead ends, rear
    corner_r = math.hypot(REAR_EDGE, HALF)
    check("sweep covers footprint", OB.SWEEP_R >= corner_r + 0.02, (OB.SWEEP_R, corner_r))
    for d, ok in ((0.20, False), (0.22, False), (0.25, True), (0.30, True), (0.40, True)):  # SWEEP_R 0.24
        v = LV(scan(circs=[(0.0, d + 0.01, 0.01)]), now)      # post beside the robot
        l, a, why, kind = fusion(now, v, empty_depth).limit(0.0, 0.6, now)
        check("turn post at %.2f %s" % (d, "ok" if ok else "blocked"), (a == 0.6) == ok, (v.sweep, a, kind))
    # U dead-end 0.5 wide, 0.45 deep: forward blocked, turn allowed (sweep), reverse allowed
    U = [(0.45, -0.35, 0.45, 0.35), (-0.5, 0.35, 0.45, 0.35), (-0.5, -0.35, 0.45, -0.35)]  # sides 0.35 > SWEEP 0.26
    f = fusion(now, LV(scan(U), now), empty_depth)
    check("U fwd slow", 0 < f.limit(0.12, 0, now)[0] <= 0.09)
    check("U turn ok", f.limit(0, 0.6, now)[1] == 0.6)
    check("U reverse ok", f.limit(-0.1, 0, now)[0] == -0.1)
    U2 = [(0.22, -0.21, 0.22, 0.21), (-0.5, 0.21, 0.22, 0.21), (-0.5, -0.21, 0.22, -0.21)]   # tight: front inside the 0.27 stop
    f = fusion(now, LV(scan(U2), now), empty_depth)
    check("tight U fwd blocked", f.limit(0.12, 0, now)[3] == "block")
    check("tight U turn blocked", f.limit(0, 0.6, now)[1] == 0)
    check("tight U reverse ok", f.limit(-0.1, 0, now)[0] < 0)
    v = LV(scan([(-0.22, -1, -0.22, 1)]), now)               # wall inside the rear stop
    check("rear wall blocks reverse", fusion(now, v, empty_depth).limit(-0.1, 0, now)[3] == "block")
    v = LV(scan([(-0.60, -1, -0.60, 1)]), now)
    l = fusion(now, v, empty_depth).limit(-0.12, 0, now)[0]
    check("rear wall 0.6 reverse ramps", -0.12 < l <= -0.04, l)
    # inside corner (two walls, robot heading into the corner at 45 deg)
    C = [(0.45, -2, 0.45, 2)]; C2 = [(0.45, -2, 0.45, 2), (-2, 0.40, 2, 0.40)]
    check("corner fwd slows", 0 < fusion(now, LV(scan(C2), now), empty_depth).limit(0.12, 0, now)[0] < 0.10)

    # ===== 5. closed loop approach at angles: max speed, scan 10 Hz, tick 20 Hz, 0.15 s controller lag, 0.007 m brake
    def approach(seg, v_cmd=0.12, ramped=True, lag=0.15, dt=0.01, tmax=60.0):
        f = OB.Fusion(); x = 0.0; t = 0.0; last_scan = -1; vel = 0.0; queue = []; cmd = 0.0; tick = -1; stopped_at = None
        while t < tmax:
            if t - last_scan >= 0.1 - 1e-9:
                last_scan = t; v = LV(scan([seg], pose=(x, 0, 0)), t); f.on_lidar(v)
                f.on_depth(OB.DepthView(t)); f.depth.valid_frac = 0.5
                f.on_odom(t, vel, 0.0, x, 0.0, 0.0); f.imu_t = t
            if t - tick >= 0.05 - 1e-9:
                tick = t; f.odom_t = t; f.imu_t = t
                l, a, why, kind = f.limit(v_cmd, 0.0, t, 0.12)
                if not ramped and kind == "slow": l = v_cmd
                cmd = l; queue.append((t + lag, cmd))
                if kind == "block" and stopped_at is None: stopped_at = t
            while queue and queue[0][0] <= t: vel = queue.pop(0)[1]
            x += vel * dt; t += dt
            if stopped_at is not None and vel == 0: break
        x += 0.007 if vel else 0.0
        # bumper gap: distance from the body rectangle to the segment
        bx = [(x + FRONT_EDGE, y) for y in np.linspace(-HALF, HALF, 21)] + [(x + s, HALF) for s in np.linspace(-REAR_EDGE, FRONT_EDGE, 21)] + [(x + s, -HALF) for s in np.linspace(-REAR_EDGE, FRONT_EDGE, 21)]
        x1, y1, x2, y2 = seg
        def dseg(px, py):
            ex, ey = x2 - x1, y2 - y1; k = max(0, min(1, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
            return math.hypot(px - (x1 + k * ex), py - (y1 + k * ey))
        return min(dseg(*p) for p in bx), t
    gaps = {}
    for ang in (0, 10, 30, 45, 60, 75):
        g, tt = approach(wall(1.5, ang)); gaps[ang] = g; check("approach %d deg stopped (not timed out)" % ang, tt < 59, tt)
        check("approach %d deg stops clear" % ang, g > 0.08, g)
    g0, _ = approach(wall(1.5, 0), ramped=False)
    print("closed-loop bumper gap after stop (m), ramped:", {k: round(v, 3) for k, v in gaps.items()}, " unramped 0 deg: %.3f" % g0)
    check("ramp improves stop precision", gaps[0] >= g0 - 0.005)
    g, _ = approach(wall(1.5, 0), lag=0.4)
    check("0.4 s lag still clear", g > 0.08, g)
    # stopping distance analytics at full speed
    react = 0.1 + 0.05 + 0.15; sd_max = 0.12 * react + 0.007; sd_ramp = OB.V_MIN * react + 0.003
    check("unramped stop distance < margin", sd_max < OB.STOP_FWD - FRONT_EDGE, sd_max)
    print("stopping distance: unramped 0.12 m/s %.3f m, ramped (0.04 m/s at stop line) %.3f m; margin at stop line %.3f m"
          % (sd_max, sd_ramp, OB.STOP_FWD - FRONT_EDGE))

    # ===== 6. depth camera alone (below / above the LiDAR plane)
    low = (0.50, 0.70, -0.10, 0.10, 0.0, 0.10)               # 10 cm high box, 0.5 m ahead: LiDAR (18.4 cm) can't see it
    d = DV(depth_img([low]), now)
    check("depth sees low box", abs(d.fwd - 0.50) < 0.03, d.fwd)
    check("lidar misses low box", LV(scan(), now).fwd > 3.0)
    f = fusion(now, LV(scan(), now), d)
    check("low box 0.5 -> slow", 0.04 <= f.limit(0.12, 0, now)[0] <= 0.10, f.limit(0.12, 0, now))
    d = DV(depth_img([(0.18, 0.36, -0.10, 0.10, 0.0, 0.10)]), now)
    check("low box 0.3 -> block via depth", fusion(now, LV(scan(), now), d).limit(0.12, 0, now)[3] == "block", d.fwd)
    over = (0.40, 0.80, -0.3, 0.3, 0.22, 0.40)                 # table edge / shelf at 22-40 cm (above the LiDAR plane)
    d = DV(depth_img([over]), now)
    check("depth sees overhang", abs(d.fwd - 0.40) < 0.03, d.fwd)
    check("floor alone not an obstacle", empty_depth.fwd == INF, empty_depth.fwd)
    check("floor noise 1% not an obstacle", DV(depth_img(noise=0.01, rng=rng), now).fwd == INF)
    check("speckle 0.5% not an obstacle", DV(depth_img(speckle=0.005, rng=rng), now).fwd == INF)
    check("speckle 2% not an obstacle", DV(depth_img(speckle=0.02, rng=rng), now).fwd == INF)
    seen = DV(depth_img([low], speckle=0.02, rng=rng), now).fwd
    check("low box + speckle still seen", 0.30 < seen < 0.80, seen)
    side = (0.40, 0.60, 0.20, 0.30, 0.0, 0.10)               # outside the corridor
    check("depth side box ignored", DV(depth_img([side]), now).fwd == INF)
    tall = (0.80, 1.0, -0.1, 0.1, 0.0, 1.0)
    check("depth wall at 0.8", abs(DV(depth_img([tall]), now).fwd - 0.80) < 0.03)
    # memory: low box seen at 0.5 m, robot drives 0.25 m, box leaves the FOV -> memory still blocks
    f = fusion(now, LV(scan(), now)); f.on_odom(now, 0.0, 0, 0.0, 0.0, 0.0)
    f.on_depth(DV(depth_img([low]), now))
    t2 = now + 2.0; f.on_odom(t2, 0.1, 0, 0.25, 0.0, 0.0); f.on_lidar(LV(scan(), t2)); f.imu_t = t2
    f.on_depth(DV(depth_img([], floor=True, drop_rows=(200, 400)), t2))      # box gone below the view, lower rows empty
    check("memory carries low box", abs(f.mem_dist(t2) - 0.25) < 0.02, f.mem_dist(t2))
    check("memory blocks", f.limit(0.12, 0, t2)[3] == "block")
    f2 = fusion(now, LV(scan(), now)); f2.on_odom(now, 0, 0, 0, 0, 0); f2.on_depth(DV(depth_img([tall]), now))
    f2.on_depth(DV(depth_img(), now + 0.2))
    check("memory cleared when depth sees it gone", f2.mem is None)
    f.on_odom(t2 + 0.1, 0, 0.5, 0.25, 0.0, 0.5)
    check("memory dropped after turning", f.mem_dist(t2 + 0.1) == INF)
    check("memory expires", fusion(now, LV(scan(), now)).mem_dist(now + 10) == INF)

    # ===== 7. encoders alone (stall / bump) and LiDAR + encoders (slip)
    st = OB.Stall(); ev = None
    for i in range(40): ev = ev or st.update(i * 0.05, 0.10, 0, 0.10 if i < 20 else 0.005, 0)
    check("stall detected after hit", ev and "track stall" in ev, ev)
    st = OB.Stall(); ev = None
    for i in range(14): ev = ev or st.update(i * 0.05, 0.10, 0, 0.0, 0)     # start: 0.7 s < grace
    check("no stall inside grace", ev is None)
    st = OB.Stall(); ev = None
    for i in range(60): ev = ev or st.update(i * 0.05, 0.10, 0, 0.09, 0)
    check("no stall when moving", ev is None)
    st = OB.Stall(); ev = None
    for i in range(60): ev = ev or st.update(i * 0.05, 0.0, 0.8, 0.0, 0.02)
    check("turn stall", ev and "turn stall" in ev, ev)
    sl = OB.Slip(); ev = None
    for i in range(30): ev = ev or sl.update(i * 0.05, True, i * 0.05 * 0.1, 0.8)   # encoders move, wall distance fixed
    check("slip detected", ev and "no progress" in ev, ev)
    sl = OB.Slip(); ev = None
    for i in range(30): ev = ev or sl.update(i * 0.05, True, i * 0.005, 0.8 - i * 0.005)
    check("no slip when closing", ev is None)
    # fusion event plumbing: stall -> event -> pop within 0.5 s, stale events dropped
    f = fusion(now, LV(scan([wall(1.0)]), now), empty_depth); f.on_cmd(now, 0.1, 0)
    for i in range(30): f.on_odom(now + i * 0.05, 0.0, 0.0, 0, 0, 0)
    check("fusion stall event", f.pop_event(now + 29 * 0.05) is not None)
    f.events = [(now, "imu", "old")]
    check("stale event dropped", f.pop_event(now + 2.0) is None)

    # ===== 8. IMU alone (jolt / tilt)
    im = OB.Imu(); ev = None
    for i in range(48): ev = ev or im.update(i / 48, -0.37, -0.39, 10.08, True)
    check("IMU rest quiet", ev is None)
    for i in range(3): ev = ev or im.update(1 + i / 48, -9.0, -0.39, 10.0, True)  # above OBS_JOLT 7
    check("IMU jolt", ev and "jolt" in ev, ev)
    im = OB.Imu(); ev = None
    for i in range(48): ev = ev or im.update(i / 48, -0.37 + rng.normal(0, 0.6), -0.39 + rng.normal(0, 0.6), 10.08, True)
    check("IMU vibration 0.6 sd quiet", ev is None, ev)
    im = OB.Imu(); ev = None
    for i in range(3): ev = ev or im.update(i / 48, -5.5, 0, 10, False)
    check("IMU jolt ignored at rest", ev is None)
    im = OB.Imu(); ev = None; g = 9.81
    for i in range(30): ev = ev or im.update(i / 48, g * math.sin(math.radians(20)), 0, g * math.cos(math.radians(20)), True)
    check("IMU tilt 20 deg", ev and "tilt" in ev, ev)
    im = OB.Imu(); ev = None
    for i in range(30): ev = ev or im.update(i / 48, g * math.sin(math.radians(8)), 0, g * math.cos(math.radians(8)), True)
    check("IMU 8 deg ramp ok", ev is None)

    # ===== 9. RGB advisory (slow only)
    f = fusion(now, LV(scan(), now), empty_depth, rgb={"t": now, "hazard": True, "label": "chair"})
    l, a, why, kind = f.limit(0.12, 0, now)
    check("RGB hazard slows", abs(l - OB.V_RGB) < 1e-9 and "chair" in why, (l, why))
    f = fusion(now, LV(scan(), now), empty_depth, rgb={"t": now - 5, "hazard": True})
    check("stale RGB ignored", f.limit(0.12, 0, now)[0] == 0.12)
    f = fusion(now, LV(scan([wall(0.20)]), now), empty_depth, rgb={"t": now, "hazard": False})
    check("RGB clear never overrides LiDAR", f.limit(0.12, 0, now)[3] == "block")
    src = open(os.path.join(ROOT, "gb_safety", "gb_rgbguard.py")).read(); tree = ast.parse(src); ns = {}
    keep = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name == "hazard_from") or
            (isinstance(n, ast.Assign) and any(getattr(t, "id", "") in ("HAZ", "CONF") for t in n.targets))]
    keep = [n for n in tree.body if isinstance(n, ast.Assign) and any(getattr(t, "id", "") == "CONF" for t in n.targets)] or keep
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.Assign) and
          any(isinstance(t, ast.Tuple) and any(getattr(e, "id", "") == "CONF" for e in t.elts) or getattr(t, "id", "") in ("HAZ", "CONF") for t in n.targets)]
          + [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "hazard_from"], type_ignores=[]), "rg", "exec"), ns)
    hz = ns["hazard_from"]
    check("rgb hazard chair centre low", hz([("chair", 0.8, 250, 200, 400, 390)], 640, 400) is not None)
    check("rgb ignore far-side", hz([("chair", 0.8, 0, 200, 120, 390)], 640, 400) is None)
    check("rgb ignore tiny/high", hz([("person", 0.9, 300, 20, 330, 120)], 640, 400) is None)
    check("rgb ignore low conf / class", hz([("chair", 0.2, 250, 200, 400, 390), ("clock", 0.9, 250, 200, 400, 390)], 640, 400) is None)

    # ===== 10. staleness / dropouts / disagreement (conservative)
    f = fusion(now, LV(scan(), now - 0.6), None)  # stale lidar AND no depth -> stop
    check("stale LiDAR + no depth stops", f.limit(0.12, 0, now)[3] == "stale")
    f = fusion(now, LV(scan(), now - 0.6), empty_depth)  # camera-first: depth alone allows creep
    check("stale LiDAR + depth creeps", 0 < f.limit(0.12, 0, now)[0] <= 0.06 + 1e-9)
    f = fusion(now, None, None); f.lidar = None
    check("missing LiDAR+depth stops", f.limit(0.1, 0, now)[3] == "stale")
    f = fusion(now, LV(scan(), now), empty_depth, odom=False)
    check("stale encoders stop motion", f.limit(0.12, 0, now)[3] == "stale" and f.limit(0, 0, now)[3] is None)
    f = fusion(now, LV(scan(), now), DV(depth_img(), now - 2))
    check("stale depth caps 0.06", abs(f.limit(0.12, 0, now)[0] - OB.V_DEGRADED) < 1e-9)
    f = fusion(now, LV(scan(), now), None)
    check("missing depth caps 0.06", abs(f.limit(0.12, 0, now)[0] - OB.V_DEGRADED) < 1e-9)
    f = fusion(now, LV(scan(), now), empty_depth, imu=False)
    check("stale IMU caps 0.06", abs(f.limit(0.12, 0, now)[0] - OB.V_DEGRADED) < 1e-9)
    f = fusion(now, LV(scan(), now), DV(depth_img([low]), now))
    check("disagree: depth near wins", f.front(now)[1] == "depth")
    f = fusion(now, LV(scan([wall(0.4)]), now), DV(depth_img([tall]), now))
    check("disagree: lidar near wins", f.front(now) [1] == "lidar" and abs(f.front(now)[0] - 0.4) < 0.01)
    f = fusion(now, LV(scan(), now), DV(depth_img([(0.30, 0.50, -0.10, 0.10, 0.0, 0.10)]), now - 1.5))
    check("stale depth obstacle not trusted but caps", 0 < f.limit(0.12, 0, now)[0] <= OB.V_DEGRADED)
    check("never above command", all(fusion(now, LV(scan([wall(dd)]), now), empty_depth).limit(c, 0, now)[0] <= c
                                       for dd in (0.4, 0.8, 2) for c in (0.02, 0.05, 0.12)))
    check("ang untouched when clear", fusion(now, LV(scan([wall(2)]), now), empty_depth).limit(0.1, 0.5, now)[1] == 0.5)

    # ===== 11. performance (this machine; the Pi is reported separately)
    r = scan([wall(1.0)], circs=[(0.5, 0.3, 0.05)]); t0 = time.perf_counter()
    for _ in range(200): LV(r, now)
    lid_ms = (time.perf_counter() - t0) / 200 * 1000
    img = depth_img([low]); t0 = time.perf_counter()
    for _ in range(100): DV(img, now)
    dep_ms = (time.perf_counter() - t0) / 100 * 1000
    f = fusion(now, LV(r, now), DV(img, now)); t0 = time.perf_counter()
    for _ in range(2000): f.limit(0.12, 0.3, now)
    lim_us = (time.perf_counter() - t0) / 2000 * 1e6
    print("perf: lidar_analyze %.2f ms/scan, depth_analyze %.2f ms/frame, limit %.0f us/tick" % (lid_ms, dep_ms, lim_us))
    check("lidar analyze < 5 ms", lid_ms < 5, lid_ms); check("depth analyze < 5 ms", dep_ms < 5, dep_ms)
    check("limit < 200 us", lim_us < 200, lim_us)

    # ===== 12. wiring: every motion path goes through the fusion
    gate = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
    check("gate imports OB", "import gb_obstacle as OB" in gate)
    check("gate guard uses fusion", "self.fz.limit(" in gate)
    check("gate collision in 3 modes", gate.count('if self.collision(') == 3, gate.count('if self.collision('))
    check("gate timed sends ramped", "self.send(_l, _a)" in gate and "self.send(self.lin, self.ang)" not in gate)
    check("gate depth+imu+encoders subs", all(k in gate for k in ("self.on_depth", "self.on_imu", "self.fz.on_odom(", "self.fz.on_cmd(")))
    check("gate no old cone", "0.5236" not in gate)
    ex = open(os.path.join(ROOT, "gb_explore.py")).read()
    check("explore imports OB", "import gb_obstacle as OB" in ex)
    check("explore send ramps", "OB.ramp(dd, MAX_LIN)" in ex and "CORRIDOR-BLOCK" in ex)
    check("explore avoid uses corridor + gate front", "depth_fwd" in ex and "corr" in ex)
    check("explore avoid 0.45", 'EXPLORE_AVOID_DIST", "0.45"' in ex)
    feat = open(os.path.join(ROOT, "gb_features.py")).read()
    check("stock driving apps off by voice", 'ALLOW_STOCK_DRIVE' in feat)
    check("features.drive via gate", 'features.drive' in feat)
    run = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read()
    check("run copies gb_obstacle", 'gb_obstacle.py' in run and 'nice -n 12' in run)
    srcm = os.path.join(ROOT, "gb_safety", "src")
    for fn in ("gb_obstacle.py", "gb_explore.py", "gb_features.py"):
        p = os.path.join(srcm, fn)
        if os.path.exists(p): check("mirror " + fn, open(p).read() == open(os.path.join(ROOT, fn)).read())
    print("test_obstacle: %d passed, %d failed" % (N_OK, N_BAD))
    return N_BAD == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
