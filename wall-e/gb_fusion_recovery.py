"""gb_fusion_recovery.py - shared LiDAR + camera FUSION STUCK-RECOVERY for every Wall-E mission (2026-11-27, John).

Why: between chair / table legs the normal LiDAR stop rules (gb_obstacle / gb_gate: 0.27 m front, 0.28 m rear,
0.24 m turn sweep, 0.15 m corridor half width) can report "too close" although a real gap wider than the chassis
exists (thin legs in the 5 cm side margin, legs inside the +/-25 deg cone but off the path, legs inside the 0.24 m
sweep circle but outside the 0.18 m body corner radius). This module lets a stuck mission (explore, wander, distance /
goto / waypoints, return-home / retrace, Nav2 relay) make SLOW, camera-verified micro-nudges through such a gap and then
hand control back to the mission. It never changes the normal stops: they stay 100% in force for every other command.

Shared by gb_explore.py (planner side, every mission type) and gb_gate.py (independent re-verification of every
fusion micro-nudge command). Pure Python + optional numpy. No ROS, never publishes, no motion by itself.

NUDGE vs REAL - a fusion micro-nudge for a candidate command (lin, ang) is allowed ONLY when ALL rules pass:
  C1  depth camera fresh (<= CAM_FRESH_S) and valid (overall valid fraction >= CAM_MIN_VALID)        else LIDAR_ONLY
  C2  the camera bins covering the candidate's path are valid (>= CAM_MIN_BIN_VALID), show floor
      (>= CAM_MIN_FLOOR samples) and no drop-off (< CAM_DROP_PTS below-floor samples)                 else REAL
  C3  camera obstacle points (0.05-0.40 m high) leave the candidate's chassis path clear to CAM_OPEN_M else REAL
  C4  RGB YOLO (if fresh) does not report a solid / animate hazard (person, pet, couch, bed, ...)
      - legged furniture (chair, table, stool) is exactly the case this module exists for             else REAL
  L1  LiDAR fresh, scan not empty, front dropouts < BLIND_FRAC_SLOW, rear dropouts < BLIND_FRAC_STOP else REAL
  L2  every LiDAR point that trips the NORMAL stop for this command belongs to a NARROW cluster
      (<= NARROW_MAX_W wide, i.e. a leg), and there are <= MAX_NEAR_CLUSTERS clusters nearby
      - any wide / dense cluster in the normal stop zone is a solid barrier                           else REAL
  L3  the PHYSICAL chassis corridor (body half width 0.10 + PHYS_MARGIN 0.02) along the candidate arc is clear
      for PASSABLE_M from the LiDAR centre (a real gap wider than chassis + margin), which also enforces the
      hard close-range floor FLOOR_FWD; any turning command needs every point >= FLOOR_SWEEP (body corner radius
      0.184 m + 3.6 cm)                                                                               else REAL
  L4  LiDAR / depth approach memories >= FLOOR_FWD + 0.05 and nothing touching the body envelope    else REAL
  K   |lin| in [0.06, 0.07] m/s (strictly above STALL_MIN_CMD 0.05) forward or pivot only (lin >= 0; reverse is
      NEVER relaxed - no rear camera), |ang| <= 0.30 rad/s
Encoder stall / bump / no-progress trips, ESTOP, disarm, abort, deadman, and FAIL_LIMIT consecutive no-progress
cycles end or blacklist a fusion escape immediately; the gate additionally voids any token older than its last trip.
"""
import json, math, os, time

E = os.environ.get
try:
    import gb_obstacle as _OB
except Exception:              # pragma: no cover - only when imported stand-alone without gb_obstacle on the path
    _OB = None


def _ob(name, default):
    return float(getattr(_OB, name, default)) if _OB is not None else float(default)


def _f(k, d):
    try:
        return float(E(k, d))
    except (TypeError, ValueError):
        return float(d)


INF = float("inf")
# ---- body / normal rules (READ ONLY mirrors of gb_obstacle; never modified here)
BODY_FRONT = _ob("BODY_FRONT", 0.13)
BODY_REAR = _ob("BODY_REAR", 0.155)
BODY_HALF_W = _ob("BODY_HALF_W", 0.10)
HALF_W = _ob("HALF_W", 0.15)                       # normal corridor half width (body + 5 cm side margin)
NORMAL_FRONT_STOP = _ob("STOP_FWD", 0.27)
NORMAL_REAR_STOP = _ob("STOP_REV", 0.28)
NORMAL_SWEEP = _ob("SWEEP_R", 0.24)
STALL_MIN_CMD = _ob("STALL_MIN_CMD", 0.05)
BLIND_FRAC_SLOW = _ob("BLIND_FRAC_SLOW", 0.25)
BLIND_FRAC_STOP = _ob("BLIND_FRAC_STOP", 0.6)
ENV_CONTACT = _ob("ENV_CONTACT", 0.02)
DEPTH_HMIN = _ob("DEPTH_HMIN", 0.05)
DEPTH_HMAX = _ob("DEPTH_HMAX", 0.40)
DEPTH_MIN_VALID_M = _ob("DEPTH_MIN_VALID_M", 0.30)
DEPTH_MAX_VALID_M = _ob("DEPTH_MAX_VALID_M", 3.00)
CAM_X = _ob("CAM_X", 0.092)
CAM_Z = _ob("CAM_Z", 0.127)
CONE = math.radians(25.0)                          # gb_explore's own front cone (+/-25 deg)
CORNER_R = math.hypot(BODY_REAR, BODY_HALF_W)      # 0.1845 m: widest point of the footprint when pivoting

# ---- fusion micro-nudge limits (bounded escape only)
NUDGE_LIN = 0.065
NUDGE_LIN_MIN = 0.06
NUDGE_LIN_MAX = 0.07
VEER_ANG = 0.10                                    # gentle re-centring veer (lateral correction into the gap)
ARC_ANG = 0.20
HARD_ARC_ANG = 0.30
PIVOT_ANG = 0.30
ANG_MAX = 0.30
PHYS_MARGIN = 0.025                                 # tight-squeeze: 5 cm side margin cut in half
PHYS_HALF_W = BODY_HALF_W + PHYS_MARGIN            # 0.125 m physical chassis corridor half width
# hard close-range floor: never closer than ~7 cm to the bumper, never below 5 cm, never at/above the normal stop
FLOOR_FWD = min(NORMAL_FRONT_STOP - 0.02, max(BODY_FRONT + 0.05, _f("GB_FR_FLOOR_FWD", 0.20)))
FLOOR_SWEEP = min(NORMAL_SWEEP - 0.01, max(CORNER_R + 0.025, _f("GB_FR_FLOOR_SWEEP", 0.22)))
PASSABLE_M = max(FLOOR_FWD + 0.10, _f("GB_FR_PASSABLE_M", 0.40))   # physical gap must extend this far (centre frame)
NARROW_MAX_W = _f("GB_FR_NARROW_MAX_W", 0.07)      # leg-like cluster width (m)
CLUSTER_GAP = 0.04                                 # neighbouring LiDAR points closer than this belong to one cluster
NEAR_R = 0.60                                      # clusters analysed within this range
MAX_NEAR_CLUSTERS = int(_f("GB_FR_MAX_NEAR_CLUSTERS", 8))
LIDAR_FRESH_S = 0.5
# camera
CAM_FRESH_S = _f("GB_FR_CAM_FRESH_S", 0.75)        # stricter than gb_obstacle DEPTH_STALE (1.0 s)
CAM_MIN_VALID = _f("GB_FR_CAM_MIN_VALID", 0.35)
CAM_MIN_BIN_VALID = _f("GB_FR_CAM_MIN_BIN_VALID", 0.30)
CAM_MIN_FLOOR = int(_f("GB_FR_CAM_MIN_FLOOR", 4))
CAM_DROP_PTS = int(_f("GB_FR_CAM_DROP_PTS", 6))
CAM_OPEN_M = _f("GB_FR_CAM_OPEN_M", 0.45)
CAM_BIN_DEG = 6.0
CAM_FOV_PAD = math.radians(2.0)
CAM_ARC_MAX_TURN = 0.6                             # rad of heading change an arc may make inside the camera check
RGB_FRESH_S = 2.5
RGB_SOLID = {"person", "dog", "cat", "couch", "sofa", "bed", "refrigerator", "suitcase", "backpack", "tv",
             "oven", "toilet", "sink", "bird", "teddy bear"}
RGB_LEGGED = {"chair", "dining table", "table", "bench", "stool", "potted plant"}
# escape state machine
SLICE_S = 0.4
PIVOT_SLICE_S = 0.2                                # pivots re-assessed sooner (fine alignment into a tight gap)
CYCLE_S = 2.5
PROGRESS_M = 0.03
CLEAR_GAIN_M = 0.03
FAIL_LIMIT = int(_f("GB_FR_FAIL_LIMIT", 5))
BLACKLIST_S = 5.0
REFUSE_BLACKLIST_S = 2.0
MAX_ESCAPE_S = _f("GB_FR_MAX_ESCAPE_S", 45.0)
MAX_TRAVEL_M = _f("GB_FR_MAX_TRAVEL_M", 1.2)
RESUME_HYST = 0.03
# token for gb_gate
TOKEN_NAME = "FUSION_NUDGE.json"
TOKEN_TTL_S = 0.6
TOKEN_MAX_SPAN_S = 1.0
ENABLED = str(E("GB_FUSION_RECOVERY", "1")).strip() != "0"

assert NUDGE_LIN_MIN > STALL_MIN_CMD and NUDGE_LIN_MAX <= 0.07 + 1e-9
assert FLOOR_FWD < NORMAL_FRONT_STOP and FLOOR_SWEEP < NORMAL_SWEEP and FLOOR_SWEEP > CORNER_R
assert PHYS_HALF_W > BODY_HALF_W


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _fin(v, default=INF):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    return x if x == x else default


# ------------------------------------------------------------------ inputs
def lidar_input(view, t=None):
    """Normalise a gb_obstacle.LidarView (gate or explore node) to the dict assess() uses. None -> None."""
    if view is None:
        return None
    try:
        env = min(_fin(getattr(view, "left", INF)), _fin(getattr(view, "right", INF)), _fin(getattr(view, "env_f", INF)))
        return {"t": float(view.t if t is None else t), "px": list(getattr(view, "px", ()) or ()),
                "py": list(getattr(view, "py", ()) or ()), "n": int(getattr(view, "n", 0) or 0),
                "blind_f": _fin(getattr(view, "blind_f", 1.0), 1.0), "blind_r": _fin(getattr(view, "blind_r", 1.0), 1.0),
                "env": env, "sweep": _fin(getattr(view, "sweep", 0.0), 0.0), "fwd": _fin(getattr(view, "fwd", 0.0), 0.0),
                "rev": _fin(getattr(view, "rev", 0.0), 0.0)}
    except Exception:
        return None


def rgb_input(d, now=None):
    if not d:
        return None
    try:
        return {"t": float(d.get("t", 0.0) or 0.0), "hazard": bool(d.get("hazard")), "label": d.get("label")}
    except Exception:
        return None


# ------------------------------------------------------------------ geometry
def path_clearance(px, py, lin, ang, half_w, horizon, step=0.01):
    """Footprint (x -BODY_REAR..+BODY_FRONT, |y| <= half_w) swept along the constant-curvature path of (lin, ang).
    Returns path length to first contact + BODY_FRONT (forward) / + BODY_REAR (reverse), INF if clear within horizon.
    Same units as gb_obstacle.arc_clearance, but with a configurable half width (physical chassis vs. normal corridor)."""
    if not lin or not len(px):
        return INF
    sgn = 1.0 if lin > 0 else -1.0
    k = float(ang) / abs(float(lin))
    n = int(max(step, horizon) / step) + 1
    lim = horizon + BODY_FRONT + BODY_REAR + half_w + 0.05
    pts = [(x, y) for x, y in zip(px, py) if x * x + y * y <= lim * lim]
    if not pts:
        return INF
    try:
        import numpy as np
        s = np.arange(n) * step
        th = k * s
        if abs(k) < 1e-6:
            X = sgn * s; Y = np.zeros_like(s)
        else:
            X = sgn * np.sin(th) / k; Y = sgn * (1.0 - np.cos(th)) / k
        P = np.asarray(pts, dtype=float)
        xs = P[:, 0][None, :]; ys = P[:, 1][None, :]
        c = np.cos(th)[:, None]; sn = np.sin(th)[:, None]
        dx = xs - X[:, None]; dy = ys - Y[:, None]
        xr = c * dx + sn * dy; yr = -sn * dx + c * dy
        hit = ((xr <= BODY_FRONT) & (xr >= -BODY_REAR) & (np.abs(yr) <= half_w)).any(axis=1)
        if not hit.any():
            return INF
        return float(s[int(np.argmax(hit))]) + (BODY_FRONT if sgn > 0 else BODY_REAR)
    except ImportError:                                            # pragma: no cover
        for i in range(n):
            si = i * step; th = k * si
            if abs(k) < 1e-6:
                X, Y = sgn * si, 0.0
            else:
                X, Y = sgn * math.sin(th) / k, sgn * (1.0 - math.cos(th)) / k
            c, sn = math.cos(th), math.sin(th)
            for x, y in pts:
                dx, dy = x - X, y - Y
                xr = c * dx + sn * dy; yr = -sn * dx + c * dy
                if -BODY_REAR <= xr <= BODY_FRONT and abs(yr) <= half_w:
                    return si + (BODY_FRONT if sgn > 0 else BODY_REAR)
        return INF


def _hit_indices(px, py, lin, ang, half_w, horizon, step=0.01):
    """Indices of points the footprint (half width half_w) touches within `horizon` of path length (forward)."""
    out = set()
    if not lin:
        return out
    k = float(ang) / abs(float(lin)); sgn = 1.0 if lin > 0 else -1.0
    n = int(max(step, horizon) / step) + 1
    lim = horizon + BODY_FRONT + BODY_REAR + half_w + 0.05
    cand = [i for i, (x, y) in enumerate(zip(px, py)) if x * x + y * y <= lim * lim]
    for j in range(n):
        si = j * step; th = k * si
        if abs(k) < 1e-6:
            X, Y = sgn * si, 0.0
        else:
            X, Y = sgn * math.sin(th) / k, sgn * (1.0 - math.cos(th)) / k
        c, sn = math.cos(th), math.sin(th)
        for i in cand:
            if i in out:
                continue
            dx, dy = px[i] - X, py[i] - Y
            xr = c * dx + sn * dy; yr = -sn * dx + c * dy
            if -BODY_REAR <= xr <= BODY_FRONT and abs(yr) <= half_w:
                out.add(i)
    return out


def clusters(px, py, r_max=NEAR_R, gap=CLUSTER_GAP):
    """Group LiDAR points within r_max into clusters ordered by bearing; a new cluster starts where consecutive points
    are more than `gap` apart. Returns [{"idx": [...], "width": m, "r": min range, "bearing": rad}]."""
    near = []
    for i, (x, y) in enumerate(zip(px, py)):
        r = math.hypot(x, y)
        if r <= r_max:
            near.append((math.atan2(y, x), i, r))
    if not near:
        return []
    near.sort()
    groups = [[near[0]]]
    for a, i, r in near[1:]:
        pa, pi_, pr = groups[-1][-1]
        if math.hypot(px[i] - px[pi_], py[i] - py[pi_]) <= gap:
            groups[-1].append((a, i, r))
        else:
            groups.append([(a, i, r)])
    if len(groups) > 1:                                             # join across the +/-pi seam
        a0, i0, _ = groups[0][0]; a1, i1, _ = groups[-1][-1]
        if math.hypot(px[i0] - px[i1], py[i0] - py[i1]) <= gap:
            groups[0] = groups.pop() + groups[0]
    out = []
    for g in groups:
        idx = [i for _a, i, _r in g]
        xs = [px[i] for i in idx]; ys = [py[i] for i in idx]
        if len(idx) <= 60:
            w = 0.0
            for p in range(len(idx)):
                for q in range(p + 1, len(idx)):
                    d = math.hypot(xs[p] - xs[q], ys[p] - ys[q])
                    if d > w:
                        w = d
        else:
            w = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        out.append({"idx": idx, "width": w, "r": min(r for _a, _i, r in g),
                    "bearing": math.atan2(sum(ys) / len(ys), sum(xs) / len(xs))})
    return out


def normal_blockers(px, py, lin, ang):
    """Indices of LiDAR points that make the NORMAL (unchanged) rules refuse this command: the normal-width corridor
    hit within the 0.27 m stop (+3 cm), the explore +/-25 deg cone inside 0.27 m, and any point inside the 0.24 m
    turn sweep when turning."""
    idx = set()
    if lin > 0:
        idx |= _hit_indices(px, py, lin, ang, HALF_W, max(0.0, NORMAL_FRONT_STOP - BODY_FRONT + 0.03))
        for i, (x, y) in enumerate(zip(px, py)):
            if x > 0 and abs(math.atan2(y, x)) <= CONE and math.hypot(x, y) < NORMAL_FRONT_STOP:
                idx.add(i)
    if ang:
        for i, (x, y) in enumerate(zip(px, py)):
            if math.hypot(x, y) < NORMAL_SWEEP:
                idx.add(i)
    return idx


def min_range(px, py):
    best = INF
    for x, y in zip(px, py):
        r = math.hypot(x, y)
        if r < best:
            best = r
    return best


# ------------------------------------------------------------------ camera (depth)
def depth_summary(img, fx, fy, cx, cy, t=None, scale_m=None, step=8, max_pts=220):
    """Gate side: compact depth-camera picture for fusion recovery from one raw depth frame (same projection as
    gb_obstacle.depth_analyze). Returns {"t", "valid", "bins": [[lo, hi, valid, floor, drop]...], "obs": [[x, y]...]}
    in the LiDAR-centre frame (x forward, y left), or None when the unit is unconfirmed."""
    import numpy as np
    try:
        scale = float(scale_m)
    except (TypeError, ValueError):
        return None
    if not (scale > 0):
        return None
    s = np.asarray(img)[::step, ::step]
    h, w = s.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    ku = ((uu * step) - cx) / fx; kv = ((vv * step) - cy) / fy
    Z = s.astype(np.float32) * scale
    valid = (Z >= DEPTH_MIN_VALID_M) & (Z <= DEPTH_MAX_VALID_M)
    Y = kv * Z; X = ku * Z
    hgt = CAM_Z - Y; xr = Z + CAM_X; yr = -X
    col_b = np.arctan2(-ku[0, :], 1.0)                              # column bearing (camera axis = robot forward)
    lo_b, hi_b = float(col_b.min()), float(col_b.max())
    bw = math.radians(CAM_BIN_DEG)
    nb = max(1, int(math.ceil((hi_b - lo_b) / bw)))
    bins = []
    near = valid & (xr <= 1.0)
    floor = near & (np.abs(hgt) <= 0.04)
    drop = near & (hgt < -0.07)
    for b in range(nb):
        lo = lo_b + b * bw; hi = min(hi_b, lo + bw)
        cols = (col_b >= lo) & (col_b <= hi + 1e-9)
        if not cols.any():
            continue
        vb = float(valid[:, cols].mean())
        bins.append([round(lo, 4), round(hi, 4), round(vb, 3), int(floor[:, cols].sum()), int(drop[:, cols].sum())])
    obs = valid & (hgt >= DEPTH_HMIN) & (hgt <= DEPTH_HMAX) & (xr <= 1.2) & (np.abs(yr) <= 0.6)
    pts = []
    if obs.any():
        q = np.round(np.stack([xr[obs], yr[obs]], axis=1) / 0.04).astype(int)
        keys, counts = np.unique(q, axis=0, return_counts=True)
        keep = keys[counts >= 2]                                    # speckle / flying-pixel rejection
        if len(keep):
            order = np.argsort(np.hypot(keep[:, 0], keep[:, 1]))[:max_pts]
            pts = [[round(float(k[0]) * 0.04, 3), round(float(k[1]) * 0.04, 3)] for k in keep[order]]
    return {"t": float(time.time() if t is None else t), "valid": round(float(valid.mean()) if valid.size else 0.0, 3),
            "bins": bins, "obs": pts}


def camera_ok(cam, now):
    """C1: (ok, reason). Offline / stale / low-validity camera -> fail safe to LiDAR-only."""
    if not cam or not isinstance(cam, dict):
        return False, "camera offline (no depth summary)"
    if cam.get("online") is False:
        return False, "camera offline (depth_online false)"
    age = now - _fin(cam.get("t"), -1e9)
    if not (-0.2 <= age <= CAM_FRESH_S):
        return False, "camera stale (%.2fs)" % age if age < 1e8 else "camera stale"
    v = _fin(cam.get("valid"), 0.0)
    if v < CAM_MIN_VALID:
        return False, "camera low validity (%.0f%% < %.0f%%)" % (100 * v, 100 * CAM_MIN_VALID)
    if not cam.get("bins"):
        return False, "camera has no bearing bins"
    return True, "camera fresh %.2fs valid %.0f%%" % (age, 100 * v)


def _path_bearings(lin, ang, s_from=0.15, s_to=None):
    """Bearings (from the LiDAR centre) of the chassis edges along the candidate path, for the camera bins."""
    s_to = CAM_OPEN_M if s_to is None else s_to
    out = []
    if not lin:
        side = 1.0 if ang > 0 else -1.0                             # pivot: the half of the view it turns toward
        return [0.0, side * math.radians(20.0)]
    k = float(ang) / abs(float(lin))
    if abs(k) > 1e-6:                                               # arcs: only the part still inside the camera view
        s_to = max(s_from, min(s_to, CAM_ARC_MAX_TURN / abs(k)))
    s = s_from
    while s <= s_to + 1e-9:
        th = k * s
        if abs(k) < 1e-6:
            X, Y = s, 0.0
        else:
            X, Y = math.sin(th) / k, (1.0 - math.cos(th)) / k
        nx, ny = -math.sin(th), math.cos(th)
        for e in (-PHYS_HALF_W, PHYS_HALF_W):
            out.append(math.atan2(Y + e * ny, X + BODY_FRONT * math.cos(th) + e * nx))
        s += 0.05
    return out


def camera_path(cam, lin, ang):
    """C2 + C3 for one candidate: (ok, reason, cam_clear_m)."""
    bins = cam.get("bins") or []
    lo_all = min(b[0] for b in bins); hi_all = max(b[1] for b in bins)
    brs = _path_bearings(lin, ang)
    blo, bhi = min(brs), max(brs)
    if blo < lo_all - CAM_FOV_PAD or bhi > hi_all + CAM_FOV_PAD:
        return False, "path leaves the camera view (%.0f..%.0f deg)" % (math.degrees(blo), math.degrees(bhi)), 0.0
    need = [b for b in bins if b[1] >= blo and b[0] <= bhi]
    if not need:
        return False, "no camera bins over the path", 0.0
    for b in need:
        if b[2] < CAM_MIN_BIN_VALID:
            return False, "camera sees a close surface / glass (bin %.0f deg valid %.0f%%)" % (
                math.degrees((b[0] + b[1]) / 2), 100 * b[2]), 0.0
    fl = sum(int(b[3]) for b in need); dr = sum(int(b[4]) for b in need)
    if dr >= CAM_DROP_PTS:
        return False, "camera sees a drop-off (%d below-floor samples)" % dr, 0.0
    if fl < CAM_MIN_FLOOR:
        return False, "camera cannot confirm open floor (%d floor samples)" % fl, 0.0
    obs = cam.get("obs") or []
    px = [p[0] for p in obs]; py = [p[1] for p in obs]
    if lin > 0:
        k = abs(float(ang) / float(lin))
        hz = CAM_OPEN_M - BODY_FRONT
        if k > 1e-6:
            hz = max(0.15, min(hz, CAM_ARC_MAX_TURN / k))
        cc = path_clearance(px, py, lin, ang, PHYS_HALF_W, hz)
        if cc < hz + BODY_FRONT:
            return False, "camera agrees an object is close on the path (%.2f m)" % cc, cc
        return True, "camera path open (floor %d)" % fl, cc
    side = 1.0 if ang > 0 else -1.0
    for x, y in zip(px, py):
        if math.hypot(x, y) < CAM_OPEN_M and (y * side) >= -PHYS_HALF_W:
            return False, "camera sees an object %.2f m on the turn side" % math.hypot(x, y), math.hypot(x, y)
    return True, "camera turn side open (floor %d)" % fl, INF


def rgb_veto(rgb, now):
    """C4: (veto, reason). Stale / missing RGB never vetoes (depth is the required camera)."""
    if not rgb or not rgb.get("hazard"):
        return False, ""
    if now - _fin(rgb.get("t"), -1e9) > RGB_FRESH_S:
        return False, ""
    lab = str(rgb.get("label") or "").strip().lower().replace("_", " ")
    if lab in RGB_LEGGED:
        return False, "rgb sees legged furniture (%s)" % lab
    return True, "RGB camera sees %s in the way" % (lab or "an object")


# ------------------------------------------------------------------ decision
def clamp_cmd(lin, ang):
    """Fusion micro-nudge bounds: lin 0 or +[0.06, 0.07] (never reverse), |ang| <= 0.30."""
    l = _fin(lin, 0.0); a = _fin(ang, 0.0)
    if l > 0:
        l = min(NUDGE_LIN_MAX, max(NUDGE_LIN_MIN, l))
    else:
        l = 0.0
    a = max(-ANG_MAX, min(ANG_MAX, a))
    return l, a


def check_command(lin, ang, lidar, cam, rgb, now, mems=None):
    """Decide ONE exact command. Returns (ok, verdict, reason) with verdict in
    'nudge' (allowed), 'lidar_only' (camera offline / stale / invalid: fail safe, normal stops stand),
    'real' (treat as a real obstacle: normal stops stand)."""
    lin = _fin(lin, 0.0); ang = _fin(ang, 0.0)
    if not ENABLED:
        return False, "lidar_only", "fusion recovery disabled (GB_FUSION_RECOVERY=0)"
    if lin < 0:
        return False, "real", "reverse is never relaxed (no rear camera)"
    if lin == 0 and ang == 0:
        return False, "real", "zero command"
    if lin > NUDGE_LIN_MAX + 1e-6 or (lin > 0 and lin < NUDGE_LIN_MIN - 1e-6) or abs(ang) > ANG_MAX + 1e-6:
        return False, "real", "outside micro-nudge bounds (lin %.3f ang %.3f)" % (lin, ang)
    if lin == 0 and abs(ang) < 0.2:
        return False, "real", "pivot below the turn-stall minimum"
    if not lidar:
        return False, "real", "no LiDAR"
    if now - _fin(lidar.get("t"), -1e9) > LIDAR_FRESH_S:
        return False, "real", "LiDAR stale"
    if int(lidar.get("n") or 0) <= 0:
        return False, "real", "LiDAR scan empty"
    if _fin(lidar.get("blind_f"), 1.0) >= BLIND_FRAC_SLOW:
        return False, "real", "LiDAR dropouts ahead %.0f%% (glass / black?)" % (100 * _fin(lidar.get("blind_f"), 1.0))
    if _fin(lidar.get("blind_r"), 1.0) >= BLIND_FRAC_STOP:
        return False, "real", "LiDAR blind behind"
    ok, why = camera_ok(cam, now)
    if not ok:
        return False, "lidar_only", why
    veto, vwhy = rgb_veto(rgb, now)
    if veto:
        return False, "real", vwhy
    mems = mems or {}
    for k in ("lidar_mem", "depth_mem"):
        if _fin(mems.get(k), INF) < FLOOR_FWD + 0.05:
            return False, "real", "%s says an obstacle is %.2f m ahead" % (k.replace("_", " "), _fin(mems.get(k)))
    if _fin(lidar.get("env"), INF) < ENV_CONTACT:
        return False, "real", "something is touching the body envelope"
    px = lidar.get("px") or []; py = lidar.get("py") or []
    # L3 physical chassis corridor / hard floor
    if ang:
        mr = min_range(px, py)
        if mr < FLOOR_SWEEP:
            return False, "real", "point %.2f m inside the pivot floor %.2f m" % (mr, FLOOR_SWEEP)
    phys = INF
    if lin > 0:
        phys = path_clearance(px, py, lin, ang, PHYS_HALF_W, PASSABLE_M - BODY_FRONT)
        if phys < FLOOR_FWD:
            return False, "real", "chassis path %.2f m inside the hard floor %.2f m" % (phys, FLOOR_FWD)
        if phys < PASSABLE_M:
            return False, "real", "no real gap: chassis path blocked at %.2f m" % phys
    # L2 narrow vs dense
    blk = normal_blockers(px, py, lin, ang)
    cl = lidar.get("_cl")
    if cl is None:
        cl = clusters(px, py)
        try:
            lidar["_cl"] = cl
        except TypeError:
            pass
    near_cl = [c for c in cl if c["r"] <= PASSABLE_M + 0.05]
    if len(near_cl) > MAX_NEAR_CLUSTERS:
        return False, "real", "dense clutter (%d clusters nearby)" % len(near_cl)
    if blk:
        owner = {}
        for c in cl:
            for i in c["idx"]:
                owner[i] = c
        for i in blk:
            c = owner.get(i)
            if c is None or c["width"] > NARROW_MAX_W:
                w = c["width"] if c else INF
                return False, "real", "wide / solid barrier in the stop zone (%.2f m wide)" % w
    # C2 + C3 camera along the path
    ok, cwhy, _cc = camera_path(cam, lin, ang)
    if not ok:
        return False, "real", cwhy
    return True, "nudge", "narrow LiDAR hits (%d pts), tight-squeeze chassis path clear >= %.2f m, %s" % (
        len(blk), min(phys, PASSABLE_M), cwhy)


def candidates(normal=None):
    """Candidate micro-nudges. 'back' is a NORMAL-rule move (no token) offered only when the rear is normally clear."""
    c = [{"name": "fwd", "lin": NUDGE_LIN, "ang": 0.0, "token": True},
         {"name": "veer_left", "lin": NUDGE_LIN, "ang": VEER_ANG, "token": True},
         {"name": "veer_right", "lin": NUDGE_LIN, "ang": -VEER_ANG, "token": True},
         {"name": "arc_left", "lin": NUDGE_LIN, "ang": ARC_ANG, "token": True},
         {"name": "arc_right", "lin": NUDGE_LIN, "ang": -ARC_ANG, "token": True},
         {"name": "hard_left", "lin": NUDGE_LIN, "ang": HARD_ARC_ANG, "token": True},
         {"name": "hard_right", "lin": NUDGE_LIN, "ang": -HARD_ARC_ANG, "token": True},
         {"name": "pivot_left", "lin": 0.0, "ang": PIVOT_ANG, "token": True},
         {"name": "pivot_right", "lin": 0.0, "ang": -PIVOT_ANG, "token": True}]
    n = normal or {}
    if (_fin(n.get("rear"), 0.0) >= NORMAL_REAR_STOP + 0.05 and _fin(n.get("corr_rev"), 0.0) >= NORMAL_REAR_STOP + 0.05
            and _fin(n.get("blind_r"), 1.0) < BLIND_FRAC_STOP):
        c.append({"name": "back", "lin": -NUDGE_LIN, "ang": 0.0, "token": False})
    return c


def _heading_after(c, s=0.3):
    if not c["lin"]:
        return math.copysign(math.pi / 4, c["ang"])
    k = c["ang"] / abs(c["lin"])
    return (k * s) * (1 if c["lin"] > 0 else -1) + (math.pi if c["lin"] < 0 else 0.0)


def assess(lidar, cam, rgb, now, goal_bearing=None, exclude=(), mems=None, normal=None):
    """Rank every candidate. Returns {"mode": "nudge"|"real"|"lidar_only", "reason", "allowed": [cands best first],
    "rejected": {name: reason}}. 'lidar_only' whenever the camera cannot be trusted (caller keeps LiDAR-only logic)."""
    ok, why = camera_ok(cam, now)
    if not ENABLED:
        return {"mode": "lidar_only", "reason": "fusion recovery disabled", "allowed": [], "rejected": {}}
    if not ok:
        return {"mode": "lidar_only", "reason": why, "allowed": [], "rejected": {}}
    allowed = []; rejected = {}
    px = (lidar or {}).get("px") or []; py = (lidar or {}).get("py") or []
    for c in candidates(normal):
        if c["name"] in exclude:
            rejected[c["name"]] = "blacklisted"
            continue
        if not c["token"]:
            if lidar and now - _fin(lidar.get("t"), -1e9) <= LIDAR_FRESH_S:
                cc = dict(c); cc["score"] = 0.2 + min(1.0, _fin((normal or {}).get("rear"), 0.0)) * 0.1
                cc["reason"] = "rear normally clear"
                allowed.append(cc)
            continue
        good, verdict, r = check_command(c["lin"], c["ang"], lidar, cam, rgb, now, mems)
        if not good:
            rejected[c["name"]] = r
            continue
        cc = dict(c); cc["reason"] = r
        phys = path_clearance(px, py, c["lin"], c["ang"], PHYS_HALF_W, 1.0) if c["lin"] else min_range(px, py)
        score = min(1.2, phys)
        if c["lin"] > 0:
            score += 0.3
        if goal_bearing is not None:
            score += 0.6 * math.cos(_wrap(_heading_after(c) - float(goal_bearing)))
        else:
            score += 0.1 * math.cos(_heading_after(c))
        cc["score"] = score
        allowed.append(cc)
    allowed.sort(key=lambda c: (-c["score"], c["name"]))
    if any(c["token"] for c in allowed):
        return {"mode": "nudge", "reason": allowed[0]["reason"], "allowed": allowed, "rejected": rejected}
    reason = "; ".join("%s: %s" % (k, v) for k, v in sorted(rejected.items())[:3]) or "no candidate"
    return {"mode": "real", "reason": reason, "allowed": [], "rejected": rejected}


def normal_clear(normal):
    """True once the NORMAL rules would let the mission drive again (front >= 0.27 + 3 cm and sweep >= 0.24)."""
    n = normal or {}
    return (_fin(n.get("front"), 0.0) >= NORMAL_FRONT_STOP + RESUME_HYST and _fin(n.get("sweep"), 0.0) >= NORMAL_SWEEP)


def trip_kind(why):
    t = (why or "").lower()
    if not t:
        return None
    if any(k in t for k in ("estop", "e-stop", "emergency", "fault", "low voltage", "tilt")):
        return "hard"
    if any(k in t for k in ("track stall", "turn stall", "bump", "jolt", "slipping", "pushing")) and "no progress" not in t:
        return "stall"
    if "no progress" in t:
        return "soft"                                              # LiDAR/depth progress checker (see step())
    if "stall" in t:
        return "stall"
    return "refused"


# ------------------------------------------------------------------ escape state machine
class FusionEscape(object):
    """Continuous slow fusion escape. step() at ~10 Hz returns
    {"event": "nudge"|"recovered"|"fallback"|"abort"|"stop", "lin", "ang", "token", "cand", "why", "fails"}.
    'nudge' commands are emitted back-to-back (no zero dwell between slices); 'recovered' hands control back to the
    mission; 'fallback' = camera became unusable or obstacle judged real -> caller uses its LiDAR-only logic;
    'abort' = FAIL_LIMIT consecutive no-progress cycles (or time / travel budget); 'stop' = ESTOP / disarm / abort."""

    def __init__(self, mission, now, odom=None, fail_limit=None):
        self.mission = mission or "mission"
        self.t0 = float(now)
        o = tuple(odom or (0.0, 0.0, 0.0))
        self.start_xy = (float(o[0]), float(o[1]))
        self.cycle_xy = self.start_xy
        self.cycle_t = float(now)
        self.cycle_front = None
        self.fails = 0
        self.fail_limit = int(fail_limit or FAIL_LIMIT)
        self.cur = None
        self.slice_until = 0.0
        self.black = {}
        self.travel = 0.0
        self.last_xy = self.start_xy
        self.nudges = 0
        self.active = True
        self.log = []

    def _blacklist(self, name, now, secs):
        if name:
            self.black[name] = max(self.black.get(name, 0.0), now + secs)

    def _done(self, ev, why, **kw):
        self.active = False
        d = {"event": ev, "lin": 0.0, "ang": 0.0, "token": False, "cand": None, "why": why, "fails": self.fails}
        d.update(kw)
        return d

    def step(self, now, lidar, cam, rgb, odom=None, normal=None, mems=None, goal_bearing=None,
             armed=True, estop=False, abort=False, trip=None, refused=False, goal_reached=False):
        now = float(now)
        if estop or abort or not armed:
            return self._done("stop", "fusion escape: stop (estop/abort/disarm)")
        if goal_reached and self.nudges > 0:                      # go-to missions: the waypoint itself was inside the gap
            return self._done("recovered", "fusion escape: reached the mission waypoint after %d nudges" % self.nudges)
        if odom:
            xy = (float(odom[0]), float(odom[1]))
            self.travel += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1]); self.last_xy = xy
        else:
            xy = self.last_xy
        tk = trip_kind(trip)
        if tk == "hard":
            return self._done("stop", "fusion escape: hard trip (%s)" % (trip or "")[:60])
        if tk == "stall":                                          # encoder stall / bump: that heading is REAL
            self._blacklist(self.cur, now, MAX_ESCAPE_S)
            self.fails += 1
            self.cur = None
            if self.fails >= self.fail_limit:
                return self._done("abort", "fusion escape: %d failed attempts (last: stall/bump)" % self.fails)
        elif tk == "soft":
            # The gate's LiDAR/depth progress checker already stopped us (and may have backed off). If the normal
            # front clearance did NOT improve, the heading is wedged -> REAL (blacklist + fail). If it improved (the
            # leg left the corridor, so the 'obstacle distance' jumped), it was the pass-through, not a wedge.
            nf_now = _fin((normal or {}).get("front"), 0.0)
            base = self.cycle_front if self.cycle_front is not None else nf_now
            if nf_now - base >= CLEAR_GAIN_M:
                self._blacklist(self.cur, now, REFUSE_BLACKLIST_S)
            else:
                self._blacklist(self.cur, now, MAX_ESCAPE_S)
                self.fails += 1
                if self.fails >= self.fail_limit:
                    return self._done("abort", "fusion escape: %d failed attempts (last: no progress)" % self.fails)
            self.cur = None
        elif tk == "refused" or refused:
            self._blacklist(self.cur, now, REFUSE_BLACKLIST_S)
            self.cur = None
        moved = math.hypot(xy[0] - self.start_xy[0], xy[1] - self.start_xy[1])
        if normal_clear(normal) and (self.nudges > 0 or moved >= PROGRESS_M):
            return self._done("recovered", "fusion escape: normal clearance restored after %d nudges, %.2f m" % (
                self.nudges, self.travel), moved=moved)
        if now - self.t0 > MAX_ESCAPE_S:
            return self._done("abort", "fusion escape: time budget %.0fs used" % MAX_ESCAPE_S)
        if self.travel > MAX_TRAVEL_M:
            return self._done("fallback", "fusion escape: travel budget %.1f m used, handing back" % MAX_TRAVEL_M)
        # cycle bookkeeping (continuous; never a dwell)
        nf = _fin((normal or {}).get("front"), 0.0)
        if self.cycle_front is None:
            self.cycle_front = nf
        if now - self.cycle_t >= CYCLE_S:
            prog = (math.hypot(xy[0] - self.cycle_xy[0], xy[1] - self.cycle_xy[1]) >= PROGRESS_M
                    or nf - self.cycle_front >= CLEAR_GAIN_M)
            if prog:
                self.fails = 0
            else:
                self.fails += 1
                self._blacklist(self.cur, now, BLACKLIST_S)
                self.cur = None
                if self.fails >= self.fail_limit:
                    return self._done("abort", "fusion escape: %d consecutive no-progress cycles" % self.fails)
            self.cycle_t = now; self.cycle_xy = xy; self.cycle_front = nf
        excl = tuple(k for k, until in self.black.items() if until > now)
        v = assess(lidar, cam, rgb, now, goal_bearing=goal_bearing, exclude=excl, mems=mems, normal=normal)
        if v["mode"] != "nudge":
            return self._done("fallback", "fusion escape: %s (%s)" % (
                "camera not trusted -> LiDAR-only" if v["mode"] == "lidar_only" else "obstacle is real", v["reason"]),
                mode=v["mode"])
        names = [c["name"] for c in v["allowed"]]
        pick = None
        if self.cur in names and now < self.slice_until:
            pick = next(c for c in v["allowed"] if c["name"] == self.cur)
        elif self.cur in names:                                     # slice ended: keep going while it is still best-ish
            pick = next(c for c in v["allowed"] if c["name"] == self.cur)
            self.slice_until = now + (SLICE_S if pick["lin"] else PIVOT_SLICE_S)
        else:
            pick = v["allowed"][0]
            self.cur = pick["name"]
            self.slice_until = now + (SLICE_S if pick["lin"] else PIVOT_SLICE_S)
        self.nudges += 1
        lin, ang = (clamp_cmd(pick["lin"], pick["ang"]) if pick["token"] else (pick["lin"], pick["ang"]))
        return {"event": "nudge", "lin": lin, "ang": ang, "token": bool(pick["token"]), "cand": pick["name"],
                "why": "fusion nudge %s (%s)" % (pick["name"], pick.get("reason", "")), "fails": self.fails}


# ------------------------------------------------------------------ token (explore node -> gate)
def token_path(estop_file):
    return os.path.join(os.path.dirname(estop_file), TOKEN_NAME)


def write_token(path, lin, ang, cand, mission, now=None, pid=None):
    now = time.time() if now is None else float(now)
    d = {"v": 1, "t": now, "exp": now + TOKEN_TTL_S, "pid": int(os.getpid() if pid is None else pid),
         "lin": round(float(lin), 4), "ang": round(float(ang), 4), "cand": cand, "mission": mission}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, path)
    return d


def read_token(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def clear_token(path):
    try:
        os.remove(path)
    except OSError:
        pass


def _pid_is_explore(pid):
    try:
        args = open("/proc/%d/cmdline" % int(pid), "rb").read().split(b"\0")
    except Exception:
        return False
    return any(a.endswith(b"gb_explore.py") for a in args) and b"--node" in args


def token_valid(tok, now, lin, ang, void_before=0.0, check_pid=True):
    """Gate side: (ok, reason). A token is a short-lived REQUEST, never a permission by itself - the gate still
    re-runs check_command() with its own sensors."""
    if not tok or tok.get("v") != 1:
        return False, "no token"
    t = _fin(tok.get("t"), -1e9); exp = _fin(tok.get("exp"), -1e9)
    if not (-0.05 <= now - t <= TOKEN_TTL_S) or now > exp or exp - t > TOKEN_MAX_SPAN_S:
        return False, "token expired"
    if t < float(void_before or 0.0):
        return False, "token older than the last gate stop/trip"
    if check_pid and not _pid_is_explore(tok.get("pid", -1)):
        return False, "token owner is not a live gb_explore node"
    l = _fin(lin, 0.0); a = _fin(ang, 0.0)
    if l < 0 or l > NUDGE_LIN_MAX + 1e-6 or abs(a) > ANG_MAX + 1e-6:
        return False, "command outside micro-nudge bounds"
    if abs(l - _fin(tok.get("lin"), 9.0)) > 0.01 or abs(a - _fin(tok.get("ang"), 9.0)) > 0.05:
        return False, "command does not match the token"
    return True, "token ok"


def cam_for_status(cam, now=None):
    """Compact copy of the gate's depth summary for /gb/gate_status (explore planner reads it)."""
    if not cam:
        return None
    now = time.time() if now is None else now
    if now - _fin(cam.get("t"), -1e9) > 2.0:
        return {"t": cam.get("t"), "valid": cam.get("valid"), "bins": [], "obs": [], "stale": True}
    obs = [p for p in (cam.get("obs") or []) if p[0] <= 0.9][:160]
    return {"t": cam.get("t"), "valid": cam.get("valid"), "bins": cam.get("bins") or [], "obs": obs}
