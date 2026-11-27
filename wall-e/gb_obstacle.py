"""gb_obstacle.py - Grok Bot obstacle fusion (pure Python + numpy, no ROS), 2026-11-05 (.bak-obstacle).

Used by gb_gate (the ONE path to the motors: voice timed drives, turns, features.drive, explore / Nav2 / follow
streams, joystick) and by gb_explore (turn-away decisions). Sensors:
  LiDAR  /scan_raw        footprint corridor (not a cone), rear corridor, 360 deg turn sweep, dropout (glass/black) rule
  depth  depth0/image_raw obstacles above/below the LiDAR plane inside the corridor (8x downsampled, ~0.2 ms/frame),
                          plus odometry-carried memory of a low obstacle once it drops under the camera's view
  encoders /odom_raw      commanded vs actual track speed -> stall / bump
  LiDAR progress          encoders say we move but the corridor distance does not shrink -> slipping / pushing
  IMU    /imu             horizontal jolt (sudden decel) or tilt -> collision / climbing
  RGB    rgb_guard.json   YOLOv8n advisory (person / furniture in the path) -> slow only, never a speed-up
Conservative fusion: the smallest distance from any trusted sensor wins; a stale critical sensor (LiDAR, encoders)
stops, a stale secondary sensor (depth, IMU) caps the speed at V_DEGRADED; speed ramps with the fused distance.
Distances are measured from the LiDAR centre along the direction of travel.
"""
import math, os, time
E = os.environ.get

def _f(k, d):
    try: return float(E(k, d))
    except (TypeError, ValueError): return float(d)

# chassis from mentorpi_description tank.xacro / base_chaais.STL (TF base_footprint): x -0.152..+0.126, y +/-0.098;
# LiDAR x=-0.002 z=0.184; depth camera x=+0.080 z=0.139, optical axis level (TF rpy -90/0/-90).
BODY_FRONT = _f("OBS_BODY_FRONT", 0.13); BODY_REAR = _f("OBS_BODY_REAR", 0.155); BODY_HALF_W = _f("OBS_BODY_HALF_W", 0.10)
SIDE_MARGIN = _f("OBS_SIDE_MARGIN", 0.05); HALF_W = BODY_HALF_W + SIDE_MARGIN          # corridor half width 0.15 m
STOP_FWD = _f("OBS_STOP_FWD", 0.27)        # ~0.14 m bumper gap (was 0.35 / 0.22 m; human-like near reaction)
STOP_REV = _f("OBS_STOP_REV", 0.28)        # reverse ~0.12 m behind rear edge (was 0.32)
SLOW_FULL = _f("OBS_SLOW_FULL", 0.70)      # ease off when object is clearly ahead (was 1.2 m)
V_MIN = _f("OBS_V_MIN", 0.04)              # creep speed at the stop distance
V_DEGRADED = _f("OBS_V_DEGRADED", 0.06)    # depth or IMU stale/missing
V_RGB = _f("OBS_V_RGB", 0.05)              # RGB advisory: person / furniture in the path
V_BLIND = V_MIN                            # LiDAR dropouts in the path (glass, black, too close)
SWEEP_R = _f("OBS_SWEEP_R", 0.24)          # turn clearance (was 0.26; corner ~0.18)
LIDAR_STALE = _f("OBS_LIDAR_STALE", 0.5); ODOM_STALE = _f("OBS_ODOM_STALE", 0.35)
# 2026-11-23 fix 4: IMU_STALE 0.35 -> 0.60 s. Measured live: the stock ros_robot_controller publishes /imu_raw (-> imu_filter
# -> /imu) at ~47 Hz but with periodic source gaps up to 0.336 s even at idle (stamp gaps 0.26 s), i.e. <15 ms below the old
# 0.35 s limit, so any executor delay in the gate (single-threaded, depth + LiDAR + IMU callbacks) under mission load
# flagged "IMU stale". 0.60 s still catches a dead IMU within ~0.6 s; jolt / tilt detection is per-sample and unchanged.
DEPTH_STALE = _f("OBS_DEPTH_STALE", 1.0); IMU_STALE = _f("OBS_IMU_STALE", 0.60); RGB_STALE = _f("OBS_RGB_STALE", 2.5)
BLIND_CONE = math.radians(45); BLIND_RUN_CONE = math.radians(30)
BLIND_FRAC_SLOW = 0.25; BLIND_FRAC_STOP = 0.6; BLIND_RUN_SLOW = 4
# 2026-11-23 blindfix (John-approved fix 1): sclidar /scan_raw reports EVERY invalid return as NaN (no inf / 0 / <range_min
# seen live: 400 rays, range_min 0.10, range_max 10.0). In an open room NaN runs of 6-8 rays sit between returns 2.3-3.0 m
# away (benign: far / absorbing), so a bare NaN cannot mean "near". A NaN run whose nearest valid neighbour (either side)
# is closer than NEAR_CTX is treated as a SOLID surface at that neighbour's range (dark / glossy / glass part of a near
# object), so it enters the corridor / rear / sweep / arc checks and STOPS at the normal stop distance.
NEAR_CTX = _f("OBS_NEAR_CTX", 1.0); NEAR_RUN_MAX = 60      # runs longer than ~94 deg are left to the blind-fraction rule
LMEM_S = _f("OBS_LMEM_S", 2.0)             # LiDAR approach memory: a corridor obstacle that turns into NaN stays 2 s
LMEM_CLEAR_HYST = _f("OBS_LMEM_CLEAR_HYST", 0.08)  # 2026-11-26: clear only when scan is beyond SLOW_FULL+this (not merely past STOP)
ARC_STEP = 0.01                            # 2026-11-23 arcfix (fix 2): swept-footprint step along a curved path (m)
# Depth camera pose relative to the LiDAR centre (x) and the floor (z). Calibrated Aurora 930 Pro mount:
# 0.092 m forward of the LiDAR, 0.127 m AGL. Overridable if a later measurement supersedes this.
CAM_X = _f("OBS_CAM_X", 0.092); CAM_Z = _f("OBS_CAM_Z", 0.127)
# Aurora 930 Pro working window is 0.30-3.00 m (axial Z). Returns inside the 0.30 m blind zone are invalid
# and must never trigger or clear a stop; 2D LiDAR owns the last 30 cm completely.
DEPTH_MIN_VALID_M = _f("OBS_DEPTH_MIN_VALID_M", 0.30)
DEPTH_MAX_VALID_M = _f("OBS_DEPTH_MAX_VALID_M", 3.00)
DEPTH_HMIN = _f("OBS_DEPTH_HMIN", 0.05); DEPTH_HMAX = _f("OBS_DEPTH_HMAX", 0.40); DEPTH_ZMAX = _f("OBS_DEPTH_ZMAX", 1.6)
DEPTH_MIN_PTS = int(_f("OBS_DEPTH_MIN_PTS", 6)); DEPTH_STEP = int(_f("OBS_DEPTH_STEP", 8)); DEPTH_CLUSTER = 0.08
MEM_S = 6.0; MEM_YAW = 0.35                # carried low-obstacle memory: 6 s, dropped after a 20 deg turn
MEM_CLEAR = 0.60                           # ...or when depth sees the spot (>0.6 m away, inside the FOV) empty
STALL_MIN_CMD = 0.05; STALL_RATIO = 0.3; STALL_GRACE = 0.8; STALL_HOLD = 0.5  # 2026-11-26: stay above V_MIN (0.04) so LiDAR-dropout creep cannot false-trigger track stall
STALL_MIN_ANG = 0.25
# 2026-11-25 noprog (John 12:59 ET): 1.2 s / 5 cm judged LiDAR noise at 0.08 m/s (phantom 'no progress' -> FAULT ESTOP 12:57).
# Odom here is command-derived, so 'moved' = commanded travel; judge over 3.5 s and >= 15 cm commanded so the expected
# closure (25 % of commanded, ~4+ cm) is well above corridor-range noise. Was: SLIP_WIN 1.2, SLIP_MIN_ODOM 0.05.
SLIP_WIN = _f("OBS_SLIP_WIN", 3.5); SLIP_MIN_ODOM = _f("OBS_SLIP_MIN_ODOM", 0.15); SLIP_FRAC = 0.25; SLIP_RANGE = 2.0
JOLT = _f("OBS_JOLT", 7.0); TILT_DEG = _f("OBS_TILT_DEG", 15.0); TILT_HOLD = 0.3  # 15-25 deg: stop, no back-off
TILT_HARD = _f("OBS_TILT_HARD_DEG", 25.0); TILT_HARD_HOLD = 0.2   # pitch OR roll (or combined) > 25 deg -> latch ESTOP
PITCH_OFF = _f("OBS_TILT_PITCH_OFFSET_DEG", 0.0); ROLL_OFF = _f("OBS_TILT_ROLL_OFFSET_DEG", 0.0)  # 2026-11-17 level reference (John: flat on stand)
# 2026-11-23 navstd (nav standards): all-sides body envelope, progress checker, gyro turn check, clearest sector
ENV_SLOW = _f("OBS_ENV_SLOW", 0.10)        # object within 10 cm of the body outline (sides / corners / ends): slow zone
ENV_CONTACT = _f("OBS_ENV_CONTACT", 0.02)  # within 2 cm (touching): creep only (V_MIN), straight escape still allowed
V_SIDE = _f("OBS_V_SIDE", 0.06)            # slow-zone cap (Nav2 CollisionMonitor 'slowdown' polygon analogue)
ENV_RANGE = 0.60                           # only rays closer than this can be inside the envelope zones
PROG_WIN = _f("OBS_PROG_WIN", 3.5)         # progress window (s)  (Nav2 SimpleProgressChecker analogue, sensor-based)
PROG_MIN_CMD = _f("OBS_PROG_MIN_CMD", 0.15)   # commanded travel in the window before judging (m)
PROG_NEAR = _f("OBS_PROG_NEAR", 0.80)      # judge only with LiDAR / depth evidence of an obstacle this close (m)
PROG_FRAC = _f("OBS_PROG_FRAC", 0.25)      # observed closure < 25 % of commanded travel -> no progress
TURN_GRACE = _f("OBS_TURN_GRACE", 1.0); TURN_WIN = _f("OBS_TURN_WIN", 0.8)
TURN_FRAC = _f("OBS_TURN_FRAC", 0.25)      # gyro |wz| < 25 % of the commanded rate over TURN_WIN -> turn stall
GYRO_PROOF = 0.15                          # gyro check arms only after the gyro has shown >= 0.15 rad/s once (self-test)
N_SECT = 12                                # 30 deg sectors for the clearest-heading search
SECT_MIN_FRAC = 0.5                        # a sector needs >= 50 % real (or near-imputed) rays, else 'unknown'
TILT_LPF = 0.2                             # gravity low-pass per IMU sample (~0.1 s at 48 Hz): rejects jolts / vibration
INF = float("inf")

def _wrap(a): return math.atan2(math.sin(a), math.cos(a))

def ramp(d, vmax, stop=STOP_FWD, full=SLOW_FULL, vmin=V_MIN):
    """Allowed speed for clearance d: 0 below `stop`, vmin at `stop`, linear to vmax at `full`; never above vmax."""
    vmax = abs(vmax)
    if not (d >= stop): return 0.0
    if vmax <= vmin: return vmax
    k = min(1.0, max(0.0, (d - stop) / max(1e-6, full - stop)))
    return min(vmax, vmin + (vmax - vmin) * k)

# ------------------------------------------------------------------ LiDAR
class LidarView:
    __slots__ = ("t", "fwd", "rev", "sweep", "blind_f", "blind_run", "blind_r", "n", "core_bad_f", "core_bad_r",
                 "imputed", "px", "py", "bad_a", "fwd_a", "rev_a", "left", "right", "env_f", "env_r", "sec")
    def __init__(self, t=0.0):
        self.t = t; self.fwd = self.rev = self.sweep = INF; self.blind_f = self.blind_r = 0.0; self.blind_run = 0; self.n = 0
        self.core_bad_f = self.core_bad_r = 0; self.imputed = 0; self.px = (); self.py = ()   # blindfix / arcfix
        self.bad_a = (); self.fwd_a = self.rev_a = None
        self.left = self.right = self.env_f = self.env_r = INF; self.sec = ()                 # navstd

_ANG = {}
def _angles(n, a0, inc):
    k = (n, round(a0, 6), round(inc, 8))
    if k not in _ANG:
        a = [_wrap(a0 + i * inc) for i in range(n)]
        cs = [(math.cos(x), math.sin(x)) for x in a]
        run = sorted((x, i) for i, x in enumerate(a) if abs(x) <= BLIND_RUN_CONE)
        sec = [int((x + math.pi) / (2 * math.pi / N_SECT)) % N_SECT for x in a]          # navstd: sector per ray
        sn = [0] * N_SECT
        for q in sec: sn[q] += 1
        _ANG.clear(); _ANG[k] = (a, cs, [i for _x, i in run], sec, sn)
    return _ANG[k]

def lidar_analyze(ranges, a0, inc, rmin, rmax, t=None):
    """Corridor clearances from one scan. Invalid = inf / NaN / 0 / < range_min / > range_max (dropout)."""
    v = LidarView(time.time() if t is None else t); n = len(ranges); v.n = n
    if not n: v.blind_f = v.blind_r = 1.0; return v           # 2026-11-17 allguard: empty scan = blind, never 'clear'
    ang, cs, run_idx, sec_i, s_n = _angles(n, a0, inc); lo = max(rmin, 0.02)
    fi = ft = ri = rt = 0; fwd = rev = sw = INF; ok = [False] * n
    for i in range(n):
        r = ranges[i]
        ok[i] = r == r and lo <= r <= rmax                         # r == r: not NaN
    # 2026-11-23 blindfix: impute near-flanked dropout runs (circular scan) with the nearer valid neighbour's range
    eff = [ranges[i] if ok[i] else None for i in range(n)]
    if any(ok) and not all(ok):
        start = next(i for i in range(n) if ok[i]); i = 1
        while i <= n:
            j = (start + i) % n
            if ok[j]: i += 1; continue
            k = i
            while not ok[(start + k) % n]: k += 1
            run = [(start + m) % n for m in range(i, k)]
            prv = ranges[(start + i - 1) % n]; nxt = ranges[(start + k) % n]
            near = min(prv, nxt)
            if near < NEAR_CTX and len(run) <= NEAR_RUN_MAX:
                for m in run: eff[m] = near
                v.imputed += len(run)
            i = k
    pxs = []; pys = []; bad_a = []; fwd_a = rev_a = None
    left = right = env_f = env_r = INF                                 # navstd: body-envelope gaps (m)
    s_min = [INF] * N_SECT; s_ok = [0] * N_SECT; sw_ = 2 * math.pi / N_SECT
    for i in range(n):
        r = eff[i]; a = ang[i]; good = ok[i]
        aa = abs(a)
        if aa <= BLIND_CONE: ft += 1; fi += not good
        elif aa >= math.pi - BLIND_CONE: rt += 1; ri += not good
        if not good:
            if aa <= BLIND_CONE or aa >= math.pi - BLIND_CONE: bad_a.append(a)
            if aa <= BLIND_RUN_CONE: v.core_bad_f += 1
            elif aa >= math.pi - BLIND_RUN_CONE: v.core_bad_r += 1
        if r is None: continue
        k = sec_i[i]; s_ok[k] += 1                                      # navstd: sector stats (imputed rays count)
        if r < s_min[k]: s_min[k] = r
        if r < sw: sw = r
        x = r * cs[i][0]; y = r * cs[i][1]
        if r <= 1.2: pxs.append(x); pys.append(y)                 # arcfix: nearby points for the swept-path check
        ay = y if y >= 0 else -y
        if r <= ENV_RANGE:                                             # navstd: gap to the real body outline
            dx = max(x - BODY_FRONT, -BODY_REAR - x, 0.0); dy = max(ay - BODY_HALF_W, 0.0)
            g = dy if dx == 0.0 else (dx if dy == 0.0 else math.hypot(dx, dy))
            if dx == 0.0:                                              # alongside the tracks
                if y > 0:
                    if g < left: left = g
                elif g < right: right = g
            if x > 0:
                if g < env_f: env_f = g
            elif g < env_r: env_r = g
        if ay <= HALF_W:
            # navstd: a point ALONGSIDE the body in the side margin (BODY_HALF_W < |y| <= HALF_W, x within the body
            # length) is not 'ahead' or 'behind' - straight motion never closes on it. It is handled by the side zone,
            # so Wally is not deadlocked beside a wall. Points inside the body width still block (contact / self).
            if x > BODY_FRONT or (x > 0 and ay <= BODY_HALF_W):
                if x < fwd: fwd = x; fwd_a = a
            elif x < -BODY_REAR or (x <= 0 and ay <= BODY_HALF_W):
                if -x < rev: rev = -x; rev_a = a
    v.px = pxs; v.py = pys; v.bad_a = bad_a; v.fwd_a = fwd_a; v.rev_a = rev_a
    v.left, v.right, v.env_f, v.env_r = left, right, env_f, env_r
    v.sec = tuple((_wrap(-math.pi + (k + 0.5) * sw_), s_min[k], (s_ok[k] / s_n[k]) if s_n[k] else 0.0) for k in range(N_SECT))
    best = cur = 0
    for i in run_idx:
        cur = cur + 1 if not ok[i] else 0
        if cur > best: best = cur
    v.fwd, v.rev, v.sweep = fwd, rev, sw
    v.blind_f = fi / ft if ft else 1.0; v.blind_r = ri / rt if rt else 1.0; v.blind_run = best
    return v

def arc_clearance(px, py, lin, ang, horizon, step=ARC_STEP):
    """2026-11-23 arcfix (fix 2): sweep the robot footprint (x -BODY_REAR..+BODY_FRONT, |y| <= HALF_W incl. side margin)
    along the constant-curvature path of (lin, ang) and return the clearance in the SAME units as the straight corridor
    (path length to first contact + BODY_FRONT forward / + BODY_REAR reverse); INF if nothing is hit within `horizon` m.
    Straight motion gives exactly the corridor distance, so the stop distances keep their meaning."""
    if not lin or not len(px): return INF
    import numpy as np
    sgn = 1.0 if lin > 0 else -1.0
    k = float(ang) / abs(float(lin))                       # yaw change per metre of travel
    s = np.arange(0.0, max(step, horizon) + 1e-9, step)
    th = k * s
    if abs(k) < 1e-6: X = sgn * s; Y = np.zeros_like(s)
    else: X = sgn * np.sin(th) / k; Y = sgn * (1.0 - np.cos(th)) / k
    xs = np.asarray(px, dtype=float)[None, :]; ys = np.asarray(py, dtype=float)[None, :]
    c = np.cos(th)[:, None]; sn = np.sin(th)[:, None]
    dx = xs - X[:, None]; dy = ys - Y[:, None]
    xr = c * dx + sn * dy; yr = -sn * dx + c * dy
    hit = ((xr <= BODY_FRONT) & (xr >= -BODY_REAR) & (np.abs(yr) <= HALF_W)).any(axis=1)
    if not hit.any(): return INF
    return float(s[int(np.argmax(hit))]) + (BODY_FRONT if sgn > 0 else BODY_REAR)

# ------------------------------------------------------------------ depth camera
class DepthView:
    __slots__ = ("t", "fwd", "valid_frac", "pts")
    def __init__(self, t=0.0): self.t = t; self.fwd = INF; self.valid_frac = 0.0; self.pts = 0

_GRID = {}
def decode_depth(encoding, data, height, width, step=0, is_bigendian=0):
    """Raw depth image. Honour the ROS row step and endian flag. No unit is applied."""
    enc = (encoding or "").strip()
    if enc in ("16UC1", "mono16"):
        item, kind = 2, "u2"
    elif enc == "32FC1":
        item, kind = 4, "f4"
    else:
        return None
    raw = data if isinstance(data, (bytes, bytearray)) else bytes(data or b"")
    try:
        height, width = int(height), int(width)
    except (TypeError, ValueError):
        return None
    if height <= 0 or width <= 0:
        return None
    need = width * item
    try:
        stride = int(step or 0)
    except (TypeError, ValueError):
        stride = 0
    if stride < need:
        if len(raw) >= height * need:
            stride = need
        else:
            return None
    if len(raw) < height * stride:
        return None
    import numpy as np
    endian = ">" if int(is_bigendian or 0) else "<"
    buf = np.frombuffer(raw[:height * stride], dtype=np.uint8).reshape(height, stride)
    packed = np.ascontiguousarray(buf[:, :need])
    return packed.view(np.dtype(endian + kind)).reshape(height, width)


def center_raw(image, cx=None, cy=None, radius=2):
    """Positive raw value on the forward optical ray. Not an image-wide median."""
    import numpy as np
    h, w = image.shape[:2]
    try:
        x = (w - 1) / 2.0 if cx is None else float(cx)
        y = (h - 1) / 2.0 if cy is None else float(cy)
    except (TypeError, ValueError):
        return None
    xi = min(max(int(round(x)), 0), w - 1)
    yi = min(max(int(round(y)), 0), h - 1)
    patch = image[max(0, yi - radius):yi + radius + 1, max(0, xi - radius):xi + radius + 1]
    vals = patch[patch > 0]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def lidar_person_range(beams, bearing_rad, calibration):
    """Range of the beam at the person's bearing. Not the forward-corridor minimum.

    calibration must already say validated and include a measured camera-to-lidar
    offset and yaw. This function does not invent those numbers.
    """
    if not isinstance(calibration, dict) or calibration.get("validated") is not True:
        return None
    for key in ("x_m", "y_m", "z_m", "yaw_rad", "measured_at"):
        if calibration.get(key) is None:
            return None
    try:
        target = float(bearing_rad) + float(calibration["yaw_rad"])
    except (TypeError, ValueError):
        return None
    import math
    best = None
    for item in beams or ():
        try:
            ang, rng = float(item[0]), float(item[1])
        except (TypeError, ValueError, IndexError):
            continue
        if rng != rng or rng <= 0.05 or abs(ang - target) > math.radians(2.0):
            continue
        if best is None or rng < best:
            best = rng
    return best


def lidar_forward_m(ranges, angle_min, angle_increment, half_rad=None):
    """Median range in the forward cone. Not the median of the whole scan."""
    import math
    half = math.radians(10) if half_rad is None else float(half_rad)
    try:
        a0 = float(angle_min)
        inc = float(angle_increment)
    except (TypeError, ValueError):
        return None
    vals = []
    for i, r in enumerate(ranges or ()):
        try:
            x = float(r)
        except (TypeError, ValueError):
            continue
        ang = a0 + i * inc
        while ang > math.pi:
            ang -= 2 * math.pi
        while ang < -math.pi:
            ang += 2 * math.pi
        if abs(ang) > half or x != x or not (0.05 < x < 30.0):
            continue
        vals.append(x)
    if len(vals) < 3:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def resolve_depth_scale(encoding, raw_samples=None, lidar_m=None, driver_scale=None, forward_raw=None, camera_ahead_m=None, tf_fresh=False):
    """Metres per raw depth unit, or (None, reason).

    32FC1 is metres by the ROS sensor_msgs convention. 16UC1 / mono16 is not
    given a unit from the encoding, from a filter range, or from how large the
    integers look. A scale is accepted only from an explicit driver calibration
    value, or when the forward-ray sample agrees with the LiDAR range on that
    same bearing. A scene-wide median is not that ray. A camera offset is
    applied only when tf_fresh says the offset was measured. A frame that does
    not agree stays unconfirmed and must not be used for ranging.
    """
    enc = (encoding or "").strip()
    if driver_scale is not None:
        try:
            s = float(driver_scale)
        except (TypeError, ValueError):
            s = 0.0
        if s > 0:
            return s, "driver"
    if enc == "32FC1":
        return 1.0, "32FC1_metres"
    if enc not in ("16UC1", "mono16"):
        return None, "unsupported_encoding"
    try:
        ref = float(lidar_m)
    except (TypeError, ValueError):
        return None, "need_lidar_crosscheck"
    # A chassis drawing is not a measured transform. Subtract an offset only when
    # the caller supplies one from a fresh TF. Otherwise compare the forward ray as-is.
    ahead = 0.0
    if tf_fresh and camera_ahead_m is not None:
        try:
            ahead = float(camera_ahead_m)
        except (TypeError, ValueError):
            return None, "tf_offset_unusable"
    expected = ref - ahead
    if expected != expected or not (0.30 <= expected <= 6.0):
        return None, "lidar_range_unusable"
    sample = None
    if forward_raw is not None:
        try:
            sample = float(forward_raw)
        except (TypeError, ValueError):
            sample = None
    else:
        vals = []
        for v in raw_samples or ():
            try:
                x = float(v)
            except (TypeError, ValueError):
                continue
            if x > 0 and x == x:
                vals.append(x)
        if 8 <= len(vals) <= 64:
            vals.sort()
            sample = vals[len(vals) // 2]
    if sample is None or not (sample > 0 and sample == sample):
        return None, "need_forward_ray"
    for scale, name in ((0.001, "crosscheck_mm"), (1.0, "crosscheck_m")):
        if abs(sample * scale - expected) / expected <= 0.30:
            return scale, name
    return None, "crosscheck_disagree"


def depth_return_valid(z_m, zmin=None, zmax=None):
    """True only for an Aurora axial-Z sample inside the 0.30-3.00 m working window."""
    zmin = DEPTH_MIN_VALID_M if zmin is None else zmin
    zmax = DEPTH_MAX_VALID_M if zmax is None else zmax
    try:
        z = float(z_m)
    except (TypeError, ValueError):
        return False
    return z == z and zmin <= z <= zmax


def depth_analyze(img_mm, fx, fy, cx, cy, t=None, step=None, scale_m=None):
    """Downsampled corridor distance from the LiDAR centre.

    scale_m is metres per raw unit. None leaves the view invalid: a mono16
    buffer is not assumed to be millimetres. Synthetic tests that build
    millimetre images pass scale_m=0.001 themselves.
    """
    import numpy as np
    step = step or DEPTH_STEP
    d = DepthView(time.time() if t is None else t)
    try:
        scale = float(scale_m)
    except (TypeError, ValueError):
        scale = 0.0
    if not (scale > 0):
        return d
    s = img_mm[::step, ::step]
    h, w = s.shape
    key = (h, w, step, fx, fy, cx, cy)
    if key not in _GRID:
        vv, uu = np.mgrid[0:h, 0:w]
        _GRID.clear(); _GRID[key] = (((uu * step) - cx) / fx, ((vv * step) - cy) / fy)
    ku, kv = _GRID[key]
    Z = s.astype(np.float32) * scale
    # 2026-11-26: discard Aurora blind-zone and out-of-range returns (LiDAR owns < 0.30 m).
    # DEPTH_MAX_VALID_M (3.00 m) is the camera working far limit; HEIGHT filters still reject the floor.
    valid = (Z >= DEPTH_MIN_VALID_M) & (Z <= DEPTH_MAX_VALID_M)
    Y = kv * Z; X = ku * Z
    hgt = CAM_Z - Y; xr = Z + CAM_X; yr = -X
    roi = valid & (np.abs(yr) <= HALF_W)
    obs = roi & (hgt >= DEPTH_HMIN) & (hgt <= DEPTH_HMAX)
    d.valid_frac = float(valid.mean()) if valid.size else 0.0
    n = int(obs.sum()); d.pts = n
    if n >= DEPTH_MIN_PTS:                 # nearest cluster of >= 6 samples within 5 cm in x: rejects speckle / flying pixels
        xs = np.sort(xr[obs]); k = DEPTH_MIN_PTS - 1
        hit = np.nonzero(xs[k:] - xs[:-k] <= DEPTH_CLUSTER)[0]
        if len(hit): d.fwd = float(xs[hit[0] + k // 2])
    return d

# ------------------------------------------------------------------ detectors
class Stall:
    """Encoders: tracks commanded to move but /odom_raw says (almost) not -> stall / bump."""
    def __init__(self): self.reset()
    def reset(self): self.since = None; self.low = None; self.sig = (0, 0)
    def update(self, t, cmd_lin, cmd_ang, v, w):
        sig = (0 if abs(cmd_lin) < STALL_MIN_CMD else (1 if cmd_lin > 0 else -1),
               0 if abs(cmd_ang) < STALL_MIN_ANG else (1 if cmd_ang > 0 else -1))
        if sig == (0, 0): self.reset(); return None
        if sig != self.sig: self.sig = sig; self.since = t; self.low = None
        if t - self.since < STALL_GRACE: return None
        lin_low = sig[0] != 0 and abs(v) < STALL_RATIO * abs(cmd_lin)
        ang_low = sig[0] == 0 and sig[1] != 0 and abs(w) < STALL_RATIO * abs(cmd_ang)
        if lin_low or ang_low:
            if self.low is None: self.low = t
            if t - self.low >= STALL_HOLD:
                return ("track stall: commanded %.2f m/s, encoders %.2f m/s" % (cmd_lin, v)) if lin_low else \
                       ("turn stall: commanded %.2f rad/s, encoders %.2f rad/s" % (cmd_ang, w))
        else: self.low = None
        return None

class Slip:
    """Encoders say we move forward but the LiDAR corridor distance does not shrink -> slipping / pushing an object."""
    def __init__(self): self.h = []
    def reset(self): self.h = []
    def update(self, t, moving_fwd, odom_s, fwd):
        if not moving_fwd or not (fwd < SLIP_RANGE): self.h = []; return None
        self.h.append((t, odom_s, fwd)); self.h = [x for x in self.h if t - x[0] <= SLIP_WIN + 0.2]
        t0, s0, f0 = self.h[0]
        if t - t0 < SLIP_WIN: return None
        moved = odom_s - s0
        if moved >= SLIP_MIN_ODOM and (f0 - fwd) < SLIP_FRAC * moved:
            return "no progress: encoders moved %.2f m, obstacle distance changed %.2f m (slipping / pushing)" % (moved, f0 - fwd)
        return None

def clearest_heading(view, exclude_rear=False):
    """navstd: (bearing rad, sector min range m) of the most open 30 deg LiDAR sector. Sectors with < SECT_MIN_FRAC
    real or near-imputed rays are UNKNOWN and never chosen (unknown is not clear). (None, 0.0) if nothing is known."""
    best = (None, 0.0, 9.0)
    for c, mn, fr in (getattr(view, "sec", ()) or ()):
        if fr < SECT_MIN_FRAC: continue
        if exclude_rear and abs(c) > math.radians(120): continue
        m = min(mn, 10.0)
        if m > best[1] + 1e-6 or (abs(m - best[1]) <= 1e-6 and abs(c) < best[2]): best = (c, m, abs(c))
    return best[0], best[1]

def side_open(view):
    """navstd: (left score, right score) = best known sector range on each side (15..105 deg). Unknown = 0."""
    l = r = 0.0
    for c, mn, fr in (getattr(view, "sec", ()) or ()):
        if fr < SECT_MIN_FRAC or not (math.radians(15) <= abs(c) <= math.radians(105)): continue
        if c > 0: l = max(l, min(mn, 10.0))
        else: r = max(r, min(mn, 10.0))
    return l, r

class Progress:
    """navstd: sensor-based progress checker (Nav2 SimpleProgressChecker idea, adapted for slipping tracks: the
    reference is COMMANDED travel, the evidence is LiDAR and depth range to the obstacle in the travel direction -
    /odom_raw on this base is command-derived and cannot see slip). Judged only when an obstacle is within PROG_NEAR,
    so open-room creeping and in-place turns never trigger it."""
    def __init__(self): self.reset()
    def reset(self): self.h = []; self.sig = 0; self.cum = 0.0; self.t = None
    def update(self, t, lin, d_lidar, d_depth):
        sig = 0 if abs(lin) < STALL_MIN_CMD else (1 if lin > 0 else -1)
        if sig == 0 or sig != self.sig: self.reset(); self.sig = sig
        if sig == 0: return None
        if self.t is not None: self.cum += abs(lin) * max(0.0, min(0.2, t - self.t))
        self.t = t
        self.h.append((t, self.cum, d_lidar, d_depth)); self.h = [x for x in self.h if t - x[0] <= PROG_WIN + 0.3]
        t0, c0, l0, z0 = self.h[0]
        if t - t0 < PROG_WIN: return None
        moved = self.cum - c0
        if moved < PROG_MIN_CMD: return None
        ev = []
        for name, a, b in (("lidar", l0, d_lidar), ("depth", z0, d_depth)):
            if a is None or b is None or not (math.isfinite(a) and math.isfinite(b)) or a >= PROG_NEAR: continue
            ev.append((name, a - b, b))
        if not ev: return None                                   # no near obstacle evidence -> cannot judge
        if any(c >= PROG_FRAC * moved for _n, c, _b in ev): return None
        near = min(b for _n, _c, b in ev); where = "ahead" if sig > 0 else "behind"
        self.h = [self.h[-1]]                                    # one event per window
        return ("no progress %s: commanded %.2f m in %.1fs, %s (obstacle %.2fm %s - slipping / pushing / wedged)"
                % ("forward" if sig > 0 else "reversing", moved, PROG_WIN,
                   ", ".join("%s closed %.2f m" % (n, c) for n, c, _b in ev), near, where))

class TurnProgress:
    """navstd: in-place turn commanded but the IMU gyro shows (almost) no yaw rate -> turn stall (independent of the
    command-derived odometry). Arms only after the gyro has proven itself (>= GYRO_PROOF rad/s seen once)."""
    def __init__(self): self.proven = False; self.reset()
    def reset(self): self.since = None; self.h = []; self.sig = 0
    def update(self, t, lin, ang, wz):
        if wz is None: return None
        if abs(wz) >= GYRO_PROOF: self.proven = True
        sig = 0 if (abs(ang) < STALL_MIN_ANG or abs(lin) >= STALL_MIN_CMD) else (1 if ang > 0 else -1)
        if sig == 0 or sig != self.sig: self.reset(); self.sig = sig
        if sig == 0 or not self.proven: return None
        if self.since is None: self.since = t
        self.h.append((t, abs(wz))); self.h = [x for x in self.h if t - x[0] <= TURN_WIN]
        if t - self.since < TURN_GRACE + TURN_WIN or len(self.h) < 3: return None
        m = sum(x[1] for x in self.h) / len(self.h)
        if m < TURN_FRAC * abs(ang):
            self.since = t; self.h = []
            return "turn stall (imu gyro): commanded %.2f rad/s, measured %.2f rad/s for %.1fs" % (ang, m, TURN_WIN)
        return None

class Imu:
    """IMU events (2026-11-05 live fix):
    * jolt: high-passed |a_xy| > JOLT while moving (2 samples)         -> 'jolt'      stop + back off (gate)
    * tilt: pitch OR roll (or combined) >= 15 and <= 25 deg, low-passed, 0.3 s -> 'tilt' stop, NO back-off
    * hard: pitch OR roll > 25 deg, or combined tilt > 25 deg, 0.2 s    -> 'tilt_hard' zero motors + latch ESTOP, NO back-off
    Axes: IMU mounted yaw -90 deg, so robot pitch = atan2(a_y, a_z), roll = atan2(-a_x, a_z) (thresholds are symmetric)."""
    def __init__(self):
        self.bx = self.by = None; self.n = 0; self.tilt_since = None; self.hard_since = None; self.last = None
        self.g = None; self.kind = None
    def update(self, t, ax, ay, az, moving):
        if self.bx is None: self.bx, self.by = ax, ay
        hp = math.hypot(ax - self.bx, ay - self.by)
        self.bx += 0.08 * (ax - self.bx); self.by += 0.08 * (ay - self.by)        # ~0.25 s baseline at 48 Hz
        if self.g is None: self.g = [ax, ay, az]
        else:
            self.g[0] += TILT_LPF * (ax - self.g[0]); self.g[1] += TILT_LPF * (ay - self.g[1]); self.g[2] += TILT_LPF * (az - self.g[2])
        gx, gy, gz = self.g
        if gz > 0.5:
            pitch = math.degrees(math.atan2(gy, gz)) - PITCH_OFF; roll = math.degrees(math.atan2(-gx, gz)) - ROLL_OFF
            if PITCH_OFF or ROLL_OFF:   # combined tilt from the level-corrected pitch/roll
                tot = math.degrees(math.atan(math.hypot(math.tan(math.radians(max(-89.0, min(89.0, pitch)))), math.tan(math.radians(max(-89.0, min(89.0, roll)))))))
            else:
                tot = math.degrees(math.atan2(math.hypot(gx, gy), gz))
        else:
            pitch = roll = tot = 90.0                                              # on its side / upside down
        axis = max(abs(pitch), abs(roll))
        ev = None; kind = None
        if moving and hp > JOLT:
            self.n += 1
            if self.n >= 2: ev, kind = "jolt %.1f m/s2 (bump / sudden stop)" % hp, "jolt"
        else: self.n = 0
        if axis >= TILT_DEG or tot >= TILT_DEG:
            if self.tilt_since is None: self.tilt_since = t
            if t - self.tilt_since >= TILT_HOLD:
                ev, kind = "tilted pitch %.0f / roll %.0f deg (climbing / stuck on something)" % (pitch, roll), "tilt"
        else: self.tilt_since = None
        if axis > TILT_HARD or tot > TILT_HARD:
            if self.hard_since is None: self.hard_since = t
            if t - self.hard_since >= TILT_HARD_HOLD:
                ev, kind = "tipping: pitch %.0f / roll %.0f deg (over %.0f deg)" % (pitch, roll, TILT_HARD), "tilt_hard"
        else: self.hard_since = None
        self.kind = kind; self.last = (t, hp, tot, pitch, roll)
        return ev

# ------------------------------------------------------------------ fusion
def approach_memory_expired(t0, now, window=None):
    """True once the 2 s approach-memory window has elapsed (fresh hit required after this)."""
    window = LMEM_S if window is None else window
    try:
        return (float(now) - float(t0)) > float(window)
    except (TypeError, ValueError):
        return True


def scan_clears_approach_memory(view, rear=False, stop=None, hyst=None, far=None):
    """True when the current LiDAR scan clearly sees free space beyond the approach band.

    Memory is planted when clearance is < SLOW_FULL, so a 'free' scan must also be at least
    SLOW_FULL plus hysteresis. A dropout / non-finite clearance does not clear memory (the
    original near object may have gone NaN up close). `stop` is accepted as an alias for `far`.
    """
    if view is None or getattr(view, "n", 0) == 0:
        return False
    if far is None:
        far = stop if stop is not None else SLOW_FULL
    hyst = LMEM_CLEAR_HYST if hyst is None else hyst
    d = view.rev if rear else view.fwd
    blind = view.blind_r if rear else view.blind_f
    if not math.isfinite(d):
        return False
    if blind >= BLIND_FRAC_STOP:
        return False
    return d >= (far + hyst)


class Fusion:
    def __init__(self):
        self.lidar = None; self.depth = None
        self.odom_t = 0.0; self.v = self.w = 0.0; self.s = 0.0; self.pose = None; self.prev_pose = None
        self.imu_t = 0.0; self.imu = Imu(); self.stall = Stall(); self.slip = Slip()
        self.rgb = None; self.mem = None; self.cmd = (0.0, 0.0); self.events = []
        self.lmem_f = self.lmem_r = None                          # 2026-11-23 blindfix: LiDAR approach memory
        self.prog = Progress(); self.tprog = TurnProgress(); self.wz = None   # navstd
    # ---- inputs
    def on_lidar(self, view):
        self.lidar = view
        yaw = self.pose[2] if self.pose is not None else None
        if view.fwd < SLOW_FULL: self.lmem_f = (view.t, view.fwd, self.s, yaw, view.fwd_a)
        elif scan_clears_approach_memory(view, rear=False):
            self.lmem_f = None
        if view.rev < SLOW_FULL: self.lmem_r = (view.t, view.rev, self.s, yaw, view.rev_a)
        elif scan_clears_approach_memory(view, rear=True):
            self.lmem_r = None
    def _drop_lmem(self, rear=False):
        if rear: self.lmem_r = None
        else: self.lmem_f = None
    def lidar_mem(self, now, rear=False):
        """2026-11-23 blindfix + 2026-11-26 decay: a corridor obstacle seen < SLOW_FULL in the last LMEM_S s,
        minus the distance travelled toward it, while the current scan has dropouts in that direction's core cone.
        Memory is a decaying observation, never a latch: it is dropped after 2 s, or immediately when a fresh
        LiDAR scan is clearly free on that side (beyond stop + hysteresis). A later dropout cannot revive it."""
        L = self.lidar; m = self.lmem_r if rear else self.lmem_f
        if m is None or L is None: return INF
        t0, d0, s0, y0, a0 = m
        if approach_memory_expired(t0, now):
            self._drop_lmem(rear); return INF
        dyaw = _wrap(self.pose[2] - y0) if (y0 is not None and self.pose is not None) else 0.0
        if abs(dyaw) > MEM_YAW:
            self._drop_lmem(rear); return INF
        if scan_clears_approach_memory(L, rear=rear):
            self._drop_lmem(rear); return INF
        if a0 is None: return INF
        want = _wrap(a0 - dyaw)                                      # where that obstacle should be now
        if not any(abs(_wrap(b - want)) <= math.radians(10) for b in L.bad_a): return INF   # it's NaN there now
        trav = (s0 - self.s) if rear else (self.s - s0)
        return d0 - max(0.0, trav)
    def on_depth(self, view):
        self.depth = view
        if view.fwd < INF and self.pose is not None:              # remember low obstacles for when they leave the FOV
            self.mem = (view.t, view.fwd, self.pose)
        elif view.fwd == INF and view.valid_frac >= 0.3 and self.mem_dist(view.t) > MEM_CLEAR:
            self.mem = None                                       # still well inside the camera's view and gone: forget
    def on_odom(self, t, v, w, x=None, y=None, yaw=None):
        self.odom_t = t; self.v, self.w = v, w
        if x is not None:
            if self.pose is not None: self.s += math.hypot(x - self.pose[0], y - self.pose[1]) * (1 if v >= 0 else -1)
            self.pose = (x, y, yaw)
        ev = self.stall.update(t, self.cmd[0], self.cmd[1], v, w)
        if ev: self.events.append((t, "encoders", ev))
    def on_imu(self, t, ax, ay, az, wz=None):
        self.imu_t = t
        if wz is not None and wz == wz:                           # navstd: gyro yaw rate for the turn-stall check
            self.wz = wz
            evt = self.tprog.update(t, self.cmd[0], self.cmd[1], wz)
            if evt: self.events.append((t, "imu-gyro", evt))
        ev = self.imu.update(t, ax, ay, az, abs(self.cmd[0]) >= STALL_MIN_CMD or abs(self.cmd[1]) >= STALL_MIN_ANG)
        if ev: self.events.append((t, {"tilt": "imu-tilt", "tilt_hard": "imu-tilt-hard"}.get(self.imu.kind, "imu"), ev))
        if len(self.events) > 50: self.events = [e for e in self.events if t - e[0] <= 1.0]   # bounded while idle
    def on_rgb(self, d): self.rgb = d
    def on_cmd(self, t, lin, ang):
        self.cmd = (lin, ang)
        if self.lidar is not None:
            ev = self.slip.update(t, lin >= STALL_MIN_CMD, self.s, self.lidar.fwd)
            if ev: self.events.append((t, "lidar+encoders", ev))
            dz = self.depth.fwd if (lin > 0 and self.depth is not None and t - self.depth.t <= DEPTH_STALE) else None
            ev = self.prog.update(t, lin, self.lidar.fwd if lin > 0 else self.lidar.rev, dz)   # navstd
            if ev: self.events.append((t, "lidar+depth progress", ev))
    def reset_detectors(self):
        self.stall.reset(); self.slip.reset(); self.prog.reset(); self.tprog.reset(); self.events = []
    def pop_event(self, now=None):
        """First collision event of the last 0.5 s (older ones are from a previous move and are dropped)."""
        now = time.time() if now is None else now
        rec = [e for e in self.events if now - e[0] <= 0.5]; self.events = []
        return next((e for e in rec if e[1] == "imu-tilt-hard"), None) or (rec[0] if rec else None)
    def snapshot(self, now=None):
        """navstd: the FULL fused sensor picture (for the gate status JSON, the explore planner and the voice AI)."""
        now = time.time() if now is None else now; fr = self.fresh(now); L = self.lidar; D = self.depth
        def r3(x): return round(x, 3) if (x is not None and math.isfinite(x)) else None
        d = {"fresh": fr, "front_fused": r3(self.front(now)[0]), "front_src": self.front(now)[1]}
        if L is not None:
            ch, cm = clearest_heading(L); lo, ro = side_open(L)
            d["lidar"] = {"fwd": r3(L.fwd), "rev": r3(L.rev), "sweep": r3(L.sweep), "left_gap": r3(L.left),
                          "right_gap": r3(L.right), "env_front": r3(L.env_f), "env_rear": r3(L.env_r),
                          "blind_f": round(L.blind_f, 2), "blind_r": round(L.blind_r, 2), "imputed": L.imputed,
                          "clearest_deg": round(math.degrees(ch)) if ch is not None else None, "clearest_m": r3(cm),
                          "open_left": r3(lo), "open_right": r3(ro), "mem_fwd": r3(self.lidar_mem(now)),
                          "mem_rev": r3(self.lidar_mem(now, rear=True))}
        d["depth"] = {"fwd": r3(D.fwd) if D else None, "valid": round(D.valid_frac, 2) if D else None,
                      "mem": r3(self.mem_dist(now))}
        il = self.imu.last
        d["imu"] = {"tilt_deg": round(il[2], 1) if il else None, "pitch": round(il[3], 1) if il else None,
                    "roll": round(il[4], 1) if il else None, "wz": r3(self.wz), "gyro_proven": self.tprog.proven}
        d["odom"] = {"v": r3(self.v), "w": r3(self.w)}
        rg = self.rgb if fr["rgb"] else None
        d["rgb"] = {"hazard": bool(rg and rg.get("hazard")), "label": (rg or {}).get("label")}
        return d
    # ---- state
    def mem_dist(self, now):
        if not self.mem or self.pose is None: return INF
        t0, d0, p0 = self.mem
        if now - t0 > MEM_S or abs(_wrap(self.pose[2] - p0[2])) > MEM_YAW: self.mem = None; return INF
        h = p0[2]; trav = (self.pose[0] - p0[0]) * math.cos(h) + (self.pose[1] - p0[1]) * math.sin(h)
        return d0 - trav
    def fresh(self, now):
        L = self.lidar
        return {"lidar": L is not None and now - L.t <= LIDAR_STALE,
                "depth": self.depth is not None and now - self.depth.t <= DEPTH_STALE,
                "odom": now - self.odom_t <= ODOM_STALE, "imu": now - self.imu_t <= IMU_STALE,
                "rgb": bool(self.rgb) and now - float(self.rgb.get("t", 0)) <= RGB_STALE}
    def front(self, now=None):
        """(fused forward clearance, which sensor). Camera depth + memory are first-class; smallest trusted wins."""
        now = time.time() if now is None else now; fr = self.fresh(now)
        c = []
        if fr["depth"]: c.append((self.depth.fwd, "depth"))          # primary for low/overhanging obstacles
        c.append((self.mem_dist(now), "depth-memory"))
        if self.lidar is not None and now - self.lidar.t <= LIDAR_STALE:
            c.append((self.lidar.fwd, "lidar")); c.append((self.lidar_mem(now), "lidar-memory"))
        return min(c) if c else (INF, "none")
    def limit(self, lin, ang, now=None, vmax=0.12, stop_fwd=STOP_FWD, stop_rev=STOP_REV, sweep_r=SWEEP_R):
        """-> (lin, ang, why, kind). kind: None (ok), 'slow' (capped, why = reason), 'block' / 'stale' / 'blind'
        (motion refused; the gate trips). Never returns a speed above the command or vmax."""
        now = time.time() if now is None else now; fr = self.fresh(now)
        if not fr["odom"] and (lin or ang):
            return 0.0, 0.0, "wheel encoders stale %.1fs" % (now - self.odom_t), "stale"
        L = self.lidar; why = None; kind = None; caps = []
        depth_ok = fr["depth"] and self.depth is not None and self.depth.valid_frac >= 0.25
        # Camera-first: if LiDAR is missing/stale but depth is fresh, still allow SLOW forward (no reverse) using depth alone.
        if not fr["lidar"]:
            if not depth_ok:
                return 0.0, 0.0, "LiDAR stale %.1fs and no depth" % (now - self.lidar.t if self.lidar else -1), "stale"
            d = min(self.depth.fwd, self.mem_dist(now))
            if lin < 0:
                return 0.0, 0.0, "no LiDAR: reverse refused (no rear sense)", "stale"
            if d < stop_fwd:
                return 0.0, 0.0, "obstacle %.2fm ahead (depth-only, stop %.2fm)" % (d, stop_fwd), "block"
            if ang and d < sweep_r:
                return 0.0, 0.0, "obstacle %.2fm (depth) inside turn sweep" % d, "block"
            cap = min(ramp(d, vmax, stop_fwd), V_DEGRADED)
            if lin > cap: lin, why, kind = cap, "slow %.3f m/s (depth-only %.2fm)" % (cap, d), "slow"
            return lin, (0.0 if d < sweep_r + 0.15 else ang), why, kind
        if L.n == 0 and (lin or ang):                                   # 2026-11-17 allguard: empty scan blocks everything
            return 0.0, 0.0, "LiDAR scan empty", "stale"
        if not fr["depth"]: caps.append((V_DEGRADED, "depth camera stale"))
        if not fr["imu"]: caps.append((V_DEGRADED, "IMU stale"))
        if (lin and L is not None):                                       # navstd: all-sides body-envelope zones
            envd = min(L.left, L.right, L.env_f if lin > 0 else L.env_r)
            if envd < ENV_CONTACT: caps.append((V_MIN, "object touching the body envelope (%.2fm) - creep only" % envd))
            elif envd < ENV_SLOW: caps.append((V_SIDE, "object %.2fm from the body (side/corner slow zone)" % envd))
        if fr["rgb"] and self.rgb.get("hazard"): caps.append((V_RGB, "camera sees %s ahead" % self.rgb.get("label", "something")))
        # 2026-11-17 allguard: the TURN check runs first (fused sweep + rear/front blind), so every refusal below can only
        # pass on a turn that already cleared it. Rear blind stops turning too: the rear corners swing widest (0.155 m).
        turn_why = None
        if ang:
            sw = L.sweep
            if fr["depth"]: sw = min(sw, self.depth.fwd)
            sw = min(sw, self.mem_dist(now))
            if sw < sweep_r:
                turn_why = "obstacle %.2fm inside the turn sweep (r %.2fm)" % (sw, sweep_r)
            elif L.blind_r >= BLIND_FRAC_STOP:
                turn_why = "LiDAR blind behind (%.0f%% dropouts) - turn refused" % (100 * L.blind_r)
            elif L.blind_f >= BLIND_FRAC_STOP:                      # 2026-11-23 blindfix: no depth-OK bypass
                turn_why = "LiDAR blind ahead (%.0f%% dropouts) - turn refused" % (100 * L.blind_f)
            if turn_why:
                if not lin: return 0.0, 0.0, turn_why, "block"
                ang = 0.0
        if lin > 0:
            d, src = self.front(now)
            if ang:                                                       # 2026-11-23 arcfix: curved swept path
                da = arc_clearance(L.px, L.py, lin, ang, SLOW_FULL - BODY_FRONT)
                if da < d: d, src = da, "lidar-arc"
            if d < stop_fwd:
                return 0.0, ang, "obstacle %.2fm ahead (%s, stop %.2fm)" % (d, src, stop_fwd), "block"
            if L.blind_f >= BLIND_FRAC_STOP:                              # 2026-11-23 blindfix: no depth-OK bypass
                return 0.0, ang, "LiDAR blind ahead (%.0f%% dropouts)" % (100 * L.blind_f), "blind"
            if L.blind_f >= BLIND_FRAC_SLOW or L.blind_run >= BLIND_RUN_SLOW:
                caps.append((V_BLIND, "LiDAR dropouts ahead (glass / black?)"))
            cap = ramp(d, vmax, stop_fwd); src_why = "%s %.2fm" % (src, d)
            for c, w in caps:
                if c < cap: cap, src_why = c, w
            if lin > cap: lin = cap; why = "slow %.3f m/s (%s)" % (cap, src_why); kind = "slow"
        elif lin < 0:
            d = L.rev; rsrc = "lidar"
            dm = self.lidar_mem(now, rear=True)                           # 2026-11-23 blindfix
            if dm < d: d, rsrc = dm, "lidar-memory"
            if ang:                                                       # 2026-11-23 arcfix
                da = arc_clearance(L.px, L.py, lin, ang, SLOW_FULL - BODY_REAR)
                if da < d: d, rsrc = da, "lidar-arc"
            if d < stop_rev:
                return 0.0, ang, "obstacle %.2fm behind (%s, stop %.2fm)" % (d, rsrc, stop_rev), "block"
            if L.blind_r >= BLIND_FRAC_STOP:                               # 2026-11-17 allguard: was slow-only (V_BLIND)
                return 0.0, ang, "LiDAR blind behind (%.0f%% dropouts) - reverse refused" % (100 * L.blind_r), "blind"
            cap = ramp(d, vmax, stop_rev); src_why = "%s rear %.2fm" % (rsrc, d)
            if L.blind_r >= BLIND_FRAC_SLOW: cap, src_why = min(cap, V_BLIND), "LiDAR dropouts behind"
            for c, w in caps:
                if c < cap: cap, src_why = c, w
            if -lin > cap: lin = -cap; why = "slow %.3f m/s (%s)" % (cap, src_why); kind = "slow"
        if turn_why:
            return lin, 0.0, turn_why, "block"
        return lin, ang, why, kind
