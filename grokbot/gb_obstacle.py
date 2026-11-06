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
DEPTH_STALE = _f("OBS_DEPTH_STALE", 1.0); IMU_STALE = _f("OBS_IMU_STALE", 0.35); RGB_STALE = _f("OBS_RGB_STALE", 2.5)
BLIND_CONE = math.radians(45); BLIND_RUN_CONE = math.radians(30)
BLIND_FRAC_SLOW = 0.25; BLIND_FRAC_STOP = 0.6; BLIND_RUN_SLOW = 4
CAM_X = 0.082; CAM_Z = 0.139               # depth camera relative to the LiDAR centre (x) and the floor (z)
DEPTH_HMIN = _f("OBS_DEPTH_HMIN", 0.05); DEPTH_HMAX = _f("OBS_DEPTH_HMAX", 0.40); DEPTH_ZMAX = _f("OBS_DEPTH_ZMAX", 1.6)
DEPTH_MIN_PTS = int(_f("OBS_DEPTH_MIN_PTS", 6)); DEPTH_STEP = int(_f("OBS_DEPTH_STEP", 8)); DEPTH_CLUSTER = 0.08
MEM_S = 6.0; MEM_YAW = 0.35                # carried low-obstacle memory: 6 s, dropped after a 20 deg turn
MEM_CLEAR = 0.60                           # ...or when depth sees the spot (>0.6 m away, inside the FOV) empty
STALL_MIN_CMD = 0.03; STALL_RATIO = 0.3; STALL_GRACE = 0.8; STALL_HOLD = 0.5
STALL_MIN_ANG = 0.25
SLIP_WIN = 1.2; SLIP_MIN_ODOM = 0.05; SLIP_FRAC = 0.25; SLIP_RANGE = 2.0
JOLT = _f("OBS_JOLT", 7.0); TILT_DEG = _f("OBS_TILT_DEG", 15.0); TILT_HOLD = 0.3  # 15-25 deg: stop, no back-off
TILT_HARD = _f("OBS_TILT_HARD_DEG", 25.0); TILT_HARD_HOLD = 0.2   # pitch OR roll (or combined) > 25 deg -> latch ESTOP
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
    __slots__ = ("t", "fwd", "rev", "sweep", "blind_f", "blind_run", "blind_r", "n")
    def __init__(self, t=0.0):
        self.t = t; self.fwd = self.rev = self.sweep = INF; self.blind_f = self.blind_r = 0.0; self.blind_run = 0; self.n = 0

_ANG = {}
def _angles(n, a0, inc):
    k = (n, round(a0, 6), round(inc, 8))
    if k not in _ANG:
        a = [_wrap(a0 + i * inc) for i in range(n)]
        cs = [(math.cos(x), math.sin(x)) for x in a]
        run = sorted((x, i) for i, x in enumerate(a) if abs(x) <= BLIND_RUN_CONE)
        _ANG.clear(); _ANG[k] = (a, cs, [i for _x, i in run])
    return _ANG[k]

def lidar_analyze(ranges, a0, inc, rmin, rmax, t=None):
    """Corridor clearances from one scan. Invalid = inf / NaN / 0 / < range_min / > range_max (dropout)."""
    v = LidarView(time.time() if t is None else t); n = len(ranges); v.n = n
    if not n: return v
    ang, cs, run_idx = _angles(n, a0, inc); lo = max(rmin, 0.02)
    fi = ft = ri = rt = 0; fwd = rev = sw = INF; ok = [False] * n
    for i in range(n):
        r = ranges[i]; a = ang[i]
        good = r == r and lo <= r <= rmax                          # r == r: not NaN
        ok[i] = good
        aa = abs(a)
        if aa <= BLIND_CONE: ft += 1; fi += not good
        elif aa >= math.pi - BLIND_CONE: rt += 1; ri += not good
        if not good: continue
        if r < sw: sw = r
        x = r * cs[i][0]; y = r * cs[i][1]
        if abs(y) <= HALF_W:
            if x > 0:
                if x < fwd: fwd = x
            elif -x < rev: rev = -x
    best = cur = 0
    for i in run_idx:
        cur = cur + 1 if not ok[i] else 0
        if cur > best: best = cur
    v.fwd, v.rev, v.sweep = fwd, rev, sw
    v.blind_f = fi / ft if ft else 1.0; v.blind_r = ri / rt if rt else 1.0; v.blind_run = best
    return v

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
    valid = (Z > 0.10) & (Z <= DEPTH_ZMAX)
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
            pitch = math.degrees(math.atan2(gy, gz)); roll = math.degrees(math.atan2(-gx, gz))
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
class Fusion:
    def __init__(self):
        self.lidar = None; self.depth = None
        self.odom_t = 0.0; self.v = self.w = 0.0; self.s = 0.0; self.pose = None; self.prev_pose = None
        self.imu_t = 0.0; self.imu = Imu(); self.stall = Stall(); self.slip = Slip()
        self.rgb = None; self.mem = None; self.cmd = (0.0, 0.0); self.events = []
    # ---- inputs
    def on_lidar(self, view): self.lidar = view
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
    def on_imu(self, t, ax, ay, az):
        self.imu_t = t
        ev = self.imu.update(t, ax, ay, az, abs(self.cmd[0]) >= STALL_MIN_CMD or abs(self.cmd[1]) >= STALL_MIN_ANG)
        if ev: self.events.append((t, {"tilt": "imu-tilt", "tilt_hard": "imu-tilt-hard"}.get(self.imu.kind, "imu"), ev))
        if len(self.events) > 50: self.events = [e for e in self.events if t - e[0] <= 1.0]   # bounded while idle
    def on_rgb(self, d): self.rgb = d
    def on_cmd(self, t, lin, ang):
        self.cmd = (lin, ang)
        if self.lidar is not None:
            ev = self.slip.update(t, lin >= STALL_MIN_CMD, self.s, self.lidar.fwd)
            if ev: self.events.append((t, "lidar+encoders", ev))
    def reset_detectors(self):
        self.stall.reset(); self.slip.reset(); self.events = []
    def pop_event(self, now=None):
        """First collision event of the last 0.5 s (older ones are from a previous move and are dropped)."""
        now = time.time() if now is None else now
        rec = [e for e in self.events if now - e[0] <= 0.5]; self.events = []
        return next((e for e in rec if e[1] == "imu-tilt-hard"), None) or (rec[0] if rec else None)
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
        if self.lidar is not None and now - self.lidar.t <= LIDAR_STALE: c.append((self.lidar.fwd, "lidar"))
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
        if not fr["depth"]: caps.append((V_DEGRADED, "depth camera stale"))
        if not fr["imu"]: caps.append((V_DEGRADED, "IMU stale"))
        if fr["rgb"] and self.rgb.get("hazard"): caps.append((V_RGB, "camera sees %s ahead" % self.rgb.get("label", "something")))
        if lin > 0:
            d, src = self.front(now)
            if d < stop_fwd:
                return 0.0, (0.0 if L.sweep < sweep_r else ang), "obstacle %.2fm ahead (%s, stop %.2fm)" % (d, src, stop_fwd), "block"
            if L.blind_f >= BLIND_FRAC_STOP and not depth_ok:
                return 0.0, ang, "LiDAR blind ahead (%.0f%% dropouts) and no depth" % (100 * L.blind_f), "blind"
            if L.blind_f >= BLIND_FRAC_SLOW or L.blind_run >= BLIND_RUN_SLOW:
                caps.append((V_BLIND, "LiDAR dropouts ahead (glass / black?)"))
            cap = ramp(d, vmax, stop_fwd); src_why = "%s %.2fm" % (src, d)
            for c, w in caps:
                if c < cap: cap, src_why = c, w
            if lin > cap: lin = cap; why = "slow %.3f m/s (%s)" % (cap, src_why); kind = "slow"
        elif lin < 0:
            d = L.rev
            if d < stop_rev:
                return 0.0, (0.0 if L.sweep < sweep_r else ang), "obstacle %.2fm behind (stop %.2fm)" % (d, stop_rev), "block"
            cap = ramp(d, vmax, stop_rev); src_why = "lidar rear %.2fm" % d
            if L.blind_r >= BLIND_FRAC_SLOW: cap, src_why = min(cap, V_BLIND), "LiDAR dropouts behind"
            for c, w in caps:
                if c < cap: cap, src_why = c, w
            if -lin > cap: lin = -cap; why = "slow %.3f m/s (%s)" % (cap, src_why); kind = "slow"
        if ang:
            sw = L.sweep
            if fr["depth"]: sw = min(sw, self.depth.fwd)
            sw = min(sw, self.mem_dist(now))
            if sw < sweep_r:
                return lin, 0.0, "obstacle %.2fm inside the turn sweep (r %.2fm)" % (sw, sweep_r), "block"
        return lin, ang, why, kind
