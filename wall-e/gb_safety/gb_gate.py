#!/usr/bin/env python3
"""gb_gate.py - Grok Bot motion gate: the ONE path from Grok Bot to /controller/cmd_vel (runs in the MentorPi container).

Inputs
  /gb/motion_req  std_msgs/String JSON, one-shot requests; the GATE owns the clock (no caller sleeps):
      {"op":"timed","lin":0.08,"ang":0.0,"dur":4,"id":"..","src":"voice"}   dur default 3 s, hard cap 10 s
      {"op":"turn","deg":90,"ang":0.48,"id":"..","src":".."}                 odom-yaw closed loop, cap 10 s
      {"op":"stop","src":".."}                                               cancel everything, zero burst
  /gb/cmd_in      geometry_msgs/Twist stream (missions, follow, Nav2 relay, guard manoeuvres, joystick):
      deadman: no fresh message within GATE_DEADMAN_S (0.5 s) -> zero.  Absolute stream cap GATE_STREAM_CAP_S.
  ESTOP file      /home/ubuntu/shared/gb_safety/ESTOP (mirrored from /home/pi/grokbot/data/ESTOP):
      while present: every request refused, motion cancelled, zero published at 10 Hz.
      2026-11-25 noestop: only in BENCH mode. On the floor (mode != bench) no latch is written or honoured: faults,
      hard tilt and the remote ESTOP button do a normal stop + disarm (see gb_floor.py); a file there is a one-shot stop.
  /dev/input/js0  optional gamepad: left stick Y = linear, right stick X = angular (caps), ESTOP button(s).
Output
  GATE_OUT (default /controller/cmd_vel) at 20 Hz while moving; after ANY stop: zero at 10 Hz for 2 s.
  /gb/gate_status std_msgs/String JSON at 10 Hz (heartbeat for gb_watchdog).
Every state change is logged with wall-clock timestamps to GATE_LOG.
"""
import json, math, os, struct, threading, time, uuid
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu
try:
    import gb_floor                # 2026-11-25 noestop: ground mode never latches (copied here by gb_safety_run.sh)
except Exception:
    gb_floor = None
import gb_obstacle as OB          # 2026-11-05 .bak-obstacle: footprint corridor + sensor fusion + speed ramp (copied here by gb_safety_run.sh)
try:
    # 2026-11-27 fusion stuck-recovery (John): a mission may REQUEST a bounded, camera-verified micro-nudge below the normal
    # stops via a short-lived token; the gate re-verifies it with its OWN LiDAR + depth + RGB before relaxing anything.
    # Import failure = feature off (normal stops only).
    import gb_fusion_recovery as FR
except Exception:
    FR = None

E = os.environ.get
OUT = E("GATE_OUT", "/controller/cmd_vel")
IN_STREAM = E("GATE_IN", "/gb/cmd_in")
REQ = E("GATE_REQ", "/gb/motion_req")
STATUS = E("GATE_STATUS", "/gb/gate_status")
ESTOP = E("GB_ESTOP_FILE", "/home/ubuntu/shared/gb_safety/ESTOP")
MAINT = E("GB_MAINT_FILE", "/home/ubuntu/shared/gb_safety/MAINT")
LOGF = E("GATE_LOG", "/home/ubuntu/shared/gb_safety/gate.log")
DEADMAN = float(E("GATE_DEADMAN_S", "0.5"))  # 2026-11-26: command freshness watchdog, already in the 0.3-0.5 s band
ZERO_HOLD = float(E("GATE_ZERO_HOLD_S", "2.0"))
HZ = 20.0
# 2026-11-26 speedcap (John, GB-48): two separate linear hard caps.
# Manual (Jarvis pad/keyboard, joystick, voice teleop) = 0.288 m/s.
# Autonomous (live MISSION_LEASE or explore/nav sources) = 1.0 m/s.
# Stop distances 0.27/0.28/0.24 m are UNCHANGED.
try:
    import gb_speed_caps as _SC
except Exception:
    _SC = None
MANUAL_MAX_LIN = float(E("GATE_MANUAL_MAX_LIN", "0.288"))
AUTO_MAX_LIN = float(E("GATE_AUTO_MAX_LIN", "1.0"))
MAX_LIN = AUTO_MAX_LIN  # absolute never-exceed; per-command clamps use lin_cap_for()
MAX_ANG = float(E("GATE_MAX_ANG", "0.8"))           # rad/s hard cap
TIMED_DEFAULT = 3.0
TIMED_CAP = 10.0
STREAM_CAP = float(E("GATE_STREAM_CAP_S", "600"))
ODOM_TOPICS = [t for t in E("GATE_ODOM", "/odom,/odom_raw").split(",") if t]
# Direction-aware LiDAR guard (ON whenever a nonzero linear or angular velocity is commanded;
# every source incl. missions/Nav2/joystick; no mission bypass). Disabled while motionless so
# sitting still cannot trip obstacle / stall / tilt / stale-sensor faults. Stop distances UNCHANGED:
#   forward -> front sector +/-30 deg, reverse -> rear sector 180+/-30 deg, turning -> 360 deg footprint sweep radius.
# 2026-11-05: distances are now the FOOTPRINT CORRIDOR (|y| <= 0.15 m) clearance from the LiDAR centre, fused over
# LiDAR + depth camera (+ carried memory), not the old +/-30 deg cone (which missed the track corners / sides).
FRONT_STOP = float(E("GATE_FRONT_STOP", str(OB.STOP_FWD)))    # voice / timed / joystick (was cone 0.33)
REAR_STOP = float(E("GATE_REAR_STOP", str(OB.STOP_REV)))      # (was 0.25)
SWEEP_R = float(E("GATE_SWEEP_R", str(OB.SWEEP_R)))           # (was 0.20)
S_FRONT_STOP = float(E("GATE_STREAM_FRONT_STOP", str(OB.STOP_FWD)))   # missions / Nav2 / follow too (was cone 0.25)
S_REAR_STOP = float(E("GATE_STREAM_REAR_STOP", str(OB.STOP_REV)))     # (was 0.20)
S_SWEEP_R = float(E("GATE_STREAM_SWEEP_R", str(OB.SWEEP_R)))          # (was 0.18)
RGB_FILE = E("GATE_RGB_FILE", "/home/ubuntu/shared/gb_safety/rgb_guard.json")
# 2026-11-23 navstd: fault escalation - FAULT_LIMIT collision events (stall / no-progress / gyro turn stall / jolt /
# tilt) within FAULT_WIN s latch ESTOP exactly like a hard tilt (host latch via ESTOP_REQ; only a human clears it).
# 2026-11-25 benchpersist (John 9:10/9:17 ET): bench-only guard bypass. The host launcher (gb_safety_run.sh gate)
# refreshes BENCH_FILE every 1 s ONLY while /home/pi/grokbot/data/BENCH_GUARD_OFF exists AND data/learn/mode.json
# says mode=bench. Stale (>BENCH_FRESH_S) / missing / expired -> guard back ON. Thresholds below are NOT changed.
BENCH_FILE = E("GATE_BENCH_FILE", "/home/ubuntu/shared/gb_safety/BENCH_GUARD_OFF"); BENCH_FRESH_S = 5.0
# 2026-11-25 noprog: stall-type events (no-progress / slip / turn stall from command odom or gyro) are SOFT: the move is
# stopped (and backed off as before) but they do not count toward FAULT_LIMIT; only SOFT_LIMIT in SOFT_WIN latch.
# Collisions (IMU jolt) and tilt keep the 3-in-60 s rule. Obstacle stops 0.27/0.28/0.24 are unaffected.
SOFT_SENSORS = ("encoders", "lidar+encoders", "lidar+depth progress", "imu-gyro")
SOFT_LIMIT = int(E("GATE_SOFT_LIMIT", "5")); SOFT_WIN = float(E("GATE_SOFT_WIN_S", "120"))
FAULT_LIMIT = int(E("GATE_FAULT_LIMIT", "3")); FAULT_WIN = float(E("GATE_FAULT_WIN_S", "60"))
# 2026-11-25 noestop (John 14:02 ET): when mode != bench (gb_floor.no_latch) fault_latch / tilt_latch / the remote
# ESTOP button do a NORMAL stop (finish -> zero + 10 Hz zero for 2 s, mission abort, disarm, log) and write NO latch
# file. A non-persistent in-memory hold refuses motion until the robot is re-armed ("wake up Wally" rewrites ARMED).
# An ESTOP / MAINT file seen in ground mode is a one-shot stop, never a block (the host launcher archives it).
ARMED_FILE = E("GATE_ARMED_FILE", os.path.join(os.path.dirname(ESTOP), "ARMED"))
LEASE_FILE = E("GB_MISSION_LEASE", os.path.join(os.path.dirname(ESTOP), "MISSION_LEASE"))
EXPLORE_ABORT = E("GATE_EXPLORE_ABORT", "/home/ubuntu/shared/grokbot/explore/abort")
BACKOFF_MARGIN = 0.05                                                 # back-off needs rear >= REAR_STOP + 5 cm, rear not blind
SENSORS_FILE = E("GATE_SENSORS_FILE", "/home/ubuntu/shared/gb_safety/sensors.json")   # full fused picture for the voice AI
BACKOFF_V = 0.05; BACKOFF_S = 1.2                                    # after a bump / stall: back off 6 cm if the rear is clear
SELF_MIN = float(E("GATE_SELF_MIN", "0.10"))         # returns closer than this are the robot's own body
SCAN_STALE = float(E("GATE_SCAN_STALE_S", str(OB.LIDAR_STALE)))   # was 1.0 s; scans arrive at 10 Hz
EVENT = E("GATE_EVENT", "/home/ubuntu/shared/gb_safety/guard_event.json")
# 2026-11-26 tripclear: Jarvis/gb_arm disarm writes this; a newer mtime releases a sticky guard/bump trip.
DISARM_EVENT = E("GATE_DISARM_EVENT", os.path.join(os.path.dirname(ESTOP), "DISARM_EVENT"))
JOY_DEV = E("GATE_JOY_DEV", "/dev/input/js0")
JOY_ON = E("GATE_JOY", "1") == "1"
JOY_LIN_AXIS = int(E("GATE_JOY_LIN_AXIS", "1")); JOY_ANG_AXIS = int(E("GATE_JOY_ANG_AXIS", "2"))
JOY_ESTOP_BTNS = [int(x) for x in E("GATE_JOY_ESTOP_BTNS", "9").split(",") if x.strip()]  # 2026-11-17: START only (USB WirelessGamepad js index 9); was 6,8
JOY_CLEAR_BTNS = [int(x) for x in E("GATE_JOY_CLEAR_BTNS", "4,5").split(",") if x.strip()]  # L1+R1 held together clears a NORMAL remote/voice stop (human action)
JOY_CLEAR_HOLD = float(E("GATE_JOY_CLEAR_HOLD_S", "2.0"))
JOY_LIN = min(MANUAL_MAX_LIN, float(E("GATE_JOY_MAX_LIN", "0.288"))); JOY_ANG = min(MAX_ANG, float(E("GATE_JOY_MAX_ANG", "0.8")))

def _now_s():
    t = time.time(); return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)

def _clamp(v, m):
    try: v = float(v)
    except Exception: return 0.0
    if not math.isfinite(v): return 0.0
    return max(-m, min(m, v))

def lin_cap_for(src="stream", lease_live=False, joystick=False):
    """Linear hard cap for this command: 0.288 manual, 1.0 autonomous."""
    if _SC is not None:
        return _SC.lin_cap_for(src, lease_live=lease_live, joystick=joystick)
    if joystick or str(src or "").strip().lower() == "joystick":
        return MANUAL_MAX_LIN
    if lease_live:
        return AUTO_MAX_LIN
    s = str(src or "").strip().lower()
    key = s.split(":")[0].split("/")[-1] if s else ""
    if key in ("explore", "nav2", "follow", "guard", "mission", "autonomous", "gb_explore", "wander", "nav") or "explore" in s:
        return AUTO_MAX_LIN
    return MANUAL_MAX_LIN


def _wrap(a): return math.atan2(math.sin(a), math.cos(a))

# 2026-11-26 motiononly (John): safety guards are disabled while motionless (zero commanded
# linear and angular velocity, and not in a motion mode). Any nonzero commanded lin or ang
# enables the LiDAR collision/obstacle guards. Stop thresholds stay 0.27 / 0.28 / 0.24 m.
MOTION_EPS_LIN = float(E("GATE_MOTION_EPS_LIN", "1e-4"))
MOTION_EPS_ANG = float(E("GATE_MOTION_EPS_ANG", "1e-4"))
MOTION_MODES = ("timed", "turn", "stream", "bounded")

def commanded_motion(lin, ang):
    """True when a nonzero linear or angular velocity is being commanded."""
    try:
        return abs(float(lin or 0.0)) > MOTION_EPS_LIN or abs(float(ang or 0.0)) > MOTION_EPS_ANG
    except Exception:
        return False

class Gate(Node):
    def __init__(self):
        super().__init__("gb_gate")
        os.makedirs(os.path.dirname(LOGF), exist_ok=True)
        self.logf = open(LOGF, "a", buffering=1)
        self.lock = threading.RLock()
        self.pub = self.create_publisher(Twist, OUT, 10)
        self.st_pub = self.create_publisher(String, STATUS, 10)
        self.create_subscription(String, REQ, self.on_req, 10)
        self.create_subscription(Twist, IN_STREAM, self.on_stream, 10)
        for t in ODOM_TOPICS:
            self.create_subscription(Odometry, t, lambda m, t=t: self.on_odom(m, t), 10)
        self.front = self.rear = self.sweep = float("inf"); self.scan_t = 0.0; self.trips = 0; self.last_trip = ""; self.trip_t = 0.0; self.trip_clear_after = 0.0; self._trip_start_cleared = False
        self.create_subscription(LaserScan, E("GATE_SCAN", "/scan_raw"), self.on_scan, qos_profile_sensor_data)
        self.fz = OB.Fusion(); self.cam_k = None; self.rgb_chk = 0.0; self.slow_why = ""; self.backoff = False
        self.create_subscription(CameraInfo, E("GATE_DEPTH_INFO", "/ascamera/camera_publisher/depth0/camera_info"), self.on_caminfo, qos_profile_sensor_data)
        self.create_subscription(Image, E("GATE_DEPTH", "/ascamera/camera_publisher/depth0/image_raw"), self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, E("GATE_IMU", "/imu"), self.on_imu, qos_profile_sensor_data)
        self.mode = "idle"; self.lin = 0.0; self.ang = 0.0; self.src = ""; self.rid = ""
        self.deadline = 0.0; self.t_start = 0.0; self.t_fresh = 0.0; self.zero_until = 0.0; self.last_zero = 0.0
        self.lease_hold = False; self.lease_hold_t = 0.0   # 2026-11-26: after stop, refuse streams until a fresh lease
        self.turn = None; self.yaw = None; self.yaw_src = None; self.yaw_t = 0.0
        self.ox = 0.0; self.oy = 0.0; self.look = None; self.poses = {}
        self.estop = os.path.exists(ESTOP); self.maint = os.path.exists(MAINT); self.last_why = "start"; self.n_out = 0; self.last_pub = (None, None)
        import collections; self.seen = collections.deque(maxlen=256)
        self.joy = {"present": False, "lin": 0.0, "ang": 0.0, "t": 0.0}
        self.faults = []; self.sens_t = 0.0                                # navstd
        self.floor_hold_t = 0.0; self._floor_sig = None; self._floor_chk = 0.0; self._floor_on = True   # noestop
        self._floor_evt = None
        try: self._floor_evt = self._evt_sig()                          # an event left from before start is not replayed
        except Exception: pass
        self.create_timer(1.0 / HZ, self.tick)
        self.create_timer(0.1, self.status)
        self.log("gate up out=%s in=%s req=%s estop_file=%s deadman=%.2fs hold=%.1fs caps manual_lin=%.3f auto_lin=%.3f ang=%.2f estop=%s"
                 % (OUT, IN_STREAM, REQ, ESTOP, DEADMAN, ZERO_HOLD, MANUAL_MAX_LIN, AUTO_MAX_LIN, MAX_ANG, self.estop))
        self.log("obstacle fusion up: corridor half-width %.2fm stop fwd %.2f rev %.2f sweep %.2f ramp %.2f->manual %.3f / auto %.3f m/s at %.2f-%.2fm; "
                 "lidar+depth+encoders(%s)+imu+rgb; stale lidar>%.1fs enc>%.2fs -> stop, depth>%.1fs imu>%.2fs -> %.2f m/s"
                 % (OB.HALF_W, FRONT_STOP, REAR_STOP, SWEEP_R, OB.V_MIN, MANUAL_MAX_LIN, AUTO_MAX_LIN, FRONT_STOP, OB.SLOW_FULL,
                    E("GATE_ENCODERS", "/odom_raw"), OB.LIDAR_STALE, OB.ODOM_STALE, OB.DEPTH_STALE, OB.IMU_STALE, OB.V_DEGRADED))
        self.log("allguard 2026-11-17: guard runs before the first nonzero output (timed/turn/stream/bounded/backoff); "
                 "rear LiDAR dropouts >= %.0f%% stop reverse and turning; empty scan blocks all; obstacle module %s"
                 % (100 * OB.BLIND_FRAC_STOP, getattr(OB, "__file__", "?")))
        self.zero_until = time.time() + ZERO_HOLD           # start-up: assert zero for 2 s
        if JOY_ON:
            threading.Thread(target=self.joy_loop, daemon=True).start()

    def log(self, msg):
        line = "%s [gate] %s" % (_now_s(), msg)
        try: self.logf.write(line + "\n")
        except Exception: pass
        self.get_logger().info(msg)

    # ------------------------------------------------------------ inputs
    def on_odom(self, m, t):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # Each topic keeps its own pose. A bounded look reads only /odom.
        self.poses[t] = (time.time(), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                         str(m.header.frame_id or ""), str(m.child_frame_id or ""))
        if t == E("GATE_ENCODERS", "/odom_raw"):                 # wheel encoders: stall / bump detection only
            self.ox = m.pose.pose.position.x; self.oy = m.pose.pose.position.y
            self.fz.on_odom(time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z, self.ox, self.oy, yaw)
        if self.yaw_src not in (None, t) and time.time() - self.yaw_t < 0.5:
            return                                        # timed turns keep the first topic that is alive
        self.yaw = yaw
        self.yaw_src = t; self.yaw_t = time.time()

    def _pinned_odom(self, now):
        """Filtered /odom only. A stale or wrong-frame sample does not fall back to /odom_raw."""
        rec = self.poses.get("/odom")
        if not rec:
            return None
        recv, x, y, yaw, frame, child = rec
        import gb_bounded
        if not gb_bounded.fresh_age(now - recv, gb_bounded.ODOM_FRESH_S):
            return None
        if frame not in ("odom", "/odom") or child not in ("base_footprint", "/base_footprint"):
            return None
        return (x, y, yaw, now - recv)

    def on_scan(self, m):
        v = OB.lidar_analyze(m.ranges, m.angle_min, m.angle_increment, max(SELF_MIN, m.range_min), m.range_max, time.time())
        self.fz.on_lidar(v)
        self.front = self.fz.front(v.t)[0]; self.rear, self.sweep, self.scan_t = v.rev, v.sweep, v.t

    def on_caminfo(self, m):
        self.cam_k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(self, m):
        if self.cam_k is None or m.encoding not in ("16UC1", "mono16", "32FC1"): return
        try:
            img = OB.decode_depth(m.encoding, bytes(m.data), m.height, m.width, m.step, getattr(m, "is_bigendian", 0))
            if img is None:
                return
            lidar_m = self.front if self.front is not None and math.isfinite(self.front) else None
            fwd = OB.center_raw(img, self.cam_k[2], self.cam_k[3])
            documented = None
            try:
                import gb_metric
                documented = gb_metric.validated_depth_scale(m.encoding)
            except Exception:
                documented = None
            if documented:
                scale, how = documented
            else:
                scale, how = OB.resolve_depth_scale(m.encoding, None, lidar_m, forward_raw=fwd, tf_fresh=False)
            if not scale:
                self.log_throttle("depth unit unconfirmed (%s, %s); not used for range" % (m.encoding, how), 10.0)
                return
            self.fz.on_depth(OB.depth_analyze(img, *self.cam_k, t=time.time(), scale_m=scale))
            if FR is not None:                                    # 2026-11-27 fusion recovery: compact depth picture
                try:
                    self.fr_cam = FR.depth_summary(img, *self.cam_k, t=time.time(), scale_m=scale)
                except Exception as _fx:
                    self.fr_cam = None
                    self.log_throttle("fusion depth summary failed: %s" % _fx, 10.0)
        except Exception as ex:
            self.log_throttle("depth frame failed: %s" % ex, 10.0)

    def on_imu(self, m):
        a = m.linear_acceleration
        self.fz.on_imu(time.time(), a.x, a.y, a.z, m.angular_velocity.z)   # navstd: gyro yaw rate -> turn-stall check

    def bench_off(self):
        """2026-11-25 benchpersist: True only while the host keeps BENCH_FILE fresh (flag + mode=bench). Bypasses the
        obstacle / depth / LiDAR / sweep stops, slow-downs and stall / no-progress / slip fault counting ONLY.
        ESTOP, MAINT, deadman, MAX_LIN / MAX_ANG caps, timed durations, the hard-tilt latch and arming all stay."""
        now = time.time()
        if now - getattr(self, "_bench_chk", 0.0) < 0.5: return self._bench_on
        self._bench_chk = now; on = False
        try:
            if now - os.stat(BENCH_FILE).st_mtime <= BENCH_FRESH_S:
                d = json.load(open(BENCH_FILE))
                exp = float(d.get("expires") or 0)
                on = d.get("mode") == "bench" and (exp <= 0 or now < exp)
                self._bench_scope = "stall" if d.get("scope") == "stall" else "all"   # navfix: stall-only bench mode
        except Exception:
            on = False
        if on != getattr(self, "_bench_on", False):
            sc = getattr(self, "_bench_scope", "all")
            self.log(("GUARD OFF (bench): obstacle/sensor stops + stall fault counting BYPASSED (ESTOP, deadman, caps, arming kept)"
                      if sc == "all" else "STALL CHECKS OFF (bench, scope=stall): obstacle stops ACTIVE; stall/no-progress/slip "
                      "faults ignored (tracks spin on the stand)") if on else "GUARD ON: bench bypass ended (flag gone/stale/expired or mode not bench)")
        self._bench_on = on
        return on

    def guard(self, lin, ang, stream=False):
        """Returns (lin, ang, why|None): fused LiDAR + depth + encoders + IMU + RGB limit (gb_obstacle.Fusion.limit).
        why is set only when motion is refused (obstacle / stale critical sensor / blind); slow-downs just cap lin.
        Fail-safe: stale LiDAR or encoders block everything, stale depth / IMU cap the speed.
        2026-11-26 motiononly (John): a zero lin+ang command is motionless — guards are disabled and
        this returns the (clamped) zero command with why=None, even if an obstacle is close or a sensor is stale.
        A nonzero commanded linear or angular velocity enables the LiDAR stops (0.27 / 0.28 / 0.24 m)."""
        cap = self._lin_cap(joystick=(getattr(self, "src", "") == "joystick"))
        if not commanded_motion(lin, ang):
            self.slow_why = ""
            return _clamp(lin, cap), _clamp(ang, MAX_ANG), None
        if getattr(self, "bench_off", lambda: False)() and getattr(self, "_bench_scope", "all") == "all":   # 2026-11-25 benchpersist (scope all only)
            self.slow_why = ""; t = time.time()
            if t - self._thr.get("_bench", 0) > 1.0:
                self._thr["_bench"] = t
                self.log("GUARD OFF (bench) applied lin=%+.3f ang=%+.3f front=%.2f rear=%.2f sweep=%.2f"
                         % (lin, ang, self.front, self.rear, self.sweep))
            return _clamp(lin, cap), _clamp(ang, MAX_ANG), None
        fs, rs, sw = (S_FRONT_STOP, S_REAR_STOP, S_SWEEP_R) if stream else (FRONT_STOP, REAR_STOP, SWEEP_R)
        now = time.time()
        if now - self.rgb_chk > 0.2:
            self.rgb_chk = now
            try: self.fz.on_rgb(json.load(open(RGB_FILE)))
            except Exception: pass
        l, a, why, kind = self.fz.limit(lin, ang, now, cap, fs, rs, sw)
        # 2026-11-27 fusion stuck-recovery: ONLY a 'block' (never stale / blind) on a mission stream (never joystick,
        # voice, timed or bounded) with a valid micro-nudge token may be re-checked; normal stops stand otherwise.
        if kind == "block" and stream and globals().get("FR") is not None and getattr(self, "src", "") != "joystick":
            _fn = getattr(self, "_fusion_nudge", None)
            _fr = _fn(lin, ang, now, why) if callable(_fn) else None
            if _fr is not None:
                self.slow_why = _fr[2]
                return _fr[0], _fr[1], None
        if kind == "slow":
            self.slow_why = why
            if now - self._thr.get("_slow", 0) > 1.0: self._thr["_slow"] = now; self.log("SLOW " + why)
            return l, a, None
        self.slow_why = ""
        return l, a, why

    def _fusion_nudge(self, lin, ang, now, why):
        """2026-11-27 fusion stuck-recovery (John). -> (lin, ang, note) or None (keep the normal block).
        Relaxes the 0.27 / 0.24 m stops for ONE command only when: mode stream, armed, live mission lease, not bench,
        a fresh (<= 0.6 s) token from a live gb_explore node issued after the last gate trip matches the command,
        the command is a slow forward / pivot micro-nudge (lin 0 or 0.06-0.07 m/s, |ang| <= 0.30, never reverse), odom
        and IMU are fresh, and gb_fusion_recovery.check_command() passes with the GATE'S OWN LiDAR, depth summary and
        RGB (camera fresh + valid + confirms open floor, LiDAR hits narrow, chassis gap real, hard floor 0.20 / 0.22 m).
        ESTOP / floor stops / deadman / caps / stall + bump detection run exactly as before."""
        _FR = globals().get("FR")
        if _FR is None:
            return None
        try:
            if getattr(self, "mode", "idle") != "stream" or getattr(self, "estop", False) or self.bench_off():
                return None
            if not os.path.exists(globals().get("ARMED_FILE", "")) or not self._lease_live():
                return None
            tok_path = _FR.token_path(globals().get("ESTOP", "/home/ubuntu/shared/gb_safety/ESTOP"))
            tok = _FR.read_token(tok_path)
            ok, r = _FR.token_valid(tok, now, lin, ang, void_before=float(getattr(self, "trip_t", 0.0) or 0.0))
            if not ok:
                if tok:
                    self.log_throttle("FUSION-NUDGE token rejected (%s); normal block stands: %s" % (r, why), 2.0)
                return None
            fr = self.fz.fresh(now)
            if not (fr.get("odom") and fr.get("imu") and fr.get("lidar")):
                self.log_throttle("FUSION-NUDGE refused: odom/IMU/LiDAR not fresh; normal block stands", 2.0)
                return None
            L = self.fz.lidar
            lid = _FR.lidar_input(L)
            rgb = _FR.rgb_input(self.fz.rgb) if fr.get("rgb") else None
            mems = {"lidar_mem": self.fz.lidar_mem(now), "depth_mem": self.fz.mem_dist(now)}
            ok, verdict, reason = _FR.check_command(lin, ang, lid, getattr(self, "fr_cam", None), rgb, now, mems)
            if not ok:
                self.log_throttle("FUSION-NUDGE refused (%s: %s); normal block stands: %s" % (verdict, reason, why), 1.0)
                return None
            l2, a2 = _FR.clamp_cmd(lin, ang)
            self.fr_last = {"t": now, "lin": l2, "ang": a2, "cand": (tok or {}).get("cand"), "reason": reason,
                            "normal_block": why}
            self.log_throttle("FUSION-NUDGE allowed lin=%+.3f ang=%+.3f (%s) - normal block was: %s" % (l2, a2, reason, why), 1.0)
            return l2, a2, "fusion micro-nudge (%s)" % reason
        except Exception as ex:
            try: self.log_throttle("FUSION-NUDGE error %s: normal block stands" % ex, 5.0)
            except Exception: pass
            return None

    def collision(self, src):
        """Encoder stall / LiDAR no-progress / IMU jolt or tilt while moving -> stop, then back off 6 cm if it was a
        forward move and the rear corridor is clear (the back-off is a gate timed move, guarded like any other)."""
        ev = self.fz.pop_event(time.time())
        if not ev: return False
        _t, sensor, why = ev
        # 2026-11-26 motiononly (John): inlined (no new globals) so AST-extracted collision()
        # in existing tests keeps working. Ignore stall / tilt / jolt / sensor events while
        # commanded velocity is zero AND we are not in a motion mode. mode=turn with lin=0 still counts.
        try:
            _ml = abs(float(getattr(self, "lin", 0.0) or 0.0))
            _ma = abs(float(getattr(self, "ang", 0.0) or 0.0))
            _mm = getattr(self, "mode", "idle")
        except Exception:
            _ml, _ma, _mm = 0.0, 0.0, "idle"
        if _ml <= 1e-4 and _ma <= 1e-4 and _mm not in ("timed", "turn", "stream", "bounded"):
            self.log("GUARD OFF (motionless): ignored %s event: %s" % (sensor, why))
            self.fz.reset_detectors(); return False
        if sensor == "imu-tilt-hard":                             # > 25 deg pitch or roll: zero + latch, no back-off
            self.tilt_latch(why, src); self.fz.reset_detectors(); return True
        if getattr(self, "bench_off", lambda: False)():                   # 2026-11-25 benchpersist: no stop, no fault count
            self.log("GUARD OFF (bench, scope=%s): ignored %s event: %s" % (getattr(self, "_bench_scope", "all"), sensor, why))
            self.fz.reset_detectors(); return False
        # 15-25 deg tilt stops with no back-off. Hard tip (>25) already latched above.
        # Skip reverse backoff on any IMU event (jolt or tilt).
        soft = sensor.startswith("imu")  # imu / imu-tilt / imu-gyro: no back-off
        fwd = self.lin > 0 and not soft
        self.trip("bump detected (%s): %s" % (sensor, why), src); self.finish("BUMP (%s): %s" % (sensor, why))
        self.fz.reset_detectors()
        now = time.time()                                                 # navstd: fault escalation (never self-clears)
        _g = globals(); _sl = _g.get("SOFT_LIMIT", 5); _sw = _g.get("SOFT_WIN", 120.0)
        if sensor in _g.get("SOFT_SENSORS", ()):                          # 2026-11-25 noprog: soft stall event
            self.soft = [x for x in getattr(self, "soft", []) if now - x <= _sw] + [now]
            self.log("SOFT stall event %d/%d in %.0fs (not counted as a collision fault): %s" % (len(self.soft), _sl, _sw, why))
            if len(self.soft) >= _sl and not self.estop:
                self.fault_latch("%d repeated stall events in %.0fs (last: %s)" % (len(self.soft), _sw, why), src)
                return True
        else:
            self.faults = [x for x in self.faults if now - x <= FAULT_WIN] + [now]
            if len(self.faults) >= FAULT_LIMIT and not self.estop:
                self.fault_latch("%d collision faults in %.0fs (last: %s)" % (len(self.faults), FAULT_WIN, why), src)
                return True
        L = self.fz.lidar
        if (fwd and not self.backoff and L and L.rev >= REAR_STOP + BACKOFF_MARGIN and L.blind_r < OB.BLIND_FRAC_STOP
                and not self.estop):
            self.backoff = True; now = time.time()
            self.mode = "timed"; self.lin = -BACKOFF_V; self.ang = 0.0; self.src = "backoff"; self.rid = "backoff"
            self.t_start = now; self.deadline = now + BACKOFF_S
            self.log("START backoff lin=%+.3f dur=%.1fs (rear %.2fm)" % (self.lin, BACKOFF_S, self.fz.lidar.rev))
        return True

    def trip(self, why, src):
        t = time.time()
        self.trips += 1; self.last_trip = why
        self.trip_t = t
        self.trip_clear_after = t + ZERO_HOLD
        if t - self._thr.get("_trip", 0) > 1.0:
            self._thr["_trip"] = t
            self.log("GUARD TRIP (%s): %s | front=%.2f rear=%.2f sweep=%.2f" % (src, why, self.front, self.rear, self.sweep))
            try:
                with open(EVENT + ".tmp", "w") as f:
                    json.dump({"t": t, "why": why, "src": src, "front": self.front if math.isfinite(self.front) else None,
                               "rear": self.rear if math.isfinite(self.rear) else None,
                               "sweep": self.sweep if math.isfinite(self.sweep) else None}, f)
                os.replace(EVENT + ".tmp", EVENT)
            except Exception: pass

    def clear_trip(self, reason="idle"):
        """Release a guard/bump trip. These are normal stops, not latches
        (John 2026-11-25: floor faults never permanently latch). Archives
        guard_event.json, clears last_trip, resets stall detectors, and
        refreshes sensors.json so Jarvis/voice drop the sticky state."""
        prev = self.last_trip or ""
        existed = False
        try:
            if os.path.exists(EVENT):
                existed = True
                archived = EVENT + ".cleared-" + time.strftime("%Y%m%d-%H%M%S")
                dest = archived
                n = 0
                while os.path.exists(dest):
                    n += 1
                    dest = archived + "-%d" % n
                os.replace(EVENT, dest)
                self.log("GUARD TRIP archived %s -> %s" % (os.path.basename(EVENT), os.path.basename(dest)))
        except Exception as ex:
            self.log("GUARD TRIP archive failed: %s" % ex)
            try:
                if os.path.exists(EVENT):
                    os.remove(EVENT)
            except Exception:
                pass
        self.last_trip = ""
        self.trip_clear_after = 0.0
        try:
            self.fz.reset_detectors()
        except Exception:
            pass
        if prev or existed:
            self.log("GUARD TRIP cleared (%s): was %s" % (reason, prev or "event-file"))
            try:
                self.sens_t = 0.0
                self.status()
            except Exception:
                pass
        return bool(prev or existed)

    def _trip_clear_reason(self, now):
        """Return a reason string if a sticky guard/bump trip should release now."""
        try:
            has_file = os.path.exists(EVENT)
        except Exception:
            has_file = False
        if not (self.last_trip or has_file):
            return None
        try:
            if os.path.exists(DISARM_EVENT):
                dt = os.path.getmtime(DISARM_EVENT)
                if dt >= (float(getattr(self, "trip_t", 0.0) or 0.0) - 0.05):
                    return "disarm"
        except Exception:
            pass
        if getattr(self, "mode", "idle") == "idle" and not commanded_motion(
                getattr(self, "lin", 0.0), getattr(self, "ang", 0.0)):
            until = float(getattr(self, "trip_clear_after", 0.0) or 0.0)
            zero_until = float(getattr(self, "zero_until", 0.0) or 0.0)
            if now >= until and now >= zero_until:
                return "idle-timeout"
        return None

    def on_req(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            self.log("bad request (not JSON): %r" % msg.data[:120]); return
        op = str(d.get("op", "")).lower(); src = str(d.get("src", "?"))[:40]; rid = str(d.get("id") or uuid.uuid4().hex[:8])
        now = time.time()
        with self.lock:
            if op in ("timed", "turn"):
                if rid in self.seen: return                   # clients repeat each request (lossy first publish); run once
                self.seen.append(rid)
            if op == "stop":
                oid = d.get("only_id")
                if oid and (oid != self.rid or self.mode == "idle"):
                    return                                    # backup stop for an earlier, already finished request
                _rev = getattr(self, "_revoke_mission_lease", None)
                if callable(_rev):
                    _rev("stop request from %s" % src)
                self.finish("stop request from %s" % src); return
            if op == "noop": return
            if self.floor_nolatch():                                      # noestop: files never block on the floor
                if self.floor_held():
                    self.log("REFUSED %s id=%s from %s: stopped (floor normal stop) - say 'wake up Wally' to re-arm" % (op, rid, src)); return
            elif os.path.exists(MAINT):
                self.log("REFUSED %s id=%s from %s: maintenance lock" % (op, rid, src)); return
            if self.estop and not self.floor_nolatch():
                self.log("REFUSED %s id=%s from %s: ESTOP latched" % (op, rid, src)); return
            if op == "timed":
                lin = _clamp(d.get("lin", 0), self._lin_cap(src=src)); ang = _clamp(d.get("ang", 0), MAX_ANG)
                try: dur = float(d.get("dur"))
                except Exception: dur = None
                if dur is None or not math.isfinite(dur) or dur <= 0:
                    dur = TIMED_DEFAULT                       # no / zero / invalid duration -> default, never indefinite
                dur = min(dur, TIMED_CAP)
                if lin == 0 and ang == 0:
                    self.finish("timed request with zero velocity from %s" % src); return
                self.mode = "timed"; self.lin = lin; self.ang = ang; self.src = src; self.rid = rid
                self.t_start = now; self.deadline = now + dur; self.turn = None
                self.log("START timed id=%s src=%s lin=%+.3f ang=%+.3f dur=%.2fs deadline=%s front=%.2f rear=%.2f sweep=%.2f"
                         % (rid, src, lin, ang, dur, time.strftime("%H:%M:%S", time.localtime(self.deadline)),
                            self.front, self.rear, self.sweep))
                _l, _a, why = self.guard(lin, ang)          # 2026-11-17 allguard: guard BEFORE the first nonzero output
                if why:
                    self.trip(why, "timed " + src); self.finish("GUARD (at start): " + why); return
                self.send(_l, _a)
            elif op == "bounded_return":
                if self.mode != "bounded" or self.look is None or not self.look.active:
                    self.log("REFUSED bounded_return id=%s: no look is running" % rid); return
                self.look.start_return()
                self.log("bounded return requested id=%s" % rid)
            elif op == "bounded":
                import gb_bounded
                pinned = self._pinned_odom(now)
                if pinned is None or not gb_bounded.fresh_age(now - self.scan_t, gb_bounded.SCAN_FRESH_S):
                    self.log("REFUSED bounded id=%s: pinned /odom or scan not fresh" % rid); return
                self.look = gb_bounded.BoundedLook()
                try:
                    cap = float(d.get("cap", gb_bounded.CAP_S))
                except (TypeError, ValueError):
                    cap = gb_bounded.CAP_S
                self.look.begin(now, pinned[:3], cap, source="/odom", frame="odom", child="base_footprint",
                                lidar_yaw=gb_bounded.LIDAR_YAW_RAD)
                if not self.look.active:
                    self.look = None
                    self.log("REFUSED bounded id=%s: %s" % (rid, "source")); return
                self.mode = "bounded"; self.src = src; self.rid = rid; self.t_start = now
                self.deadline = now + self.look.cap; self.lin = 0.0; self.ang = 0.0; self.turn = None
                self.log("START bounded id=%s src=%s cap=%.0fs source=/odom home=(%.2f,%.2f)"
                         % (rid, src, self.look.cap, pinned[0], pinned[1]))
            elif op == "turn":
                try: deg = float(d.get("deg", 90))
                except Exception: deg = 90.0
                deg = max(-360.0, min(360.0, deg))
                w = abs(_clamp(d.get("ang", 0.48), MAX_ANG)) or 0.48
                w = max(0.3, w)                               # tracks need ~0.3 rad/s to turn in place
                if self.yaw is None or now - self.yaw_t > 1.0:
                    self.log("REFUSED turn id=%s: no fresh odometry yaw" % rid); return
                tgt = math.radians(deg)
                cap = min(TIMED_CAP, abs(tgt) / w * 1.6 + 2.0)
                self.mode = "turn"; self.lin = 0.0; self.ang = math.copysign(w, tgt); self.src = src; self.rid = rid
                self.t_start = now; self.deadline = now + cap
                self.turn = {"target": tgt, "done": 0.0, "prev": self.yaw, "start_yaw": self.yaw}
                self.log("START turn id=%s src=%s deg=%+.1f w=%.2f cap=%.1fs yaw0=%.3f (%s)"
                         % (rid, src, deg, w, cap, self.yaw, self.yaw_src))
                _l, _a, why = self.guard(0.0, self.ang)     # 2026-11-17 allguard: guard BEFORE the first nonzero output
                if why:
                    self.trip(why, "turn " + src); self.finish("GUARD (at start): " + why); return
                self.send(0.0, _a)
            elif op != "noop":
                self.log("unknown op %r from %s" % (op, src))

    def on_stream(self, m):
        now = time.time()
        with self.lock:
            lin = _clamp(m.linear.x, self._lin_cap(src="stream")); ang = _clamp(m.angular.z, MAX_ANG)
            if self.floor_nolatch():                                      # noestop
                if self.floor_held():
                    if lin or ang: self.log_throttle("REFUSED stream: stopped (floor normal stop) until re-armed")
                    return
            elif os.path.exists(MAINT):
                if lin or ang: self.log_throttle("REFUSED stream: maintenance lock")
                return
            if self.estop and not self.floor_nolatch():
                if lin or ang: self.log_throttle("REFUSED stream: ESTOP latched")
                return
            if self.mode in ("timed", "turn"):
                return                                        # a timed/turn command owns the motors until it ends
            _lease_ok = getattr(self, "_stream_lease_ok", None)
            if callable(_lease_ok) and not _lease_ok(now, joystick=False):
                if lin or ang:
                    self.log_throttle("REFUSED stream: mission lease revoked; waiting for a fresh lease")
                    if self.mode == "stream":
                        self.finish("stream refused: mission lease revoked")
                return
            if lin == 0 and ang == 0:
                if self.mode == "stream": self.finish("stream sent zero")
                return
            if self.mode != "stream":
                self.mode = "stream"; self.t_start = now; self.src = "stream"; self.rid = ""
                self.log("START stream lin=%+.3f ang=%+.3f" % (lin, ang))
            self.lin, self.ang, self.t_fresh = lin, ang, now
            self.deadline = self.t_start + STREAM_CAP

    _thr = {}
    def log_throttle(self, msg, every=2.0):
        t = time.time()
        if t - self._thr.get(msg, 0) > every:
            self._thr[msg] = t; self.log(msg)

    # ------------------------------------------------------------ joystick (optional, through the gate)
    def joy_loop(self):
        import select
        axes = {}; fd = None; btns = {}; clear_t0 = None
        while rclpy.ok():
            try:
                if fd is None:
                    if not os.path.exists(JOY_DEV):
                        self.joy["present"] = False; time.sleep(1.0); continue
                    fd = open(JOY_DEV, "rb", buffering=0); self.joy["present"] = True
                    self.log("joystick connected: %s (estop buttons %s)" % (JOY_DEV, JOY_ESTOP_BTNS))
                if clear_t0 is not None and time.time() - clear_t0 >= JOY_CLEAR_HOLD:
                    clear_t0 = None
                    if self.estop:
                        self._write_latch(os.path.join(os.path.dirname(ESTOP), "CLEAR_REQ"), {"t": time.time(), "src": "remote", "why": "L1+R1 hold"})
                        self.log("remote clear requested (buttons %s held %.1fs); host mirror decides" % (JOY_CLEAR_BTNS, JOY_CLEAR_HOLD))
                if not select.select([fd], [], [], 0.1)[0]:
                    continue
                ev = fd.read(8)
                if not ev or len(ev) < 8: raise OSError("short read")
                _t, val, typ, num = struct.unpack("IhBB", ev)
                init = bool(typ & 0x80)
                typ &= ~0x80
                if typ == 1:
                    btns[num] = bool(val)
                    held = bool(JOY_CLEAR_BTNS) and all(btns.get(b) for b in JOY_CLEAR_BTNS)
                    if not held: clear_t0 = None
                    elif clear_t0 is None and not init: clear_t0 = time.time()
                if typ == 1 and val and not init and num in JOY_ESTOP_BTNS and self.floor_nolatch():   # noestop
                    with self.lock: self.floor_stop("remote button %d" % num, "remote", was="remote-estop")
                    clear_t0 = None
                elif typ == 1 and val and not init and num in JOY_ESTOP_BTNS:
                    body = {"t": time.time(), "src": "remote", "kind": "normal", "why": "remote button %d" % num}
                    self._write_latch(os.path.join(os.path.dirname(ESTOP), "ESTOP_REQ"), body)   # host mirror -> host latch (voice speaks)
                    self._write_latch(ESTOP, body)
                    self.estop = True; clear_t0 = None; self.finish("REMOTE ESTOP button %d" % num)
                elif typ == 2:
                    axes[num] = val / 32767.0
                    l = -axes.get(JOY_LIN_AXIS, 0.0); a = -axes.get(JOY_ANG_AXIS, 0.0)
                    l = 0.0 if abs(l) < 0.12 else l; a = 0.0 if abs(a) < 0.12 else a
                    self.joy.update(lin=l * JOY_LIN, ang=a * JOY_ANG, t=time.time())
            except Exception as ex:
                if fd is not None:
                    self.log("joystick lost (%s) -> zero" % ex)
                    try: fd.close()
                    except Exception: pass
                fd = None; self.joy.update(present=False, lin=0.0, ang=0.0)
                with self.lock:
                    if self.mode == "stream" and self.src == "joystick": self.finish("joystick disconnected")
                time.sleep(0.5)

    # ------------------------------------------------------------ core
    def _no_range_blocks_autonomy(self):
        """Fail closed for every non-joystick source when health is missing, stale, or not a validated range.

        Joystick teleop is the only source left through. Estop, tip, battery, deadman,
        and the watchdog still apply to it and are handled elsewhere in tick.
        """
        try:
            import gb_health
            snap = gb_health.read_shared()
            block = gb_health.gate_blocks_autonomy(self.src, self.mode, snap)
        except Exception:
            block = self.mode != "idle" and self.src != "joystick"
        if not block:
            return False
        self.finish("autonomous refused: health monitor missing, stale, or no validated range")
        return True

    def fault_latch(self, why, src):
        """navstd: repeated collision faults -> same latch path as a hard tilt (host ESTOP via ESTOP_REQ + container ESTOP).
        The gate never clears it; a human must (voice/app clear, as for any ESTOP).
        2026-11-25 noestop: on the floor (mode != bench) -> normal stop, no latch."""
        if getattr(self, "floor_nolatch", lambda: False)():
            self.soft = []
            return self.floor_stop("repeated collisions: " + why, src, was="fault-latch")
        d = {"t": time.time(), "src": "gate-faults", "kind": "safety", "why": why}
        for f in (os.path.join(os.path.dirname(ESTOP), "ESTOP_REQ"), ESTOP):
            self._write_latch(f, d)
        self.trip("repeated collisions - emergency stop latched (%s)" % why, src)
        self.estop = True; self.backoff = False; self.faults = []; self.finish("FAULT ESTOP: " + why)

    def tilt_latch(self, why, src):
        """Hard tilt: ESTOP_REQ -> gb_safety_run.sh mirror writes the HOST latch (data/ESTOP, same as voice / clap),
        container ESTOP written too so this gate and the watchdog hold zero immediately.
        2026-11-25 noestop: on the floor (mode != bench) -> normal stop, no latch."""
        if getattr(self, "floor_nolatch", lambda: False)():
            return self.floor_stop("tipping over: " + why, src, was="tilt-latch")
        d = {"t": time.time(), "src": "gate-tilt", "kind": "safety", "why": why}
        for f in (os.path.join(os.path.dirname(ESTOP), "ESTOP_REQ"), ESTOP):
            self._write_latch(f, d)
        self.trip("tipping over - emergency stop latched (%s)" % why, src)
        self.estop = True; self.backoff = False; self.finish("TILT ESTOP: " + why)

    # ------------------------------------------------------------ 2026-11-25 noestop (floor: never latch)
    def floor_nolatch(self):
        """True unless mode.json (mirrored to FLOOR_MODE.json) says bench. Missing/stale info counts as floor."""
        now = time.time()
        if now - self._floor_chk >= 0.5:
            self._floor_chk = now
            try: on = gb_floor.no_latch() if gb_floor else True
            except Exception: on = True
            if on != self._floor_on:
                self.log("noestop: %s" % ("GROUND/floor mode -> faults, tilt, remote button and latch files do a NORMAL stop "
                                          "(no latch, re-arm always allowed)" if on else "BENCH mode -> legacy latching ESTOP behaviour"))
            self._floor_on = on
        return self._floor_on

    def floor_stop(self, why, src, was="estop", event=True):
        """Normal stop instead of an emergency latch: zero now + 10 Hz zero for ZERO_HOLD, end the mission, disarm,
        log the cause. No latch file. Motion stays refused (in memory only) until ARMED is rewritten (wake up Wally)."""
        self.trip("normal stop (floor, no latch): %s" % why, src)
        self.backoff = False; self.faults = []
        self.finish("FLOOR STOP (normal, no latch; was %s): %s" % (was, why))
        self.floor_hold_t = time.time()
        try:
            os.makedirs(os.path.dirname(EXPLORE_ABORT), exist_ok=True)
            with open(EXPLORE_ABORT, "w") as f: f.write("floor stop: %s" % why[:120])
        except Exception as ex:
            self.log("floor stop: mission abort file failed: %s" % ex)
        try:
            if os.path.exists(ARMED_FILE):
                os.remove(ARMED_FILE)
                with open(os.path.join(os.path.dirname(ARMED_FILE), "arm.log"), "a") as f:
                    f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + "DISARMED: gate floor stop (%s)\n" % why[:120])
        except Exception as ex:
            self.log("floor stop: disarm failed: %s" % ex)
        if event and gb_floor:
            try: gb_floor.request_stop("gate:" + str(src)[:30], why, was=was)
            except Exception as ex: self.log("floor stop event failed: %s" % ex)
        self.log("NORMAL STOP (floor): %s | was=%s src=%s -> disarmed, mission aborted, no latch; say 'wake up Wally' to re-arm"
                 % (why, was, src))
        return True

    def floor_held(self):
        """In-memory hold after a floor stop; released as soon as ARMED is (re)written after the stop."""
        if not self.floor_hold_t: return False
        try:
            if os.stat(ARMED_FILE).st_mtime > self.floor_hold_t:
                self.log("noestop: re-armed after the floor stop -> motion allowed again")
                self.floor_hold_t = 0.0; return False
        except OSError:
            pass
        return True

    def _evt_sig(self):
        try:
            p = gb_floor.stop_event_path() if gb_floor else None
            if not p: return None
            st = os.stat(p)
        except OSError:
            return None
        try: body = json.load(open(p))
        except Exception: body = {}
        return (st.st_mtime_ns, st.st_size, body if isinstance(body, dict) else {})

    def _latch_sig(self):
        sig = []
        for f in (ESTOP, MAINT):
            try: st = os.stat(f); sig.append((f, st.st_mtime_ns, st.st_size))
            except OSError: pass
        return tuple(sig) or None

    def _write_latch(self, path, body):
        """Replace a stop file while holding it, so a voice clear cannot unlink a hard fault mid-write."""
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
            try:
                import fcntl
                fcntl.flock(fd, fcntl.LOCK_EX)
                os.ftruncate(fd, 0)
                os.lseek(fd, 0, os.SEEK_SET)
                os.write(fd, json.dumps(body).encode())
            finally:
                os.close(fd)
        except Exception as ex:
            self.log("latch write failed %s: %s" % (path, ex))

    def send(self, lin, ang):
        self.fz.on_cmd(time.time(), float(lin), float(ang))      # the stall / slip detectors compare against what we SENT
        m = Twist(); m.linear.x = float(lin); m.angular.z = float(ang)
        self.pub.publish(m); self.n_out += 1
        key = (round(lin, 3), round(ang, 3))
        if key != self.last_pub:
            self.log("OUT lin=%+.3f ang=%+.3f mode=%s" % (lin, ang, self.mode)); self.last_pub = key

    def _revoke_mission_lease(self, why):
        """Immediately drop MISSION_LEASE and hold /gb/cmd_in until a newer live lease is granted."""
        self.lease_hold = True
        self.lease_hold_t = time.time()
        cleared = False
        try:
            import gb_arm
            cleared = bool(gb_arm.clear_mission_lease(why))
        except Exception:
            try:
                p = LEASE_FILE
                if os.path.exists(p):
                    os.remove(p); cleared = True
            except Exception:
                pass
        if cleared or why:
            try: self.log("MISSION LEASE revoked (%s)" % why)
            except Exception: pass

    def _lease_live(self):
        """True while a live MISSION_LEASE is held (autonomous / explore)."""
        try:
            import gb_arm
            return bool(gb_arm.lease_is_live())
        except Exception:
            try:
                return os.path.exists(LEASE_FILE)
            except Exception:
                return False

    def _lin_cap(self, src=None, joystick=False):
        return lin_cap_for(src if src is not None else getattr(self, "src", ""),
                           lease_live=self._lease_live(), joystick=joystick)

    def _stream_lease_ok(self, now=None, joystick=False):
        if joystick:
            return True
        if not getattr(self, "lease_hold", False):
            return True
        hold_t = float(getattr(self, "lease_hold_t", 0.0) or 0.0)
        try:
            import gb_arm
            # One policy: a newer mission lease lifts the hold, and so does an
            # armed manual /gb/cmd_in driver once no mission lease is active
            # (after the short revoke drain, or a fresh arm after the hold).
            if gb_arm.stream_allowed_after_hold(True, hold_t, now=now, joystick=False):
                if self.lease_hold:
                    try: self.log("MISSION LEASE hold cleared (fresh lease or armed manual drive, no active mission)")
                    except Exception: pass
                self.lease_hold = False
                return True
            return False
        except Exception:
            try:
                p = LEASE_FILE
                if os.path.exists(p) and os.path.getmtime(p) > hold_t:
                    self.lease_hold = False
                    return True
                armed = os.path.exists(ARMED_FILE)
                if (not os.path.exists(p)) and armed:
                    armed_t = os.path.getmtime(ARMED_FILE)
                    tnow = time.time() if now is None else now
                    if armed_t > hold_t or (float(tnow) - hold_t) >= 1.0:
                        self.lease_hold = False
                        return True
            except Exception:
                pass
            return False

    def finish(self, why):
        was = self.mode
        try:
            import gb_arm
            revoke = gb_arm.why_revokes_lease(why)
        except Exception:
            w = (why or "").lower()
            revoke = any(k in w for k in ("stop request", "estop", "floor stop", "maintenance lock", "floor hold", "disarm"))
        if revoke:
            fn = getattr(self, "_revoke_mission_lease", None)
            if callable(fn):
                fn(why)
            else:
                try:
                    import gb_arm
                    gb_arm.clear_mission_lease(why)
                except Exception:
                    pass
        self.mode = "idle"; self.lin = self.ang = 0.0; self.turn = None
        if self.look is not None:
            self.look.cancel(why); self.look = None
        if self.src == "backoff": self.backoff = False
        now = time.time(); self.zero_until = now + ZERO_HOLD; self.last_why = why
        self.send(0.0, 0.0); self.last_zero = now
        if was != "idle" or "ESTOP" in why or "stop" in why:
            self.log("STOP (%s) was=%s ran=%.2fs -> zero now, then 10 Hz zero for %.1fs"
                     % (why, was, (now - self.t_start) if was != "idle" else 0.0, ZERO_HOLD))

    def tick(self):
        t0 = time.perf_counter()
        try:
            self._tick_locked()
        finally:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            if dt_ms >= 15.0:
                try:
                    import gb_health
                    gb_health.note_control_ms(dt_ms)
                except Exception:
                    pass

    def _tick_locked(self):
        now = time.time()
        with self.lock:
            # 2026-11-26 tripclear: guard/bump trips are normal stops. Release the sticky
            # last_trip + guard_event.json on startup (stale file), disarm, or idle timeout.
            if not getattr(self, "_trip_start_cleared", False):
                self._trip_start_cleared = True
                try:
                    if self.mode == "idle" and (self.last_trip or os.path.exists(EVENT)):
                        self.clear_trip("startup-stale")
                except Exception:
                    pass
            else:
                try:
                    _cr = self._trip_clear_reason(now)
                    if _cr:
                        self.clear_trip(_cr)
                    if self.mode == "idle" and getattr(self, "slow_why", ""):
                        self.slow_why = ""
                except Exception:
                    pass
            if self.floor_nolatch():                                      # 2026-11-25 noestop: ground never latches
                sig = self._latch_sig()
                if sig and sig != self._floor_sig:
                    self.floor_stop("latch file present in ground mode (%s) - one-shot stop, host archives it"
                                    % ",".join(os.path.basename(x[0]) for x in sig), "file", was="latch-file", event=False)
                self._floor_sig = sig
                esig = self._evt_sig()                                    # FLOOR_STOP from voice / clap / host
                if esig != self._floor_evt:
                    self._floor_evt = esig
                    if esig and esig[2] and not str(esig[2].get("src", "")).startswith("gate:"):
                        self.floor_stop("%s (%s)" % (esig[2].get("why", "stop"), esig[2].get("src", "?")), "event",
                                        was=str(esig[2].get("was", "estop")), event=False)
                if self.estop or self.maint:
                    self.estop = False; self.maint = False
                    self.log("noestop: ground mode -> latched state released (no persistent ESTOP/MAINT on the floor)")
                held = self.floor_held()
                if held and self.mode != "idle": self.finish("floor hold: not re-armed since the floor stop")
                if held:
                    if now < self.zero_until and now - self.last_zero >= 0.09:
                        self.send(0.0, 0.0); self.last_zero = now
                    return
            elif os.path.exists(MAINT):
                if not self.maint:
                    self.maint = True; self.finish("MAINTENANCE lock")
                if now - self.last_zero >= 0.09:
                    self.send(0.0, 0.0); self.last_zero = now
                return
            else:
                self.maint = False
            e = os.path.exists(ESTOP) and not self.floor_nolatch()
            if e and not self.estop:
                self.estop = True; self.finish("ESTOP file present")
            elif not e and self.estop:
                self.estop = False; self.log("ESTOP cleared (file removed) - motion allowed again")
            if self.estop:
                if now - self.last_zero >= 0.09:
                    self.send(0.0, 0.0); self.last_zero = now
                return
            if self._no_range_blocks_autonomy():
                return
            # joystick feeds the stream path (same deadman + caps)
            j = self.joy
            if j["present"] and (j["lin"] or j["ang"]) and now - j["t"] < 5.0 and self.mode in ("idle", "stream"):
                if self.mode != "stream": self.mode = "stream"; self.t_start = now; self.log("START joystick stream")
                self.src = "joystick"; self.lin, self.ang, self.t_fresh = j["lin"], j["ang"], now
                self.deadline = self.t_start + STREAM_CAP
            elif self.mode == "stream" and self.src == "joystick" and j["present"] and not (j["lin"] or j["ang"]):
                self.finish("joystick centred")
            if self.mode == "timed":
                if now >= self.deadline:
                    self.finish("deadline reached id=%s (commanded %.2fs)" % (self.rid, self.deadline - self.t_start)); return
                if self.collision("timed " + self.src): return
                _l, _a, why = self.guard(self.lin, self.ang)
                if why:
                    self.trip(why, "timed " + self.src); self.finish("GUARD: " + why); return
                self.send(_l, _a)                                 # speed ramped by the fused distance
            elif self.mode == "turn":
                tr = self.turn
                if self.yaw is None or now - self.yaw_t > 0.5:
                    self.finish("turn aborted: odometry stale"); return
                tr["done"] += _wrap(self.yaw - tr["prev"]); tr["prev"] = self.yaw
                rem = tr["target"] - tr["done"]
                if abs(rem) < 0.03 or (rem * tr["target"] < 0):
                    self.finish("turn complete id=%s turned=%.1fdeg target=%.1fdeg" % (
                        self.rid, math.degrees(tr["done"]), math.degrees(tr["target"]))); return
                if now >= self.deadline:
                    self.finish("turn deadline id=%s turned=%.1fdeg of %.1fdeg" % (
                        self.rid, math.degrees(tr["done"]), math.degrees(tr["target"]))); return
                if self.collision("turn " + self.src): return
                _l, _a, why = self.guard(0.0, rem)
                if why:
                    self.trip(why, "turn " + self.src); self.finish("GUARD: " + why); return
                w = abs(self.ang)
                if abs(rem) < 0.25: w = max(0.3, w * 0.7)
                self.send(0.0, math.copysign(w, rem))
            elif self.mode == "bounded":
                import gb_bounded
                pinned = self._pinned_odom(now)
                if self.look is None or pinned is None or not gb_bounded.fresh_age(now - self.scan_t, gb_bounded.SCAN_FRESH_S):
                    if self.look is not None:
                        self.look.cancel("sensors_stale")
                    self.finish("bounded sensors stale"); return
                cmd = self.look.step(now, self.front, self.rear, self.sweep, pinned[:3],
                                     now - self.scan_t, pinned[3], estop=self.estop, source="/odom")
                if not cmd.get("allow"):
                    self.finish("bounded " + str(cmd.get("reason"))); return
                _l, _a, why = self.guard(cmd.get("lin") or 0.0, cmd.get("ang") or 0.0)
                if why:
                    self.trip(why, "bounded " + self.src); self.finish("GUARD: " + why); return
                self.send(_l, _a)
            elif self.mode == "stream":
                if now - self.t_fresh > DEADMAN:
                    self.finish("DEADMAN: no fresh stream command for %.2fs" % (now - self.t_fresh)); return
                if now >= self.deadline:
                    self.finish("stream cap %.0fs reached" % STREAM_CAP); return
                if self.collision(self.src): return
                l, a, why = self.guard(self.lin, self.ang, stream=(self.src != "joystick"))
                if why: self.trip(why, self.src)
                self.send(l, a)
            else:
                if now < self.zero_until and now - self.last_zero >= 0.09:
                    self.send(0.0, 0.0); self.last_zero = now
                # 2026-11-26 motiononly (John): drop idle-time stall/tilt/jolt events so they
                # cannot trip the moment the next move starts.
                try:
                    ev = self.fz.pop_event(now)
                    if ev:
                        self.log("GUARD OFF (motionless): discarded idle %s event: %s" % (ev[1], ev[2]))
                        self.fz.reset_detectors()
                except Exception:
                    pass

    def status(self):
        d = {"t": time.time(), "mode": self.mode, "lin": self.lin, "ang": self.ang, "src": self.src, "id": self.rid,
             "estop": self.estop, "deadline": self.deadline if self.mode != "idle" else 0, "why": self.last_why,
             "joy": self.joy["present"], "out": OUT, "n_out": self.n_out,
             "front": round(self.front, 3) if math.isfinite(self.front) else None,
             "rear": round(self.rear, 3) if math.isfinite(self.rear) else None,
             "sweep": round(self.sweep, 3) if math.isfinite(self.sweep) else None, "trips": self.trips, "last_trip": self.last_trip}
        d["bench_guard_off"] = bool(self.bench_off())                       # 2026-11-25 benchpersist
        d["motion_guards"] = commanded_motion(self.lin, self.ang) or (self.mode in MOTION_MODES)  # 2026-11-26 motiononly
        d["lease_hold"] = bool(getattr(self, "lease_hold", False))
        d["bench_scope"] = getattr(self, "_bench_scope", "all") if d["bench_guard_off"] else None   # navfix
        try:
            now = d["t"]; fr = self.fz.fresh(now); L = self.fz.lidar; D = self.fz.depth
            d["fusion"] = {"lidar_fwd": round(L.fwd, 3) if L and math.isfinite(L.fwd) else None,
                           "depth_fwd": round(D.fwd, 3) if D and math.isfinite(D.fwd) else None,
                           "front_src": self.fz.front(now)[1], "fresh": fr, "blind": round(L.blind_f, 2) if L else None,
                           "cap": round(OB.ramp(self.front, self._lin_cap()), 3), "slow": self.slow_why}
            snap = self.fz.snapshot(now)                                  # navstd: full fused sensor suite
            d["sensors"] = snap
            d["faults"] = {"recent": len([x for x in self.faults if now - x <= FAULT_WIN]), "limit": FAULT_LIMIT,
                           "win_s": FAULT_WIN}
            if now - self.sens_t >= 0.5:                                  # 2 Hz file for the host voice AI / health
                self.sens_t = now
                try:
                    with open(SENSORS_FILE + ".tmp", "w") as f:
                        json.dump({"t": now, "estop": self.estop, "mode": self.mode, "front": d["front"], "rear": d["rear"],
                                   "sweep": d["sweep"], "last_trip": self.last_trip, "slow": self.slow_why,
                                   "faults": d["faults"], "sensors": snap,
                                   "bench_guard_off": d["bench_guard_off"], "bench_scope": d.get("bench_scope"),
                                   "motion_guards": d.get("motion_guards")}, f)
                    os.replace(SENSORS_FILE + ".tmp", SENSORS_FILE)
                except Exception: pass
        except Exception: pass
        if FR is not None:                                                # 2026-11-27 fusion stuck-recovery
            try:
                d["fr"] = {"cam": FR.cam_for_status(getattr(self, "fr_cam", None), d["t"]),
                           "last": getattr(self, "fr_last", None), "floor_fwd": FR.FLOOR_FWD, "floor_sweep": FR.FLOOR_SWEEP,
                           "squeeze_active": bool(getattr(self, "fr_last", None)
                                                  and d["t"] - float(self.fr_last.get("t", 0)) <= FR.TOKEN_TTL_S)}
            except Exception:
                pass
        if self.turn: d["turned_deg"] = round(math.degrees(self.turn["done"]), 1)
        m = String(); m.data = json.dumps(d); self.st_pub.publish(m)

def main():
    rclpy.init()
    g = Gate()
    try:
        rclpy.spin(g)
    except (KeyboardInterrupt, Exception) as ex:
        try: g.log("gate exiting (%s) -> zero burst" % type(ex).__name__)
        except Exception: pass
    finally:
        try:
            for _ in range(10):
                g.send(0.0, 0.0); time.sleep(0.05)
        except Exception: pass
        try: g.destroy_node(); rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
