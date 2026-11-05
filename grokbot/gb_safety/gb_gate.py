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
import gb_obstacle as OB          # 2026-11-05 .bak-obstacle: footprint corridor + sensor fusion + speed ramp (copied here by gb_safety_run.sh)

E = os.environ.get
OUT = E("GATE_OUT", "/controller/cmd_vel")
IN_STREAM = E("GATE_IN", "/gb/cmd_in")
REQ = E("GATE_REQ", "/gb/motion_req")
STATUS = E("GATE_STATUS", "/gb/gate_status")
ESTOP = E("GB_ESTOP_FILE", "/home/ubuntu/shared/gb_safety/ESTOP")
LOGF = E("GATE_LOG", "/home/ubuntu/shared/gb_safety/gate.log")
DEADMAN = float(E("GATE_DEADMAN_S", "0.5"))
ZERO_HOLD = float(E("GATE_ZERO_HOLD_S", "2.0"))
HZ = 20.0
MAX_LIN = float(E("GATE_MAX_LIN", "0.12"))          # m/s hard cap (Hiwonder app cap is 0.2)
MAX_ANG = float(E("GATE_MAX_ANG", "0.8"))           # rad/s hard cap
TIMED_DEFAULT = 3.0
TIMED_CAP = 10.0
STREAM_CAP = float(E("GATE_STREAM_CAP_S", "600"))
ODOM_TOPICS = [t for t in E("GATE_ODOM", "/odom,/odom_raw").split(",") if t]
# Direction-aware LiDAR guard (ALWAYS ON, every source incl. missions/Nav2/joystick; no mission bypass):
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
BACKOFF_V = 0.05; BACKOFF_S = 1.2                                    # after a bump / stall: back off 6 cm if the rear is clear
SELF_MIN = float(E("GATE_SELF_MIN", "0.10"))         # returns closer than this are the robot's own body
SCAN_STALE = float(E("GATE_SCAN_STALE_S", str(OB.LIDAR_STALE)))   # was 1.0 s; scans arrive at 10 Hz
EVENT = E("GATE_EVENT", "/home/ubuntu/shared/gb_safety/guard_event.json")
JOY_DEV = E("GATE_JOY_DEV", "/dev/input/js0")
JOY_ON = E("GATE_JOY", "1") == "1"
JOY_LIN_AXIS = int(E("GATE_JOY_LIN_AXIS", "1")); JOY_ANG_AXIS = int(E("GATE_JOY_ANG_AXIS", "2"))
JOY_ESTOP_BTNS = [int(x) for x in E("GATE_JOY_ESTOP_BTNS", "6,8").split(",") if x.strip()]  # SELECT/L2 on Hiwonder pads
JOY_LIN = min(MAX_LIN, float(E("GATE_JOY_MAX_LIN", "0.12"))); JOY_ANG = min(MAX_ANG, float(E("GATE_JOY_MAX_ANG", "0.8")))

def _now_s():
    t = time.time(); return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)

def _clamp(v, m):
    try: v = float(v)
    except Exception: return 0.0
    if not math.isfinite(v): return 0.0
    return max(-m, min(m, v))

def _wrap(a): return math.atan2(math.sin(a), math.cos(a))

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
        self.front = self.rear = self.sweep = float("inf"); self.scan_t = 0.0; self.trips = 0; self.last_trip = ""
        self.create_subscription(LaserScan, E("GATE_SCAN", "/scan_raw"), self.on_scan, qos_profile_sensor_data)
        self.fz = OB.Fusion(); self.cam_k = None; self.rgb_chk = 0.0; self.slow_why = ""; self.backoff = False
        self.create_subscription(CameraInfo, E("GATE_DEPTH_INFO", "/ascamera/camera_publisher/depth0/camera_info"), self.on_caminfo, qos_profile_sensor_data)
        self.create_subscription(Image, E("GATE_DEPTH", "/ascamera/camera_publisher/depth0/image_raw"), self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, E("GATE_IMU", "/imu"), self.on_imu, qos_profile_sensor_data)
        self.mode = "idle"; self.lin = 0.0; self.ang = 0.0; self.src = ""; self.rid = ""
        self.deadline = 0.0; self.t_start = 0.0; self.t_fresh = 0.0; self.zero_until = 0.0; self.last_zero = 0.0
        self.turn = None; self.yaw = None; self.yaw_src = None; self.yaw_t = 0.0
        self.estop = os.path.exists(ESTOP); self.last_why = "start"; self.n_out = 0; self.last_pub = (None, None)
        import collections; self.seen = collections.deque(maxlen=256)
        self.joy = {"present": False, "lin": 0.0, "ang": 0.0, "t": 0.0}
        self.create_timer(1.0 / HZ, self.tick)
        self.create_timer(0.1, self.status)
        self.log("gate up out=%s in=%s req=%s estop_file=%s deadman=%.2fs hold=%.1fs caps lin=%.2f ang=%.2f estop=%s"
                 % (OUT, IN_STREAM, REQ, ESTOP, DEADMAN, ZERO_HOLD, MAX_LIN, MAX_ANG, self.estop))
        self.log("obstacle fusion up: corridor half-width %.2fm stop fwd %.2f rev %.2f sweep %.2f ramp %.2f->%.2f m/s at %.2f-%.2fm; "
                 "lidar+depth+encoders(%s)+imu+rgb; stale lidar>%.1fs enc>%.2fs -> stop, depth>%.1fs imu>%.2fs -> %.2f m/s"
                 % (OB.HALF_W, FRONT_STOP, REAR_STOP, SWEEP_R, OB.V_MIN, MAX_LIN, FRONT_STOP, OB.SLOW_FULL,
                    E("GATE_ENCODERS", "/odom_raw"), OB.LIDAR_STALE, OB.ODOM_STALE, OB.DEPTH_STALE, OB.IMU_STALE, OB.V_DEGRADED))
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
        if t == E("GATE_ENCODERS", "/odom_raw"):                 # wheel encoders: stall / bump detection + pose for memory
            q = m.pose.pose.orientation
            self.fz.on_odom(time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z, m.pose.pose.position.x,
                            m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.yaw_src not in (None, t) and time.time() - self.yaw_t < 0.5:
            return                                        # prefer the first topic that is alive
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw_src = t; self.yaw_t = time.time()

    def on_scan(self, m):
        v = OB.lidar_analyze(m.ranges, m.angle_min, m.angle_increment, max(SELF_MIN, m.range_min), m.range_max, time.time())
        self.fz.on_lidar(v)
        self.front = self.fz.front(v.t)[0]; self.rear, self.sweep, self.scan_t = v.rev, v.sweep, v.t

    def on_caminfo(self, m):
        self.cam_k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(self, m):
        if self.cam_k is None or m.encoding not in ("16UC1", "mono16"): return
        try:
            import numpy as np
            img = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            self.fz.on_depth(OB.depth_analyze(img, *self.cam_k, t=time.time()))
        except Exception as ex:
            self.log_throttle("depth frame failed: %s" % ex, 10.0)

    def on_imu(self, m):
        a = m.linear_acceleration
        self.fz.on_imu(time.time(), a.x, a.y, a.z)

    def guard(self, lin, ang, stream=False):
        """Returns (lin, ang, why|None): fused LiDAR + depth + encoders + IMU + RGB limit (gb_obstacle.Fusion.limit).
        why is set only when motion is refused (obstacle / stale critical sensor / blind); slow-downs just cap lin.
        Fail-safe: stale LiDAR or encoders block everything, stale depth / IMU cap the speed."""
        fs, rs, sw = (S_FRONT_STOP, S_REAR_STOP, S_SWEEP_R) if stream else (FRONT_STOP, REAR_STOP, SWEEP_R)
        now = time.time()
        if now - self.rgb_chk > 0.2:
            self.rgb_chk = now
            try: self.fz.on_rgb(json.load(open(RGB_FILE)))
            except Exception: pass
        l, a, why, kind = self.fz.limit(lin, ang, now, MAX_LIN, fs, rs, sw)
        if kind == "slow":
            self.slow_why = why
            if now - self._thr.get("_slow", 0) > 1.0: self._thr["_slow"] = now; self.log("SLOW " + why)
            return l, a, None
        self.slow_why = ""
        return l, a, why

    def collision(self, src):
        """Encoder stall / LiDAR no-progress / IMU jolt or tilt while moving -> stop, then back off 6 cm if it was a
        forward move and the rear corridor is clear (the back-off is a gate timed move, guarded like any other)."""
        ev = self.fz.pop_event(time.time())
        if not ev: return False
        _t, sensor, why = ev
        if sensor == "imu-tilt-hard":                             # > 25 deg pitch or roll: zero + latch, no back-off
            self.tilt_latch(why, src); self.fz.reset_detectors(); return True
        # Soft contacts (jolt / stall / soft tilt): never latch ESTOP. Tip-over (>25 deg) already handled above.
        # Skip reverse backoff on IMU jolt — carpet vibration was causing thrash; explore / driver renegotiates.
        soft = sensor.startswith("imu")  # imu / imu-tilt (not imu-tilt-hard)
        fwd = self.lin > 0 and not soft
        self.trip("bump detected (%s): %s" % (sensor, why), src); self.finish("BUMP (%s): %s" % (sensor, why))
        self.fz.reset_detectors()
        if fwd and not self.backoff and self.fz.lidar and self.fz.lidar.rev > REAR_STOP + 0.05 and not self.estop:
            self.backoff = True; now = time.time()
            self.mode = "timed"; self.lin = -BACKOFF_V; self.ang = 0.0; self.src = "backoff"; self.rid = "backoff"
            self.t_start = now; self.deadline = now + BACKOFF_S
            self.log("START backoff lin=%+.3f dur=%.1fs (rear %.2fm)" % (self.lin, BACKOFF_S, self.fz.lidar.rev))
        return True

    def trip(self, why, src):
        self.trips += 1; self.last_trip = why
        t = time.time()
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
                self.finish("stop request from %s" % src); return
            if op == "noop": return
            if self.estop:
                self.log("REFUSED %s id=%s from %s: ESTOP latched" % (op, rid, src)); return
            if op == "timed":
                lin = _clamp(d.get("lin", 0), MAX_LIN); ang = _clamp(d.get("ang", 0), MAX_ANG)
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
                self.send(lin, ang)
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
                self.send(0.0, self.ang)
            elif op != "noop":
                self.log("unknown op %r from %s" % (op, src))

    def on_stream(self, m):
        now = time.time()
        with self.lock:
            lin = _clamp(m.linear.x, MAX_LIN); ang = _clamp(m.angular.z, MAX_ANG)
            if self.estop:
                if lin or ang: self.log_throttle("REFUSED stream: ESTOP latched")
                return
            if self.mode in ("timed", "turn"):
                return                                        # a timed/turn command owns the motors until it ends
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
        axes = {}; fd = None
        while rclpy.ok():
            try:
                if fd is None:
                    if not os.path.exists(JOY_DEV):
                        self.joy["present"] = False; time.sleep(1.0); continue
                    fd = open(JOY_DEV, "rb", buffering=0); self.joy["present"] = True
                    self.log("joystick connected: %s (estop buttons %s)" % (JOY_DEV, JOY_ESTOP_BTNS))
                ev = fd.read(8)
                if not ev or len(ev) < 8: raise OSError("short read")
                _t, val, typ, num = struct.unpack("IhBB", ev)
                typ &= ~0x80
                if typ == 1 and val and num in JOY_ESTOP_BTNS:
                    try:
                        with open(ESTOP, "w") as f: json.dump({"t": time.time(), "src": "joystick", "why": "button %d" % num}, f)
                    except Exception as ex: self.log("joystick estop write failed: %s" % ex)
                    self.estop = True; self.finish("JOYSTICK ESTOP button %d" % num)
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
    def tilt_latch(self, why, src):
        """Hard tilt: ESTOP_REQ -> gb_safety_run.sh mirror writes the HOST latch (data/ESTOP, same as voice / clap),
        container ESTOP written too so this gate and the watchdog hold zero immediately."""
        d = {"t": time.time(), "src": "gate-tilt", "why": why}
        for f in (os.path.join(os.path.dirname(ESTOP), "ESTOP_REQ"), ESTOP):
            try:
                with open(f + ".tmp", "w") as fh: json.dump(d, fh)
                os.replace(f + ".tmp", f)
            except Exception as ex: self.log("tilt latch write failed %s: %s" % (f, ex))
        self.trip("tipping over - emergency stop latched (%s)" % why, src)
        self.estop = True; self.backoff = False; self.finish("TILT ESTOP: " + why)

    def send(self, lin, ang):
        self.fz.on_cmd(time.time(), float(lin), float(ang))      # the stall / slip detectors compare against what we SENT
        m = Twist(); m.linear.x = float(lin); m.angular.z = float(ang)
        self.pub.publish(m); self.n_out += 1
        key = (round(lin, 3), round(ang, 3))
        if key != self.last_pub:
            self.log("OUT lin=%+.3f ang=%+.3f mode=%s" % (lin, ang, self.mode)); self.last_pub = key

    def finish(self, why):
        was = self.mode
        self.mode = "idle"; self.lin = self.ang = 0.0; self.turn = None
        if self.src == "backoff": self.backoff = False
        now = time.time(); self.zero_until = now + ZERO_HOLD; self.last_why = why
        self.send(0.0, 0.0); self.last_zero = now
        if was != "idle" or "ESTOP" in why or "stop" in why:
            self.log("STOP (%s) was=%s ran=%.2fs -> zero now, then 10 Hz zero for %.1fs"
                     % (why, was, (now - self.t_start) if was != "idle" else 0.0, ZERO_HOLD))

    def tick(self):
        now = time.time()
        with self.lock:
            e = os.path.exists(ESTOP)
            if e and not self.estop:
                self.estop = True; self.finish("ESTOP file present")
            elif not e and self.estop:
                self.estop = False; self.log("ESTOP cleared (file removed) - motion allowed again")
            if self.estop:
                if now - self.last_zero >= 0.09:
                    self.send(0.0, 0.0); self.last_zero = now
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

    def status(self):
        d = {"t": time.time(), "mode": self.mode, "lin": self.lin, "ang": self.ang, "src": self.src, "id": self.rid,
             "estop": self.estop, "deadline": self.deadline if self.mode != "idle" else 0, "why": self.last_why,
             "joy": self.joy["present"], "out": OUT, "n_out": self.n_out,
             "front": round(self.front, 3) if math.isfinite(self.front) else None,
             "rear": round(self.rear, 3) if math.isfinite(self.rear) else None,
             "sweep": round(self.sweep, 3) if math.isfinite(self.sweep) else None, "trips": self.trips, "last_trip": self.last_trip}
        try:
            now = d["t"]; fr = self.fz.fresh(now); L = self.fz.lidar; D = self.fz.depth
            d["fusion"] = {"lidar_fwd": round(L.fwd, 3) if L and math.isfinite(L.fwd) else None,
                           "depth_fwd": round(D.fwd, 3) if D and math.isfinite(D.fwd) else None,
                           "front_src": self.fz.front(now)[1], "fresh": fr, "blind": round(L.blind_f, 2) if L else None,
                           "cap": round(OB.ramp(self.front, MAX_LIN), 3), "slow": self.slow_why}
        except Exception: pass
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
