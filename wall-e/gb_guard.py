#!/usr/bin/env python3
"""gb_guard.py - LiDAR front obstacle guard for the MentorPi T1 (tracked).

Watches /scan_raw. Front = lidar angle 0 deg (static TF base_link->lidar_frame is identity
rotation; positive angles = robot's left, REP-103). If a valid reading in the front
sector (+/-GUARD_SECTOR_DEG) is closer than GUARD_STOP_DIST while the current motor command
(last Twist on /controller/cmd_vel, /cmd_vel or /app/cmd_vel) is forward (linear.x > 0):
  1. zero Twist on /controller/cmd_vel at 20 Hz for 0.5 s
  2. reverse 0.1 m/s for 0.5 s (skipped if the rear is blocked)
  3. turn in place toward the clearer side at 0.8 rad/s (capped so the whole maneuver <= 2.0 s)
  4. repeated zero Twists (always)
Forward commands that arrive while the obstacle is still within range are answered with an
immediate zero Twist (suppression). Cooldown 2 s between maneuvers.
Kill switch: flag file (GUARD_FLAG, synced from ~/grokbot/guard.disabled) or GUARD_DISABLED=1
-> no guard motion at all.
Mission bypass: while an autonomous gb_explore mission node runs (explore/map/distance/wander), it writes
GUARD_MISSION_FLAG = {"pid", "t", "mission", "dry"} on every heartbeat. The guard skips its stop/back-off
maneuver only while that flag is fresh (< GUARD_MISSION_STALE_S), the pid is alive and is a gb_explore --node
process, it is not a dry run, and no mission abort file exists. Anything else (flag missing, stale, dead pid,
unreadable) = guard fully active. Manual/simple drive commands never write the flag. GUARD_TURN=0 -> stop-only. GUARD_DRY_RUN=1 -> log only, never publish.
"""
import json, math, os, signal, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

E = os.environ.get
def fenv(k, d): 
    try: return float(E(k, d))
    except Exception: return float(d)
def benv(k, d): return str(E(k, d)).strip().lower() in ("1", "true", "yes", "on")

STOP_DIST   = fenv("GUARD_STOP_DIST", 0.127)     # m, measured from the LiDAR centre
RELEASE_HYS = fenv("GUARD_RELEASE_HYS", 0.03)    # m, extra clearance needed to release "blocked"
SECTOR      = math.radians(fenv("GUARD_SECTOR_DEG", 30))
FRONT_OFF   = math.radians(fenv("GUARD_FRONT_DEG", 0))  # lidar angle that faces forward
SIDE_LO, SIDE_HI = math.radians(30), math.radians(90)
REAR_HALF   = math.radians(30)
REAR_MIN    = fenv("GUARD_REAR_MIN", 0.15)
STOP_T, REV_T, TURN_T = fenv("GUARD_STOP_T", 0.5), fenv("GUARD_REV_T", 0.5), fenv("GUARD_TURN_T", 1.0)
REV_V, TURN_W = abs(fenv("GUARD_REV_V", 0.1)), abs(fenv("GUARD_TURN_W", 0.8))
MAX_T       = min(fenv("GUARD_MAX_T", 2.0), 2.0)  # hard ceiling for the whole maneuver
FINAL_N     = 5                                    # final zero Twists at 20 Hz (0.25 s)
COOLDOWN    = fenv("GUARD_COOLDOWN", 2.0)
TURN        = benv("GUARD_TURN", "1")
DRY         = benv("GUARD_DRY_RUN", "0")
ASSUME_FWD  = benv("GUARD_ASSUME_FORWARD", "0")    # dry-run testing only
ESTOP_FILE  = E("GB_ESTOP_FILE", "/home/ubuntu/shared/gb_safety/ESTOP")   # gb_safety latch: no manoeuvres
GATE_IN     = E("GUARD_GATE_IN", "/gb/cmd_in")
FLAG        = E("GUARD_FLAG", "/home/ubuntu/shared/gb_guard/guard.disabled")
STATUS      = E("GUARD_STATUS", "/home/ubuntu/shared/gb_guard/status.json")
OUT_TOPIC   = E("GUARD_OUT_TOPIC", "/controller/cmd_vel")
WATCH       = [t for t in E("GUARD_WATCH", "/controller/cmd_vel,/cmd_vel,/app/cmd_vel").split(",") if t]
LOG_EVERY   = fenv("GUARD_LOG_EVERY", 5.0)
MISSION_FLAG  = E("GUARD_MISSION_FLAG", "/home/ubuntu/shared/gb_guard/mission_active")
MISSION_ABORT = E("GUARD_MISSION_ABORT", "/home/ubuntu/shared/grokbot/explore/abort")
MISSION_STALE = fenv("GUARD_MISSION_STALE_S", 3.0)
MISSION_BYPASS_ON = benv("GUARD_MISSION_BYPASS", "0")
# Capability gates from Grok Bot (gb_caps). LiDAR gated off -> front stop from the depth camera (stop-only, no
# reverse/turn); depth missing/stale or camera also off -> forward motion always suppressed.
CAPS_FILE   = E("GUARD_CAPS", "/home/ubuntu/shared/gb_guard/caps.json")
DEPTH_TOPIC = E("GUARD_DEPTH_TOPIC", "/ascamera/camera_publisher/depth0/image_raw")
DEPTH_STOP  = fenv("GUARD_DEPTH_STOP", 0.30)      # m from the depth camera
DEPTH_STALE = fenv("GUARD_DEPTH_STALE_S", 0.6)

def read_caps():
    """Missing file: no user disable (lidar guard may run). Malformed file: do not treat that as both sensors on."""
    try:
        with open(CAPS_FILE) as fh: d = json.load(fh)
    except FileNotFoundError:
        return True, True
    except Exception:
        return False, False
    if not isinstance(d, dict) or not isinstance(d.get("lidar"), bool) or not isinstance(d.get("camera"), bool):
        return False, False
    return d["lidar"], d["camera"]

def mission_state(now=None):
    """(bypass: bool, why: str). Fail-safe: any doubt -> (False, reason) = guard active."""
    if not MISSION_BYPASS_ON: return False, "bypass disabled by GUARD_MISSION_BYPASS=0"
    now = time.time() if now is None else now
    try:
        with open(MISSION_FLAG) as fh: d = json.load(fh)
    except FileNotFoundError: return False, "no mission"
    except Exception as ex: return False, "flag unreadable (%s)" % type(ex).__name__
    try:
        pid = int(d.get("pid", 0)); age = now - float(d.get("t", 0))
    except Exception: return False, "flag malformed"
    if d.get("dry"): return False, "dry-run mission (guard stays active)"
    if age > MISSION_STALE or age < -1.0: return False, "flag stale (%.1fs)" % age
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh: cmd = fh.read().replace(b"\0", b" ")
    except Exception: return False, "mission pid %d dead" % pid
    if b"gb_explore.py" not in cmd or b"--node" not in cmd: return False, "pid %d is not a mission node" % pid
    if os.path.exists(MISSION_ABORT): return False, "mission aborting"
    return True, "%s mission pid %d" % (d.get("mission", "?"), pid)
HZ = 20.0

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Guard(Node):
    def __init__(self):
        super().__init__("gb_guard")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Twist, OUT_TOPIC, q)
        self.gpub = self.create_publisher(Twist, GATE_IN, q)     # non-zero manoeuvres go through gb_gate
        self.create_subscription(LaserScan, "/scan_raw", self.on_scan, qos_profile_sensor_data)
        for t in WATCH:
            self.create_subscription(Twist, t, lambda m, t=t: self.on_cmd(m, t), q)
        self.front = self.left = self.right = self.rear = float("inf")
        self.front_invalid = 0.0
        self.blocked = False; self.last_scan = 0.0; self.scans = 0
        self.cmd_fwd = False; self.cmd_src = ""; self.cmd_t = 0.0
        self.phase = None; self.t0 = 0.0; self.turn_dir = 0.0; self.turn_end = 0.0
        self.final_left = 0; self.rev_ok = True
        self.cool_until = 0.0; self.last_log = 0.0; self.last_status = 0.0
        self.n_trig = 0; self.n_supp = 0; self._cache = None
        self._mb = (0.0, False, "no mission"); self._mb_logged = None
        self._caps = (0.0, True, True); self._mode_logged = None; self.turn_ok = TURN
        self.depth_front = float("inf"); self.depth_invalid = 1.0; self.depth_t = 0.0
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0 / HZ, self.tick)
        self.get_logger().info(
            "gb_guard up: stop<%.3fm front=%.0fdeg +/-%.0fdeg turn=%s dry_run=%s assume_fwd=%s out=%s watch=%s"
            % (STOP_DIST, math.degrees(FRONT_OFF), math.degrees(SECTOR), TURN, DRY, ASSUME_FWD, OUT_TOPIC, WATCH))

    # ---------------------------------------------------------------- helpers
    def disabled(self):
        return benv("GUARD_DISABLED", "0") or os.path.exists(FLAG)

    def caps(self):
        now = time.time()
        if now - self._caps[0] > 0.5:
            lid, cam = read_caps(); self._caps = (now, lid, cam)
            mode = "lidar" if lid else ("depth" if cam else "blind")
            if mode != self._mode_logged:
                self._mode_logged = mode
                self.get_logger().warn({"lidar": "sensor mode: LiDAR guard",
                                        "depth": "sensor mode: user switched LiDAR off -> depth-camera front stop (stop-only)",
                                        "blind": "sensor mode: user switched LiDAR and camera off -> forward motion suppressed"}[mode])
        return self._caps[1], self._caps[2]

    def depth_mode(self):
        return not self.caps()[0]

    def _set_front(self, front, invalid, src):
        was = self.blocked; self.front = front; self.front_invalid = invalid
        stop = STOP_DIST if src == "scan" else DEPTH_STOP
        if self.front < stop:
            self.blocked = True
        elif self.blocked and self.front >= stop + RELEASE_HYS and self.front_invalid < 0.3:
            self.blocked = False
        if self.blocked != was:
            self.get_logger().info("front %s (%s): min=%.3fm invalid=%.0f%%" % ("BLOCKED" if self.blocked else "clear", src,
                                   self.front, 100 * self.front_invalid))

    def on_depth(self, m):
        try:
            import numpy as np
            if m.encoding not in ("mono16", "16UC1"): return
            a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            roi = a[int(m.height * 0.25):int(m.height * 0.75), int(m.width * 0.30):int(m.width * 0.70)]
            v = roi[(roi > 120) & (roi < 4000)]
            invalid = 1.0 - v.size / max(1, roi.size)
            # nearest point = 3rd percentile (ignores speckle); mostly-invalid ROI = something too close -> 0
            front = float(np.percentile(v, 3)) / 1000.0 if v.size > 0.2 * roi.size else (0.0 if invalid > 0.8 else float("inf"))
            self.depth_front, self.depth_invalid, self.depth_t = front, invalid, time.time()
            if self.depth_mode() and self.caps()[1]:          # blind mode (camera gated too): never release
                self._set_front(front, invalid, "depth"); self.last_scan = self.depth_t
                if self.blocked and self.forward_now(): self.trigger("depth")
        except Exception as ex:
            self.get_logger().warn("depth frame error: %s" % ex)

    def mission_bypass(self):
        """True while an autonomous mission node is alive (cached 0.2 s). Logs every ON/OFF transition."""
        now = time.time()
        if now - self._mb[0] > 0.2:
            on, why = mission_state(now); self._mb = (now, on, why)
            if on != self._mb_logged:
                self._mb_logged = on
                self.get_logger().warn(("MISSION BYPASS ON: guard stop skipped (%s)" if on else
                                        "mission bypass OFF: guard active (%s)") % why)
        return self._mb[1]

    def send(self, lin, ang, why=""):
        if DRY:
            return
        m = Twist(); m.linear.x = float(lin); m.angular.z = float(ang)
        if float(lin) == 0.0 and float(ang) == 0.0:
            self.pub.publish(m); self.gpub.publish(m)        # zero: straight to the controller (always allowed) + gate
        elif os.path.exists(ESTOP_FILE):
            return                                           # ESTOP latched: zero only, no reverse/turn manoeuvre
        else:
            self.gpub.publish(m)                             # reverse/turn manoeuvre: via the gate (caps, deadman, ESTOP)

    def _masks(self, s):
        key = (len(s.ranges), s.angle_min, s.angle_increment)
        if self._cache and self._cache[0] == key: return self._cache[1]
        f, l, r, b = [], [], [], []
        for i in range(len(s.ranges)):
            a = wrap(s.angle_min + i * s.angle_increment - FRONT_OFF)
            if abs(a) <= SECTOR: f.append(i)
            if SIDE_LO <= a <= SIDE_HI: l.append(i)
            if -SIDE_HI <= a <= -SIDE_LO: r.append(i)
            if abs(wrap(a - math.pi)) <= REAR_HALF: b.append(i)
        self._cache = (key, (f, l, r, b)); return self._cache[1]

    # ---------------------------------------------------------------- callbacks
    def on_scan(self, s):
        if self.depth_mode():
            return                                   # LiDAR gated off by Grok Bot: don't use it at all
        f, l, r, b = self._masks(s)
        lo = max(s.range_min, 0.02); hi = s.range_max
        def vals(idx): return [s.ranges[i] for i in idx if math.isfinite(s.ranges[i]) and lo <= s.ranges[i] <= hi]
        fv = vals(f)
        self.front = min(fv) if fv else float("inf")
        self.front_invalid = 1.0 - len(fv) / max(1, len(f))
        lv, rv, bv = vals(l), vals(r), vals(b)
        # side clearance: 10th percentile (robust to single noisy returns); no returns = open
        def p10(v): 
            if not v: return float(hi)
            v = sorted(v); return v[len(v) // 10]
        self.left, self.right = p10(lv), p10(rv)
        self.rear = min(bv) if bv else float(hi)
        # release only on a clearly valid, clear front sector (objects closer than range_min read invalid)
        self._set_front(self.front, self.front_invalid, "scan")
        self.last_scan = time.time(); self.scans += 1
        if self.blocked and self.forward_now():
            self.trigger("scan")

    def on_cmd(self, m, topic):
        fwd = m.linear.x > 1e-3
        self.cmd_fwd = fwd; self.cmd_src = topic; self.cmd_t = time.time()
        if not fwd or self.disabled() or self.mission_bypass():
            return
        if self.phase is not None:
            # someone pushed forward during our maneuver: re-assert our current command now
            self.n_supp += 1
            self.send(*self.current_cmd(), why="override")
            return
        if self.blocked:
            self.trigger("cmd " + topic)

    def forward_now(self):
        return self.cmd_fwd or (DRY and ASSUME_FWD)

    # ---------------------------------------------------------------- maneuver
    def trigger(self, why):
        if self.disabled() or self.mission_bypass():
            return
        now = time.time()
        if self.phase is not None:
            return
        if now < self.cool_until:
            # cooldown: no new course change, but still suppress forward motion
            if self.cmd_fwd:
                self.n_supp += 1
                self.send(0.0, 0.0, why="suppress")
                self.cmd_fwd = False
                self.get_logger().info("%sSUPPRESS forward (%s) front=%.3fm, cooldown %.1fs left"
                                       % ("[DRY] would " if DRY else "", why, self.front, self.cool_until - now))
            return
        self.n_trig += 1
        self.turn_ok = TURN and not self.depth_mode()          # depth mode: stop-only (no rear/side view)
        self.rev_ok = self.rear > REAR_MIN and self.turn_ok
        self.turn_dir = 1.0 if self.left >= self.right else -1.0   # +z = CCW = left
        self.phase = "stop"; self.t0 = now
        self.send(0.0, 0.0, why="stop")
        self.cmd_fwd = False
        plan = "stop %.1fs" % STOP_T
        if self.turn_ok:
            plan += ", %s, turn %s %.1f rad/s <=%.2fs" % (
                ("reverse %.2f m/s %.1fs" % (REV_V, REV_T)) if self.rev_ok else "NO reverse (rear %.2fm)" % self.rear,
                "LEFT" if self.turn_dir > 0 else "RIGHT", TURN_W, self.turn_budget())
        plan += ", then %d zero Twists" % FINAL_N
        self.get_logger().warn("%sOBSTACLE %.3fm in front (trigger: %s). L=%.2fm R=%.2fm rear=%.2fm -> %s"
                               % ("[DRY] would act: " if DRY else "", self.front, why, self.left, self.right, self.rear, plan))

    def turn_budget(self):
        return max(0.0, min(TURN_T, MAX_T - STOP_T - REV_T - FINAL_N / HZ))

    def current_cmd(self):
        if self.phase == "rev": return (-REV_V if self.rev_ok else 0.0, 0.0)
        if self.phase == "turn": return (0.0, self.turn_dir * TURN_W)
        return (0.0, 0.0)

    def tick(self):
        now = time.time()
        if self.phase is None:
            if self.depth_mode():
                lid, cam = self.caps()
                if (not cam or now - self.depth_t > DEPTH_STALE) and not self.blocked:
                    self.blocked = True; self.front = 0.0
                    self.get_logger().warn("depth mode without fresh depth (camera %s) -> forward motion suppressed"
                                           % ("gated off" if not cam else "stale"))
                if not cam or now - self.depth_t > DEPTH_STALE:
                    self.last_scan = now
            if self.blocked and self.forward_now() and now >= self.cool_until and now - self.last_scan < 0.5:
                self.trigger("tick")
            self.housekeeping(now)
            return
        el = now - self.t0
        if self.disabled() and self.phase != "final":
            self.get_logger().warn("kill switch set during maneuver -> final zeros")
            self.phase = "final"; self.final_left = FINAL_N
        if el >= MAX_T - FINAL_N / HZ and self.phase != "final":   # hard deadline
            self.phase = "final"; self.final_left = FINAL_N
        if self.phase == "stop" and el >= STOP_T:
            if self.turn_ok:
                self.phase = "rev"
            else:
                self.phase = "final"; self.final_left = FINAL_N
        if self.phase == "rev" and el >= STOP_T + REV_T:
            self.phase = "turn"; self.turn_end = now + self.turn_budget()
        if self.phase == "turn" and now >= self.turn_end:
            self.phase = "final"; self.final_left = FINAL_N
        if self.phase == "final":
            self.send(0.0, 0.0, why="final")
            self.final_left -= 1
            if self.final_left <= 0:
                self.phase = None; self.cool_until = now + COOLDOWN; self.cmd_fwd = False
                self.get_logger().info("%smaneuver done in %.2fs; cooldown %.1fs; front now %.3fm"
                                       % ("[DRY] " if DRY else "", el, COOLDOWN, self.front))
            return
        self.send(*self.current_cmd(), why=self.phase)

    def housekeeping(self, now):
        if now - self.last_status >= 1.0:
            self.last_status = now
            st = {"t": round(now, 2), "front_min": self.front, "front_invalid": round(self.front_invalid, 2),
                  "left": self.left, "right": self.right, "rear": self.rear, "blocked": self.blocked,
                  "cmd_forward": self.cmd_fwd, "cmd_src": self.cmd_src, "disabled": self.disabled(),
                  "mission_bypass": self.mission_bypass(), "mission_why": self._mb[2],
                  "sensor_mode": self._mode_logged, "depth_front": None if math.isinf(self.depth_front) else round(self.depth_front, 3),
                  "dry_run": DRY, "turn": TURN, "triggers": self.n_trig, "suppressed": self.n_supp,
                  "scan_age": round(now - self.last_scan, 2) if self.last_scan else None}
            try:
                tmp = STATUS + ".tmp"
                with open(tmp, "w") as fh: json.dump({k: (v if not (isinstance(v, float) and math.isinf(v)) else None) for k, v in st.items()}, fh)
                os.replace(tmp, STATUS)
            except Exception:
                pass
        if LOG_EVERY > 0 and now - self.last_log >= LOG_EVERY:
            self.last_log = now
            age = now - self.last_scan if self.last_scan else -1
            self.get_logger().info("front_min=%.3fm (invalid %.0f%%) L=%.2f R=%.2f rear=%.2f blocked=%s cmd_fwd=%s disabled=%s scans=%d age=%.1fs"
                                   % (self.front, 100 * self.front_invalid, self.left, self.right, self.rear, self.blocked,
                                      self.cmd_fwd, self.disabled(), self.scans, age))

    def safe_shutdown(self):
        if self.phase is not None and not DRY:
            for _ in range(FINAL_N):
                self.send(0.0, 0.0); time.sleep(1.0 / HZ)

def main():
    rclpy.init()
    g = Guard()
    def bye(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, bye)
    try:
        rclpy.spin(g)
    except KeyboardInterrupt:
        pass
    finally:
        try: g.safe_shutdown()
        except Exception: pass
        g.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()