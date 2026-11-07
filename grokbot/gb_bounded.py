"""45 second odometry look through the gate. Not a map and not Nav2.

Home is the filtered /odom pose when the look starts. Wheel /odom_raw is not
used and is not a fallback. The lidar yaw is the URDF zero plus the user's
8:04 confirmation that it is aligned. That is not an instrumented beam test
and not a camera extrinsic calibration. The 0.15 m stop is an odometry
threshold, not a surveyed accuracy.
"""
import json, math, os
import gb_obstacle as OB

CAP_S = 45.0
RETURN_CAP_S = 45.0
MAX_LIN = 0.08
MAX_ANG = 0.5
SCAN_FRESH_S = 0.5
ODOM_FRESH_S = 0.5
ODOM_NEAR_M = 0.15
JUMP_M = 0.40
YAW_JUMP = 1.2
ALIGN_RAD = 0.35
MAX_SPAN_M = MAX_LIN * CAP_S + 0.5
PIN_SOURCE = "/odom"
PIN_FRAME = "odom"
PIN_CHILD = "base_footprint"
LIDAR_YAW_RAD = 0.0
LIDAR_ALIGN_NOTE = "urdf rpy 0 and user t906 8:04 it is aligned; not an instrumented scan and not camera extrinsics"


def wrap(a):
    a = float(a)
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def fresh_age(age, limit):
    try:
        age = float(age)
    except (TypeError, ValueError):
        return False
    return age == age and 0.0 <= age <= float(limit)


def _finite_pose(odom):
    try:
        x, y, yaw = float(odom[0]), float(odom[1]), float(odom[2])
    except (TypeError, ValueError, IndexError):
        return None
    if x != x or y != y or yaw != yaw:
        return None
    return (x, y, yaw)


class BoundedLook(object):
    def __init__(self):
        self.requested = False
        self.active = False
        self.t0 = None
        self.home = None
        self.phase = "idle"
        self.reason = None
        self.source = None
        self.frame = None
        self.child = None
        self.last = None
        self.cap = CAP_S
        self.return_t0 = None
        self.return_from = None
        self.return_budget = None
        self.lidar_yaw = LIDAR_YAW_RAD

    def load(self, path):
        """A file from an earlier boot is not a mission. Motion stays off."""
        self.requested = False
        self.active = False
        self.phase = "idle"
        self.home = None
        self.source = None
        try:
            with open(path) as f:
                body = json.load(f)
            if isinstance(body, dict) and body.get("home"):
                self.reason = "not_resumed"
        except (OSError, ValueError):
            self.reason = None
        return self

    def save(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        body = {
            "requested": False, "active": False, "home": self.home, "phase": "idle",
            "reason": self.reason, "source": self.source, "accuracy_claimed": False,
        }
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(body, f)
        os.replace(tmp, path)

    def request(self):
        """Ask for a look. Does not move and does not mark home."""
        self.requested = True
        self.active = False
        self.phase = "wait_sensors"
        self.reason = None
        return self

    def begin(self, now, odom, cap_s=None, source=PIN_SOURCE, frame=PIN_FRAME, child=PIN_CHILD, lidar_yaw=LIDAR_YAW_RAD):
        pose = _finite_pose(odom)
        if pose is None or source != PIN_SOURCE or frame != PIN_FRAME or child != PIN_CHILD:
            self.active = False
            self.phase = "idle"
            self.reason = "source_refused"
            return self
        try:
            yaw = LIDAR_YAW_RAD if lidar_yaw is None else float(lidar_yaw)
        except (TypeError, ValueError):
            yaw = None
        if yaw is None or abs(wrap(yaw)) > 0.2:
            self.active = False
            self.phase = "idle"
            self.reason = "lidar_yaw_disagrees"
            return self
        self.requested = True
        self.active = True
        self.t0 = float(now)
        self.home = pose
        self.last = pose
        self.phase = "out"
        self.reason = None
        self.source = PIN_SOURCE
        self.frame = PIN_FRAME
        self.child = PIN_CHILD
        self.lidar_yaw = LIDAR_YAW_RAD
        self.return_t0 = None
        self.return_from = None
        self.return_budget = None
        try:
            cap = CAP_S if cap_s is None else float(cap_s)
        except (TypeError, ValueError):
            cap = CAP_S
        self.cap = max(5.0, min(90.0, cap))
        return self

    def start_return(self):
        if not self.active or self.home is None:
            return False
        self.phase = "return"
        return True

    def cancel(self, why):
        self.requested = False
        self.active = False
        self.phase = "idle"
        self.reason = why
        return {"op": "stop", "lin": 0.0, "ang": 0.0, "allow": False, "reason": why, "accuracy_claimed": False}

    def step(self, now, front_m, rear_m, sweep_m, odom, scan_age, odom_age, estop=False, maint=False, source=PIN_SOURCE):
        """One gate request. Stops inside the existing 0.27 / 0.28 / 0.24 limits."""
        if not self.active:
            return {"op": "idle", "lin": 0.0, "ang": 0.0, "allow": False, "reason": self.phase, "accuracy_claimed": False}
        if estop or maint:
            return self.cancel("estop" if estop else "maintenance")
        if source != self.source:
            return self.cancel("source_switch")
        if not fresh_age(scan_age, SCAN_FRESH_S) or not fresh_age(odom_age, ODOM_FRESH_S):
            return self.cancel("sensors_stale")
        pose = _finite_pose(odom)
        try:
            front, rear, sweep = float(front_m), float(rear_m), float(sweep_m)
        except (TypeError, ValueError):
            pose = None
        if pose is None or front != front or rear != rear or sweep != sweep:
            return self.cancel("sensors_stale")
        jumped = self._jump(pose)
        if jumped is not None:
            return jumped
        now = float(now)
        if self.phase == "out" and now - self.t0 >= self.cap:
            self.phase = "return"
        if self.phase == "return":
            return self._return(now, pose, front, rear, sweep)
        if front <= OB.STOP_FWD or sweep <= OB.SWEEP_R:
            if sweep <= OB.SWEEP_R or rear <= OB.STOP_REV:
                return self._stop("obstacle")
            return self._cmd(0.0, MAX_ANG if front <= OB.STOP_FWD else 0.0, "turn_away")
        return self._cmd(MAX_LIN, 0.0, "bounded_out")

    def _jump(self, pose):
        if self.last is None:
            self.last = pose
            return None
        dist = math.hypot(pose[0] - self.last[0], pose[1] - self.last[1])
        dyaw = abs(wrap(pose[2] - self.last[2]))
        self.last = pose
        if dist > JUMP_M or dyaw > YAW_JUMP:
            return self.cancel("odom_jump")
        return None

    def _stop(self, why):
        return {"op": "stop", "lin": 0.0, "ang": 0.0, "allow": False, "reason": why, "accuracy_claimed": False}

    def _cmd(self, lin, ang, why):
        lin = max(-MAX_LIN, min(MAX_LIN, float(lin)))
        ang = max(-MAX_ANG, min(MAX_ANG, float(ang)))
        return {"op": "timed", "lin": lin, "ang": ang, "allow": True, "reason": why, "accuracy_claimed": False}

    def _return(self, now, pose, front, rear, sweep):
        if self.home is None:
            return self.cancel("no_home")
        if front <= OB.STOP_FWD or rear <= OB.STOP_REV or sweep <= OB.SWEEP_R:
            return self.cancel("obstacle")
        dx = self.home[0] - pose[0]
        dy = self.home[1] - pose[1]
        dist = math.hypot(dx, dy)
        if dist > MAX_SPAN_M:
            return self.cancel("odom_implausible")
        if dist < ODOM_NEAR_M:
            done = self.cancel("odom_near")
            done["accuracy_claimed"] = False
            return done
        if self.return_t0 is None:
            self.return_t0 = float(now)
            self.return_from = (pose[0], pose[1])
            self.return_budget = min(MAX_SPAN_M, dist * 1.25 + 0.25)
        if float(now) - self.return_t0 > RETURN_CAP_S:
            return self.cancel("return_timeout")
        traveled = math.hypot(pose[0] - self.return_from[0], pose[1] - self.return_from[1])
        if traveled > self.return_budget:
            return self.cancel("return_budget")
        err = wrap(math.atan2(dy, dx) - pose[2])
        if abs(err) > ALIGN_RAD:
            return self._cmd(0.0, math.copysign(MAX_ANG, err), "return_turn")
        return self._cmd(min(MAX_LIN, dist), max(-MAX_ANG, min(MAX_ANG, err)), "return_odom")
