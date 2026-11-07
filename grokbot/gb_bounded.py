"""Short room look through the gate. Not Nav2 and not a saved-map mission.

Home is the odometry pose at the moment a person asks. A restart loads nothing
into motion. The 92 mm / 127 mm camera notes are not used.
"""
import json, os
import gb_obstacle as OB

CAP_S = 45.0
MAX_LIN = 0.08
MAX_ANG = 0.5
SCAN_FRESH_S = 0.5
ODOM_FRESH_S = 0.5


class BoundedLook(object):
    def __init__(self):
        self.requested = False
        self.active = False
        self.t0 = None
        self.home = None
        self.phase = "idle"
        self.reason = None

    def load(self, path):
        """A file from an earlier boot is not a mission. Motion stays off."""
        self.requested = False
        self.active = False
        self.phase = "idle"
        self.home = None
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
        body = {"requested": False, "active": False, "home": self.home, "phase": "idle", "reason": self.reason}
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(body, f)
        os.replace(tmp, path)

    def request(self):
        """Ask for a look. Does not move and does not mark home until odometry is fresh."""
        self.requested = True
        self.active = False
        self.phase = "wait_sensors"
        self.reason = None
        return self

    def begin(self, now, odom):
        self.requested = True
        self.active = True
        self.t0 = float(now)
        self.home = (float(odom[0]), float(odom[1]), float(odom[2]))
        self.phase = "out"
        self.reason = None
        return self

    def cancel(self, why):
        self.requested = False
        self.active = False
        self.phase = "idle"
        self.reason = why
        return {"op": "stop", "lin": 0.0, "ang": 0.0, "allow": False, "reason": why}

    def step(self, now, front_m, rear_m, sweep_m, odom, scan_age, odom_age, estop=False, maint=False):
        """One gate request. Stops inside the existing 0.27 / 0.28 / 0.24 limits."""
        if not self.active:
            return {"op": "idle", "lin": 0.0, "ang": 0.0, "allow": False, "reason": self.phase}
        if estop or maint:
            return self.cancel("estop" if estop else "maintenance")
        try:
            if float(scan_age) > SCAN_FRESH_S or float(odom_age) > ODOM_FRESH_S:
                return self.cancel("sensors_stale")
            front, rear, sweep = float(front_m), float(rear_m), float(sweep_m)
        except (TypeError, ValueError):
            return self.cancel("sensors_stale")
        now = float(now)
        if self.phase == "out" and now - self.t0 >= CAP_S:
            self.phase = "return"
        if self.phase == "return":
            return self._return(odom)
        if front <= OB.STOP_FWD or sweep <= OB.SWEEP_R:
            if sweep <= OB.SWEEP_R or rear <= OB.STOP_REV:
                return {"op": "stop", "lin": 0.0, "ang": 0.0, "allow": False, "reason": "obstacle"}
            return {"op": "timed", "lin": 0.0, "ang": MAX_ANG if front <= OB.STOP_FWD else 0.0,
                    "allow": True, "reason": "turn_away"}
        return {"op": "timed", "lin": MAX_LIN, "ang": 0.0, "allow": True, "reason": "bounded_out"}

    def _return(self, odom):
        if self.home is None:
            return self.cancel("no_home")
        dx = self.home[0] - float(odom[0])
        dy = self.home[1] - float(odom[1])
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 0.15:
            return self.cancel("home")
        return {"op": "timed", "lin": min(MAX_LIN, dist), "ang": 0.0, "allow": True, "reason": "return_odom"}
