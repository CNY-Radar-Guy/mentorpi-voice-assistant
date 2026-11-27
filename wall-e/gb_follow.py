"""gb_follow.py - slow person follow at 2 metres.

LiDAR range is primary. A fresh depth range is the fallback. Neither means
stop and say the no-range line. The controller never commands a closing speed
once the person is inside the 2 m standoff, and it stops well before contact.

This is an original distance-hold loop in the same family as the common ROS
"follow the range" behaviour. It does not copy Nav2 or turtlebot follower source.
Nav2 itself is Apache-2.0 (https://github.com/ros-navigation/navigation2).
"""
import gb_health

STANDOFF_M = float(__import__("os").environ.get("FOLLOW_RANGE_M", "2.0"))
DEADBAND_M = 0.25
MAX_LIN = 0.08
MAX_ANG = 0.40
CONTACT_M = 1.50  # inside this, do not drive toward the person


def follow_command(range_m, source, bearing=0.0, fresh=True, autonomy=False, protections=None):
    """Returns dict command, speech, allow.

    source: 'lidar', 'depth', or 'none'. RGB is not a source.
    """
    protections = protections or {}
    if source == "rgb":
        source = "none"
    pol = gb_health.motion_policy("autonomous", "none" if not fresh else source, protections)
    if protections.get("estop") or protections.get("tip_hard") or protections.get("tip_soft") or protections.get("battery_shutdown") or protections.get("watchdog"):
        return {"lin": 0.0, "ang": 0.0, "speech": pol.speech, "allow": False, "reason": pol.reason}
    if not autonomy:
        return {"lin": 0.0, "ang": 0.0, "speech": "Autonomy is off. I will not follow.", "allow": False, "reason": "autonomy_off"}
    if source not in ("lidar", "depth") or not fresh or range_m is None:
        return {"lin": 0.0, "ang": 0.0, "speech": gb_health.NO_RANGE, "allow": False, "reason": "no_range"}
    try:
        dist = float(range_m)
        bear = float(bearing or 0.0)
    except (TypeError, ValueError):
        return {"lin": 0.0, "ang": 0.0, "speech": gb_health.NO_RANGE, "allow": False, "reason": "no_range"}
    if dist != dist:
        return {"lin": 0.0, "ang": 0.0, "speech": gb_health.NO_RANGE, "allow": False, "reason": "no_range"}
    lin = 0.0
    if dist >= CONTACT_M and dist > STANDOFF_M + DEADBAND_M:
        lin = min(MAX_LIN, 0.12 * (dist - STANDOFF_M))
        if source == "depth":
            lin = min(lin, MAX_LIN * 0.5)
    ang = 0.0
    if abs(bear) > 0.20 and dist >= CONTACT_M:
        ang = max(-MAX_ANG, min(MAX_ANG, bear))
        if abs(bear) > 0.50:
            lin = 0.0  # turn in place rather than arc into an unknown side
    return {"lin": lin, "ang": ang, "speech": None, "allow": True, "reason": source}


CONF_MIN = 0.75
LOST_S = 1.0
MAX_RUN_S = 120.0
DEPTH_FRESH_S = 0.5


def _usable_track(track):
    """Return (track, reason). A missing confidence is not a person."""
    if not track or not track.get("fresh"):
        return None, "person_lost"
    if track.get("ambiguous"):
        return None, "ambiguous"
    try:
        conf = float(track["conf"])
    except (KeyError, TypeError, ValueError):
        return None, "confidence"
    if conf < CONF_MIN:
        return None, "confidence"
    if track.get("source") == "depth":
        try:
            age = float(track.get("depth_age", 0))
        except (TypeError, ValueError):
            return None, "depth_stale"
        if age > DEPTH_FRESH_S:
            return None, "depth_stale"
        if track.get("unit") not in gb_health.VALIDATED_DEPTH_UNITS:
            return None, "depth_unit"
    return track, None


class FollowLoop(object):
    """Closed loop. Each step is one command. Loss, timeout, and safety cancel it.

    The loop does not invent a range. Autonomy off cancels rather than arming a later move.
    """

    def __init__(self):
        self.active = False
        self.t0 = None
        self.lost_since = None
        self.reason = None
        self.had_track = False

    def start(self, now):
        self.active = True
        self.t0 = float(now)
        self.lost_since = None
        self.reason = None
        self.had_track = False

    def cancel(self, why):
        self.active = False
        self.reason = why
        speech = {
            "person_lost": "I can't get a clear distance on you. Can you stand back further, about three meters, so I can track you?",
            "person_lost_later": "I lost the person, so I stopped.",
            "timeout": "I followed for too long, so I stopped.",
            "confidence": "I'm not sure that's a person, so I stopped.",
            "ambiguous": "I see more than one person, so I won't follow.",
            "depth_stale": "The depth reading is stale, so I stopped.",
            "depth_unit": "The depth unit is not checked, so I won't follow.",
            "estop": "Emergency stop.",
            "maintenance": "The maintenance lock is on, so I won't follow.",
            "health": "I won't follow until the range check is fresh.",
            "autonomy_off": "Autonomy is off. I will not follow.",
        }.get(why, "I'm staying put.")
        return {"lin": 0.0, "ang": 0.0, "allow": False, "op": "stop", "reason": why, "speech": speech}

    def step(self, now, health_snap, track, autonomy, protections=None, maint=False):
        if not self.active:
            return {"lin": 0.0, "ang": 0.0, "allow": False, "op": "idle", "reason": "idle", "speech": None}
        now = float(now)
        protections = protections or {}
        if maint or protections.get("maint"):
            return self.cancel("maintenance")
        if protections.get("estop") or protections.get("tip_hard") or protections.get("tip_soft") or protections.get("battery_shutdown") or protections.get("watchdog"):
            return self.cancel("estop" if protections.get("estop") else "safety")
        if now - self.t0 > MAX_RUN_S:
            return self.cancel("timeout")
        if not autonomy:
            return self.cancel("autonomy_off")
        usable, why = _usable_track(track)
        if usable is None:
            if why in ("confidence", "ambiguous", "depth_stale", "depth_unit"):
                return self.cancel(why)
            if not self.had_track:
                return self.cancel("person_lost")
            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since >= LOST_S:
                return self.cancel("person_lost_later")
            return {"lin": 0.0, "ang": 0.0, "allow": False, "op": "hold", "reason": "person_gap", "speech": None}
        self.had_track = True
        self.lost_since = None
        if not gb_health.autonomy_allowed(health_snap, now):
            return self.cancel("health")
        cmd = follow_command(usable["depth_m"], usable["source"], usable.get("bearing") or 0.0, True,
                             autonomy=True, protections=protections)
        cmd["op"] = "timed" if cmd.get("allow") and (cmd.get("lin") or cmd.get("ang")) else "hold"
        if not cmd.get("allow"):
            return self.cancel(cmd.get("reason") or "refused")
        return cmd
