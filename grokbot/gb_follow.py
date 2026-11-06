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
