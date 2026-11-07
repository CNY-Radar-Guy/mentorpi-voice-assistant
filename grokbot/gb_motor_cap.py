"""Half of the documented app command box, per track motor.

Source, installed controller node odom_publisher_node.py:
  app_cmd_vel_callback clamps only the cmd_vel topic to ±0.2 m/s and ±0.5 rad/s.
  The same node builds MecanumChassis(wheelbase=0.1368, track_width=0.1446,
  wheel_diameter=0.075) and converts with rps = v / (pi * diameter).
  controller/cmd_vel is not clamped there. set_motor_speed writes the float
  with no limit. Motor type JGB37 (0x01) has no maximum in the SDK.

50% means half of the fastest track speed at a corner of that ±0.2 / ±0.5 box,
in either direction, including reverse and a differential spin. It is a ROS
command ceiling. It is not a firmware rating. A write to /dev/rrc does not
pass through it. This module does not publish.
"""
import json, math, os

APP_LIN_MAX = 0.2
APP_ANG_MAX = 0.5
WHEELBASE = 0.1368
TRACK_WIDTH = 0.1446
WHEEL_DIAMETER = 0.075
SOURCE = "odom_publisher_node.py app_cmd_vel_callback ±0.2 m/s ±0.5 rad/s; MecanumChassis wheel_diameter 0.075 track_width 0.1446 wheelbase 0.1368"
CORRECTION_YAML = "/home/ubuntu/software/chassis_adjustment/robot_correction_factors.yaml"
STRAIGHT_CORRECTION_FACTOR = 5.50


def _span():
    return (WHEELBASE + TRACK_WIDTH) / 2.0


def track_mps(lin, ang):
    return abs(float(lin)) + abs(float(ang)) * _span()


def mps_to_rps(speed):
    return float(speed) / (math.pi * WHEEL_DIAMETER)


def peak_rps():
    return mps_to_rps(track_mps(APP_LIN_MAX, APP_ANG_MAX))


def half_rps():
    return peak_rps() / 2.0


def tank_rps_map(lin, ang, linear_y=0.0):
    """Per-motor rps from the installed tank call.

    odom_publisher builds MecanumChassis(0.1368, 0.1446, 0.075) and, for
    MentorPi_Tank, calls set_velocity(linear_x, 0, angular_z). mecanum.py
    then stores [-m1, -m2, m3, m4] with m1 = vx - vy - w*span. The tank
    odometry factor 0.52 is applied to the pose integral, not to these rps.
    A left/right correction that rewrites a straight command is not applied
    here; motors that differ from this map are rejected.
    """
    span = _span()
    vx = float(lin)
    vy = float(linear_y)
    w = float(ang)
    motor1 = vx - vy - w * span
    motor2 = vx + vy - w * span
    motor3 = vx + vy + w * span
    motor4 = vx - vy + w * span
    raw = (-motor1, -motor2, motor3, motor4)
    return [(i + 1, mps_to_rps(v)) for i, v in enumerate(raw)]


def load_track_correction(path=None):
    """Left and right factors from the same file odom_publisher reads.

    The incident archive did not contain robot_correction_factors.yaml.
    A missing file uses 1.0 and 1.0, which is that publisher's own fallback.
    """
    path = path or os.environ.get("GB_CORRECTION_YAML") or CORRECTION_YAML
    left = right = 1.0
    try:
        import yaml
        with open(path) as handle:
            factors = yaml.safe_load(handle) or {}
        if isinstance(factors, dict):
            left = float(factors.get("left_correction_factor", 1.0))
            right = float(factors.get("right_correction_factor", 1.0))
    except Exception:
        left = right = 1.0
    if left != left or right != right or math.isinf(left) or math.isinf(right):
        return 1.0, 1.0
    return left, right


def correct_tank_twist(lin, ang, left=1.0, right=1.0):
    """The tank branch in odom_publisher cmd_vel_callback, before set_velocity.

    Only a straight non-negative command is rewritten, and only when a factor
    is not 1. Turning commands are unchanged. linear_y stays 0.
    """
    lin = float(lin)
    ang = float(ang)
    if lin >= 0.0 and ang == 0.0 and (float(left) != 1.0 or float(right) != 1.0):
        ang = lin * (float(right) - float(left)) * STRAIGHT_CORRECTION_FACTOR
    return lin, ang


def motors_match_twist(pairs, lin, ang, tol=1e-4):
    """True when the four rps values are the tank map of this twist."""
    try:
        expected = {mid: rps for mid, rps in tank_rps_map(lin, ang)}
    except (TypeError, ValueError):
        return False
    got = {}
    for item in pairs or ():
        try:
            mid = int(item[0])
            rps = float(item[1])
        except (TypeError, ValueError, IndexError):
            return False
        if mid in got or rps != rps or math.isinf(rps):
            return False
        got[mid] = rps
    if set(got) != set(expected):
        return False
    return all(abs(got[i] - expected[i]) <= tol for i in expected)


def clamp_motors(pairs):
    """[(id, rps), ...] clamped to ±half_rps, or None if the list is unusable."""
    if not isinstance(pairs, (list, tuple)) or not pairs:
        return None
    limit = half_rps()
    out = []
    seen = set()
    for item in pairs:
        try:
            mid = int(item[0])
            rps = float(item[1])
        except (TypeError, ValueError, IndexError):
            return None
        if mid < 1 or mid > 4 or mid in seen or rps != rps or math.isinf(rps):
            return None
        seen.add(mid)
        if rps > limit:
            rps = limit
        elif rps < -limit:
            rps = -limit
        out.append((mid, rps))
    return out


def config_body():
    return {
        "source": SOURCE,
        "app_lin_max_m_s": APP_LIN_MAX,
        "app_ang_max_rad_s": APP_ANG_MAX,
        "wheel_diameter_m": WHEEL_DIAMETER,
        "wheelbase_m": WHEELBASE,
        "track_width_m": TRACK_WIDTH,
        "peak_rps": peak_rps(),
        "half_rps": half_rps(),
        "fraction": 0.5,
        "provisional_software_cap": True,
        "hardware_top_speed_verified": False,
        "firmware_cap": False,
        "serial_bypass": True,
        "enforced_on_robot": False,
        "physical_speed_verified": False,
    }


def config_ok(body):
    """A saved file may not raise the half-speed ceiling."""
    try:
        if body.get("firmware_cap") is not False or body.get("serial_bypass") is not True:
            return False
        if body.get("hardware_top_speed_verified") is not False or body.get("provisional_software_cap") is not True:
            return False
        if abs(float(body.get("half_rps")) - half_rps()) > 1e-9:
            return False
        if float(body.get("fraction")) != 0.5:
            return False
    except (TypeError, ValueError, AttributeError):
        return False
    return True


def write_config(path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(config_body(), f)
    os.replace(tmp, path)
    return path


def read_config(path):
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    if not config_ok(body):
        return None
    return body
