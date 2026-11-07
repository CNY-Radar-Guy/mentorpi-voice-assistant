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
