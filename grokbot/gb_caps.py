"""gb_caps.py - persisted robot capability + speed state shared by the voice service, features, missions and gb-guard.
State file ~/grokbot/data/caps.json = {"lidar": bool, "camera": bool, "speed_pct": 50..100}.
"Off" is a software gate (the ROS drivers keep running): nothing in Grok Bot uses a gated sensor, and gb-guard
switches to a depth-camera front stop when the LiDAR is gated. A copy is mirrored into gb-guard's shared folder.
Revert: delete caps.json (defaults: both on, 100 %)."""
import json, os
DATA = os.path.expanduser(os.environ.get("GB_DATA", "~/grokbot/data"))
F = os.path.join(DATA, "caps.json")
GUARD_MIRROR = os.path.expanduser(os.environ.get("GUARD_CAPS_HOST", "/home/pi/docker/tmp/gb_guard/caps.json"))
DEFAULT = {"lidar": True, "camera": True, "speed_pct": 100}
MIN_PCT, MAX_PCT, STEP = 50, 100, 10

def _atomic(path, d):
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + ".tmp", "w") as f: json.dump(d, f)
        os.replace(path + ".tmp", path)
    except OSError:
        pass

def get():
    d = dict(DEFAULT)
    try:
        with open(F) as f: d.update(json.load(f))
    except Exception:
        pass
    d["lidar"] = bool(d["lidar"]); d["camera"] = bool(d["camera"])
    try: p = int(round(float(d["speed_pct"]) / STEP) * STEP)
    except Exception: p = 100
    d["speed_pct"] = max(MIN_PCT, min(MAX_PCT, p))
    return d

def save(d):
    _atomic(F, d); _atomic(GUARD_MIRROR, d); return d

def sync():
    """Re-mirror the current state for gb-guard (call at service start)."""
    return save(get())

def set_sensor(name, on):
    d = get(); d[name] = bool(on); return save(d)

def set_speed_pct(p):
    d = get(); d["speed_pct"] = max(MIN_PCT, min(MAX_PCT, int(round(p / STEP) * STEP))); return save(d)

def speed_mult():
    return get()["speed_pct"] / 100.0

MSG_LIDAR_OFF = "My LiDAR is off, so I can only do short camera-guarded drives; I can't map or explore."
MSG_CAMERA_OFF = "My camera is off, so I'm limited to the LiDAR and can't see that."
MSG_BOTH_OFF = "My LiDAR and camera are both off, so I won't move. Turn one of them back on first."

def motion_block(kind="drive"):
    """kind: drive (short timed drive) | mission (SLAM/Nav2/explore) | lidar_feature | camera_feature. None = allowed."""
    d = get()
    if not d["lidar"] and not d["camera"]:
        return MSG_BOTH_OFF
    if kind in ("mission", "lidar_feature") and not d["lidar"]:
        return MSG_LIDAR_OFF
    if kind == "camera_feature" and not d["camera"]:
        return MSG_CAMERA_OFF
    return None

def vision_block():
    return None if get()["camera"] else MSG_CAMERA_OFF