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

def load():
    """(state, why). why is ok, missing, or malformed. Malformed does not become 'both on'."""
    if not os.path.exists(F):
        return dict(DEFAULT), "missing"
    try:
        with open(F) as f:
            raw = json.load(f)
    except Exception:
        return None, "malformed"
    if not isinstance(raw, dict) or not isinstance(raw.get("lidar"), bool) or not isinstance(raw.get("camera"), bool):
        return None, "malformed"
    try:
        p = int(round(float(raw.get("speed_pct", 100)) / STEP) * STEP)
    except (TypeError, ValueError):
        return None, "malformed"
    return {"lidar": raw["lidar"], "camera": raw["camera"], "speed_pct": max(MIN_PCT, min(MAX_PCT, p))}, "ok"

def get():
    """Fresh read every call. Missing file keeps the in-memory default and is not a hardware fault.
    A saved false stays off. Malformed JSON does not grant the default."""
    d, why = load()
    if why == "malformed" or d is None:
        return {"lidar": False, "camera": False, "speed_pct": 100, "trusted": False, "reason": "malformed"}
    d["trusted"] = why == "ok"
    d["reason"] = why
    return d

def save(d):
    """Write only the user gate and speed. Fault strings are not stored and cannot turn a sensor off."""
    clean = {"lidar": bool(d["lidar"]), "camera": bool(d["camera"]), "speed_pct": int(d["speed_pct"])}
    _atomic(F, clean); _atomic(GUARD_MIRROR, clean); return clean

def sync():
    """Re-mirror a valid user gate. A missing or malformed file is left untouched."""
    d, why = load()
    if why != "ok":
        return {"trusted": False, "reason": why}
    return save(d)

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

def startup_banner(health_mode=None, user=None):
    """Spoken/logged status from a fresh caps read plus physical health. Nothing here is cached.

    health_mode is 'lidar', 'depth', 'none', or None when health has not reported yet.
    A saved lidar:false is 'switched off by choice', never a port fault.
    """
    user = get() if user is None else user
    if user.get("reason") == "malformed" or user.get("trusted") is False and user.get("reason") == "malformed":
        return "The capability file is unreadable, so I will not drive myself."
    if not user.get("lidar", True) and not user.get("camera", True):
        return MSG_BOTH_OFF
    if not user.get("lidar", True):
        return "LiDAR is switched off by choice. The depth camera can still guard short drives."
    if health_mode == "none":
        return "I cannot drive. I have no LiDAR and no depth camera. Remote control input only."
    if health_mode == "depth":
        return "LiDAR unavailable, switching to depth camera."
    if health_mode == "lidar":
        return "LiDAR online"
    return "LiDAR and camera are switched on. Health is reported separately from this switch."