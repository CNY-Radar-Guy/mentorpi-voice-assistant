"""gb_health.py - LiDAR and RGB-D freshness for Wally.

LiDAR is the primary range sensor for navigation, obstacle avoidance, and the
2 m follow. A fresh, valid depth frame is the fallback. RGB without metric
depth is not a range sensor. An old LiDAR fault is a note, not a latch: a
valid depth frame is allowed immediately.

Unknown sectors (too few real returns) are not clear. Autonomous commands
stop when neither sensor is fresh. Teleop is still allowed, and it still
loses to emergency stop, tip, battery shutdown, and the watchdog.

Spoken lines (exact):
  LiDAR online
  RGB-D camera online
  LiDAR unavailable, switching to depth camera.
  I cannot drive. I have no LiDAR and no depth camera. Remote control input only.
"""
import json, os, time

LIDAR_ONLINE = "LiDAR online"
DEPTH_ONLINE = "RGB-D camera online"
LIDAR_FALLBACK = "LiDAR unavailable, switching to depth camera."
NO_RANGE = "I cannot drive. I have no LiDAR and no depth camera. Remote control input only."

LIDAR_FRESH_S = float(os.environ.get("HEALTH_LIDAR_FRESH_S", "0.5"))
DEPTH_FRESH_S = float(os.environ.get("HEALTH_DEPTH_FRESH_S", "1.0"))
# A healthy /scan_raw is about 400 beams with ~384-394 finite. Half-empty is not "online".
MIN_VALID_FRAC = float(os.environ.get("HEALTH_MIN_VALID_FRAC", "0.50"))
# Autonomous depth is off until a real unit check. A driver scale string alone is not enough.
VALIDATED_DEPTH_UNITS = {"crosscheck_mm", "crosscheck_m", "32FC1_metres", "documented_mm"}
GOOD_N = int(os.environ.get("HEALTH_GOOD_N", "2"))
BAD_N = int(os.environ.get("HEALTH_BAD_N", "2"))
# Pi 5 is 4 cores. A healthy idle near 2.2 must not shed. Optional vision
# stands down only above this 1-minute load. Safety loops ignore it.
LOAD_SHED = float(os.environ.get("WALLY_LOAD_SHED", "3.5"))
def _default_health_path():
    explicit = os.environ.get("HEALTH_FILE") or os.environ.get("GB_HEALTH_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    try:
        import gb_paths
        return os.path.join(os.path.dirname(gb_paths.shared_estop()), "health.json")
    except Exception:
        return os.path.expanduser("~/docker/tmp/gb_safety/health.json")


SHARED = _default_health_path()


def depth_valid_fraction(samples, scale_m, min_m=0.15, max_m=6.0):
    """Fraction of pixels that are real metres. scale_m None or <= 0 => 0 (unit unknown)."""
    try:
        scale = float(scale_m)
    except (TypeError, ValueError):
        return 0.0
    if not (scale > 0):
        return 0.0
    n = len(samples)
    if n <= 0:
        return 0.0
    good = 0
    for z in samples:
        if z is None:
            continue
        try:
            metres = float(z) * scale
        except (TypeError, ValueError):
            continue
        if metres == metres and min_m <= metres <= max_m:
            good += 1
    return good / float(n)


def sector_is_clear(valid_count, total, nearest_m, stop_m, min_frac=0.5):
    """A sector with too few real ranges is unknown, and unknown is not clear."""
    if total <= 0:
        return False
    if valid_count / float(total) < min_frac:
        return False
    if nearest_m is None:
        return False
    try:
        d = float(nearest_m)
    except (TypeError, ValueError):
        return False
    if d != d:  # NaN
        return False
    return d >= float(stop_m)


# Observed /scan_raw offer on the robot: one publisher, reliable, volatile.
# The health node subscribes best-effort volatile, which is compatible with that offer.
# A transient_local (latched) subscriber is not, and receives nothing while health stays fresh.
SCAN_PUBLISHER_QOS = {"reliability": "reliable", "durability": "volatile"}
HEALTH_SCAN_QOS = {"reliability": "best_effort", "durability": "volatile", "history": "keep_last", "depth": 5}
ONEOFF_SCAN_QOS = {"reliability": "reliable", "durability": "volatile", "history": "keep_last", "depth": 5}


def qos_compatible(publisher, subscriber):
    """ROS durability and reliability. Incompatible QoS is silence, not a dead lidar."""
    try:
        pub_r = publisher["reliability"]
        pub_d = publisher["durability"]
        sub_r = subscriber["reliability"]
        sub_d = subscriber["durability"]
    except (TypeError, KeyError):
        return False
    if sub_r == "best_effort":
        rel_ok = pub_r in ("best_effort", "reliable")
    elif sub_r == "reliable":
        rel_ok = pub_r == "reliable"
    else:
        rel_ok = False
    if sub_d == "volatile":
        dur_ok = pub_d in ("volatile", "transient_local")
    elif sub_d == "transient_local":
        dur_ok = pub_d == "transient_local"
    else:
        dur_ok = False
    return rel_ok and dur_ok


def sampler_silence(publisher, subscriber, receive_age_s, limit=1.0):
    """Why a one-off read can be empty while a long-running subscriber is fresh."""
    if not qos_compatible(publisher, subscriber):
        return "qos_incompatible"
    return "scan_raw" if receive_time_fresh(receive_age_s, limit=limit) else "no_receive"


def scan_source_ready(topics, receive_age_s, limit=1.0):
    """/scan_raw with a fresh receive time. Absence of /scan is not a fault.

    Freshness is when this process received the message, not the header stamp.
    """
    names = set(topics or [])
    if "/scan_raw" not in names:
        return False, "scan_raw_absent"
    try:
        age = float(receive_age_s)
    except (TypeError, ValueError):
        return False, "scan_raw_stale"
    if age < -1.0 or age > float(limit):
        return False, "scan_raw_stale"
    return True, "scan_raw"


def receive_time_fresh(receive_age, header_age=None, limit=1.0):
    """A header in another clock domain cannot extend a stale receive time."""
    try:
        age = float(receive_age)
    except (TypeError, ValueError):
        return False
    if age < -1.0 or age > float(limit):
        return False
    return True


def lidar_status(ranges, frame_id=None):
    """Classify one /scan_raw. A missing device is the caller's problem; this does not open a serial port.

    A present but wrong frame is a fault. A short or sparse scan is not 'online'.
    A full scan with most beams finite is healthy even if an old log once said the port was down.
    """
    vals = []
    for r in ranges or []:
        try:
            x = float(r)
        except (TypeError, ValueError):
            continue
        if x == x and 0.05 < x < 30.0:
            vals.append(x)
    n = len(ranges or [])
    frac = (len(vals) / float(n)) if n else 0.0
    ref = None
    if vals:
        ordered = sorted(vals)
        ref = ordered[len(ordered) // 2]
    if frame_id and frame_id not in ("lidar_frame",):
        return {"fresh": False, "valid_frac": frac, "driver_ok": False, "fault": "unexpected frame", "ref_m": ref}
    ok = n >= 100 and frac >= MIN_VALID_FRAC
    return {"fresh": ok, "valid_frac": frac, "driver_ok": ok,
            "fault": None if ok else "sparse_or_short", "ref_m": ref}


def depth_status(msg, lidar_m=None):
    """Metric depth only after the unit is resolved. Nonzero mono16 bytes are not enough."""
    enc = ""
    samples = []
    if isinstance(msg, dict):
        enc = msg.get("encoding") or ""
        samples = _preview_samples(msg)
    else:
        enc = getattr(msg, "encoding", "") or ""
    try:
        import gb_obstacle as OB
        scale, how = OB.resolve_depth_scale(enc, samples, lidar_m)
    except Exception:
        scale, how = None, "resolver_failed"
    frac = depth_valid_fraction(samples, scale) if scale else 0.0
    ranging = how in VALIDATED_DEPTH_UNITS and frac >= MIN_VALID_FRAC
    return {"fresh": ranging, "valid_frac": frac, "unit_ok": ranging, "unit_how": how,
            "scale_m": scale, "fault": None if ranging else how}


def _preview_samples(msg, limit=64):
    data = msg.get("data")
    enc = (msg.get("encoding") or "")
    raw = b""
    if isinstance(data, (bytes, bytearray)):
        raw = bytes(data)
    elif isinstance(data, str) and data:
        import base64
        try:
            raw = base64.b64decode(data)
        except Exception:
            return []
    elif isinstance(data, list):
        return data[:limit]
    if not raw:
        return []
    import struct
    if enc == "32FC1" and len(raw) >= 4:
        n = min(limit, len(raw) // 4)
        return list(struct.unpack("<%df" % n, raw[:n * 4]))
    if enc in ("16UC1", "mono16") and len(raw) >= 2:
        n = min(limit, len(raw) // 2)
        return list(struct.unpack("<%dH" % n, raw[:n * 2]))
    return []


def under_budget(load1, limit=None):
    """True when optional work (embeddings, extra vision) may run. Safety never consults this."""
    cap = LOAD_SHED if limit is None else float(limit)
    try:
        return float(load1) < cap
    except (TypeError, ValueError):
        return False


class Decision(object):
    def __init__(self, allow, scale, reason, speech=None):
        self.allow = bool(allow)
        self.scale = float(scale)
        self.reason = reason
        self.speech = speech

    def as_tuple(self):
        return self.allow, self.scale, self.reason, self.speech


def effective_mode(health, caps=None):
    """Combine a physical health object with the user gate.

    caps None, or a missing file, means both sensors are allowed. A saved
    lidar:false is respected and is not turned back on. A stale fault string
    on the health object is ignored. Depth stays available when the user left
    the camera on, even if the LiDAR switch is off or the LiDAR stream is stale.
    """
    if caps is None:
        lidar_user, cam_user = True, True
    else:
        lidar_user = bool(caps.get("lidar", True))
        cam_user = bool(caps.get("camera", True))
    lidar_ok = bool(getattr(health, "lidar_online", False)) and lidar_user
    depth_ok = bool(getattr(health, "depth_online", False)) and cam_user
    if lidar_ok:
        return "lidar"
    if depth_ok:
        return "depth"
    return "none"


def motion_policy(provenance, mode, protections=None):
    """provenance: 'teleop' or 'autonomous'. mode: 'lidar', 'depth', or 'none'.

    Teleop may run with no ranging. It never bypasses estop, tip, battery, or watchdog.
    Soft tip stops with no reverse. Autonomous motion with no ranging is refused.
    """
    p = protections or {}
    if p.get("estop") or p.get("watchdog") or p.get("battery_shutdown"):
        return Decision(False, 0.0, "safety")
    if p.get("tip_hard"):
        return Decision(False, 0.0, "tip_hard")
    if p.get("tip_soft"):
        return Decision(False, 0.0, "tip_soft")  # 15-25 deg: stop, no back-off
    prov = (provenance or "autonomous").lower()
    if prov == "teleop":
        # Sensorless teleop is allowed. Scale < 1 is a cap, not a claim that space is clear.
        scale = 1.0 if mode in ("lidar", "depth") else 0.5
        return Decision(True, scale, "teleop")
    if mode == "lidar":
        return Decision(True, 1.0, "lidar")
    if mode == "depth":
        return Decision(True, 0.5, "depth")
    return Decision(False, 0.0, "no_range", NO_RANGE)


class Health(object):
    def __init__(self, now=None):
        self.mode = "none"
        self.lidar_online = False
        self.depth_online = False
        self.lidar_valid_frac = 0.0
        self.depth_valid_frac = 0.0
        self.depth_unit = None
        self.rgb_online = False
        self.last_lidar_fault = None
        self.goal_epoch = 0
        self.ever_online = False
        self.grace_s = 20.0
        self.t0 = time.time() if now is None else float(now)
        self._lg = self._lb = self._dg = self._db = 0
        self._announced_none = False

    def snapshot(self, caps=None):
        eff = effective_mode(self, caps)
        user_lidar = True if caps is None else bool(caps.get("lidar", True))
        user_camera = True if caps is None else bool(caps.get("camera", True))
        return {
            "mode": eff,
            "health_mode": self.mode,
            "lidar_online": self.lidar_online,
            "lidar_valid_frac": self.lidar_valid_frac,
            "depth_online": self.depth_online,
            "depth_ranging": self.mode == "depth" or (self.depth_online and eff == "depth"),
            "depth_unit": self.depth_unit,
            "depth_valid_frac": self.depth_valid_frac,
            "rgb_online": self.rgb_online,
            "user_lidar": user_lidar,
            "user_camera": user_camera,
            "goal_epoch": self.goal_epoch,
            "t": time.time(),
        }

    def resume_goal(self):
        """Recovery never restarts a goal that was cancelled."""
        return False

    def update(self, lidar=None, depth=None, rgb=None, now=None):
        """Each argument is a sample dict or None if that sensor was not observed this tick.

        lidar: {fresh: bool, valid_frac: float, driver_ok: bool}
        depth: {fresh: bool, valid_frac: float}   metric depth only
        rgb:   {fresh: bool}                       never promotes mode
        """
        now = time.time() if now is None else float(now)
        phrases = []
        if lidar is not None:
            self.lidar_valid_frac = float(lidar.get("valid_frac") or 0)
            ok = bool(lidar.get("fresh")) and bool(lidar.get("driver_ok", True)) and self.lidar_valid_frac >= MIN_VALID_FRAC
            if ok:
                self._lg += 1
                self._lb = 0
                self.last_lidar_fault = None
            else:
                self._lb += 1
                self._lg = 0
                self.last_lidar_fault = lidar.get("fault") or "stale"
            if self._lg >= GOOD_N:
                self.lidar_online = True
            elif self._lb >= BAD_N:
                self.lidar_online = False
        if depth is not None:
            how = depth.get("unit_how")
            frac = float(depth.get("valid_frac") or 0)
            self.depth_unit = how if how in VALIDATED_DEPTH_UNITS else None
            self.depth_valid_frac = frac
            ok = bool(depth.get("fresh")) and how in VALIDATED_DEPTH_UNITS and frac >= MIN_VALID_FRAC
            if ok:
                self._dg += 1
                self._db = 0
            else:
                self._db += 1
                self._dg = 0
            if self._dg >= GOOD_N:
                if not self.depth_online:
                    phrases.append(DEPTH_ONLINE)
                self.depth_online = True
            elif self._db >= BAD_N:
                self.depth_online = False
        if rgb is not None:
            self.rgb_online = bool(rgb.get("fresh"))
        prev = self.mode
        if self.lidar_online:
            self.mode = "lidar"
            if prev != "lidar":
                phrases.append(LIDAR_ONLINE)
                self.ever_online = True
        elif self.depth_online:
            self.mode = "depth"
            self.ever_online = True
            if prev == "lidar":
                phrases.append(LIDAR_FALLBACK)
        else:
            self.mode = "none"
        if self.mode == "none" and prev != "none":
            self.goal_epoch += 1
            quiet = (not self.ever_online) and (now - self.t0) < self.grace_s
            if not quiet and not self._announced_none:
                phrases.append(NO_RANGE)
                self._announced_none = True
        if self.mode != "none":
            self._announced_none = False
        # A stored fault must not hide a live depth fallback.
        if self.mode == "depth" and self.last_lidar_fault:
            pass
        return phrases

    def request_autonomous(self):
        """Called when a goal is asked for. Repeats the exact no-range line. Does not resume old goals."""
        if self.mode == "none":
            self.goal_epoch += 1
            return NO_RANGE
        return None


def sample_time(body):
    """Epoch seconds from t, time, or a ROS stamp. Milliseconds only above 1e12."""
    if not isinstance(body, dict):
        return None
    raw = body.get("t", body.get("time"))
    if raw is None and isinstance(body.get("stamp"), dict):
        try:
            raw = float(body["stamp"].get("sec") or 0) + float(body["stamp"].get("nanosec") or 0) / 1e9
        except (TypeError, ValueError):
            return None
    elif raw is None:
        raw = body.get("stamp")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if value > 1e14:
        value = value / 1e9
    elif value > 1e12:
        value = value / 1000.0
    if value < 1e9:
        return None
    return value


def read_shared(path=None, now=None):
    """Fresh health snapshot, or None if the monitor is missing, stale, or malformed.

    The file may be written by a process that is not the enabled unit. A missing
    or old timestamp is still a refusal. Three seconds remains the limit.
    """
    if path:
        p = path
    elif os.environ.get("HEALTH_FILE") or os.environ.get("GB_HEALTH_FILE"):
        p = _default_health_path()
    else:
        p = SHARED
    now = time.time() if now is None else float(now)
    try:
        with open(p) as f:
            d = json.load(f)
    except Exception:
        return None
    if not isinstance(d, dict):
        return None
    stamped = sample_time(d)
    if stamped is None:
        return None
    age = now - stamped
    if age < -1.0 or age > 3.0:
        return None
    d = dict(d)
    d["t"] = stamped
    if d.get("depth_topic_online") and d.get("depth_metric_valid") is not True:
        d["depth_online"] = False
    return d


def watchdog_report(maint_present, zeroing, release, now, why):
    """Current watchdog state. Maintenance is true only while the lock file exists."""
    return {
        "t": float(now),
        "maint": bool(maint_present),
        "zeroing": bool(zeroing),
        "release": release or "",
        "why": why or "",
    }


def autonomy_allowed(snap, now=None):
    """Autonomous motion only with a fresh physical health record.

    A missing, stale, or malformed monitor file is a refusal. User-capability
    defaults are not a substitute. Depth counts only with a validated unit and
    coverage. Joystick teleop is decided by the caller, not here.
    """
    if not isinstance(snap, dict):
        return False
    if now is not None:
        try:
            age = float(now) - float(snap["t"])
        except (KeyError, TypeError, ValueError):
            return False
        if age < -1.0 or age > 3.0:
            return False
    mode = snap.get("mode")
    if mode == "lidar":
        if snap.get("lidar_online") is not True or snap.get("user_lidar") is False:
            return False
        try:
            return float(snap.get("lidar_valid_frac", 0)) >= MIN_VALID_FRAC
        except (TypeError, ValueError):
            return False
    if mode == "depth":
        if snap.get("depth_online") is not True or snap.get("user_camera") is False:
            return False
        if snap.get("depth_unit") not in VALIDATED_DEPTH_UNITS:
            return False
        try:
            return float(snap.get("depth_valid_frac", 0)) >= MIN_VALID_FRAC
        except (TypeError, ValueError):
            return False
    return False


def gate_blocks_autonomy(src, motion_mode, snap, now=None):
    """True when this command must be zeroed.

    Only an explicit joystick source is teleop. Any other source, including a
    plain velocity stream, is autonomous and fails closed without fresh health.
    """
    if motion_mode == "idle":
        return False
    if src == "joystick":
        return False
    return not autonomy_allowed(snap, now)


_NOTE_T = 0.0

def note_control_ms(ms, path=None):
    """Best-effort latency sample for the stream watchdog. Never raises. At most twice a second."""
    global _NOTE_T
    now = time.time()
    if now - _NOTE_T < 0.5:
        return
    _NOTE_T = now
    path = path or os.environ.get("CONTROL_MS_FILE", "/home/pi/docker/tmp/gb_safety/control_ms")
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_NONBLOCK, 0o644)
        try:
            os.write(fd, ("%d\n" % int(ms)).encode())
        finally:
            os.close(fd)
    except Exception:
        return


def read_control_ms(path=None, max_age=2.0):
    path = path or os.environ.get("CONTROL_MS_FILE", "/home/pi/docker/tmp/gb_safety/control_ms")
    try:
        if time.time() - os.path.getmtime(path) > max_age:
            return None
        return float(open(path).read().strip() or "nan")
    except Exception:
        return None


def audio_io_kind(exc, cancelling=False):
    """Classify an arecord/read error.

    Interrupted system call while a playback cancel is in progress is expected.
    Any other interrupt or I/O error stays a real failure and must not be dropped.
    """
    text = str(exc).lower()
    errno = getattr(exc, "errno", None)
    interrupted = isinstance(exc, InterruptedError) or errno == 4 or "interrupted system call" in text
    if interrupted and cancelling:
        return "cancel"
    if interrupted:
        return "interrupted"
    return "failed"


def write_shared(health, path=None, caps=None):
    p = path or SHARED
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = p + ".tmp"
        with open(tmp, "w") as f:
            json.dump(health.snapshot(caps), f)
        os.replace(tmp, p)
    except OSError:
        return False
    return True
