"""Two-clap stop with a quiet gate. Local, no network, no microphone device.

The gate arms only after a beat of stable quiet relative to the adaptive noise
floor. A continuous voice or an echo tail keeps it unarmed. The first peak does
not demand another quiet beat before the second peak. This does not claim that
a quiet interval makes false triggers impossible, and it does not run while the
microphone never gets that quiet beat.

Default beat is 0.40 s (CLAP_QUIET_S). That is longer than a gap inside a
sentence and long enough for a short echo tail to finish on the microphone.
The speaking flag is not the arm signal.
"""
import json, os, time

QUIET_BEAT_S = float(os.environ.get("CLAP_QUIET_S", "0.40") or "0.40")
QUIET_ABOVE_DB = 8.0
QUIET_STABLE_DB = 6.0
PAIR_MIN_S = 0.12
PAIR_MAX_S = 0.50
REFRACTORY_S = 1.5
CONTINUOUS_S = 0.08
HOP_S = 0.001
PRESETS = {
    "low": {"rise": 18.0, "margin": 24.0, "abs_min": -22.0},
    "med": {"rise": 13.0, "margin": 17.0, "abs_min": -27.0},
    "high": {"rise": 10.0, "margin": 12.0, "abs_min": -32.0},
    "max": {"rise": 7.0, "margin": 8.0, "abs_min": -38.0},
}


def _pct(values, p):
    if not values:
        return -90.0
    s = sorted(values)
    i = int(round((len(s) - 1) * (p / 100.0)))
    return float(s[max(0, min(len(s) - 1, i))])


class QuietGate(object):
    def __init__(self, sens="high", beat_s=None):
        cfg = PRESETS.get(sens, PRESETS["high"])
        self.rise = cfg["rise"]
        self.margin = cfg["margin"]
        self.abs_min = cfg["abs_min"]
        self.sens = sens
        self.beat = QUIET_BEAT_S if beat_s is None else float(beat_s)
        self.floor = -90.0
        self.hist = []
        self.env = []
        self.n = 0
        self.t = 0.0
        self.quiet_for = 0.0
        self.loud_for = 0.0
        self.state = "listen"
        self.peak_t = None
        self.peak_db = None
        self.peak_decayed = False
        self.refractory_until = 0.0
        self.events = []
        self.seeded = False

    def step(self, db):
        """One millisecond. Returns an event dict or None. Does not publish motion."""
        self.n += 1
        self.t += HOP_S
        db = float(db)
        self.env.append(db)
        self.hist.append(db)
        if len(self.hist) > 2000:
            self.hist.pop(0)
        if len(self.env) > 80:
            self.env.pop(0)
        if self.n >= 200 and self.n % 50 == 0:
            candidate = _pct(self.hist, 20)
            if not self.seeded:
                self.floor = candidate
                self.seeded = True
            elif candidate < self.floor:
                self.floor = candidate
            else:
                self.floor = min(candidate, self.floor + 0.5)
        floor_ready = self.n >= 200
        above = floor_ready and db > self.floor + QUIET_ABOVE_DB
        stable = floor_ready and (max(self.env) - min(self.env)) <= QUIET_STABLE_DB
        quiet = floor_ready and (not above) and stable and self.t >= self.refractory_until
        if self.t < self.refractory_until:
            self.quiet_for = 0.0
            self.state = "listen"
            self.peak_t = None
            return None
        if self.state == "listen":
            self.peak_t = None
            if not quiet:
                self.quiet_for = 0.0
                return None
            self.quiet_for += HOP_S
            if self.quiet_for >= self.beat:
                self.state = "armed"
            return None
        onset = self._onset(db)
        if above and not onset:
            self.loud_for += HOP_S
        else:
            self.loud_for = 0.0
        if self.loud_for >= CONTINUOUS_S:
            self._unarm()
            return None
        if self.peak_t is not None and not self.peak_decayed:
            if db <= self.peak_db - 8.0 or db <= self.floor + 4.0:
                self.peak_decayed = True
        if not onset:
            return None
        if self.state == "armed":
            self.state = "peak"
            self.peak_t = self.t
            self.peak_db = db
            self.peak_decayed = False
            self.loud_for = 0.0
            return None
        gap = self.t - self.peak_t
        if gap < PAIR_MIN_S:
            return None
        if gap > PAIR_MAX_S or not self.peak_decayed:
            self.peak_t = self.t
            self.peak_db = db
            self.peak_decayed = False
            self.state = "peak"
            return None
        ev = {
            "t": round(self.t, 3),
            "gap_ms": round(gap * 1000),
            "peak_db": round(db, 1),
            "floor_db": round(self.floor, 1),
            "sens": self.sens,
            "quiet_s": self.beat,
            "echo_ref": False,
        }
        self.events.append(ev)
        self.refractory_until = self.t + REFRACTORY_S
        self._unarm()
        return ev

    def _onset(self, db):
        if len(self.env) < 16 or self.n < 200:
            return False
        base = _pct(self.env[-16:-6], 50)
        return (db - base >= self.rise and db - self.floor >= self.margin and db >= self.abs_min
                and db - self.env[-6] >= self.rise * 0.5)

    def _unarm(self):
        self.state = "listen"
        self.quiet_for = 0.0
        self.loud_for = 0.0
        self.peak_t = None
        self.peak_db = None
        self.peak_decayed = False


def run_envelope(samples, sens="high", beat_s=None):
    gate = QuietGate(sens, beat_s)
    found = []
    for db in samples:
        ev = gate.step(db)
        if ev:
            found.append(ev)
    return found, gate


def latch_enabled(cfg, now=None):
    """A test window never turns into a live latch when it expires or the process restarts.

    latch:true is an explicit arm. A missing test_until means the running
    service is allowed to latch. An expired test_until stays disarmed.
    """
    now = time.time() if now is None else float(now)
    if not isinstance(cfg, dict):
        return False
    if cfg.get("latch") is True:
        return True
    if "test_until" in cfg:
        return False
    return True


def inhibit_effect(maint_present, estop_present):
    """What the gate does while a lock file is held. No motor publish."""
    held = bool(maint_present or estop_present)
    return {"zero": held, "cancel_missions": bool(estop_present), "velocity_command": False}


def latch_clap(host, shared, maint_path, abort_path, now=None):
    """Write a normal clap stop to both ESTOP paths. Do not clear MAINT or a hard fault."""
    now = time.time() if now is None else float(now)
    try:                                              # 2026-11-25 noestop: floor -> normal stop, no latch file
        import gb_floor
        floor = gb_floor.no_latch()
    except Exception:
        floor = True
    if floor:
        if abort_path:
            parent = os.path.dirname(abort_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(abort_path, "w") as f:
                f.write("floor stop: clap\n")
        try:
            import gb_arm; gb_arm.disarm("floor normal stop: double clap")
        except Exception:
            pass
        try:
            gb_floor.request_stop("clap", "double clap", was="clap-estop")
        except Exception:
            pass
        return {"wrote": [], "kept_safety": [], "maint_present": bool(maint_path and os.path.lexists(maint_path)),
                "motion": False, "transport": "floor-stop"}
    body = {"t": now, "src": "clap", "kind": "normal", "why": "double clap"}
    wrote = []
    kept_safety = []
    for path in (host, shared):
        if not path:
            continue
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        existing = None
        if os.path.lexists(path):
            try:
                with open(path) as f:
                    existing = json.load(f)
            except (OSError, ValueError):
                existing = None
        if isinstance(existing, dict) and (existing.get("kind") == "safety"
                                            or existing.get("src") in ("gate-tilt", "tilt", "battery", "watchdog", "physical", "joystick", "hardware")):
            kept_safety.append(path)
            continue
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(body, f)
        os.replace(tmp, path)
        wrote.append(path)
    if abort_path:
        parent = os.path.dirname(abort_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(abort_path, "w") as f:
            f.write("ESTOP clap\n")
    return {
        "wrote": wrote,
        "kept_safety": kept_safety,
        "maint_present": bool(maint_path and os.path.lexists(maint_path)),
        "motion": False,
        "transport": "estop-file",
    }
