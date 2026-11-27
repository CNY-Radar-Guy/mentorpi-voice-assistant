#!/usr/bin/env python3
"""Wall-E / Jarvis Root Cause Analysis generator.

Standing rule (John, 27 Nov 2026 ET): every crash or unexpected behavior must
produce a structured RCA report so the recursive learning loop can learn from
it, fix it, and prevent recurrence.

This module is a *read-only observer* plus a writer for files under the RCA
directory only. It never arms or moves Wall-E, never touches gb_gate motion
logic, and never writes or deletes ESTOP / MAINT / CLEAR_REQ / ARMED.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional
from zoneinfo import ZoneInfo

ET = ZoneInfo("America/New_York")

TRIGGER_TYPES = (
    "service_crash",
    "guard_trip",
    "lidar_dropout",
    "unexpected_behavior",
)

WATCHED_UNITS = (
    "grokbot.service",
    "gb-gate.service",
    "gb-guard.service",
    "gb-watchdog.service",
    "gb-intercom.service",
    "gb-armguard.service",
)

# Paths this program must never create, rewrite, or delete.
FORBIDDEN_BASENAMES = ("ESTOP", "MAINT", "CLEAR_REQ", "ARMED")

DEFAULT_COOLDOWN_S = 900.0  # 1 RCA per component/trigger(/reason) per 15 min
DEFAULT_JOURNAL_LINES = 80
DEFAULT_GATE_LOG_LINES = 80

PLACEHOLDER_ROOT = (
    "_Draft — fill in after capturing evidence. What actually broke, and why?_"
)
PLACEHOLDER_FIX = (
    "_Draft — describe the fix applied (or 'not yet fixed; draft only')._"
)
PLACEHOLDER_REGRESSION = (
    "_Draft — list the test or live check that proves the fix and guards against recurrence._"
)
PLACEHOLDER_PREVENTION = (
    "_Draft — the standing rule or check the learning loop should enforce next time._"
)
PLACEHOLDER_FOLLOWUPS = "_None yet._"
PLACEHOLDER_TIMELINE = "_Add a short chronological list once the evidence is reviewed._"

_SLUG_RE = re.compile(r"[^a-z0-9]+")


def _env_path(name: str, default: str) -> Path:
    return Path(os.environ.get(name, default))


@dataclass
class RcaConfig:
    rca_dir: Path = field(default_factory=lambda: _env_path("GB_RCA_DIR", "/home/pi/grokbot/data/rca"))
    data_dir: Path = field(default_factory=lambda: _env_path("GB_DATA", "/home/pi/grokbot/data"))
    safety_dir: Path = field(default_factory=lambda: _env_path("GB_SAFETY", "/home/pi/docker/tmp/gb_safety"))
    docs_dir: Path = field(default_factory=lambda: _env_path("GB_DOCS", "/home/pi/grokbot/docs"))
    template_path: Optional[Path] = None
    cooldown_s: float = field(default_factory=lambda: float(os.environ.get("GB_RCA_COOLDOWN_S", str(DEFAULT_COOLDOWN_S))))
    journal_lines: int = DEFAULT_JOURNAL_LINES
    gate_log_lines: int = DEFAULT_GATE_LOG_LINES
    tz: Any = ET

    def __post_init__(self) -> None:
        self.rca_dir = Path(self.rca_dir)
        self.data_dir = Path(self.data_dir)
        self.safety_dir = Path(self.safety_dir)
        self.docs_dir = Path(self.docs_dir)
        if self.template_path is None:
            self.template_path = self.docs_dir / "rca" / "RCA_TEMPLATE.md"
        else:
            self.template_path = Path(self.template_path)

    @property
    def index_path(self) -> Path:
        return self.rca_dir / "rca_index.jsonl"

    @property
    def state_path(self) -> Path:
        return self.rca_dir / "rca_state.json"


def et_dt(ts: Optional[float] = None, tz=ET) -> datetime:
    if ts is None:
        ts = time.time()
    return datetime.fromtimestamp(float(ts), tz=tz)


def format_et(ts: Optional[float] = None, tz=ET) -> str:
    return et_dt(ts, tz).strftime("%Y-%m-%d %H:%M:%S ET")


def slugify(text: str, max_len: int = 48) -> str:
    s = _SLUG_RE.sub("-", (text or "").lower()).strip("-")
    return (s or "event")[:max_len].strip("-")


def normalize_reason(text: str) -> str:
    s = re.sub(r"\s+", " ", (text or "").strip().lower())
    s = re.sub(r"\d+\.\d+", "N", s)
    s = re.sub(r"\b\d+\b", "N", s)
    return s[:160]


def dedup_key(component: str, trigger: str, symptom: str = "") -> str:
    comp = slugify(component, 40)
    trig = trigger if trigger in TRIGGER_TYPES else "unexpected_behavior"
    if trig in ("service_crash", "lidar_dropout"):
        return f"{comp}|{trig}"
    return f"{comp}|{trig}|{normalize_reason(symptom)}"


def rca_id(when: datetime, component: str, slug: str) -> str:
    stamp = when.strftime("%Y%m%d-%H%M%S")
    return f"RCA-{stamp}-{slugify(component, 32)}-{slugify(slug, 40)}"


def ensure_rca_dir(cfg: RcaConfig) -> Path:
    cfg.rca_dir.mkdir(parents=True, exist_ok=True)
    return cfg.rca_dir


def _read_text(path: Path, default: str = "") -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return default


def _read_json(path: Path) -> Any:
    try:
        return json.loads(_read_text(path, "null") or "null")
    except Exception:
        return None


def _tail_text(path: Path, max_lines: int, max_bytes: int = 256_000) -> str:
    if not path.is_file():
        return f"(missing: {path})"
    try:
        size = path.stat().st_size
        with path.open("rb") as f:
            if size > max_bytes:
                f.seek(size - max_bytes)
                f.readline()
            data = f.read().decode("utf-8", errors="replace")
        lines = data.splitlines()
        return "\n".join(lines[-max_lines:])
    except Exception as ex:
        return f"(error reading {path}: {ex})"


def _run(cmd: list[str], timeout: int = 12) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception as ex:
        return f"(command failed: {' '.join(cmd)}: {ex})"
    out = (r.stdout or "").strip()
    err = (r.stderr or "").strip()
    if r.returncode != 0 and not out:
        return f"(exit {r.returncode}: {err or 'no output'})"
    return out or (err and f"(stderr: {err})") or "(empty)"


def collect_snapshot(cfg: RcaConfig) -> dict[str, Any]:
    """Read-only snapshot of arm/mode/sensors. Never writes these files."""
    safety = cfg.safety_dir
    data = cfg.data_dir
    armed_path = safety / "ARMED"
    estop_path = safety / "ESTOP"
    data_estop = data / "ESTOP"
    snap: dict[str, Any] = {
        "captured_at_et": format_et(tz=cfg.tz),
        "armed": armed_path.is_file(),
        "estop": estop_path.is_file() or data_estop.is_file(),
        "mode": None,
        "sensors": _read_json(safety / "sensors.json"),
        "health": _read_json(safety / "health.json"),
        "watchdog_state": _read_json(safety / "watchdog_state.json"),
        "rgb_guard": _read_json(safety / "rgb_guard.json"),
        "floor_mode": _read_json(safety / "FLOOR_MODE.json"),
        "disarm_event": _read_json(safety / "DISARM_EVENT"),
        "battery": _read_json(data / "battery.json"),
        "caps": _read_json(data / "caps.json"),
        "learn_mode": _read_json(data / "learn" / "mode.json"),
        "loopwatch": None,
    }
    sensors = snap["sensors"] if isinstance(snap["sensors"], dict) else {}
    snap["mode"] = sensors.get("mode")
    # Keep loopwatch compact: units + last alerts only.
    lw = _read_json(data / "loopwatch.json") or _read_json(safety / "loopwatch.json")
    if isinstance(lw, dict):
        snap["loopwatch"] = {
            "t": lw.get("t"),
            "units": lw.get("units"),
            "recent_alerts": lw.get("recent_alerts"),
        }
    if armed_path.is_file():
        raw = _read_text(armed_path).strip()
        try:
            snap["armed_file"] = json.loads(raw)
        except Exception:
            snap["armed_file"] = raw[:500]
    return snap


def collect_guard_event(cfg: RcaConfig) -> Any:
    return _read_json(cfg.safety_dir / "guard_event.json")


def collect_journal(cfg: RcaConfig, component: str, lines: Optional[int] = None) -> str:
    n = int(lines or cfg.journal_lines)
    unit = component if component.endswith(".service") else None
    # Map common component names to units when possible.
    if unit is None:
        raw = component.strip()
        if raw in {u.replace(".service", "") for u in WATCHED_UNITS} or raw in (
            "gb-loopwatch",
            "gb-rgbguard",
            "grokbot-lowv",
        ):
            unit = raw if raw.endswith(".service") else raw + ".service"
        elif raw in ("grokbot_voice", "voice", "wally-voice"):
            unit = "grokbot.service"
        elif raw in ("sclidar", "sclidar_node", "lidar"):
            unit = None
    if unit:
        return _run(["journalctl", "-u", unit, "-n", str(n), "--no-pager"])
    # Multi-unit fallback for lidar / unknown components.
    units = [f"-u={u}" for u in WATCHED_UNITS]
    return _run(["journalctl", *units, "-n", str(n), "--no-pager"])


def collect_gate_log(cfg: RcaConfig, lines: Optional[int] = None) -> str:
    return _tail_text(cfg.safety_dir / "gate.log", int(lines or cfg.gate_log_lines))


def collect_evidence(
    cfg: RcaConfig,
    component: str,
    *,
    skip_cmds: bool = False,
    evidence: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    ev: dict[str, Any] = {
        "snapshot": collect_snapshot(cfg) if not (evidence and "snapshot" in evidence) else evidence["snapshot"],
        "guard_event": collect_guard_event(cfg) if not (evidence and "guard_event" in evidence) else evidence["guard_event"],
        "gate_log": collect_gate_log(cfg) if not (evidence and "gate_log" in evidence) else evidence["gate_log"],
        "journal": "(skipped in test / dry collect)" if skip_cmds and not (evidence and "journal" in evidence) else (
            evidence["journal"] if evidence and "journal" in evidence else collect_journal(cfg, component)
        ),
        "extra": (evidence or {}).get("extra", ""),
    }
    return ev


def load_index(cfg: RcaConfig) -> list[dict[str, Any]]:
    path = cfg.index_path
    if not path.is_file():
        return []
    rows = []
    for line in _read_text(path).splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def last_index_time(rows: Iterable[Mapping[str, Any]], key: str) -> Optional[float]:
    last = None
    for row in rows:
        if row.get("dedup_key") == key:
            try:
                last = float(row.get("ts_epoch"))
            except (TypeError, ValueError):
                continue
    return last


def load_state(cfg: RcaConfig) -> dict[str, Any]:
    data = _read_json(cfg.state_path)
    return data if isinstance(data, dict) else {"cooldowns": {}}


def save_state(cfg: RcaConfig, state: Mapping[str, Any]) -> None:
    ensure_rca_dir(cfg)
    tmp = cfg.state_path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, cfg.state_path)


def should_write(cfg: RcaConfig, key: str, now: Optional[float] = None, *, force: bool = False) -> tuple[bool, str]:
    """Return (allowed, reason). Rate-limit / dedup: one RCA per key per cooldown window."""
    if force:
        return True, "force"
    now = time.time() if now is None else float(now)
    last = last_index_time(load_index(cfg), key)
    state = load_state(cfg)
    cd = (state.get("cooldowns") or {}).get(key)
    try:
        if cd is not None:
            last = max(last or 0.0, float(cd))
    except (TypeError, ValueError):
        pass
    if last is None:
        return True, "no-prior"
    elapsed = now - last
    if elapsed < cfg.cooldown_s:
        return False, "cooldown %.0fs remaining for %s" % (cfg.cooldown_s - elapsed, key)
    return True, "cooldown-expired"


def _pretty_json(value: Any) -> str:
    if value is None or value == "":
        return "(none)"
    if isinstance(value, str):
        return value
    try:
        return json.dumps(value, indent=2, default=str)
    except Exception:
        return str(value)


def load_template(cfg: RcaConfig) -> str:
    path = cfg.template_path
    if path and path.is_file():
        return path.read_text(encoding="utf-8")
    # Bundled fallback so generation still works if docs are not installed yet.
    bundled = Path(__file__).resolve().parent / "docs" / "rca" / "RCA_TEMPLATE.md"
    # release tree vs /home/pi/grokbot/docs
    candidates = [
        bundled,
        Path("/home/pi/grokbot/docs/rca/RCA_TEMPLATE.md"),
        Path("/home/pi/grokbot/releases/e48d3d8/docs/rca/RCA_TEMPLATE.md"),
    ]
    for c in candidates:
        if c.is_file():
            return c.read_text(encoding="utf-8")
    raise FileNotFoundError("RCA template not found; expected docs/rca/RCA_TEMPLATE.md")


def render_template(template: str, fields: Mapping[str, str]) -> str:
    out = template
    for key, value in fields.items():
        out = out.replace("{{" + key + "}}", value if value is not None else "")
    # Leave any leftover placeholders visible rather than crashing.
    return out


def append_jsonl(path: Path, entry: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(entry, default=str, sort_keys=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()


def _unique_path(directory: Path, stem: str) -> Path:
    candidate = directory / f"{stem}.md"
    n = 2
    while candidate.exists():
        candidate = directory / f"{stem}-{n}.md"
        n += 1
    return candidate


def generate_rca(
    cfg: RcaConfig,
    *,
    component: str,
    trigger: str,
    symptom: str,
    title: Optional[str] = None,
    reporter: str = "automated",
    status: str = "draft",
    root_cause: str = PLACEHOLDER_ROOT,
    fix_applied: str = PLACEHOLDER_FIX,
    regression_test: str = PLACEHOLDER_REGRESSION,
    prevention_rule: str = PLACEHOLDER_PREVENTION,
    followups: str = PLACEHOLDER_FOLLOWUPS,
    timeline: str = PLACEHOLDER_TIMELINE,
    related: str = "none",
    ts: Optional[float] = None,
    force: bool = False,
    skip_cmds: bool = False,
    evidence: Optional[Mapping[str, Any]] = None,
    extra_evidence: str = "",
) -> dict[str, Any]:
    """Write a timestamped Markdown RCA and append a JSONL index row.

    Returns a result dict: ok, skipped, path, entry, reason.
    """
    if trigger not in TRIGGER_TYPES:
        raise ValueError(f"trigger must be one of {TRIGGER_TYPES}, got {trigger!r}")
    if any(name in str(cfg.rca_dir) for name in ("/ESTOP", "/MAINT")):
        raise RuntimeError("refusing to use an RCA dir that looks like a safety lock path")

    now = time.time() if ts is None else float(ts)
    when = et_dt(now, cfg.tz)
    key = dedup_key(component, trigger, symptom)
    allowed, reason = should_write(cfg, key, now, force=force)
    if not allowed:
        return {"ok": False, "skipped": True, "reason": reason, "path": None, "entry": None, "dedup_key": key}

    ev = collect_evidence(cfg, component, skip_cmds=skip_cmds, evidence=evidence)
    if extra_evidence:
        ev["extra"] = ((ev.get("extra") or "") + "\n" + extra_evidence).strip()

    slug = slugify(title or symptom or trigger, 40)
    ident = rca_id(when, component, slug)
    ensure_rca_dir(cfg)
    path = _unique_path(cfg.rca_dir, ident)
    ident = path.stem

    fields = {
        "id": ident,
        "id_short": ident.replace("RCA-", "", 1),
        "title": title or symptom.strip().split("\n", 1)[0][:120] or trigger.replace("_", " "),
        "timestamp_et": format_et(now, cfg.tz),
        "status": status,
        "trigger": trigger,
        "component": component,
        "reporter": reporter,
        "related": related or "none",
        "symptom": symptom.strip() or "(not specified)",
        "timeline": timeline,
        "snapshot_json": _pretty_json(ev.get("snapshot")),
        "guard_event": _pretty_json(ev.get("guard_event")),
        "gate_log": (ev.get("gate_log") or "(none)").rstrip() or "(none)",
        "journal": (ev.get("journal") or "(none)").rstrip() or "(none)",
        "extra_evidence": ev.get("extra") or "",
        "root_cause": root_cause,
        "fix_applied": fix_applied,
        "regression_test": regression_test,
        "prevention_rule": prevention_rule,
        "followups": followups,
    }
    body = render_template(load_template(cfg), fields)
    path.write_text(body, encoding="utf-8")

    entry = {
        "id": ident,
        "path": str(path),
        "ts_et": format_et(now, cfg.tz),
        "ts_epoch": now,
        "component": component,
        "trigger": trigger,
        "symptom": symptom.strip().split("\n", 1)[0][:300],
        "title": fields["title"],
        "status": status,
        "reporter": reporter,
        "root_cause_summary": re.sub(r"\s+", " ", root_cause).strip()[:400],
        "prevention_rule": re.sub(r"\s+", " ", prevention_rule).strip()[:400],
        "dedup_key": key,
        "draft": status != "complete",
    }
    append_jsonl(cfg.index_path, entry)
    state = load_state(cfg)
    state.setdefault("cooldowns", {})[key] = now
    state["last_id"] = ident
    save_state(cfg, state)
    return {"ok": True, "skipped": False, "reason": reason, "path": str(path), "entry": entry, "dedup_key": key}


# ---------------------------------------------------------------------------
# Seeded example: 27 Nov 2026 sclidar dropout / "Guard, I held still"
# ---------------------------------------------------------------------------
SEED_TS = datetime(2026, 11, 27, 9, 38, 49, tzinfo=ET).timestamp()

SEED_SYMPTOM = (
    "John tried to manually drive Wall-E and got the spoken / UI message "
    "\"Guard, I held still\". The safety gate blocked the command. At the moment "
    "of the trip, front range was stuck at 0.192 m (sweep 0.193 m) and the trip "
    "reason was \"no LiDAR: reverse refused (no rear sense)\"."
)

SEED_TIMELINE = """\
- 08:44:29 ET — Last useful LiDAR-related gate note before the silent gap: `SLOW slow 0.040 m/s (LiDAR dropouts behind)` during the morning explore session.
- ~08:44–09:38 ET — `sclidar_node` silently stopped publishing scans (~54–56 minutes). `gb_guard` / `gb_gate` kept the last ranges (front 0.192 m, rear 0.302 m, sweep 0.193 m) instead of treating the sensor as offline for all directions.
- 09:38:42 ET — John armed Wall-E from the Jarvis UI to drive manually.
- 09:38:49 ET — `GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19`. Voice / UI reported \"Guard, I held still\".
- 09:38:51 ET — Trip cleared on idle-timeout; stale ranges unchanged.
- 09:40:17 ET — Second manual attempt: `STOP (autonomous refused: health monitor missing, stale, or no validated range)` immediately followed by the same `no LiDAR: reverse refused` trip with identical stale ranges.
- 09:40:20 ET — Second trip cleared on idle-timeout.
- 09:41:32 ET — Remote disarm (Jarvis).
- 09:42 ET — `sclidar_node` (`ros2 launch peripherals lidar.launch.py`, scan remapped to `scan_raw`) was restarted inside the MentorPi container. Fresh scans resumed; front opened to about 1.33 m.
- After the restart — Manual drive path was no longer blocked by the stale 0.19 m reading. This RCA records the incident for the learning loop.
"""

SEED_GATE_LOG = """\
2026-11-27 13:44:29.872 [gate] SLOW slow 0.040 m/s (LiDAR dropouts behind)
2026-11-27 14:38:49.008 [gate] GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
2026-11-27 14:38:51.810 [gate] GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
2026-11-27 14:40:17.309 [gate] STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.02s -> zero now, then 10 Hz zero for 2.0s
2026-11-27 14:40:17.409 [gate] STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.01s -> zero now, then 10 Hz zero for 2.0s
2026-11-27 14:40:17.908 [gate] GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
2026-11-27 14:40:20.311 [gate] GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
"""

SEED_JOURNAL = """\
Nov 27 08:44:29 wally bash[5754]: [INFO] [gb_gate]: SLOW slow 0.040 m/s (LiDAR dropouts behind)
Nov 27 09:38:49 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
Nov 27 09:38:51 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
Nov 27 09:40:17 wally bash[5754]: [INFO] [gb_gate]: STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.02s -> zero now, then 10 Hz zero for 2.0s
Nov 27 09:40:17 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
Nov 27 09:40:20 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
"""

SEED_GUARD_EVENTS = {
    "first": {
        "t": 1795790329.008666,
        "why": "no LiDAR: reverse refused (no rear sense)",
        "src": "stream",
        "front": 0.19183446823212913,
        "rear": 0.30194129944373,
        "sweep": 0.19300000369548798,
        "cleared_as": "guard_event.json.cleared-20261127-143851",
    },
    "second": {
        "t": 1795790417.908803,
        "why": "no LiDAR: reverse refused (no rear sense)",
        "src": "stream",
        "front": 0.19183446823212913,
        "rear": 0.30194129944373,
        "sweep": 0.19300000369548798,
        "cleared_as": "guard_event.json.cleared-20261127-144020",
    },
}

SEED_SNAPSHOT = {
    "captured_at_et": "2026-11-27 09:38:49 ET (incident; reconstructed from guard_event + gate.log)",
    "armed": True,
    "estop": False,
    "mode": "stream",
    "note": "Live sensors.json was not snapshotted at the exact trip instant. Values below are the latched ranges written to guard_event.json, which stayed bit-identical across both trips 88 seconds apart — evidence the scan was stale, not a new measurement.",
    "stale_ranges_m": {"front": 0.19183446823212913, "rear": 0.30194129944373, "sweep": 0.19300000369548798},
    "trip_why": "no LiDAR: reverse refused (no rear sense)",
    "lidar_process_after_fix": "sclidar_node restarted 2026-11-27 09:42 ET via ros2 launch peripherals lidar.launch.py (scan:=scan_raw)",
}

SEED_ROOT = """\
The `sclidar` ROS 2 node inside the MentorPi container silently stopped publishing LaserScan messages for about 56 minutes. It did not crash the `gb-guard` or `gb-gate` systemd units, so those services stayed "active" while their last ranges froze (front 0.192 m, sweep 0.193 m). When John tried to drive manually, the gate correctly refused reverse because it had no rear LiDAR sense, and the voice path announced \"Guard, I held still\". The safety gate was not misreading the joystick and there was no stale ESTOP latch. The misleading part was that the frozen 0.19 m front reading looked like a real close obstacle rather than an offline lidar. Identical range bits on the 09:38:49 ET and 09:40:17 ET trips confirm the values were latched, not freshly measured.
"""

SEED_FIX = """\
The `sclidar_node` / `ros2 launch peripherals lidar.launch.py` process was restarted inside the MentorPi container at 09:42 ET. Fresh scans started flowing again and the front range opened to about 1.33 m, after which manual drive was no longer blocked by the stale 0.19 m reading. No motion-gate logic was changed as part of that recovery. This RCA generator / watcher is the follow-up so the same class of failure produces a saved report automatically next time.
"""

SEED_REGRESSION = """\
- Confirm `sclidar_node` is running in the MentorPi container and `/scan` (or `scan_raw`) is publishing.
- Confirm `sensors.json` shows `sensors.fresh.lidar == true` and a changing front range (not stuck at 0.192 m).
- Armed manual drive forward/reverse is allowed only when LiDAR is fresh; a missing scan must trip with an explicit \"no LiDAR\" reason, not a stale close-range obstacle.
- Unit tests in `tests/test_gb_rca_20261127.py` cover RCA generation, cooldown dedup, and JSONL indexing.
- After deploy: safety services remain active, Wall-E remains disarmed, zero motion, `gb-rca-watch.service` is running.
"""

SEED_PREVENTION = """\
1. Treat LiDAR *scan freshness* as a first-class health signal, independent of the last range value. If scans stop for longer than the short memory window, fail to \"LiDAR offline\" in every direction and never keep advertising a latched 0.19 m (or any last range) as a live obstacle.
2. Voice / Jarvis UI should say that LiDAR is stale or offline when the trip reason is \"no LiDAR\", not only \"Guard, I held still\", so a frozen range is not mistaken for a chair leg.
3. `gb-rca-watch` must auto-draft an RCA on `lidar_dropout`, on a new `guard_event.json` trip, and on a systemd crash/restart of grokbot / gb-gate / gb-guard / gb-watchdog / gb-intercom / gb-armguard, rate-limited to one report per cooldown window so a crash loop cannot spam the index.
4. Recurring `sclidar` silence should be visible in loopwatch / health status even when gb-gate itself is still heartbeating.
"""

SEED_FOLLOWUPS = """\
- Consider a dedicated scan-topic freshness probe (read-only) alongside `sensors.json` `fresh.lidar`.
- Keep this report in the learning-loop index (`rca_index.jsonl`) as the reference example for stale-LiDAR false obstacles.
- Do not treat a later GitHub README / docs sync as a reason to include anyone's full legal name; use \"John\" only.
"""

SEED_EXTRA = """\
### Archived `guard_event.json` (both trips, identical stale ranges)

```json
%s
```

`sclidar_node` process after the 09:42 ET restart (MentorPi container, UTC times in `ps`):
`/home/ubuntu/third_party_ros2/third_party_ws/install/sclidar_ros2/lib/sclidar_ros2/sclidar --ros-args -r __node:=sclidar_node -r scan:=scan_raw` started 14:42 UTC.
""" % json.dumps(SEED_GUARD_EVENTS, indent=2)


def seed_sclidar_dropout_rca(cfg: RcaConfig, *, force: bool = True) -> dict[str, Any]:
    ev = {
        "snapshot": SEED_SNAPSHOT,
        "guard_event": SEED_GUARD_EVENTS["first"],
        "gate_log": SEED_GATE_LOG.strip(),
        "journal": SEED_JOURNAL.strip(),
        "extra": SEED_EXTRA,
    }
    return generate_rca(
        cfg,
        component="gb_guard",
        trigger="lidar_dropout",
        symptom=SEED_SYMPTOM,
        title="sclidar dropout left stale 0.19m front and blocked manual drive",
        reporter="John (incident) / RCA generator (seeded 27 Nov 2026 ET)",
        status="complete",
        root_cause=SEED_ROOT.strip(),
        fix_applied=SEED_FIX.strip(),
        regression_test=SEED_REGRESSION.strip(),
        prevention_rule=SEED_PREVENTION.strip(),
        followups=SEED_FOLLOWUPS.strip(),
        timeline=SEED_TIMELINE.strip(),
        related="manual-drive \"Guard, I held still\" / sclidar_node restart 09:42 ET",
        ts=SEED_TS,
        force=force,
        skip_cmds=True,
        evidence=ev,
    )


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Generate a Wall-E / Jarvis RCA report (writes only under the RCA directory).",
    )
    p.add_argument("--manual", action="store_true", help="Semi-automated RCA for an unexpected behavior or reviewed incident.")
    p.add_argument("--auto", action="store_true", help="Automated draft RCA (used by gb_rca_watch).")
    p.add_argument("--seed-sclidar-dropout", action="store_true", help="Write the 27 Nov 2026 sclidar dropout example RCA.")
    p.add_argument("--component", default="", help="Affected service or component (e.g. gb_guard, grokbot.service, sclidar).")
    p.add_argument("--trigger", default="", choices=("",) + TRIGGER_TYPES)
    p.add_argument("--symptom", default="", help="What was observed.")
    p.add_argument("--title", default="")
    p.add_argument("--root-cause", default="")
    p.add_argument("--fix", dest="fix_applied", default="")
    p.add_argument("--regression", dest="regression_test", default="")
    p.add_argument("--prevention", dest="prevention_rule", default="")
    p.add_argument("--timeline", default="")
    p.add_argument("--followups", default="")
    p.add_argument("--reporter", default="")
    p.add_argument("--status", default="", help="draft (default for auto/manual) or complete.")
    p.add_argument("--force", action="store_true", help="Bypass cooldown / dedup.")
    p.add_argument("--skip-cmds", action="store_true", help="Do not call journalctl (tests / offline).")
    p.add_argument("--rca-dir", default="", help="Override RCA output directory.")
    p.add_argument("--data-dir", default="")
    p.add_argument("--safety-dir", default="")
    p.add_argument("--docs-dir", default="")
    p.add_argument("--template", default="")
    p.add_argument("--cooldown", type=float, default=-1.0)
    return p


def config_from_args(args: argparse.Namespace) -> RcaConfig:
    cfg = RcaConfig()
    if args.rca_dir:
        cfg = replace(cfg, rca_dir=Path(args.rca_dir))
    if args.data_dir:
        cfg = replace(cfg, data_dir=Path(args.data_dir))
    if args.safety_dir:
        cfg = replace(cfg, safety_dir=Path(args.safety_dir))
    if args.docs_dir:
        cfg = replace(cfg, docs_dir=Path(args.docs_dir), template_path=None)
    if args.template:
        cfg = replace(cfg, template_path=Path(args.template))
    if args.cooldown is not None and args.cooldown >= 0:
        cfg = replace(cfg, cooldown_s=float(args.cooldown))
    return cfg


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    cfg = config_from_args(args)
    if args.seed_sclidar_dropout:
        result = seed_sclidar_dropout_rca(cfg, force=True)
        print(json.dumps({k: result[k] for k in ("ok", "skipped", "reason", "path", "dedup_key")}, indent=2))
        return 0 if result.get("ok") else 1
    if not (args.manual or args.auto):
        _build_parser().print_help()
        return 2
    component = args.component.strip()
    symptom = args.symptom.strip()
    if not component or not symptom:
        print("error: --component and --symptom are required", file=sys.stderr)
        return 2
    trigger = args.trigger or ("unexpected_behavior" if args.manual else "unexpected_behavior")
    status = args.status or "draft"
    reporter = args.reporter or ("manual" if args.manual else "gb-rca-watch")
    kwargs = dict(
        component=component,
        trigger=trigger,
        symptom=symptom,
        title=args.title or None,
        reporter=reporter,
        status=status,
        force=args.force,
        skip_cmds=args.skip_cmds,
    )
    if args.root_cause:
        kwargs["root_cause"] = args.root_cause
    if args.fix_applied:
        kwargs["fix_applied"] = args.fix_applied
    if args.regression_test:
        kwargs["regression_test"] = args.regression_test
    if args.prevention_rule:
        kwargs["prevention_rule"] = args.prevention_rule
    if args.timeline:
        kwargs["timeline"] = args.timeline
    if args.followups:
        kwargs["followups"] = args.followups
    result = generate_rca(cfg, **kwargs)
    printable = {k: result[k] for k in ("ok", "skipped", "reason", "path", "dedup_key")}
    if result.get("entry"):
        printable["id"] = result["entry"].get("id")
    print(json.dumps(printable, indent=2))
    if result.get("skipped"):
        return 0
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
