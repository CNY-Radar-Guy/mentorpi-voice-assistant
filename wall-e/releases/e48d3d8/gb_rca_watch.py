#!/usr/bin/env python3
"""gb-rca-watch: read-only observer that drafts RCAs on crashes and guard trips.

Never arms, never moves, never writes ESTOP / MAINT / CLEAR_REQ / ARMED,
never imports or patches gb_gate, and never runs systemctl start/stop/restart
on any Wall-E unit. The only writes are under GB_RCA_DIR.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional

# Allow `python gb_rca_watch.py` from the release directory or /home/pi/grokbot/rca.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gb_rca as RCA

POLL_S = float(os.environ.get("GB_RCA_POLL_S", "5"))
LIDAR_DROPOUT_S = float(os.environ.get("GB_RCA_LIDAR_DROPOUT_S", "15"))
STATUS_NAME = "rca_watch_status.json"


def log(*a) -> None:
    print(time.strftime("%Y-%m-%d %H:%M:%S"), "[rca-watch]", *a, flush=True)


def _run(cmd, timeout=10):
    return RCA._run(list(cmd), timeout=timeout)


def systemd_state(units) -> dict[str, dict[str, str]]:
    raw = _run(["systemctl", "show", "-p", "Id,ActiveState,NRestarts,SubState,Result"] + list(units))
    out, cur = {}, {}
    for line in (raw or "").splitlines() + [""]:
        if not line.strip() or line.startswith("(exit ") or line.startswith("(command failed"):
            if cur.get("Id"):
                out[cur["Id"]] = cur
            cur = {}
            continue
        if "=" not in line:
            continue
        k, _, v = line.partition("=")
        cur[k] = v
    return out


def write_status(cfg: RCA.RcaConfig, payload: dict[str, Any]) -> None:
    RCA.ensure_rca_dir(cfg)
    path = cfg.rca_dir / STATUS_NAME
    tmp = path.with_suffix(".json.tmp")
    try:
        tmp.write_text(json.dumps(payload, default=str, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, path)
    except Exception as ex:
        log("status write failed:", ex)


def _draft(cfg: RCA.RcaConfig, *, component: str, trigger: str, symptom: str, timeline: str = "", extra: str = "") -> Optional[dict[str, Any]]:
    try:
        result = RCA.generate_rca(
            cfg,
            component=component,
            trigger=trigger,
            symptom=symptom,
            reporter="gb-rca-watch",
            status="draft",
            timeline=timeline or RCA.PLACEHOLDER_TIMELINE,
            force=False,
            skip_cmds=False,
            extra_evidence=extra,
        )
    except Exception as ex:
        log("generate failed:", ex)
        return None
    if result.get("skipped"):
        log("dedup:", result.get("reason"))
    else:
        log("wrote", result.get("path"))
    return result


def _nrestarts(st: dict[str, str]) -> int:
    try:
        return int(st.get("NRestarts") or 0)
    except ValueError:
        return 0


def check_units(cfg: RCA.RcaConfig, prev: dict[str, dict[str, str]], now_map: dict[str, dict[str, str]]) -> None:
    for unit in RCA.WATCHED_UNITS:
        cur = now_map.get(unit) or {}
        old = prev.get(unit) or {}
        if not cur:
            continue
        active = cur.get("ActiveState", "unknown")
        n = _nrestarts(cur)
        prev_n = _nrestarts(old) if old else n
        prev_active = old.get("ActiveState") if old else None
        crashed = False
        why = []
        if old and n > prev_n:
            crashed = True
            why.append("NRestarts increased %s -> %s" % (prev_n, n))
        if active == "failed" and prev_active != "failed":
            crashed = True
            why.append("ActiveState=failed (Result=%s)" % cur.get("Result", "?"))
        if prev_active == "active" and active in ("activating", "deactivating", "inactive", "failed"):
            # A running unit left 'active' — likely a crash + Restart=.
            crashed = True
            why.append("left active (%s -> %s)" % (prev_active, active))
        if crashed:
            _draft(
                cfg,
                component=unit,
                trigger="service_crash",
                symptom="systemd %s: %s. SubState=%s Result=%s."
                % (unit, "; ".join(why) or "unexpected state change", cur.get("SubState", "?"), cur.get("Result", "?")),
                timeline="- %s ET — %s" % (RCA.format_et(tz=cfg.tz), "; ".join(why)),
            )


def _event_token(ev: Any) -> Optional[str]:
    if not isinstance(ev, dict):
        return None
    why = ev.get("why") or ev.get("reason") or ev.get("msg")
    t = ev.get("t") or ev.get("ts")
    if not why and t is None:
        return None
    return "%s|%s" % (t, why)


def check_guard_event(cfg: RCA.RcaConfig, last_token: Optional[str]) -> Optional[str]:
    ev = RCA.collect_guard_event(cfg)
    token = _event_token(ev)
    if token and token != last_token:
        why = ev.get("why") or ev.get("reason") or "guard trip"
        front = ev.get("front")
        rear = ev.get("rear")
        sweep = ev.get("sweep")
        trigger = "lidar_dropout" if "no lidar" in str(why).lower() or "lidar" in str(why).lower() and "stale" in str(why).lower() else "guard_trip"
        # "no LiDAR" is primarily a lidar_dropout; other trips stay guard_trip.
        if "no lidar" in str(why).lower():
            trigger = "lidar_dropout"
        _draft(
            cfg,
            component="gb_guard",
            trigger=trigger,
            symptom="guard_event.json trip: %s (front=%s rear=%s sweep=%s, src=%s)"
            % (why, front, rear, sweep, ev.get("src")),
            timeline="- %s ET — new guard_event.json (%s)" % (RCA.format_et(tz=cfg.tz), why),
            extra="### Live guard_event.json\n\n```json\n%s\n```" % json.dumps(ev, indent=2, default=str),
        )
        return token
    # File gone (cleared / renamed) — keep last token so a rewrite of the same event does not double-fire
    # if the watcher missed the clear; a new event has a new `t`.
    return token if token is not None else last_token


def lidar_fresh(snapshot: dict[str, Any]) -> Optional[bool]:
    sensors = snapshot.get("sensors") if isinstance(snapshot, dict) else None
    if not isinstance(sensors, dict):
        return None
    block = sensors.get("sensors") if isinstance(sensors.get("sensors"), dict) else sensors
    fresh = block.get("fresh") if isinstance(block.get("fresh"), dict) else {}
    if "lidar" in fresh:
        return bool(fresh.get("lidar"))
    return None


def check_lidar(cfg: RCA.RcaConfig, stale_since: Optional[float], now: float) -> Optional[float]:
    snap = RCA.collect_snapshot(cfg)
    fresh = lidar_fresh(snap)
    if fresh is True:
        return None
    if fresh is None:
        return stale_since
    # fresh is False
    if stale_since is None:
        return now
    if now - stale_since >= LIDAR_DROPOUT_S:
        _draft(
            cfg,
            component="sclidar",
            trigger="lidar_dropout",
            symptom="sensors.json reports sensors.fresh.lidar=false for at least %.0f s (Wall-E may be holding a stale last range)."
            % (now - stale_since),
            timeline="- %s ET — lidar freshness false for %.0f s" % (RCA.format_et(now, cfg.tz), now - stale_since),
        )
        # Keep stale_since so cooldown (not this flag) suppresses repeats; reset when fresh again.
        return stale_since
    return stale_since


def main() -> int:
    cfg = RCA.RcaConfig()
    RCA.ensure_rca_dir(cfg)
    log(
        "start poll=%.1fs lidar_dropout=%.0fs cooldown=%.0fs rca_dir=%s (read-only observer; no motion, no safety writes)"
        % (POLL_S, LIDAR_DROPOUT_S, cfg.cooldown_s, cfg.rca_dir)
    )
    prev: dict[str, dict[str, str]] = {}
    last_guard: Optional[str] = _event_token(RCA.collect_guard_event(cfg))
    stale_since: Optional[float] = None
    # Seed prev so we do not RCA every unit on first poll after a watcher restart.
    prev = systemd_state(RCA.WATCHED_UNITS)
    while True:
        now = time.time()
        units = systemd_state(RCA.WATCHED_UNITS)
        try:
            check_units(cfg, prev, units)
            last_guard = check_guard_event(cfg, last_guard)
            stale_since = check_lidar(cfg, stale_since, now)
        except Exception as ex:
            log("poll error:", ex)
        prev = units or prev
        write_status(cfg, {
            "t": now,
            "ts_et": RCA.format_et(now, cfg.tz),
            "poll_s": POLL_S,
            "lidar_dropout_s": LIDAR_DROPOUT_S,
            "cooldown_s": cfg.cooldown_s,
            "units": {u: {"active": (units.get(u) or {}).get("ActiveState"), "NRestarts": (units.get(u) or {}).get("NRestarts")} for u in RCA.WATCHED_UNITS},
            "guard_token": last_guard,
            "lidar_stale_since": stale_since,
            "note": "observer only; never arms/moves or touches gb_gate",
        })
        time.sleep(POLL_S)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        log("stop")
        sys.exit(0)
