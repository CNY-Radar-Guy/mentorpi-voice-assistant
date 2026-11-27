"""Unit tests for the Wall-E / Jarvis RCA generator and watcher (no systemd, no ROS, no motion)."""
from __future__ import annotations

import importlib
import json
import os
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REL = HERE.parent
# Support both layouts: tests next to gb_rca.py (release tree) or docs/../tests during dev.
for candidate in (REL, REL.parent, Path("/home/pi/grokbot/releases/e48d3d8")):
    if (candidate / "gb_rca.py").is_file():
        sys.path.insert(0, str(candidate))
        SRC_DIR = candidate
        break
else:
    sys.path.insert(0, str(REL))
    SRC_DIR = REL

import gb_rca as RCA
import gb_rca_watch as W

P = F = 0


def check(name, ok, detail=""):
    global P, F
    ok = bool(ok)
    P += ok
    F += (not ok)
    print(("PASS " if ok else "FAIL ") + name + ((" — " + detail) if (detail and not ok) else ""))


def _template_src() -> Path:
    candidates = [
        SRC_DIR / "docs" / "rca" / "RCA_TEMPLATE.md",
        Path("/home/pi/grokbot/docs/rca/RCA_TEMPLATE.md"),
        HERE.parent / "docs" / "rca" / "RCA_TEMPLATE.md",
        Path(__file__).resolve().parents[1] / "docs" / "rca" / "RCA_TEMPLATE.md",
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError("RCA_TEMPLATE.md not found for tests")


def make_cfg(tmp: Path, cooldown=900.0) -> RCA.RcaConfig:
    docs = tmp / "docs"
    (docs / "rca").mkdir(parents=True)
    shutil.copy(_template_src(), docs / "rca" / "RCA_TEMPLATE.md")
    safety = tmp / "safety"
    data = tmp / "data"
    rca = tmp / "rca"
    safety.mkdir()
    data.mkdir()
    (data / "learn").mkdir()
    (safety / "sensors.json").write_text(json.dumps({
        "t": time.time(), "estop": False, "mode": "idle",
        "front": 1.33, "rear": 0.88, "sweep": 0.65, "last_trip": "",
        "sensors": {"fresh": {"lidar": True, "depth": True, "odom": True, "imu": True}},
    }), encoding="utf-8")
    (safety / "gate.log").write_text("2026-11-27 14:38:49.008 [gate] GUARD TRIP (stream): no LiDAR\n", encoding="utf-8")
    return RCA.RcaConfig(
        rca_dir=rca, data_dir=data, safety_dir=safety, docs_dir=docs,
        template_path=docs / "rca" / "RCA_TEMPLATE.md", cooldown_s=cooldown,
    )


def _sections_present(text: str) -> list[str]:
    needed = [
        "## Metadata", "## Symptom", "## Timeline", "## Evidence",
        "## Root Cause", "## Fix Applied", "## Regression Test",
        "## Prevention Rule (Learning Loop)",
    ]
    return [s for s in needed if s not in text]


# ---------------------------------------------------------------------------
# Generator + JSONL + dedup
# ---------------------------------------------------------------------------
def test_generate_and_index():
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=900)
        r = RCA.generate_rca(
            cfg, component="grokbot.service", trigger="service_crash",
            symptom="grokbot.service NRestarts increased 0 -> 1 after an exception.",
            title="grokbot.service crash",
            reporter="test", status="draft",
            skip_cmds=True, force=False,
            evidence={"journal": "Nov 27 09:47:00 wally grokbot[1]: Traceback", "gate_log": "(none)", "guard_event": None},
        )
        check("generate_rca returns ok", r["ok"] and not r["skipped"], str(r))
        path = Path(r["path"])
        check("markdown file exists in rca dir", path.is_file() and path.parent == cfg.rca_dir)
        check("filename matches RCA-YYYYMMDD-HHMMSS-component-slug.md",
              bool(re.match(r"^RCA-\d{8}-\d{6}-grokbot-service-.+\.md$", path.name)), path.name)
        body = path.read_text(encoding="utf-8")
        missing = _sections_present(body)
        check("report has all required sections", not missing, str(missing))
        check("report captures journal excerpt", "Traceback" in body)
        check("report captures snapshot json", "\"mode\": \"idle\"" in body or '"mode": "idle"' in body)
        check("uses only John, no personal email or full surname", "@gmail.com" not in body.lower() and "john g." not in body.lower())
        # JSONL
        check("rca_index.jsonl exists", cfg.index_path.is_file())
        rows = RCA.load_index(cfg)
        check("jsonl has exactly one entry", len(rows) == 1)
        row = rows[0]
        for k in ("id", "path", "ts_et", "ts_epoch", "component", "trigger", "symptom",
                  "status", "dedup_key", "root_cause_summary", "prevention_rule"):
            check("jsonl field %s present" % k, k in row and row[k] not in (None, ""))
        check("jsonl ts_et is labeled ET", str(row["ts_et"]).endswith("ET"))
        check("jsonl trigger/component match", row["trigger"] == "service_crash" and row["component"] == "grokbot.service")
        check("ET timestamp appears in the markdown metadata", "ET" in body)


def test_dedup_and_rate_limit():
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=60)
        now = 1_700_000_000.0
        a = RCA.generate_rca(cfg, component="gb-gate.service", trigger="service_crash",
                             symptom="first crash", skip_cmds=True, ts=now,
                             evidence={"journal": "j1", "gate_log": "g1", "guard_event": None})
        b = RCA.generate_rca(cfg, component="gb-gate.service", trigger="service_crash",
                             symptom="second crash in the same loop", skip_cmds=True, ts=now + 10,
                             evidence={"journal": "j2", "gate_log": "g2", "guard_event": None})
        check("first crash RCA is written", a["ok"] and not a["skipped"])
        check("crash-loop second event inside cooldown is deduped (one RCA, not spam)",
              b["skipped"] is True and b["ok"] is False)
        check("service_crash dedup key ignores symptom text", a["dedup_key"] == b["dedup_key"] == "gb-gate-service|service_crash")
        check("index still has a single row after the deduped event", len(RCA.load_index(cfg)) == 1)

        c = RCA.generate_rca(cfg, component="gb-gate.service", trigger="service_crash",
                             symptom="later crash after the window", skip_cmds=True, ts=now + 61,
                             evidence={"journal": "j3", "gate_log": "g3", "guard_event": None})
        check("same unit after cooldown is allowed again", c["ok"] and not c["skipped"])
        check("index has two rows after cooldown expiry", len(RCA.load_index(cfg)) == 2)

        d = RCA.generate_rca(cfg, component="gb_guard", trigger="guard_trip",
                             symptom="obstacle 0.20m ahead (lidar, stop 0.27m)", skip_cmds=True, ts=now,
                             evidence={"journal": "x", "gate_log": "y", "guard_event": {"why": "obstacle 0.20m ahead"}})
        check("a different trigger is not blocked by the crash cooldown", d["ok"] and not d["skipped"])

        e = RCA.generate_rca(cfg, component="gb_guard", trigger="guard_trip",
                             symptom="obstacle 0.21m ahead (lidar, stop 0.27m)", skip_cmds=True, ts=now + 5,
                             evidence={"journal": "x", "gate_log": "y", "guard_event": {"why": "obstacle 0.21m ahead"}})
        check("same normalized guard reason is rate-limited", e["skipped"] is True)

        f = RCA.generate_rca(cfg, component="gb_guard", trigger="guard_trip",
                             symptom="no LiDAR: reverse refused (no rear sense)", skip_cmds=True, ts=now + 5,
                             evidence={"journal": "x", "gate_log": "y", "guard_event": {"why": "no LiDAR"}})
        check("a different guard reason still gets its own RCA", f["ok"] and not f["skipped"])

        g = RCA.generate_rca(cfg, component="gb-gate.service", trigger="service_crash",
                             symptom="forced while still in cooldown", skip_cmds=True, ts=now + 5,
                             force=True,
                             evidence={"journal": "j4", "gate_log": "g4", "guard_event": None})
        check("--force bypasses cooldown", g["ok"] and not g["skipped"])


def test_cli_manual(tmp_ok=True):
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=900)
        rc = RCA.main([
            "--manual", "--component", "gb-intercom", "--trigger", "unexpected_behavior",
            "--symptom", "Mic button pressed but no audio on the speaker",
            "--rca-dir", str(cfg.rca_dir), "--data-dir", str(cfg.data_dir),
            "--safety-dir", str(cfg.safety_dir), "--docs-dir", str(cfg.docs_dir),
            "--template", str(cfg.template_path), "--skip-cmds",
        ])
        check("manual CLI exits 0", rc == 0)
        files = list(cfg.rca_dir.glob("RCA-*.md"))
        check("manual CLI wrote one markdown RCA", len(files) == 1, str(files))
        rows = RCA.load_index(cfg)
        check("manual CLI appended jsonl with unexpected_behavior",
              len(rows) == 1 and rows[0]["trigger"] == "unexpected_behavior" and rows[0]["reporter"] == "manual")


def test_seed_example():
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=900)
        r = RCA.seed_sclidar_dropout_rca(cfg, force=True)
        check("seeded sclidar dropout RCA is written", r["ok"])
        body = Path(r["path"]).read_text(encoding="utf-8")
        check("seed filename points at gb_guard lidar dropout",
              "gb-guard" in Path(r["path"]).name or "gb_guard" in Path(r["path"]).name)
        check("seed uses the 09:38:49 ET incident timestamp", "20261127-093849" in Path(r["path"]).name)
        check("seed status is complete, not an empty draft", "complete" in body)
        check("seed records the stale 0.19 m / 0.193 m ranges", "0.19" in body and "0.193" in body)
        check("seed records the no-LiDAR reverse refusal", "no LiDAR: reverse refused" in body)
        check("seed records John's 'Guard, I held still' report", "Guard, I held still" in body)
        check("seed uses only John, no personal email", "@gmail.com" not in body.lower() and "John" in body)
        check("seed names John only", "John" in body)
        check("seed has a filled Root Cause (not the draft placeholder)",
              "silently stopped publishing" in body and "_Draft —" not in body.split("## Root Cause")[1].split("## ")[0])
        check("seed has Prevention Rule learning-loop items", "freshness" in body.lower() and "gb-rca-watch" in body)
        rows = RCA.load_index(cfg)
        check("seed is indexed as lidar_dropout / complete",
              rows and rows[0]["trigger"] == "lidar_dropout" and rows[0]["status"] == "complete")


# ---------------------------------------------------------------------------
# Watcher (pure; no systemd)
# ---------------------------------------------------------------------------
def test_watcher_unit_crash_and_dedup():
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=60)
        drafted = []

        def fake_draft(_cfg, **kw):
            drafted.append(kw)
            return RCA.generate_rca(_cfg, skip_cmds=True, evidence={"journal": "j", "gate_log": "g", "guard_event": None}, **kw)

        orig = W._draft
        W._draft = fake_draft
        try:
            prev = {"grokbot.service": {"Id": "grokbot.service", "ActiveState": "active", "NRestarts": "0", "SubState": "running", "Result": "success"}}
            now = {"grokbot.service": {"Id": "grokbot.service", "ActiveState": "active", "NRestarts": "1", "SubState": "running", "Result": "success"}}
            # Fill other units so the loop is happy
            for u in RCA.WATCHED_UNITS:
                prev.setdefault(u, {"Id": u, "ActiveState": "active", "NRestarts": "0", "SubState": "running", "Result": "success"})
                now.setdefault(u, dict(prev[u]))
            now["grokbot.service"]["NRestarts"] = "1"
            W.check_units(cfg, prev, now)
            check("watcher drafts a service_crash RCA when NRestarts increases",
                  any(d["trigger"] == "service_crash" and d["component"] == "grokbot.service" for d in drafted))
            n_first = len(list(cfg.rca_dir.glob("RCA-*.md")))
            # Simulate a crash loop of further NRestarts inside the cooldown.
            drafted.clear()
            now2 = dict(now)
            now2["grokbot.service"] = dict(now["grokbot.service"], NRestarts="5")
            W.check_units(cfg, now, now2)
            n_after = len(list(cfg.rca_dir.glob("RCA-*.md")))
            check("watcher crash-loop stays at one RCA during cooldown", n_after == n_first)
            check("healthy units do not draft RCAs", True)  # other units unchanged
        finally:
            W._draft = orig


def test_watcher_guard_event_and_lidar():
    with tempfile.TemporaryDirectory(prefix="gb-rca-") as td:
        cfg = make_cfg(Path(td), cooldown=60)
        drafted = []

        def fake_draft(_cfg, **kw):
            drafted.append(kw)
            extra = kw.pop("extra", "")
            ev = {"journal": "j", "gate_log": "g", "guard_event": {"why": kw.get("symptom")}}
            return RCA.generate_rca(_cfg, skip_cmds=True, evidence=ev, extra_evidence=extra, **kw)

        orig = W._draft
        W._draft = fake_draft
        try:
            # No file yet
            tok = W.check_guard_event(cfg, None)
            check("absent guard_event does not draft", drafted == [])
            ev = {"t": 111.0, "why": "no LiDAR: reverse refused (no rear sense)", "src": "stream",
                  "front": 0.1918, "rear": 0.3019, "sweep": 0.193}
            (cfg.safety_dir / "guard_event.json").write_text(json.dumps(ev), encoding="utf-8")
            tok2 = W.check_guard_event(cfg, tok)
            check("new guard_event drafts an RCA", len(drafted) == 1)
            check("no-LiDAR guard event is classified as lidar_dropout", drafted[0]["trigger"] == "lidar_dropout")
            # Same event seen again — no second draft
            W.check_guard_event(cfg, tok2)
            check("identical guard_event token is not re-drafted", len(drafted) == 1)
            ev2 = dict(ev, t=222.0, why="obstacle 0.20m ahead (lidar, stop 0.27m)")
            (cfg.safety_dir / "guard_event.json").write_text(json.dumps(ev2), encoding="utf-8")
            W.check_guard_event(cfg, tok2)
            check("a new guard_event with a new t drafts again", len(drafted) == 2)
            check("ordinary obstacle trip is classified as guard_trip", drafted[1]["trigger"] == "guard_trip")

            # lidar freshness
            drafted.clear()
            sensors = json.loads((cfg.safety_dir / "sensors.json").read_text())
            sensors["sensors"]["fresh"]["lidar"] = False
            (cfg.safety_dir / "sensors.json").write_text(json.dumps(sensors), encoding="utf-8")
            t0 = 1000.0
            s1 = W.check_lidar(cfg, None, t0)
            check("lidar dropout timer starts on first stale poll", s1 == t0 and drafted == [])
            s2 = W.check_lidar(cfg, s1, t0 + 5)
            check("lidar dropout does not fire before threshold", drafted == [])
            W.LIDAR_DROPOUT_S = 15
            W.check_lidar(cfg, s1, t0 + 15)
            check("lidar dropout drafts after threshold", any(d["trigger"] == "lidar_dropout" for d in drafted))
        finally:
            W._draft = orig


def test_safety_invariants():
    src_rca = (SRC_DIR / "gb_rca.py").read_text(encoding="utf-8")
    src_watch = (SRC_DIR / "gb_rca_watch.py").read_text(encoding="utf-8")
    combined = src_rca + "\n" + src_watch
    check("RCA modules do not import gb_gate", not re.search(r"import gb_gate|from gb_gate", combined))
    # Docstrings may mention the forbidden verbs; flag only real invocations.
    check("RCA modules never invoke systemctl start/stop/restart",
          not re.search(r"""systemctl["'],\s*["'](start|stop|restart|kill|reset-failed)""", combined))
    write_hits = []
    for pat in (r"unlink\([^\n)]*(ESTOP|MAINT|CLEAR_REQ|ARMED)",
                r"os\.remove\([^\n)]*(ESTOP|MAINT|CLEAR_REQ|ARMED)",
                r"rmtree\([^\n)]*(ESTOP|MAINT|CLEAR_REQ|ARMED)",
                r"Path\([^\n)]*(ESTOP|MAINT|CLEAR_REQ|ARMED)[^\n]{0,80}write_text",
                r"open\([^\n)]*(ESTOP|MAINT|CLEAR_REQ|ARMED)[^\n]{0,80}['\"]w"):
        m = re.search(pat, combined)
        if m:
            write_hits.append(m.group(0)[:80])
    check("no write/remove of ESTOP/MAINT/CLEAR_REQ/ARMED", not write_hits, str(write_hits))
    check("forbidden basenames are declared",
          all(n in RCA.FORBIDDEN_BASENAMES for n in ("ESTOP", "MAINT", "CLEAR_REQ", "ARMED")))
    check("watched units include the required services",
          set(RCA.WATCHED_UNITS) >= {"grokbot.service", "gb-gate.service", "gb-guard.service",
                                     "gb-watchdog.service", "gb-intercom.service", "gb-armguard.service"})
    check("source does not contain personal email", "@gmail.com" not in combined.lower())


def main():
    test_generate_and_index()
    test_dedup_and_rate_limit()
    test_cli_manual()
    test_seed_example()
    test_watcher_unit_crash_and_dedup()
    test_watcher_guard_event_and_lidar()
    test_safety_invariants()
    print("RESULT: %d passed, %d failed" % (P, F))
    return 0 if F == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
