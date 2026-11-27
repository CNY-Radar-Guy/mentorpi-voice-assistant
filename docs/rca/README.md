# Wall-E / Jarvis Root Cause Analysis (RCA)

Standing rule from John (27 Nov 2026, America/New_York): **every crash or unexpected behavior** — service crashes, guard trips, LiDAR dropouts, stalls, regressions — must produce a structured Root Cause Analysis report and be saved so the recursive learning loop can learn from it, fix it, and prevent recurrence.

Public-facing reports and docs use only "John" (never a full legal name).

## Where things live

| What | Path |
|---|---|
| Template | `/home/pi/grokbot/docs/rca/RCA_TEMPLATE.md` (mirrored on ALPHA-ONE at `D:\AI-Models\jarvis\docs\rca\`) |
| Generator / CLI | `/home/pi/grokbot/releases/e48d3d8/gb_rca.py` and `/home/pi/grokbot/rca/gb_rca.py` |
| Watcher | `/home/pi/grokbot/releases/e48d3d8/gb_rca_watch.py` — systemd unit `gb-rca-watch.service` |
| Reports + JSONL index | `/home/pi/grokbot/data/rca/RCA-YYYYMMDD-HHMMSS-<component>-<slug>.md` and `rca_index.jsonl` |
| Unit tests | `/home/pi/grokbot/releases/e48d3d8/tests/test_gb_rca_20261127.py` |

The watcher and generator are **read-only observers** with respect to the robot. They never arm or move Wall-E, never import or edit `gb_gate`, and never write `ESTOP`, `MAINT`, `CLEAR_REQ`, or `ARMED`. The only writes are under `/home/pi/grokbot/data/rca/`.

## How crashes and guard events trigger a report

`gb-rca-watch.service` polls every 5 seconds (nice 15, 5% CPU quota) and drafts at most **one RCA per dedup key per 15-minute cooldown** (a crash loop produces one report, not a folder of spam):

1. **`service_crash`** — `NRestarts` increases, or a watched unit leaves `active` / becomes `failed`, for:
   `grokbot`, `gb-gate`, `gb-guard`, `gb-watchdog`, `gb-intercom`, `gb-armguard`.
2. **`guard_trip`** — a new ` /home/pi/docker/tmp/gb_safety/guard_event.json` appears (new `t` / reason). Cleared/renamed files do not re-fire the same event.
3. **`lidar_dropout`** — `sensors.json` reports `sensors.fresh.lidar == false` for ≥ 15 s, *or* a guard event whose reason is `no LiDAR`.
4. Auto drafts have status `draft` with evidence pre-filled (`journalctl` excerpt, `gate.log` tail, `guard_event.json`, sensor/arm/mode snapshot). A person (or a later `--manual` pass) fills **Root Cause**, **Fix Applied**, **Regression Test**, and **Prevention Rule (Learning Loop)**.

Existing safety units are not modified. There is no `OnFailure=` drop-in on `gb-gate` / `gb-guard` / `gb-watchdog` / `gb-armguard` on purpose: a standalone observer cannot bounce those services and `gb-armguard`'s `ExecStartPre` must not be retriggered (it removes `ARMED`).

## Manual / semi-automated CLI

```bash
# Unexpected behavior that did not trip a watcher hook
/home/pi/grokbot/venv/bin/python /home/pi/grokbot/rca/gb_rca.py \
  --manual --component gb-intercom --trigger unexpected_behavior \
  --symptom "Mic button pressed but nothing came out of Wall-E's speaker"

# Fill the learning-loop sections at the same time
/home/pi/grokbot/venv/bin/python /home/pi/grokbot/rca/gb_rca.py \
  --manual --component gb_guard --trigger lidar_dropout \
  --symptom "..." --status complete \
  --root-cause "..." --fix "..." --regression "..." --prevention "..."
```

`--force` bypasses the cooldown. `--skip-cmds` skips `journalctl` (tests / offline).

## Seeded example (27 Nov 2026 ET)

`RCA-20261127-093849-gb-guard-sclidar-dropout-left-stale-0-19m-front-a.md` records the 56-minute `sclidar` dropout that left `gb_guard` stuck on a stale 0.193 m sweep / 0.192 m front and blocked John's manual drive with "Guard, I held still". It was already complete when the generator was introduced, so the index is useful immediately.

Regenerate it (force) only if the file was lost:

```bash
/home/pi/grokbot/venv/bin/python /home/pi/grokbot/rca/gb_rca.py --seed-sclidar-dropout
```
