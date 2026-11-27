# RCA-20261127-093849-gb-guard-sclidar-dropout-left-stale-0-19m-front-a — sclidar dropout left stale 0.19m front and blocked manual drive

> Standing rule (John, 27 Nov 2026 ET): every crash or unexpected behavior on Wall-E / Jarvis must produce a structured Root Cause Analysis and be saved for the recursive learning loop.

## Metadata

| Field | Value |
|---|---|
| RCA ID | `RCA-20261127-093849-gb-guard-sclidar-dropout-left-stale-0-19m-front-a` |
| Timestamp (ET) | 2026-11-27 09:38:49 ET |
| Status | complete |
| Trigger type | `lidar_dropout` |
| Affected component / service | gb_guard |
| Reporter | John (incident) / RCA generator (seeded 27 Nov 2026 ET) |
| Related reports | manual-drive "Guard, I held still" / sclidar_node restart 09:42 ET |

## Symptom

John tried to manually drive Wall-E and got the spoken / UI message "Guard, I held still". The safety gate blocked the command. At the moment of the trip, front range was stuck at 0.192 m (sweep 0.193 m) and the trip reason was "no LiDAR: reverse refused (no rear sense)".

## Timeline (America/New_York)

- 08:44:29 ET — Last useful LiDAR-related gate note before the silent gap: `SLOW slow 0.040 m/s (LiDAR dropouts behind)` during the morning explore session.
- ~08:44–09:38 ET — `sclidar_node` silently stopped publishing scans (~54–56 minutes). `gb_guard` / `gb_gate` kept the last ranges (front 0.192 m, rear 0.302 m, sweep 0.193 m) instead of treating the sensor as offline for all directions.
- 09:38:42 ET — John armed Wall-E from the Jarvis UI to drive manually.
- 09:38:49 ET — `GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19`. Voice / UI reported "Guard, I held still".
- 09:38:51 ET — Trip cleared on idle-timeout; stale ranges unchanged.
- 09:40:17 ET — Second manual attempt: `STOP (autonomous refused: health monitor missing, stale, or no validated range)` immediately followed by the same `no LiDAR: reverse refused` trip with identical stale ranges.
- 09:40:20 ET — Second trip cleared on idle-timeout.
- 09:41:32 ET — Remote disarm (Jarvis).
- 09:42 ET — `sclidar_node` (`ros2 launch peripherals lidar.launch.py`, scan remapped to `scan_raw`) was restarted inside the MentorPi container. Fresh scans resumed; front opened to about 1.33 m.
- After the restart — Manual drive path was no longer blocked by the stale 0.19 m reading. This RCA records the incident for the learning loop.

## Evidence

### Sensor / arm / mode snapshot

```json
{
  "captured_at_et": "2026-11-27 09:38:49 ET (incident; reconstructed from guard_event + gate.log)",
  "armed": true,
  "estop": false,
  "mode": "stream",
  "note": "Live sensors.json was not snapshotted at the exact trip instant. Values below are the latched ranges written to guard_event.json, which stayed bit-identical across both trips 88 seconds apart \u2014 evidence the scan was stale, not a new measurement.",
  "stale_ranges_m": {
    "front": 0.19183446823212913,
    "rear": 0.30194129944373,
    "sweep": 0.19300000369548798
  },
  "trip_why": "no LiDAR: reverse refused (no rear sense)",
  "lidar_process_after_fix": "sclidar_node restarted 2026-11-27 09:42 ET via ros2 launch peripherals lidar.launch.py (scan:=scan_raw)"
}
```

### `guard_event.json`

```json
{
  "t": 1795790329.008666,
  "why": "no LiDAR: reverse refused (no rear sense)",
  "src": "stream",
  "front": 0.19183446823212913,
  "rear": 0.30194129944373,
  "sweep": 0.19300000369548798,
  "cleared_as": "guard_event.json.cleared-20261127-143851"
}
```

### `gate.log` excerpt

```
2026-11-27 13:44:29.872 [gate] SLOW slow 0.040 m/s (LiDAR dropouts behind)
2026-11-27 14:38:49.008 [gate] GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
2026-11-27 14:38:51.810 [gate] GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
2026-11-27 14:40:17.309 [gate] STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.02s -> zero now, then 10 Hz zero for 2.0s
2026-11-27 14:40:17.409 [gate] STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.01s -> zero now, then 10 Hz zero for 2.0s
2026-11-27 14:40:17.908 [gate] GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
2026-11-27 14:40:20.311 [gate] GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
```

### `journalctl` excerpt

```
Nov 27 08:44:29 wally bash[5754]: [INFO] [gb_gate]: SLOW slow 0.040 m/s (LiDAR dropouts behind)
Nov 27 09:38:49 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
Nov 27 09:38:51 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
Nov 27 09:40:17 wally bash[5754]: [INFO] [gb_gate]: STOP (autonomous refused: health monitor missing, stale, or no validated range) was=stream ran=0.02s -> zero now, then 10 Hz zero for 2.0s
Nov 27 09:40:17 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP (stream): no LiDAR: reverse refused (no rear sense) | front=0.19 rear=0.30 sweep=0.19
Nov 27 09:40:20 wally bash[5754]: [INFO] [gb_gate]: GUARD TRIP cleared (idle-timeout): was no LiDAR: reverse refused (no rear sense)
```

### Archived `guard_event.json` (both trips, identical stale ranges)

```json
{
  "first": {
    "t": 1795790329.008666,
    "why": "no LiDAR: reverse refused (no rear sense)",
    "src": "stream",
    "front": 0.19183446823212913,
    "rear": 0.30194129944373,
    "sweep": 0.19300000369548798,
    "cleared_as": "guard_event.json.cleared-20261127-143851"
  },
  "second": {
    "t": 1795790417.908803,
    "why": "no LiDAR: reverse refused (no rear sense)",
    "src": "stream",
    "front": 0.19183446823212913,
    "rear": 0.30194129944373,
    "sweep": 0.19300000369548798,
    "cleared_as": "guard_event.json.cleared-20261127-144020"
  }
}
```

`sclidar_node` process after the 09:42 ET restart (MentorPi container, UTC times in `ps`):
`/home/ubuntu/third_party_ros2/third_party_ws/install/sclidar_ros2/lib/sclidar_ros2/sclidar --ros-args -r __node:=sclidar_node -r scan:=scan_raw` started 14:42 UTC.


## Root Cause

The `sclidar` ROS 2 node inside the MentorPi container silently stopped publishing LaserScan messages for about 56 minutes. It did not crash the `gb-guard` or `gb-gate` systemd units, so those services stayed "active" while their last ranges froze (front 0.192 m, sweep 0.193 m). When John tried to drive manually, the gate correctly refused reverse because it had no rear LiDAR sense, and the voice path announced "Guard, I held still". The safety gate was not misreading the joystick and there was no stale ESTOP latch. The misleading part was that the frozen 0.19 m front reading looked like a real close obstacle rather than an offline lidar. Identical range bits on the 09:38:49 ET and 09:40:17 ET trips confirm the values were latched, not freshly measured.

## Fix Applied

The `sclidar_node` / `ros2 launch peripherals lidar.launch.py` process was restarted inside the MentorPi container at 09:42 ET. Fresh scans started flowing again and the front range opened to about 1.33 m, after which manual drive was no longer blocked by the stale 0.19 m reading. No motion-gate logic was changed as part of that recovery. This RCA generator / watcher is the follow-up so the same class of failure produces a saved report automatically next time.

## Regression Test

- Confirm `sclidar_node` is running in the MentorPi container and `/scan` (or `scan_raw`) is publishing.
- Confirm `sensors.json` shows `sensors.fresh.lidar == true` and a changing front range (not stuck at 0.192 m).
- Armed manual drive forward/reverse is allowed only when LiDAR is fresh; a missing scan must trip with an explicit "no LiDAR" reason, not a stale close-range obstacle.
- Unit tests in `tests/test_gb_rca_20261127.py` cover RCA generation, cooldown dedup, and JSONL indexing.
- After deploy: safety services remain active, Wall-E remains disarmed, zero motion, `gb-rca-watch.service` is running.

## Prevention Rule (Learning Loop)

1. Treat LiDAR *scan freshness* as a first-class health signal, independent of the last range value. If scans stop for longer than the short memory window, fail to "LiDAR offline" in every direction and never keep advertising a latched 0.19 m (or any last range) as a live obstacle.
2. Voice / Jarvis UI should say that LiDAR is stale or offline when the trip reason is "no LiDAR", not only "Guard, I held still", so a frozen range is not mistaken for a chair leg.
3. `gb-rca-watch` must auto-draft an RCA on `lidar_dropout`, on a new `guard_event.json` trip, and on a systemd crash/restart of grokbot / gb-gate / gb-guard / gb-watchdog / gb-intercom / gb-armguard, rate-limited to one report per cooldown window so a crash loop cannot spam the index.
4. Recurring `sclidar` silence should be visible in loopwatch / health status even when gb-gate itself is still heartbeating.

## Follow-ups

- Consider a dedicated scan-topic freshness probe (read-only) alongside `sensors.json` `fresh.lidar`.
- Keep this report in the learning-loop index (`rca_index.jsonl`) as the reference example for stale-LiDAR false obstacles.
- Do not treat a later GitHub README / docs sync as a reason to include anyone's full legal name; use "John" only.
