# Build log: iterative, ground-up

Wall-E (Hiwonder MentorPi T1, Raspberry Pi 5) and Jarvis (local console on ALPHA-ONE, RTX 5080) are built in short loops: run on a real floor, watch what breaks, diagnose from logs and sensors, deploy a fix with tests, run again. Each entry below is a real failure and what changed because of it.

## 1. Guard-bump false stalls from silent LiDAR dropouts

**Symptom.** Wall-E stops mid-explore with "guard bump detected", or refuses manual drive with "I held still".

**Cause.** The LiDAR node can stop publishing scans without crashing. The guard keeps the last scan, so a stale close reading (one time a 0.19 m "obstacle" in front, with no rear coverage) blocks motion indefinitely. Separately, during a dropout the gate creeps at `0.04 m/s`, which used to sit above `STALL_MIN_CMD = 0.03 m/s`, so the creep looked like a stall and latched a trip file that never cleared.

**Fix.** Restart the LiDAR node to restore fresh scans. `STALL_MIN_CMD` raised to `0.05 m/s`. `clear_trip()` on startup, on disarm, and after `2.0 s` idle. Freshness checks (`LIDAR_FRESH_S`) so stale scans are treated as blind rather than as obstacles.

## 2. Obstacle reposition and continuous nudge recovery

**Symptom.** In tight spaces explore gave up or oscillated.

**Fix.** Wall-E repositions away from the obstacle, re-checks the clearest sector, and keeps making slow, gated nudges until there is room to resume (`gb_explore.py`, `tests/test_explore_reposition_20261127.py`, `tests/test_obstacle_recheck_20261126.py`).

## 3. Temporary clearance reduction for tight spots (camera-verified)

**Symptom.** Stuck between chair legs: the normal stop rules (`0.27 m` front, `0.28 m` rear, `0.24 m` turn sweep, `0.15 m` corridor half width) report "too close" even though a gap wider than the chassis exists.

**Fix.** `gb_fusion_recovery.py`, shared by every mission type (explore, wander, go-to / waypoints, return-home) and re-verified independently in `gb_safety/gb_gate.py`. A stuck mission may cut the side margin in half (physical corridor `0.125 m` half width) and make slow micro-nudges (`0.06–0.07 m/s`, `|ang| <= 0.30 rad/s`, forward or pivot only; reverse is never relaxed) only when:

- the depth camera is fresh, sees floor along the path, and sees no drop-off or obstacle in the chassis path;
- YOLO reports no solid or animate hazard (people, pets, couches...);
- every LiDAR point tripping the normal stop belongs to a narrow cluster (a leg), and the physical chassis corridor is clear.

Any stall, bump, ESTOP, disarm, deadman, or repeated no-progress ends the escape. The reduced clearance lives only in a short-lived per-command token; once Wall-E is clear the normal clearances are back in force and the mission continues. Tests: `tests/test_fusion_recovery_20261127.py`, `tests/test_fusion_recovery_missions_20261127.py`.

## 4. Wake persistence after missions

**Symptom.** After finishing a mission Wall-E fell asleep and needed the wake word again.

**Fix.** A stay-awake session opens on wake and persists after missions until power-off or bench mode (plugging in) returns him to sleep (`grokbot_voice.py`, `tests/test_stayawake_post_mission_20261127.py`).

## 5. Two-way intercom and expanded Vision UI

**What.** A half-duplex intercom: Jarvis streams John's voice to Wall-E's speaker while he talks, then switches to Wall-E's onboard mic so John can hear the room (Auto-VAD with ~0.9 s silence switch-back, or push-to-talk). Noise gate, capped speech boost, and auto-close of the speaker after 0.6 s of silence keep it from hissing. With the intercom off Wall-E is back to normal dual-listen (Jarvis commands plus onboard mic). Pi side: `wall-e/intercom/gb_intercom.py` (audio only, never touches motion). Jarvis side: `jarvis/wally-intercom.js` and the mic button in `jarvis/public/index.html`.

**UI.** The Jarvis chat column was narrowed and the Wall-E Vision panel (LiDAR map, FPV, YOLO click-to-label) expanded, with a drag handle.

## Earlier lessons (November 5–26)

- Split speed caps: manual `0.288 m/s`, autonomous explore `1.0 m/s` hard cap starting at `0.1012 m/s` and stepping up only while sensors allow.
- Nav2-style slow / contact / escape zones, IMU and encoder stall checks, and a 3-in-60 s fault ESTOP latch.
- Voice: echo masking, VAD tuning, barge-in, and wake-to-mission handoff.
