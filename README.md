# Jarvis and Wall-E

John's working notes for a tracked rover built with two kinds of AI at once: a centralized partner for design and diagnosis, and a local model that stays on the workbench.

The robot is **Wall-E** (Wally), a Hiwonder MentorPi. The console in front of it is **Jarvis**, on a salvaged workstation named **ALPHA-ONE**. This page is the project intro. As of November 27, 2026 the live Wall-E code (Pi release `e48d3d8`) and the Jarvis console code from ALPHA-ONE are both in this repository under [`wall-e/`](wall-e/) and [`jarvis/`](jarvis/).

## A hybrid way to build

Everyday AI tasks are one end of what these systems can do. This project explores the far end: a real tracked robot, live sensors, and a local command console.

**Grok Bot**, from the xAI team, acts as a centralized conductor for design, diagnosis, and iteration.

**Qwen** (`qwen3:14b`) runs locally through **Ollama** on ALPHA-ONE, a salvaged workstation with an RTX 5080. Putting the model there takes the heavy compute off the MentorPi, toward longer-term self-sufficiency: more of the rover's work can stay in the room.

Central and local AI working together is the practical pattern this project is testing.

## An iterative, ground-up build

This is not a kit demo or a finished product. Wall-E and Jarvis are being built from the ground up, one real-world failure at a time: John drives or sends the robot on a mission, something goes wrong on a real floor, the cause is diagnosed from logs and sensor data, a fix plus unit tests is deployed to the Pi, and the next run tests it again. The code in this repository is a snapshot of that loop, not a polished release. See [`docs/BUILD_LOG.md`](docs/BUILD_LOG.md) for the full list of lessons.

### Lessons learned so far

- **Guard-bump false stalls from silent LiDAR dropouts.** The LiDAR node can stop publishing without crashing. The safety guard then keeps acting on the last (stale) scan, for example a 0.19 m "obstacle" in front, and blocks motion, which shows up as "guard bump" or "I held still" even during manual drive. Restarting the LiDAR node restores fresh scans and clears the block. Related fixes: the dropout creep (`0.04 m/s`) no longer counts as a stall (`STALL_MIN_CMD` raised to `0.05 m/s`), and stale trips are cleared on startup, disarm, and after `2.0 s` idle.
- **Obstacle reposition and continuous nudge recovery.** In tight spaces Wall-E backs off and repositions instead of giving up, then keeps making slow, sensor-checked nudges toward the clearest sector until he has room to continue the mission.
- **Temporary, camera-verified clearance reduction for tight spots.** Between chair or table legs the normal LiDAR stop rules can report "too close" even when a real gap wider than the chassis exists. The shared fusion recovery module ([`wall-e/gb_fusion_recovery.py`](wall-e/gb_fusion_recovery.py)) lets a stuck mission (explore or go-to-destination) halve the side margin and squeeze through with slow micro-nudges, but only when the depth camera confirms floor and a clear path, YOLO shows no solid or animate hazard, and the LiDAR points in the way are narrow legs. The gate re-checks every nudge independently, reverse is never relaxed, and the normal clearances are restored automatically once Wall-E is clear.
- **Wake persistence after missions.** After a mission Wall-E now stays awake and listening until he is powered off or put into bench mode (plugged in), instead of dropping back to sleep and needing the wake word again.
- **Two-way Jarvis ↔ Wall-E intercom and a bigger Vision UI.** A half-duplex intercom ([`wall-e/intercom/gb_intercom.py`](wall-e/intercom/gb_intercom.py) on the Pi, [`jarvis/wally-intercom.js`](jarvis/wally-intercom.js) on ALPHA-ONE) sends John's voice to Wall-E's speaker while he talks and switches to Wall-E's onboard mic when he stops (Auto-VAD or push-to-talk, with a noise gate and capped speech boost). With the intercom off, Wall-E returns to normal dual-listen mode. The Jarvis layout now gives the Wall-E Vision panel most of the screen.

## Repository layout

| Path | What it is |
| --- | --- |
| `wall-e/` | Python code running on Wall-E's Raspberry Pi 5 (release `e48d3d8`): voice (`grokbot_voice.py`), explore and missions (`gb_explore.py`), fusion stuck-recovery (`gb_fusion_recovery.py`), obstacle and stop rules (`gb_obstacle.py`), ESTOP, wake gate, mapping, follow, low-voltage, arm and more |
| `wall-e/gb_safety/` | Independent safety gate (`gb_gate.py`), RGB guard, watchdog, clap detector, and their run scripts |
| `wall-e/intercom/` | Two-way intercom daemon |
| `wall-e/systemd/` | systemd units and drop-ins as installed on the Pi (secrets are loaded from files that are not in this repo) |
| `wall-e/tests/` | Unit tests, including the November 27 fusion-recovery and stay-awake tests |
| `jarvis/` | Jarvis local ops console on ALPHA-ONE: `server.js`, `public/index.html`, the `wally-*.js` Wall-E modules (drive, battery, intercom, tool), the orchestrator, and `tools/wally/remote.py` |
| `docs/` | Build log and screenshots |

Not included on purpose: runtime data, logs, audio, model weights (`*.onnx`), `.bak` backups, `__pycache__`, the Pi's `grokbot.env` / `secrets.env`, Jarvis `data/`, the browser profile, and the Roborock helper. Use `wall-e/grokbot.env.example` as the template for local settings.

## Hardware and stack

**Wall-E (Wally).** Hiwonder MentorPi T1 Advanced tracked rover. The host is a Raspberry Pi 5 with 16 GB of RAM. ROS 2 Humble runs in Docker under the name `MentorPi`. The ranging sensor is an LDROBOT STL-19P (D500) DTOF LiDAR.

**ALPHA-ONE.** Salvaged RTX 5080 workstation. Ollama serves `qwen3:14b` at about 45 tok/s, measured November 23, 2026. **Jarvis** (`D:\AI-Models\jarvis`) is the local ops console on that machine. It covers RTX 5080 and Ollama telemetry, Wall-E control and vision, and a panel for a Roborock Qrevo L.

## How the pieces connect

```mermaid
flowchart LR
  GrokBot["Grok Bot<br/>centralized conductor"] --> Jarvis["Jarvis<br/>local ops console"]
  Qwen["Qwen qwen3:14b<br/>Ollama on RTX 5080"] --> Jarvis
  Jarvis <-->|"control and vision"| Wally["Wall-E / Wally<br/>MentorPi T1"]
  Sensors["STL-19P LiDAR<br/>and FPV camera"] --> Wally
  Qrevo["Roborock Qrevo L"] --> Jarvis
```

Grok Bot and Qwen both feed Jarvis. Jarvis and Wall-E exchange commands, the map, and the camera. The LiDAR and the FPV camera feed Wall-E. The Qrevo L reports into its own panel on Jarvis.

## Live from Jarvis

Both frames are from the same session: November 26, 2026, about 7:34 PM PT. Wall-E was armed in ground mode. Ollama was serving `qwen3:14b` on the RTX 5080. The vision column shows the live LiDAR map, the FPV camera, and an `rfdetr_nano` YOLO overlay at 10.9 FPS. The drive pad is the 3×3 diagonal pad, and the speed slider is at the manual cap of `0.288 m/s`.

The overlay in these frames is classifying incorrectly (a chair comes back as `tv`, part of the doorway as `remote`). It is scheduled to move to a stronger model, such as YOLOE or a detector trained on household classes. The same note is printed on the images.

![Full Jarvis dashboard on ALPHA-ONE: Ollama with qwen3:14b, Wall-E status, the Roborock Qrevo L panel, live LiDAR, FPV with a YOLO overlay, a 0.288 m/s speed slider, and the diagonal drive pad.](docs/screenshots/jarvis-dashboard-live.png)

*Full Jarvis dashboard on ALPHA-ONE, November 26, 2026, about 7:34 PM PT. Left: Ollama `qwen3:14b` on the RTX 5080, plus host telemetry. Center: Wall-E arm and mode controls, and the Roborock Qrevo L panel. Right: live LiDAR, FPV, the `rfdetr_nano` overlay at 10.9 FPS, the `0.288 m/s` slider, and the diagonal drive pad.*

![Close-up of the Wall-E Vision column: live LiDAR map, FPV camera, YOLO overlay, 0.288 m/s speed slider, Arm and Disarm, and the 3 by 3 diagonal D-pad.](docs/screenshots/jarvis-vision-and-drive.png)

*Wall-E Vision column from the same capture. Live LiDAR map, FPV camera, the same `rfdetr_nano` overlay at 10.9 FPS, Arm / Disarm, the `0.288 m/s` speed slider, and the 3×3 combined-axis diagonal D-pad.*

## Verified timeline

November 5–7 is the original voice foundation. Everything after that was built live on the Pi and on ALPHA-ONE and was synced into `main` on November 27, 2026.

### November 5–7, 2026 — on GitHub

The repository was created on November 5 ([`69303d5`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/commit/69303d5dadc5b06dbbf47ce239c0098ffd0d471f)). On `main`, that commit is the license and a one-line README.

The voice-controlled rover foundation is also on GitHub, on [`cursor/publish-mentorpi-voice-8581`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/tree/cursor/publish-mentorpi-voice-8581), from [`70fb756`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/commit/70fb756464533bede5ba32156f006615ac101785) (November 5, "Publish the sanitized MentorPi voice assistant tree") through [`bb5bd86`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/commit/bb5bd864ac501750631947b7c1566ae12eaaa1e9) (November 7). It includes:

- `grokbot_voice.py`
- software wake: `wake up Wally`
- identity phrase: `I am John` (the voice code treats this as identity, not as a credential)
- WonderEcho hardware wake: `Hello Hiwonder` (the factory firmware phrase; this tree does not reflash that chip)

The post–November 7 tree now lives on `main` under `wall-e/` and `jarvis/`.

### November 7–23, 2026 — local on ALPHA-ONE and the Pi

- The local Qwen / Ollama stack was checked on the RTX 5080: `qwen3:14b`, about 45 tok/s on November 23.
- Jarvis, the local ops console, was started on ALPHA-ONE (`D:\AI-Models\jarvis`).
- The onboard safety gate was deployed to the Pi at release [`e48d3d8`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/commit/e48d3d8cf7c2e71d18f74fc2a80d54ee43864294) (November 7 on the publish branch). Stop distances are `0.27 m` front, `0.28 m` rear, and `0.24 m` sweep, with a `0.5 s` deadman. The gate uses Nav2-style slow, contact, and escape zones, and it distinguishes bench mode from ground mode.

Those stop distances and the deadman are in the published gate (`STOP_FWD` / `STOP_REV` / `SWEEP_R` in `gb_obstacle.py` on that branch). The speed split and the stall-latch fix below are local to the Pi.

### November 26, 2026 — Jarvis and the Pi

- Live LiDAR viewer on Jarvis.
- FPV plus a YOLO overlay, with click-to-label and auto-reconnect. The running model is `rfdetr_nano` at 10.9 FPS. The labels in the frames above are wrong, and the model is queued for a replacement (YOLOE, or custom household classes).
- A 3×3 combined-axis diagonal D-pad, keyboard combinations for the diagonals, and a fix so the speed slider keeps focus.
- Split speed caps. Manual drive has a hard cap of `0.288 m/s`. Autonomous explore has a hard cap of `1.0 m/s`, starts at a `0.1012 m/s` cruise, and may pick up speed in `0.10 m/s` steps only while the sensors still allow the step. Stop distances are unchanged.
- A stuck `"guard bump detected"` latch was diagnosed and fixed, then checked at 8:00 PM PT. During a LiDAR dropout the gate creeps at `0.04 m/s`. `STALL_MIN_CMD` was `0.03 m/s`, so that creep looked like a stall, and the trip file was never cleared. The fix calls `clear_trip()` on startup, on disarm, and after `2.0 s` idle, and raises `STALL_MIN_CMD` to `0.05 m/s`. The published tree still has `STALL_MIN_CMD = 0.03`.

### November 27, 2026 — synced to `main`

- Silent LiDAR dropout diagnosed as the cause of a manual-drive block ("I held still"): the guard was holding a stale 0.19 m front reading. LiDAR node restarted; fresh scans cleared the guard.
- Shared LiDAR + camera fusion stuck recovery with temporary halved side margin for tight squeezes, used by explore and go-to-destination missions, re-verified by the gate (`gb_fusion_recovery.py`, `gb_explore.py`, `gb_safety/gb_gate.py`).
- Stay-awake after missions until bench mode or power-off (`grokbot_voice.py`).
- Two-way intercom between Jarvis and Wall-E's speaker and mic, and the resized Jarvis layout.

## Limits and safety

Numbers in force on the Pi as of the November 26 evening check. Stop distances and the deadman were already in the November 7 release. The manual / explore speed split and the stall floor are local.

| Limit | Value | Notes |
| --- | --- | --- |
| Front stop | `0.27 m` | From the LiDAR centre, forward |
| Rear stop | `0.28 m` | Reverse |
| Sweep stop | `0.24 m` | Turn clearance |
| Deadman | `0.5 s` | A motion command has to keep refreshing |
| Zones | slow, contact, escape | Nav2-style, inside the gate |
| Mode | bench or ground | Bench is the stand; ground is the floor |
| Manual hard cap | `0.288 m/s` | Jarvis speed slider, November 26 |
| Explore hard cap | `1.0 m/s` | Autonomous explore only |
| Explore cruise | `0.1012 m/s` | Speed explore starts at |
| Pick-up step | `0.10 m/s` | Sensor-bounded; refused when the next step is not clear |
| Dropout creep | `0.04 m/s` | LiDAR-dropout speed that tripped the stall latch |
| `STALL_MIN_CMD` before | `0.03 m/s` | Published tree; the `0.04 m/s` creep counted as a stall |
| `STALL_MIN_CMD` after | `0.05 m/s` | Local fix, verified 8:00 PM PT on November 26 |
| Trip clear | startup, disarm, `2.0 s` idle | `clear_trip()`; the trip file no longer sticks across a stop |

## What's next

- Keep syncing each round of real-world fixes into this repository.
- Replace `rfdetr_nano` with a stronger detector: YOLOE, or a model trained on household classes. The overlay in the November 26 frames mislabels ordinary room objects.
- A Wall-E charging dock beside the vacuum, with QR alignment and a log of battery cycles.
- Return-to-start that uses the LiDAR and the FPV camera together.
- An outdoor autonomous patrol rover: Jetson Orin, GPS/RTK, and a flight controller still to choose between Pixhawk and SpeedyBee.
- Raspberry Pi nodes at radar sites, linked over MeshCore.

## Technical Setup & Reference

Before this page, `main` held only the original README:

```markdown
# mentorpi-voice-assistant
Voice-controlled Hiwonder MentorPi tracked rover with Grok Bot as the onboard AI brain
```

The November 5–7 technical reference — kit notes, setup, the voice-stack architecture diagram, the wake-phrase table, and configuration — is the README on [`cursor/publish-mentorpi-voice-8581`](https://github.com/CNY-Radar-Guy/mentorpi-voice-assistant/blob/cursor/publish-mentorpi-voice-8581/README.md). The post–November 7 Pi tree is in `wall-e/` and the Jarvis console is in `jarvis/` (see `jarvis/README.txt`).
