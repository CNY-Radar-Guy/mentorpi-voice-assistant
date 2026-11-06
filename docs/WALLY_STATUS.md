# Wally integration status

This file is the acceptance matrix for the tree on this branch. It is not a claim that the robot has been driven or that speech has been heard.

## Software wake

The software phrase is exactly `wake up Wally`. Case and punctuation are ignored. `Hello Wally`, `Hey Wally`, `Hi Wally`, and a bare `Wally` are rejected, including when a leftover allow-list contains `wally` or a fuzzy window is close to the real phrase.

`Hello Hiwonder` is only the WonderEcho Pro factory firmware packet. Nothing in this tree flashes that device. A passing unit test is not spoken acceptance on the robot.

`deploy/migrate_wake_env.py` rewrites only `WAKE_WORDS`, `WAKE_ANYWHERE`, and `WAKE_PROMPT` in an existing env file, after writing a timestamped backup. It leaves `WAKE_ACK` and every other line, including keys, alone. It refuses `secrets.env`.

## Autonomous health

The motion gate treats only `src == "joystick"` as teleop. Every other velocity source is autonomous. Autonomous motion is refused when the health file is missing, older than 3 seconds, malformed, or lacks a fresh physical LiDAR sample (valid fraction at least 0.50). Depth is an autonomous range only when `depth_unit` is `crosscheck_mm`, `crosscheck_m`, or `32FC1_metres` and coverage meets the same fraction. A user capability default of "on", or a missing `caps.json`, does not by itself allow autonomy.

A saved `caps.json` with boolean `lidar: false` stays off across `sync()`. Malformed JSON is not rewritten into `lidar: true`. Fault strings are not stored.

## Camera overlay

`gb_robot.py` still reads snapshots from `http://127.0.0.1:8080/snapshot`. That port belongs to the stock snapshot server. `gb_stream.py` does not bind 8080 when something is already listening, does not kill that process, and exits 0 so the unit (`Restart=no`) does not loop. The overlay default is `192.168.50.125:8081` in `gb-stream.service`. The page is read-only. There is no token in the URL. Anyone on the LAN who can open the port can see the camera; do not forward it on the router.

The stream keeps one latest frame, draws a person box on a copy, and only when that box timestamp matches the frame. It does not open a camera device and does not run a second detector. The box fields are added to the existing `gb_rgbguard.py` inference output.

## What unit tests covered

`python3 tests/test_wally.py` — 130 passed, 0 failed, on this workspace, without a robot, microphone, or network API.

A simulated 5 ms loop, 30 iterations, on this workspace (not the Pi) printed median iteration time. Two runs:

- idle 0.000086 s, push latest frame 0.000101 s, draw on the control thread 0.000120 s
- idle 0.000134 s, push 0.000114 s, draw 0.000140 s

Push stayed under the draw cost in both runs. The second run shows push inside the noise of an empty loop, so the difference is not a stable microsecond budget. This is not speech or motor latency on the Pi, and it is not zero impact.

Also re-run in this workspace earlier in the same build: `test_tilt.py` 74/0, `test_reexplore.py` 33/0, `test_obstacle.py` 144/0, and the estop phrase tests (`test_drivestop`, `test_natural`, `test_bargein`, `test_relatch`, `test_autoclear`) passed. Those phrase tests were not repeated after the wake-grammar edit because they do not import the wake matcher.

## Not tested on the robot

Spoken "wake up Wally", WonderEcho "Hello Hiwonder", Grok API reachability, depth-unit cross-check against a real Aurora frame, the overlay in a phone browser, CPU on the Pi 5, and any track motion. Tracks-off-ground motion is not required for the caps or wake changes. The parent runs any live check.
