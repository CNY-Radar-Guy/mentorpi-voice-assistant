# Wally integration status

Not ready to test on the robot. The 2026-11-06 17:44 ET preflight is a constraint on this handoff. Nothing in that preflight was changed on the Pi.

Port 8080 is owned by stock `web_video_server` (pid 4830, user pi, inside the MentorPi container, host network, `0.0.0.0:8080`). HEAD `/` returned HTML 200. A GET of `/snapshot` with no `topic` query was empty. That does not mean the endpoint is missing: `gb_robot.snapshot()` calls `http://127.0.0.1:8080/snapshot?topic=` plus the camera topic and requires JPEG bytes (`FFD8`). No TCP clients in that sample does not mean nothing uses the port. This tree does not stop that process and does not move it. The phone URL `http://192.168.50.125:8080/` is unmet. `gb-stream` and `gb-front` stay inactive (`enable_stream` false, `Restart=no`). Port 8081 was free. It is only a low-priority overlay, and only if a later preflight finds 8080 free and a JPEG snapshot upstream already answering. Until then the overlay is not started.

`rclpy` is not installed for host Python. It imports inside the container. New ROS nodes have to use the existing `docker exec` wrapper and the rclpy wait in `gb_safety_run.sh`. Do not start them with host `python3`.

The bind list in that preflight showed `/home/pi/docker_ros2/tmp` mounted at `/home/ubuntu/share/tmp` (`share`, not `shared`). The safety launchers still name `/home/pi/docker/tmp` and `/home/ubuntu/shared`. Those strings are not a mount. `deploy/mount_check.py` refuses a safety-module copy unless that exact pair is in `docker inspect`. `wally_deploy.sh` then puts the previous launchers and injected nodes back from the backup it just took. It does not restart services and does not delete older `backup-wally-*` directories. Root was about 88% used with 6.6 GB free. The archive excludes data, models, voices, and wavs so it does not copy that bulk.

## Feature matrix

| Feature | In this tree | Exercised here | On the robot |
| --- | --- | --- | --- |
| Wake phrase exactly "wake up Wally" | Yes. Hello/Hey/bare Wally rejected in `gb_wally.classify_wake`, even if the env allow-list still contains `wally`. | Unit tests | Not spoken. WonderEcho firmware is still Hello Hiwonder and was not flashed. |
| "I am John" | Not authentication. | Unit test | Not spoken. |
| Emergency stop | Existing latch, phrases, clap, tip, watchdog. Not bypassed by teleop. | Phrase tests and policy tests | Already active on the Pi. This archive does not restart it. |
| Autonomous drive | Default off (`ALLOW_AUTONOMY=0`). Gate fails closed without a fresh validated health file. Joystick is the only teleop source. | Unit tests of the policy | Not accepted. Do not enable it for these checks. |
| LiDAR / depth health speech | Monitor code exists. Depth is not an autonomous range until the unit is `crosscheck_mm`, `crosscheck_m`, or `32FC1_metres` and coverage is at least half the samples. | Synthetic samples | Live sensors were healthy at 10 Hz and 12 Hz. This tree has not announced the phrases on the speaker. |
| User LiDAR off switch | A boolean `lidar: false` in caps.json is kept. Malformed JSON is not rewritten to on. | Unit test | Live caps were both true. This archive does not write caps.json. |
| Object teaching | Multi-turn SQLite and a perceptual hash in `gb_teach.py`, reached from `gb_wally.handle` after the exact wake. | Synthetic images | No camera capture and no spoken confirmation. |
| Pointed mission | State machine in `gb_point.py`. Ambiguous targets do not drive. Goals are not reloaded as motion. Nav2 is a goal dict plus an optional transport, not a proven plan on this map. | Synthetic ticks | No motion, no localization, no person track. |
| Speech cadence | `gb_cadence.py` maps words-per-second to a Piper length scale. | Numeric test | Piper was not run. |
| 2 m follow | `gb_follow.py` speed command. Stops inside 1.5 m. No range means no motion. | Synthetic ranges | Not wired to a live track and not driven. |
| Phone overlay | `gb_stream.py` would listen on 8081. `gb_front.py` would forward `/` there and `/snapshot?topic=` to a snapshot upstream, only when 8080 is free and that upstream already returns a JPEG. | Decision tests and a simulated loop | 8080 is unmet. Both units stay inactive. Not viewed on a phone. |
| Safety service wiring | Launchers wait for container `rclpy`, then refuse to copy unless the assumed bind is actually present. | The mount checker rejects the reported bind list. | Live units are already active. Do not restart them from this handoff. |

Unit tests are not acceptance.

## Measurements on this workspace, not the Pi

`python3 tests/test_wally.py` — 130 passed, 0 failed, before this handoff note. Re-run after these edits and record the new count in the commit message if it changes.

A simulated 5 ms loop, 30 iterations, printed median iteration time. Two earlier runs:

- idle 0.000086 s, push latest frame 0.000101 s, draw on the control thread 0.000120 s
- idle 0.000134 s, push 0.000114 s, draw 0.000140 s
- later idle 0.000087 s, push 0.000092 s, draw 0.000103 s

Push stayed under the draw cost. The second run puts push inside the noise of an empty loop. That is not speech or motor latency on the Pi, and it is not zero impact.

`test_tilt.py` 74/0 and `test_reexplore.py` 33/0 were run in this workspace before the handoff edits. Phrase tests (`test_drivestop`, `test_natural`, `test_bargein`, `test_relatch`, `test_autoclear`) passed earlier and do not import the wake matcher.

## Apply

From the unpacked archive, on the Pi, after it has been copied to a staging directory:

`WALLY_DEPLOY_APPLY=1 deploy/wally_deploy.sh`

Dry-run is the default. Apply copies host modules, backs up the previous code without data/models/voices, migrates only `WAKE_*` lines in `grokbot.env`, and restores safety launchers from that backup when the mount check fails. It does not restart services, does not play audio, and does not write `secrets.env` or `caps.json`.

Rollback, given the backup directory the apply printed:

`deploy/wally_rollback.sh /home/pi/grokbot/backup-wally-<stamp>`

That restores the backed-up code and disables `gb-front` and `gb-stream` only. It does not stop `web_video_server`.
