# Wally integration status

Not deployed. The phone page on port 8080 is not ready, and `web_video_server` has not been moved. Full readiness stays blocked until the parent runs the reviewed activation on the Pi and the checks below pass there.

## Live mount, 2026-11-06 5:49 PM ET

`docker inspect` `.Mounts` and `/proc/1/mountinfo` both show `/home/pi/docker/tmp` mounted at `/home/ubuntu/shared`, same inode 2130768. The saved `HostConfig.Binds` entry `/home/pi/docker_ros2/tmp` -> `/home/ubuntu/share/tmp` does not exist. `deploy/mount_check.py` now allows a safety copy only from live `.Mounts`. A saved bind string, even one that looks right, is not enough. Optional mountinfo must name `/home/ubuntu/shared` when it is supplied. If both inodes are supplied and they differ, the copy is refused. The apply script no longer reads `HostConfig.Binds`, so a stale saved bind does not restore the old safety files over a good mount.

Running safety code already lives at `container:/home/ubuntu/shared/gb_safety/{gb_gate,gb_watchdog,gb_rgbguard}.py` and `.../gb_guard/gb_guard.py`. The host launchers copy those files when the services start. This archive does not restart them. Shared ESTOP was absent. Status and caps were already correct. Do not clear a stop latch to "fix" that.

## Port 8080

The stock server is started by `ros2 launch bringup bringup.launch.py` inside the container. `GET /snapshot?topic=/ascamera/camera_publisher/rgb0/image` returned HTTP 200, a JPEG, 73327 bytes, 640x400. The stock root page links `stream_viewer` and snapshot for depth, rgb, ir, and hand_trajectory. That server has no proxy and no redirect. Port 8081 was free.

This repository does not contain `bringup.launch.py`. The ROS parameter on `web_video_server` is `port`. No launch argument name is invented here. `deploy/port8080_migrate.sh` prints the remaining requirements and the inverse rollback, then refuses to apply, including when `WALLY_PORT8080_APPLY=1`. Restarting the whole bringup would restart the drivers, so that is not the migration. Until the parent finds a bringup argument that changes only the video-server port, 8080 stays unmet. An overlay on 8081 does not fulfill the phone root. `gb-front` and `gb-stream` stay inactive.

`rclpy` is not installed for host Python. It imports inside the container. New ROS nodes have to use the existing `docker exec` wrapper. Do not start them with host `python3`.

Root was about 88% used with 6.6 GB free. The archive excludes data, models, voices, and wavs. Apply does not delete older `backup-wally-*` directories.

## Feature matrix

| Feature | In this tree | Exercised here | On the robot |
| --- | --- | --- | --- |
| Wake phrase exactly "wake up Wally" | Yes. Hello/Hey/bare Wally rejected in `gb_wally.classify_wake`, even if the env allow-list still contains `wally`. | Unit tests | Not spoken. WonderEcho firmware is still Hello Hiwonder and was not flashed. |
| "I am John" | Not authentication. | Unit test | Not spoken. |
| Emergency stop | Existing latch, phrases, clap, tip, watchdog. Not bypassed by teleop. | Phrase tests and policy tests | Already active on the Pi. This archive does not restart it. |
| Autonomous drive | Default off (`ALLOW_AUTONOMY=0`). Gate fails closed without a fresh validated health file. Joystick is the only teleop source. | Unit tests of the policy | Not accepted. Do not enable it for these checks. |
| LiDAR / depth health speech | Monitor code exists. Depth is not an autonomous range until the unit is `crosscheck_mm`, `crosscheck_m`, or `32FC1_metres` and coverage is at least half the samples. | Synthetic samples | Live sensors were healthy at 10 Hz and 12 Hz. This tree has not announced the phrases on the speaker. |
| User LiDAR off switch | A boolean `lidar: false` in caps.json is kept. Malformed JSON is not rewritten to on. | Unit test | Live caps were both true. This archive does not write caps.json. |
| Object teaching | `gb_wally.handle` calls the teacher. `grokbot_voice.main` installs a capture that decodes `gb_robot.snapshot()` (the existing JPEG URL). | Synthetic images, plus a source check that main installs that capture | Not spoken. A failed snapshot does not save an object. |
| Pointed mission | State machine in `gb_point.py`, reached from `handle`. Ambiguous targets do not drive. | Synthetic ticks | Not a ROS timer and not published to the gate. That loop is not started. |
| Speech cadence | The voice loop calls `gb_cadence.observe`. Piper `tts()` uses `current_scale()`. The xAI TTS body has no speed field, so a cloud voice does not change rate. | Numeric test | Not spoken. |
| 2 m follow | `follow_once` reads the health file and `PERSON_TRACK_FILE`. No file, no range, no motion. One allowed step can go to the gate as `src=wally` for 0.25 s. There is no follow loop. | Unit test with a fresh health file and a lidar track | Not driven. Do not start a long follow. |
| Phone overlay | `gb_stream.py` would listen on 8081. `gb_front.py` would forward `/` there and `/snapshot?topic=` to a snapshot upstream, only when 8080 is free and that upstream already returns a JPEG. | Decision tests | 8080 is unmet. Both units stay inactive. Not viewed on a phone. |
| Safety service wiring | Launchers copy only after live `.Mounts` match. A fast path exists for the activation window. | Stale-bind versus live-mount tests | Live units are already active. Apply does not restart them. |

Unit tests are not acceptance.

## Measurements on this workspace, not the Pi

`python3 grokbot/tests/test_handoff.py` — 56 passed, 0 failed. `python3 grokbot/tests/test_wally.py` — 130 passed, 0 failed. This run's simulated loop medians were idle 0.000097 s, push 0.000099 s, draw 0.000131 s.

A simulated 5 ms loop, 30 iterations, printed median iteration time. Earlier runs:

- idle 0.000086 s, push latest frame 0.000101 s, draw on the control thread 0.000120 s
- idle 0.000134 s, push 0.000114 s, draw 0.000140 s
- later idle 0.000087 s, push 0.000092 s, draw 0.000103 s

Push stayed under the draw cost. That is not speech or motor latency on the Pi, and it is not zero impact.

`test_tilt.py` 74/0 and `test_reexplore.py` 33/0 were run in this workspace before the handoff edits. Phrase tests (`test_drivestop`, `test_natural`, `test_bargein`, `test_relatch`, `test_autoclear`) passed earlier and do not import the wake matcher.

## Apply

From the unpacked archive, on the Pi, as user pi:

`deploy/wally_deploy.sh`

That is a dry run. It does not copy and does not restart anything.

`WALLY_DEPLOY_APPLY=1 deploy/wally_deploy.sh`

Apply copies host modules, writes the backup path to `~/grokbot/.wally-last-code-backup`, and migrates only `WAKE_*` lines in `grokbot.env`. The wake migration records its own backup path in `~/grokbot/.wally-last-wake-backup`. Safety files stay in place when live `.Mounts` show `/home/pi/docker/tmp` -> `/home/ubuntu/shared`. If that check fails, those safety files are put back from the backup just taken. Apply does not restart services, does not play audio, and does not write `secrets.env` or `caps.json`.

## Activation

`deploy/wally_activate.sh` prints the plan and exits. It does not restart anything.

On the Pi, after the apply, and only if you want the safety processes reloaded:

`WALLY_ACTIVATE=1 deploy/wally_activate.sh`

The script publishes zero Twist on `/controller/cmd_vel` before the restart. If `data/ESTOP` is missing it stops grokbot first, so the voice process cannot speak a rising-edge "Emergency stop.", then it creates a latch. An existing latch is not rewritten and is not deleted. It restarts `gb-gate` and `gb-watchdog` only, and exits with an error if both Python processes are not visible within 2 seconds. It does not start grokbot again: the default 15 second auto-clear would release the latch and speak. It does not play a readiness phrase, does not follow, and does not change port 8080.

## Rollback

`deploy/wally_rollback.sh /home/pi/grokbot/backup-wally-<stamp>`

Camera units are disabled first, including when the backup path is missing or the restore fails, and including when more than one `grokbot.env.bak-wake-*` file exists. The script tells you the apply-specific wake backup when `.wally-last-wake-backup` points at a real file, otherwise the newest one. It does not copy that file over `grokbot.env`. It does not stop `web_video_server`, grokbot, or the motion gate, and it does not delete `data/ESTOP`.

Port 8080 inverse rollback, for a move that has not been made, is printed by `deploy/port8080_migrate.sh`. That script refuses to apply.
