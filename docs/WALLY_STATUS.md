# Wally integration status

Not deployed. The phone page on port 8080 is not ready, and `web_video_server` has not been moved. Full readiness stays blocked until the parent runs the reviewed activation on the Pi and the checks below pass there.

## Live mount, 2026-11-06 5:49 PM ET

`docker inspect` `.Mounts` and `/proc/1/mountinfo` both show `/home/pi/docker/tmp` mounted at `/home/ubuntu/shared`, same inode 2130768. The saved `HostConfig.Binds` entry `/home/pi/docker_ros2/tmp` -> `/home/ubuntu/share/tmp` does not exist. `deploy/mount_check.py` now allows a safety copy only from live `.Mounts`. A saved bind string, even one that looks right, is not enough. Optional mountinfo must name `/home/ubuntu/shared` when it is supplied. If both inodes are supplied and they differ, the copy is refused. The apply script no longer reads `HostConfig.Binds`, so a stale saved bind does not restore the old safety files over a good mount.

Running safety code already lives at `container:/home/ubuntu/shared/gb_safety/{gb_gate,gb_watchdog,gb_rgbguard}.py` and `.../gb_guard/gb_guard.py`. The host launchers copy those files when the services start. This archive does not restart them. Shared ESTOP was absent. Status and caps were already correct. Do not clear a stop latch to "fix" that.

## Port 8080

The stock server is started by `ros2 launch bringup bringup.launch.py` with no arguments and no parameters: `Node(package='web_video_server', executable='web_video_server', output='screen')`. Package 1.0.0 reads `port` once on the private node `_web_video_server` and uses 8080 when that parameter is unset. There is no parameter callback. `ros2 param set` does not move the bind. A second process would also bind 8080, so this tree does not start one. `GET /snapshot?topic=/ascamera/camera_publisher/rgb0/image` returned HTTP 200, a JPEG, 73327 bytes, 640x400. Snapshots stay on that URL. The phone page on 8080 is unmet. `gb-front` and `gb-stream` stay inactive. An overlay on 8081 does not fulfill the phone root. Read-only probes are `ros2 param list /web_video_server` and that snapshot GET.

`rclpy` is not installed for host Python. It imports inside the container. New ROS nodes have to use the existing `docker exec` wrapper. Do not start them with host `python3`.

Root was about 88% used with 6.6 GB free. The archive excludes data, models, voices, and wavs. Apply does not delete older `backup-wally-*` directories.

## Feature matrix

| Feature | In this tree | Exercised here | On the robot |
| --- | --- | --- | --- |
| Wake phrase exactly "wake up Wally" | Yes. Hello/Hey/bare Wally rejected in `gb_wally.classify_wake`, even if the env allow-list still contains `wally`. | Unit tests | Not spoken. WonderEcho firmware is still Hello Hiwonder and was not flashed. |
| "I am John" | Not authentication. | Unit test | Not spoken. |
| Normal stop | Voice or manual stop stays until resume/unlock or a new fresh teleop command. The timer defaults to 0. A latch that predates the process is kept across restart. Stop words in the same utterance block the unlock. Negated, quoted, hypothetical, stale, replayed, and model text do not unlock. | `test_policy.py` | Live `ESTOP_AUTOCLEAR_S` is still 15 until the parent runs the env migration. This archive does not edit the live file. |
| Safety fault | Tilt, battery, watchdog, and the joystick button write `kind: safety`. A drive command and resume do not clear that file. | Policy test | Not restarted here. |
| Maintenance lock | `data/MAINT`, also mirrored to the shared mount. Gate and watchdog zero every source, including the joystick. Resume, a move, and the 15 s timer do not clear it. | Policy test and source check | Not created on the Pi by this archive. |
| Autonomous drive | Default off (`ALLOW_AUTONOMY=0`). Gate fails closed without a fresh validated health file. Joystick is the only teleop source. | Unit tests of the policy | Not accepted. Do not enable it for these checks. |
| LiDAR / depth health speech | Monitor code exists. Depth is not an autonomous range until the unit is `crosscheck_mm`, `crosscheck_m`, or `32FC1_metres` and coverage is at least half the samples. | Synthetic samples | Live sensors were healthy at 10 Hz and 12 Hz. This tree has not announced the phrases on the speaker. |
| User LiDAR off switch | A boolean `lidar: false` in caps.json is kept. Malformed JSON is not rewritten to on. | Unit test | Live caps were both true. This archive does not write caps.json. |
| Object teaching | `gb_wally.handle` calls the teacher. `grokbot_voice.main` installs a capture that decodes `gb_robot.snapshot()` (the existing JPEG URL). | Synthetic images, plus a source check that main installs that capture | Not spoken. A failed snapshot does not save an object. |
| Pointed mission | `PointMission` is ticked by the voice runtime while a mission is active, and a non-zero step is sent to the gate. A target needs one fresh measured depth, confidence at least 0.55, and no model caption. Map coordinates are kept only when the observation already has them. Return without a pose fix stops. The Nav2 action client is not opened. | Synthetic ticks and `ground_observation` | Not driven. No live candidate file was attached. |
| Speech cadence | The voice loop calls `gb_cadence.observe`. Piper `tts()` uses `current_scale()`. The xAI TTS body has no speed field, so a cloud voice does not change rate. | Numeric test | Not spoken. |
| 2 m follow | `FollowLoop` stays active and is stepped every 0.2 s by the voice runtime. It cancels on loss, low confidence, ambiguity, stale depth, timeout, estop, and the maintenance lock. It does not invent a range. | Loop tests | Not driven. Nothing in this tree writes a metric `PERSON_TRACK_FILE` yet. |
| Phone overlay | `gb_stream.py` would listen on 8081. `gb_front.py` would forward `/` there and `/snapshot?topic=` to a snapshot upstream, only when 8080 is free and that upstream already returns a JPEG. | Decision tests | 8080 is unmet. Both units stay inactive. Not viewed on a phone. |
| Safety service wiring | Launchers copy only after live `.Mounts` match. A fast path exists for the activation window. | Stale-bind versus live-mount tests | Live units are already active. Apply does not restart them. |

Unit tests are not acceptance.

## Measurements on this workspace, not the Pi

`python3 grokbot/tests/test_policy.py` — 39 passed, 0 failed. `python3 grokbot/tests/test_handoff.py` — 57 passed, 0 failed. `python3 grokbot/tests/test_wally.py` — 130 passed, 0 failed. `python3 grokbot/tests/test_autoclear.py` — 18 passed, 0 failed. This run's simulated loop medians were idle 0.000090 s, push 0.000097 s, draw 0.000104 s.

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

Apply copies host modules, writes the backup path to `~/grokbot/.wally-last-code-backup`, migrates only `WAKE_*` lines, and sets `ESTOP_AUTOCLEAR_S=0` with its own backup. It does not change `WAKE_ACK` or `STARTUP_GREETING`. The live file has both of those empty. Safety files stay in place when live `.Mounts` show `/home/pi/docker/tmp` -> `/home/ubuntu/shared`. If that check fails, those safety files are put back from the backup just taken. Apply does not restart services, does not play audio, and does not write `secrets.env` or `caps.json`.

## Activation

`deploy/wally_activate.sh` prints the plan and exits. It does not restart anything.

On the Pi, after the apply, and only if you want the safety processes reloaded:

`WALLY_ACTIVATE=1 deploy/wally_activate.sh`

The script writes `data/MAINT` and the shared copy, publishes zero Twist, then restarts `gb-gate` and `gb-watchdog` only. It leaves `grokbot` running so wake and the spoken stop listener stay up. It does not create or delete `data/ESTOP`. It exits with an error if both Python processes are not visible within 2 seconds. The maintenance lock stays until `WALLY_MAINT_CLEAR=1 deploy/maint_clear.sh`. A drive command and resume do not clear it. Empty `STARTUP_GREETING` is not a suppress switch; a later voice reload needs `SUPPRESS_STARTUP_SPEECH=1`. It does not play a readiness phrase, does not follow, and does not change port 8080.

## Rollback

`deploy/wally_rollback.sh /home/pi/grokbot/backup-wally-<stamp>`

Camera units are disabled first, including when the backup path is missing or the restore fails, and including when more than one `grokbot.env.bak-wake-*` file exists. The script tells you the apply-specific wake backup when `.wally-last-wake-backup` points at a real file, otherwise the newest one. It does not copy that file over `grokbot.env`. It does not stop `web_video_server`, grokbot, or the motion gate, and it does not delete `data/ESTOP`.

Port 8080 inverse rollback, for a move that has not been made, is printed by `deploy/port8080_migrate.sh`. That script refuses to apply.
