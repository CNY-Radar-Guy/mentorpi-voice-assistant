# Wally integration status

`490d29a` activation succeeded at 18:42 ET. Maintenance is still held. Autonomy is off. The live timer is 0 and startup speech is suppressed. Port 8081 was stopped at 18:47 ET and is not started by this archive. Stock port 8080 was measured at 48 JPEGs in 4 seconds, 12 fps. The depth unit is not confirmed.

The previous health check compared a whole-frame median (about 1.689 m if the counts were millimetres) with one LiDAR value (0.98 m) and failed the 30 percent test. This tree compares the center of the depth image with the forward LiDAR cone. A camera offset is used only when a fresh transform supplies it. The drawing's 8 cm / 4.5 cm figure is not applied. Aurora's 150–4000 filter and `16UC1` suggest millimetres and are not treated as a unit. `depth_metric_valid` stays false until that forward-ray check agrees. `depth_topic_online` is separate and does not authorize motion.

At 18:50 ET the host was about 58 percent busy (load 6.7–7.4, not throttled). The largest shares of one core were odom about 70 percent, Aurora 43, rosbridge 37, voice 27, and the gate 21. With maintenance held, the gate and the watchdog each publish a zero Twist at about 10 Hz on `/controller/cmd_vel`, and the watchdog also publishes a zero `set_motor`. Voice `zero_now` is a 10-message burst only when a stop is latched, not a standing publisher. The 10 Hz inhibit was not slowed: the controller's hold and a competing command were not revalidated, and maintenance was not cleared, so a no-lock baseline was not measured. No extra diagnostic node is added.

The live wake line is `wake up wally`, `WAKE_ANYWHERE` is empty, and `WAKE_FUZZ=1.0`. A blank `WAKE_FUZZ` crashed voice import; the migration now writes `1.0` and a blank value parses as an exact match. The next activation overwrites those three fields only. `WAKE_ACK` stays `Hello John, are you ready to rock and roll?`. A WonderEcho serial event does not open a session. The chip is not reflashed. NavigateToPose is not running and `hand_landmarker.task` is not installed.

A wake does not create a stop or a maintenance file, and it does not clear one that is already there. Only a deliberate unquoted "emergency stop", and a quiet-gate clap when that service is explicitly enabled, write a persistent emergency lock. "stop", "halt", "sit", and "hold up" cancel the current motion and do not write that lock. A hard fault is unchanged. After a successful activation the script removes only the deploy interlock it just created. It does not arm a bench hold and it does not enable `gb-clap`. An expired clap test window does not become a live latch on restart. The quiet gate still waits 0.40 s. A fixture is not an acoustic proof.

Apply only stages `~/grokbot/releases/<id>/`. Activation restarts the watchdog first, while the previous gate stays up, and continues to the gate only after `watchdog_state.json` says this release is holding maintenance and sending zeros. A failure restarts only the unit that just moved. Zero Twist is a supplement; it is not the inhibit for both safety processes. The previous gate and watchdog do not understand `MAINT` and are not asked to. New code is copied to `/home/pi/docker/tmp/gb_code/<id>/` (`/home/ubuntu/shared/gb_code/<id>/` in the container). `/home/ubuntu/shared/gb_safety/` stays the shared state directory and its existing Python files are not replaced. `gb_estop.clear` never removes a `kind: safety` file or `MAINT`.

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
| Speech cadence | The voice loop calls `gb_cadence.observe`. Piper `tts()` uses `current_scale()`. The xAI TTS body has no speed field, so a cloud voice does not change rate. | Numeric test | Not spoken. |
| 2 m follow | `FollowLoop` steps every 0.2 s. A follow request writes `demand.json` first, so `gb_rgbguard` subscribes before the gate is moving and writes `person_track.json` from the same YOLOv8n pass plus depth. Mono16 is used only when the health file has a cross-checked unit. "I am John" does not select a person. | Geometry tests | Not driven. The container has not been restarted onto this code. |
| Pointing | Wrist-to-index geometry is in `gb_metric.py`. The hand landmarker is loaded only when MediaPipe and the documented task file are present. Otherwise the point file says unavailable and no target is invented. | Geometry test and the missing-model result | The model is not installed by this archive. `deploy/container_ready.sh` prints the install commands. |
| Pointed mission | A goal is sent only through `NavigateToPose` (`gb_nav.py`) after a measured map pose. Ticks publish zero velocity. No server, no map pose, or a lost fix cancels. Old status files are not reloaded as motion. | Scripted action client | The live Nav2 server was not called. |
| Health publisher | `gb_health_node.py` and `gb-health.service` write `/home/ubuntu/shared/gb_safety/health.json` using `resolve_depth_scale`. The unit is not enabled by apply. | Record test | Not started on the Pi. |
| Safety service wiring | Launchers copy only after live `.Mounts` match. A fast path exists for the activation window. | Stale-bind versus live-mount tests | Live units are already active. Apply does not restart them. |

Unit tests are not acceptance.

## Measurements on this workspace, not the Pi

`python3 grokbot/tests/test_safety_seq.py` — 24 passed, 0 failed. `python3 grokbot/tests/test_release.py` — 25 passed, 0 failed. `python3 grokbot/tests/test_policy.py` — 39 passed, 0 failed. `python3 grokbot/tests/test_wally.py` — 114 passed, 0 failed. `python3 grokbot/tests/test_handoff.py` — 57 passed, 0 failed. `python3 grokbot/tests/test_autoclear.py` — 18 passed, 0 failed. `python3 grokbot/tests/test_stream_jpeg.py` — 13 passed, 0 failed. This run's simulated loop medians were idle 0.000092 s, push 0.000101 s, draw 0.000104 s.

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

Apply copies the tree to `~/grokbot/releases/<id>/` and leaves the live `*.py` files in place. It does not edit `grokbot.env`, does not change pins, and does not restart services.

`WALLY_ACTIVATE=1 WALLY_RELEASE=<id> deploy/wally_activate.sh`

Activation writes the maintenance lock, publishes zero Twist as a supplement, and restarts `gb-watchdog` alone. The previous gate keeps running. The new watchdog must write `watchdog_state.json` with this release id, `maint` true, and `zeroing` true before `gb-gate` is restarted. If the watchdog does not, only `gb-watchdog` is returned to its previous unit. If the gate does not stay up, only `gb-gate` is returned and the new watchdog is left zeroing. Voice is next, with `SUPPRESS_STARTUP_SPEECH=1` and `ESTOP_AUTOCLEAR_S=0`. `gb-rgbguard` and `gb-health` follow, one at a time, from that release's code directory, so a view demand can run the same YOLOv8n detector without a motor command. A failure of either restarts only that unit. `data/ESTOP` and the maintenance lock are not deleted. The previous gate is not asked to read `MAINT`.

## Activation

`deploy/wally_activate.sh` prints the plan and exits. It does not restart anything.

On the Pi, after the apply, and only if you want the safety processes reloaded:

`WALLY_ACTIVATE=1 deploy/wally_activate.sh`

The script writes `data/MAINT` and the shared copy, publishes zero Twist, then restarts `gb-watchdog` and, only after its maintenance ack, `gb-gate`. Those two restarts are separate commands. Voice, `gb-rgbguard`, and `gb-health` follow one at a time. It does not create or delete `data/ESTOP`. A normal stop can be cleared by resume or a fresh move. A hard fault cannot, including when `clear(..., only_normal=False)` is passed. The maintenance lock stays until `WALLY_MAINT_CLEAR=1 deploy/maint_clear.sh`. Empty `STARTUP_GREETING` is not a suppress switch. It does not play a readiness phrase, does not follow, and does not change port 8080. Until this command is run on the Pi, the live timer remains 15 and the live wake process remains the previous one.

## Rollback

`deploy/wally_rollback.sh /home/pi/grokbot/backup-wally-<stamp>`

Camera units are disabled first, including when the backup path is missing or the restore fails, and including when more than one `grokbot.env.bak-wake-*` file exists. The script tells you the apply-specific wake backup when `.wally-last-wake-backup` points at a real file, otherwise the newest one. It does not copy that file over `grokbot.env`. It does not stop `web_video_server`, grokbot, or the motion gate, and it does not delete `data/ESTOP`.

Port 8080 inverse rollback, for a move that has not been made, is printed by `deploy/port8080_migrate.sh`. That script refuses to apply.
