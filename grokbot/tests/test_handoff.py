"""Mount, rollback, 8080, and activation decisions. No robot and no Docker."""
import json, os, subprocess, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, REPO)
import gb_front
import gb_health
import gb_wally
from deploy.mount_check import safety_copy_allowed, decide_from_inspect
from deploy.wake_backup import select_wake_backup
from deploy.activate_plan import voice_restart_allowed, write_transition_latch, within_window, PLAN
from deploy.port8080_plan import migration_decision

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

# web_video_server binds 0.0.0.0:8080. Empty /snapshot without a topic is irrelevant.
d = gb_front.preflight_decision(True, snapshot_without_topic_empty=True, upstream_snapshot_jpeg=False)
check("8080 owned blocks the phone URL", d["state"] == "blocked" and d["phone_url_8080"] is False and d["unmet_8080"] is True)
check("stream stays inactive", d["enable_stream"] is False and d["enable_front"] is False)
check("empty snapshot is not a reason to take the port", "left running" in d["reason"])

# The 17:44 saved bind is not live evidence. The 17:49 .Mounts entry is.
stale = ["/home/pi/docker_ros2/tmp:/home/ubuntu/share/tmp:rw"]
ok, problems, mapping = safety_copy_allowed(stale)
check("stale HostConfig.Binds alone do not allow a copy", ok is False and mapping == {}, problems)
check("saved binds are named as not live", any("HostConfig.Binds" in p for p in problems), problems)

# A string that happens to match the expected pair is still not .Mounts.
ok_str, _, _ = safety_copy_allowed(["/home/pi/docker/tmp:/home/ubuntu/shared:rw"])
check("a matching bind string is still not live evidence", ok_str is False)

live = {
    "HostConfig": {"Binds": ["/home/pi/docker_ros2/tmp:/home/ubuntu/share/tmp:rw"]},
    "Mounts": [{"Type": "bind", "Source": "/home/pi/docker/tmp", "Destination": "/home/ubuntu/shared"}],
}
info = "849 831 8:1 /docker/tmp /home/ubuntu/shared rw,relatime - ext4 /dev/root rw\n"
ok_live, problems_live, mapping_live = safety_copy_allowed(live, mountinfo=info, inodes=("2130768", "2130768"))
check("live Mounts allow the expected pair despite stale binds", ok_live is True and problems_live == [], problems_live)
check("mapping comes from Mounts", mapping_live.get("/home/ubuntu/shared") == "/home/pi/docker/tmp")
decision = decide_from_inspect(live, mountinfo=info)
check("stale bind is reported and ignored", decision["safety_modules"] == "allowed" and any("docker_ros2" in s for s in decision["ignored_stale_binds"]))

ok_bad_info, problems_info, _ = safety_copy_allowed(live, mountinfo="123 45 8:1 / / rw - ext4 /dev/root rw\n")
check("mountinfo that omits the destination blocks", ok_bad_info is False and any("mountinfo" in p for p in problems_info), problems_info)
ok_empty_info, _, _ = safety_copy_allowed(live, mountinfo="")
check("empty mountinfo is optional", ok_empty_info is True)
ok_inode, _, _ = safety_copy_allowed(live, inodes=("2130768", "999"))
check("inode mismatch blocks", ok_inode is False)
ok_one_inode, _, _ = safety_copy_allowed(live, inodes=("2130768", ""))
check("one-sided inode does not block", ok_one_inode is True)

wrong = {"Mounts": [{"Source": "/home/pi/docker_ros2/tmp", "Destination": "/home/ubuntu/share/tmp"}]}
ok_wrong, _, mapping_wrong = safety_copy_allowed(wrong)
check("wrong live mount blocks", ok_wrong is False and "/home/ubuntu/shared" not in mapping_wrong)

alias = {"Mounts": [{"Source": "/home/pi/docker_ros2/tmp", "Destination": "/home/ubuntu/shared"}]}
rp = {"/home/pi/docker/tmp": "/canon/docker-tmp", "/home/pi/docker_ros2/tmp": "/canon/docker-tmp"}
ok_rp, _, _ = safety_copy_allowed(alias, realpaths=rp)
check("verified realpath equivalence allows", ok_rp is True)
ok_half, _, _ = safety_copy_allowed(alias, realpaths={"/home/pi/docker/tmp": "/canon/docker-tmp"})
check("one-sided realpath does not allow", ok_half is False)

# Rollback script: camera units are disabled even when several wake backups exist.
rollback = open(os.path.join(REPO, "deploy", "wally_rollback.sh")).read()
check("rollback does not stop web_video_server", "web_video" in rollback and "was not stopped" in rollback)
check("rollback does not kill docker", "docker kill" not in rollback and "docker rm" not in rollback)
check("rollback does not glob-test wake backups", 'grokbot.env.bak-wake-"*' not in rollback)
check("rollback does not use errexit", "\nset -e" not in rollback and "set -euo" not in rollback)
check("rollback leaves the stop latch", "data/ESTOP was not removed" in rollback)
check("cameras are disabled before the restore", rollback.find("disable_cameras\n") < rollback.find("tar -C \"$BACK\""))

tmp = tempfile.mkdtemp(prefix="wally_rollback_")
back = os.path.join(tmp, "backup-wally")
dest = os.path.join(tmp, "grokbot")
os.makedirs(back)
os.makedirs(dest)
with open(os.path.join(back, "marker.txt"), "w") as f:
    f.write("restored\n")
older = os.path.join(dest, "grokbot.env.bak-wake-20261106-010000")
newer = os.path.join(dest, "grokbot.env.bak-wake-20261106-020000")
with open(older, "w") as f:
    f.write("old\n")
with open(newer, "w") as f:
    f.write("new\n")
os.utime(older, (1_000, 1_000))
os.utime(newer, (2_000, 2_000))
chosen, why = select_wake_backup(dest, None)
check("newest of several wake backups", chosen == newer and why == "newest", chosen)
check("no wake backup", select_wake_backup(tempfile.mkdtemp(), None)[1] == "no-match")
pointer = os.path.join(tmp, "specific.env")
with open(pointer, "w") as f:
    f.write("specific\n")
chosen_p, why_p = select_wake_backup(dest, pointer + "\n")
check("apply-specific pointer wins over newer files", chosen_p == pointer and why_p == "apply-specific")

stub = os.path.join(tmp, "bin")
os.makedirs(stub)
with open(os.path.join(stub, "sudo"), "w") as f:
    f.write("#!/bin/bash\nexec \"$@\"\n")
with open(os.path.join(stub, "systemctl"), "w") as f:
    f.write("#!/bin/bash\nprintf '%s\\n' \"$*\" >> \"$STUB_LOG\"\nexit 0\n")
os.chmod(os.path.join(stub, "sudo"), 0o755)
os.chmod(os.path.join(stub, "systemctl"), 0o755)
log = os.path.join(tmp, "systemctl.log")
env = os.environ.copy()
env["PATH"] = stub + os.pathsep + env.get("PATH", "")
env["WALLY_DEST"] = dest
env["STUB_LOG"] = log
proc = subprocess.run(["bash", os.path.join(REPO, "deploy", "wally_rollback.sh"), back],
                      capture_output=True, text=True, env=env)
check("rollback exits 0 with several wake backups", proc.returncode == 0, proc.stderr + proc.stdout)
check("rollback disabled camera units", "disable --now gb-front gb-stream" in open(log).read() if os.path.exists(log) else "", proc.stdout)
check("rollback restored the code", open(os.path.join(dest, "marker.txt")).read() == "restored\n" if os.path.exists(os.path.join(dest, "marker.txt")) else False)
check("rollback named the newest wake backup", newer in proc.stdout, proc.stdout)
missing = subprocess.run(["bash", os.path.join(REPO, "deploy", "wally_rollback.sh")],
                         capture_output=True, text=True, env=env)
check("missing backup still disables cameras", missing.returncode == 2 and "disabled" in missing.stdout, missing.stdout + missing.stderr)

# Activation does not clear a latch, and a restart that would speak or auto-clear is refused.
check("suppress flag allows a silent reload", voice_restart_allowed("SUPPRESS_STARTUP_SPEECH=1\nESTOP_AUTOCLEAR_S=0\n", False)[0] is True)
check("empty greeting is not the suppress switch", voice_restart_allowed("STARTUP_GREETING=\n", False)[0] is False)
check("warmup alone is not the suppress switch", voice_restart_allowed("TTS_WARMUP=0\n", False)[0] is False)
check("health speech alone is not the suppress switch", voice_restart_allowed("HEALTH_SPEAK=0\n", False)[0] is False)
check("autonomy on blocks a voice reload", voice_restart_allowed("SUPPRESS_STARTUP_SPEECH=1\nALLOW_AUTONOMY=1\n", False)[0] is False)
check("timer still set blocks a voice reload", voice_restart_allowed("SUPPRESS_STARTUP_SPEECH=1\nESTOP_AUTOCLEAR_S=15\n", True)[0] is False)
check("two second window", within_window(2.0) and not within_window(2.01))
check("plan holds zero before restart", "zero Twist" in PLAN[2] and "within 2 seconds" in PLAN[3])
check("plan leaves the voice process running", any("Leave grokbot running" in line for line in PLAN))
check("plan does not delete a user ESTOP", any("Do not create or delete data/ESTOP" in line for line in PLAN))

latch_dir = tempfile.mkdtemp(prefix="wally_estop_")
keep = os.path.join(latch_dir, "ESTOP")
with open(keep, "w") as f:
    f.write("already")
fresh = os.path.join(latch_dir, "new", "ESTOP")
created = write_transition_latch([keep, fresh], now=time.time())
check("existing latch is not rewritten", created == [fresh] and open(keep).read() == "already", created)
check("new latch is activation source", json.load(open(fresh))["src"] == "activation")
again = write_transition_latch([keep, fresh])
check("second call creates nothing and deletes nothing", again == [] and os.path.exists(keep) and os.path.exists(fresh))

# 8080 stays unmet even if a reviewer later has every input.
moved = migration_decision(True, True, True)
check("reviewed inputs still do not claim the server moved", moved["server_moved"] is False and moved["phone_ready"] is False and moved["unmet_8080"] is True)
check("overlay is not the phone root", moved["overlay_8081_fulfills_8080"] is False)
check("inverse rollback does not kill the container", any("docker-kill" in line for line in moved["inverse_rollback"]))
check("bringup launch is the named starter", moved["launch"] == "ros2 launch bringup bringup.launch.py")
check("snapshot topic is the confirmed one", "/ascamera/camera_publisher/rgb0/image" in moved["snapshot_url"])

# Follow uses health and a measured track. No track, no publisher call.
now = time.time()
health_path = os.path.join(tempfile.mkdtemp(prefix="wally_health_"), "health.json")
gb_health.SHARED = health_path
with open(health_path, "w") as f:
    json.dump({"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.96,
               "user_lidar": True, "user_camera": True}, f)
gb_wally.set_autonomy(True)
calls = []
gb_wally.install_runtime(publisher=lambda cmd: calls.append(cmd))
said = gb_wally.handle("follow me")
check("follow without a person range does not publish", calls == [] and "measured distance" in (said or ""), said)
track = os.path.join(tempfile.mkdtemp(prefix="wally_track_"), "person.json")
with open(track, "w") as f:
    json.dump({"person": {"t": now, "depth_m": 3.2, "source": "lidar", "bearing": 0.0, "conf": 0.9}}, f)
os.environ["PERSON_TRACK_FILE"] = track
try:
    said2 = gb_wally.handle("follow me")
finally:
    os.environ.pop("PERSON_TRACK_FILE", None)
    gb_wally._runtime = False
    gb_wally._publisher = None
    gb_wally.set_autonomy(False)
again = gb_wally.follow_loop().step(now + 0.2, gb_health.read_shared(now=now + 0.2),
                                   gb_wally.read_person_track(track, now=now + 0.2), True)
check("follow loop keeps a second step", again.get("allow") is True and gb_wally.follow_loop().active, again)
check("fresh lidar track publishes the first step", len(calls) == 1 and calls[0]["allow"] is True and said2 and "2 metres" in said2, (calls, said2))
stale_track = gb_wally.read_person_track(track, now=now + 5)
check("stale person track is ignored", stale_track is None)
depth_bad = os.path.join(tempfile.mkdtemp(), "bad.json")
with open(depth_bad, "w") as f:
    json.dump({"person": {"t": now, "depth_m": 3.0, "source": "depth", "unit": "driver"}}, f)
check("unvalidated depth unit is not a follow range", gb_wally.read_person_track(depth_bad, now=now) is None)

voice = open(os.path.join(ROOT, "grokbot_voice.py")).read()
check("voice installs the snapshot capture", "_install_wally_runtime()" in voice and "gb_robot.snapshot()" in voice)
check("voice follow publish is not joystick", '"src": "wally"' in voice)
launchers = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read() + open(os.path.join(ROOT, "gb_guard_run.sh")).read()
check("launchers do not inspect HostConfig.Binds", "{{json .HostConfig.Binds}}" not in launchers)
check("launchers read live Mounts", ".Mounts" in launchers)

print("test_handoff: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
