"""Preflight and mount decisions from the 2026-11-06 17:44 ET sample. No robot, no Docker."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(ROOT))
import gb_front
from deploy.mount_check import safety_copy_allowed

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
check("empty snapshot is not a reason to take the port", "not stopped" in d["reason"] or "left running" in d["reason"])

# The reported bind is not the path the launchers assume.
reported = ["/home/pi/docker_ros2/tmp:/home/ubuntu/share/tmp:rw"]
ok, problems, mapping = safety_copy_allowed(reported)
check("share/tmp is not shared", ok is False and mapping.get("/home/ubuntu/share/tmp") == "/home/pi/docker_ros2/tmp", problems)
check("assumed shared path is absent", any("/home/ubuntu/shared" in p for p in problems))

ok2, problems2, _ = safety_copy_allowed(["/home/pi/docker/tmp:/home/ubuntu/shared:rw"])
check("exact assumed bind would allow a copy", ok2 is True and problems2 == [])

text = open(os.path.join(os.path.dirname(ROOT), "deploy", "wally_rollback.sh")).read()
check("rollback does not stop web_video_server", "web_video" in text and "was not stopped" in text)
check("rollback does not kill docker", "docker kill" not in text and "docker rm" not in text)

print("test_handoff: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
