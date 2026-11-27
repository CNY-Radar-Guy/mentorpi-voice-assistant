"""Run the suite from /tmp with an empty PYTHONPATH while the Pi's absolute paths exist.

The files planted here are decoys. They are removed at the end. A path that
already exists is left untouched.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import glob, json, os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MARK = "HERMETIC-DECOY-NOT-A-ROBOT-RECORD"
OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

def plant(path, text):
    """Write a decoy only at a path that is not this source tree."""
    real = os.path.realpath(path)
    if real.startswith(os.path.realpath(ROOT) + os.sep) or real == os.path.realpath(ROOT):
        return False
    if os.path.exists(path):
        return False
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return True

created = []
try:
    record = json.dumps({
        "scale_m_per_count": 0.001,
        "marker": MARK,
        "provenance": {"tape": "7ft_to_feet", "landmark": "torso", "reference_m": 2.2098,
                       "feet_closer_than_torso_m": 0.0762},
        "note": "decoy so a test that forgets its fixture fails",
    })
    caps = json.dumps({"lidar": False, "camera": False, "speed_pct": 50, "marker": MARK})
    voice = 'reasoning_effort = "high"\n"effort"\n# %s\n' % MARK
    obstacle = "TILT_DEG = 20.0  # %s\n" % MARK
    targets = {
        "/home/pi/docker/tmp/gb_safety/depth_scale_record.json": record,
        "/home/ubuntu/shared/gb_safety/depth_scale_record.json": record,
        "/home/pi/docker/tmp/gb_guard/caps.json": caps,
        "/home/pi/grokbot/data/caps.json": caps,
        "/home/pi/grokbot/grokbot_voice.py": voice,
        "/home/pi/grokbot/gb_obstacle.py": obstacle,
    }
    for path, text in targets.items():
        try:
            if plant(path, text):
                created.append(path)
        except OSError as ex:
            print("could not plant", path, ex)
    depth_decoy = "/home/pi/docker/tmp/gb_safety/depth_scale_record.json"
    check("the canonical depth-record path holds a decoy, not the robot file",
          depth_decoy in created and MARK in open(depth_decoy).read(), created)
    sandbox = tempfile.mkdtemp(prefix="gbhermetic_")
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env["GB_SHARED_DIR"] = sandbox
    env["GB_DATA"] = os.path.join(sandbox, "data")
    env["GB_SHARED_ESTOP"] = os.path.join(sandbox, "ESTOP")
    env["OBS_TILT_DEG"] = "20"
    tilted = subprocess.run([sys.executable, os.path.join(HERE, "test_reexplore.py")],
                            cwd="/tmp", env=env, capture_output=True, text=True)
    check("soft tilt stays 15 when the shell says 20 and /home/pi/grokbot exists",
          tilted.returncode == 0 and "0 failed" in tilted.stdout, tilted.stdout[-400:] + tilted.stderr[-400:])
    env.pop("OBS_TILT_DEG", None)
    failed = []
    for path in sorted(glob.glob(os.path.join(HERE, "test_*.py"))):
        if os.path.basename(path) == "test_hermetic_pi.py":
            continue
        proc = subprocess.run([sys.executable, path], cwd="/tmp", env=env, capture_output=True, text=True)
        line = (proc.stdout.strip().splitlines() or [""])[-1]
        if proc.returncode != 0:
            failed.append((os.path.basename(path), line, proc.stderr[-300:]))
        print("%s %s" % ("OK" if proc.returncode == 0 else "FAIL", line or os.path.basename(path)))
    check("every other test passes with the decoys present and cwd /tmp", not failed, failed)
finally:
    for path in created:
        try:
            raw = open(path).read()
        except OSError:
            continue
        if MARK in raw:
            os.remove(path)
    for directory in ("/home/pi/docker/tmp/gb_safety", "/home/pi/docker/tmp/gb_guard",
                      "/home/pi/docker/tmp", "/home/pi/docker",
                      "/home/ubuntu/shared/gb_safety", "/home/ubuntu/shared"):
        try:
            real = os.path.realpath(directory)
            if real.startswith(os.path.realpath(ROOT) + os.sep):
                continue
            if os.path.isdir(directory) and not os.path.islink(directory) and not os.listdir(directory):
                os.rmdir(directory)
        except OSError:
            pass

print("test_hermetic_pi: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
