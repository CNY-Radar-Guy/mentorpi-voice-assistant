"""gb_mapping.py - start/save/stop Hiwonder's LiDAR SLAM (slam_toolbox) inside the MentorPi container.

Hiwonder's own mapping launch is  ros2 launch slam slam.launch.py  = include/robot.launch.py (controller, lidar,
depth camera, joystick, init_pose - ALREADY running from 'ros2 launch bringup bringup.launch.py' at container start)
+ include/slam_base.launch.py (slam_toolbox sync_slam_toolbox_node, config/slam.yaml, scan_raw).
Running slam.launch.py again would start duplicate drivers (two ros_robot_controller on /dev/ttyACM0), so we launch
only slam_base.launch.py. Maps are saved like Hiwonder's slam/map_save.py does (nav2_map_server map_saver_cli),
but into the shared folder (host /home/pi/docker/tmp = container /home/ubuntu/shared) and copied onto the SD card at
~/grokbot/data/maps/<name>.yaml/.pgm. Mapping itself never drives the robot.
"""
import os, re, shutil, subprocess, time
import gb_store
from gb_store import atomic_write

E = os.environ.get
CONTAINER = E("ROS_CONTAINER", "MentorPi")
HOST_SHARED = os.path.expanduser(E("ROS_SHARED_HOST", "/home/pi/docker/tmp"))
CONT_SHARED = E("ROS_SHARED_CONTAINER", "/home/ubuntu/shared")
SLAM_LAUNCH = E("SLAM_LAUNCH", "/home/ubuntu/ros2_ws/src/slam/launch/include/slam_base.launch.py")
HOST_DIR = os.path.join(HOST_SHARED, "grokbot"); CONT_DIR = CONT_SHARED + "/grokbot"
MAPS = os.path.join(gb_store.DATA, "maps")
LOG = os.path.join(gb_store.DATA, "mapping.log")
PIDF_HOST = os.path.join(HOST_DIR, "mapping.pid")       # PID inside the container namespace
ROSRUN = """#!/bin/bash
# run a command inside the MentorPi container with the exact ROS environment of the bringup launch
P=$(pgrep -f "ros2 launch bringup" | head -1)
while IFS= read -r -d "" l; do export "$l"; done < /proc/$P/environ
export PATH=/opt/ros/humble/bin:/usr/local/bin:/usr/bin:/bin
exec "$@"
"""

def _setup():
    os.makedirs(os.path.join(HOST_DIR, "maps"), exist_ok=True); os.makedirs(MAPS, exist_ok=True)
    p = os.path.join(HOST_DIR, "rosrun.sh")
    if not os.path.exists(p) or open(p).read() != ROSRUN:
        atomic_write(p, ROSRUN)
    os.chmod(p, 0o755)

def _dx(args, timeout=20, user="1000"):
    return subprocess.run(["docker", "exec", "-u", user, CONTAINER] + args, capture_output=True, text=True, timeout=timeout)

def _log(msg):
    with open(LOG, "a") as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + msg + "\n"); f.flush(); os.fsync(f.fileno())

def running():
    """container PID of our mapping launch, or of any slam_toolbox node, else None"""
    try:
        if os.path.exists(PIDF_HOST):
            pid = open(PIDF_HOST).read().strip()
            if pid and _dx(["kill", "-0", pid], 5).returncode == 0:
                return pid
        r = _dx(["pgrep", "-f", "slam_toolbox_node"], 5)
        return r.stdout.split()[0] if r.returncode == 0 and r.stdout.split() else None
    except Exception:
        return None

def start():
    _setup()
    pid = running()
    if pid:
        return False, "Mapping is already running."
    cmd = (f'echo $$ > {CONT_DIR}/mapping.pid; exec {CONT_DIR}/rosrun.sh ros2 launch {SLAM_LAUNCH} '
           f'enable_save:=true use_sim_time:=false')
    logf = open(os.path.join(gb_store.DATA, "mapping_ros.log"), "ab")
    subprocess.Popen(["docker", "exec", "-u", "1000", CONTAINER, "bash", "-c", cmd], stdout=logf, stderr=logf,
                     stdin=subprocess.DEVNULL, start_new_session=True)
    for _ in range(40):                                   # wait for slam_toolbox to come up (max ~20 s)
        time.sleep(0.5)
        if _dx(["pgrep", "-f", "sync_slam_toolbox_node"], 5).returncode == 0:
            pid = open(PIDF_HOST).read().strip() if os.path.exists(PIDF_HOST) else "?"
            _log(f"started slam_toolbox (container pid {pid})"); gb_store.Store().event("mapping_start", pid)
            return True, ("Mapping started. I won't drive myself, so steer me around slowly with the app or "
                          "gamepad, then say save the map.")
    _log("start failed: slam_toolbox did not appear")
    stop()
    return False, "I couldn't start mapping, slam toolbox didn't come up. Check the mapping log."

def stop():
    pid = running()
    if not pid:
        return False, "Mapping isn't running."
    _dx(["kill", "-INT", pid], 5)                         # ros2 launch shuts its nodes down on SIGINT
    for _ in range(20):
        time.sleep(0.5)
        if _dx(["pgrep", "-f", "slam_toolbox_node"], 5).returncode != 0:
            break
    else:
        _dx(["pkill", "-TERM", "-f", "slam_toolbox_node"], 5); time.sleep(1)
        _dx(["pkill", "-KILL", "-f", "slam_toolbox_node"], 5)
    _dx(["pkill", "-INT", "-f", "slam_base.launch.py"], 5)
    try: os.remove(PIDF_HOST)
    except OSError: pass
    ok = _dx(["pgrep", "-f", "slam_toolbox_node"], 5).returncode != 0
    _log(f"stopped mapping (pid {pid}) ok={ok}")
    return ok, "Mapping stopped." if ok else "I tried to stop mapping but slam toolbox is still running."

def clean_name(name):
    n = re.sub(r"[^a-z0-9_-]+", "_", (name or "").lower()).strip("_")
    return n or time.strftime("map_%Y%m%d_%H%M%S")

def save(name=None, stop_after=True, store=None):
    """map_saver_cli -> shared dir -> copy to ~/grokbot/data/maps. Returns (ok, message, info)."""
    _setup()
    if not running():
        return False, "Mapping isn't running, so there's no map to save. Say start mapping first.", None
    name = clean_name(name)
    if os.path.exists(os.path.join(MAPS, name + ".yaml")):
        name += time.strftime("_%H%M%S")
    target = f"{CONT_DIR}/maps/{name}"
    r = _dx([f"{CONT_DIR}/rosrun.sh", "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", target,
             "--ros-args", "-p", "map_subscribe_transient_local:=true", "-p", "save_map_timeout:=10.0"], timeout=40)
    src_y, src_p = os.path.join(HOST_DIR, "maps", name + ".yaml"), os.path.join(HOST_DIR, "maps", name + ".pgm")
    if not (os.path.exists(src_y) and os.path.exists(src_p) and os.path.getsize(src_p) > 0):
        _log(f"map_saver_cli failed rc={r.returncode}: {(r.stdout + r.stderr)[-300:]}")
        try:                                              # fallback: grab /map over rosbridge
            import gb_robot
            stem, res, w, h, origin = gb_robot.save_map(name, MAPS)
            if store is not None:
                store.add_map(name, stem + ".yaml", stem + ".pgm", res, w, h, origin)
            _log(f"saved via rosbridge fallback {stem}")
            msg = f"Map saved as {name.replace('_', ' ')}, {w} by {h} cells, on my SD card."
            if stop_after:
                ok, m = stop(); msg += " Mapping stopped." if ok else " " + m
            return True, msg, {"name": name, "yaml": stem + ".yaml", "pgm": stem + ".pgm", "width": w, "height": h}
        except Exception as ex:
            _log(f"rosbridge fallback failed: {ex}")
        return False, "Saving the map failed, no map has been received yet. Drive a little and try again.", None
    dst_p, dst_y = os.path.join(MAPS, name + ".pgm"), os.path.join(MAPS, name + ".yaml")
    atomic_write(dst_p, open(src_p, "rb").read())
    y = open(src_y).read()
    y = re.sub(r"^image:.*$", f"image: {name}.pgm", y, flags=re.M)   # relative path so the pair is portable
    atomic_write(dst_y, y)
    meta = dict(re.findall(r"^(\w+):\s*(.+)$", y, re.M))
    w = h = None
    with open(dst_p, "rb") as f:                          # PGM header: P5 <w> <h> <max>
        hdr = f.read(200).split(b"\n")
        dims = [l for l in hdr[1:] if l and not l.startswith(b"#")][0].split()
        w, h = int(dims[0]), int(dims[1])
    info = {"name": name, "yaml": dst_y, "pgm": dst_p, "width": w, "height": h,
            "resolution": float(meta.get("resolution", 0.05)), "origin": meta.get("origin", "")}
    if store is not None:
        store.add_map(name, dst_y, dst_p, info["resolution"], w, h, info["origin"])
    _log(f"saved {dst_y} {dst_p} {w}x{h}")
    msg = f"Map saved as {name.replace('_', ' ')}, {w} by {h} cells, on my SD card."
    if stop_after:
        ok, m = stop(); msg += " Mapping stopped." if ok else " " + m
    return True, msg, info
