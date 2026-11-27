"""gb_mapping.py - start/save/stop Hiwonder's LiDAR SLAM (slam_toolbox) inside the MentorPi container.

Hiwonder's own mapping launch is  ros2 launch slam slam.launch.py  = include/robot.launch.py (controller, lidar,
depth camera, joystick, init_pose - ALREADY running from 'ros2 launch bringup bringup.launch.py' at container start)
+ include/slam_base.launch.py (slam_toolbox sync_slam_toolbox_node, config/slam.yaml, scan_raw).
Running slam.launch.py again would start duplicate drivers (two ros_robot_controller on /dev/ttyACM0), so we launch
only slam_base.launch.py. Maps are saved like Hiwonder's slam/map_save.py does (nav2_map_server map_saver_cli),
but into the shared folder (host /home/pi/docker/tmp = container /home/ubuntu/shared) and copied onto the SD card at
~/grokbot/data/maps/<name>.yaml/.pgm. Mapping itself never drives the robot.

2026-11-26 continuous densification: auto/exploration saves merge into one growing master pose-graph
(~/grokbot/data/maps/master.posegraph + .data, plus master.yaml/.pgm for the live viewer). start() reloads
that master graph via /slam_toolbox/deserialize_map so new LiDAR scans densify the same map. Custom-named
maps are preserved separately and never auto-purged.
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

# Continuous densification: one growing master map across exploration runs.
MASTER_MAP_NAME = E("MASTER_MAP_NAME", "master")
# slam_toolbox/srv/SerializePoseGraph and DeserializePoseGraph (ROS 2 Humble):
#   SerializePoseGraph: string filename  ---  uint8 result (RESULT_SUCCESS=0, RESULT_FAILED_TO_WRITE_FILE=255)
#   DeserializePoseGraph: string filename, int8 match_type, geometry_msgs/Pose2D initial_pose
#     UNSET=0, START_AT_FIRST_NODE=1, START_AT_GIVEN_POSE=2, LOCALIZE_AT_POSE=3
SERIALIZE_SRV = "/slam_toolbox/serialize_map"
SERIALIZE_TYPE = "slam_toolbox/srv/SerializePoseGraph"
DESERIALIZE_SRV = "/slam_toolbox/deserialize_map"
DESERIALIZE_TYPE = "slam_toolbox/srv/DeserializePoseGraph"
MATCH_UNSET = 0
MATCH_START_AT_FIRST_NODE = 1
MATCH_START_AT_GIVEN_POSE = 2
MATCH_LOCALIZE_AT_POSE = 3
RESULT_SUCCESS = 0
RESULT_FAILED_TO_WRITE_FILE = 255
MAP_EXTS = (".yaml", ".pgm", ".posegraph", ".data")
GRID_EXTS = (".yaml", ".pgm")
GRAPH_EXTS = (".posegraph", ".data")
PGM_UNKNOWN = 205
PGM_FREE = 254
PGM_OCCUPIED = 0

def _setup():
    os.makedirs(os.path.join(HOST_DIR, "maps"), exist_ok=True); os.makedirs(MAPS, exist_ok=True)
    p = os.path.join(HOST_DIR, "rosrun.sh")
    if not os.path.exists(p) or open(p).read() != ROSRUN:
        atomic_write(p, ROSRUN)
    os.chmod(p, 0o755)

def _dx(args, timeout=20, user="1000"):
    return subprocess.run(["docker", "exec", "-u", user, CONTAINER] + args, capture_output=True, text=True, timeout=timeout)

def _ros2_call(args, timeout=20):
    """Run a ros2 CLI command inside MentorPi with the bringup environment (via rosrun.sh)."""
    return _dx([f"{CONT_DIR}/rosrun.sh"] + list(args), timeout=timeout)

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

def start(autonomous=False):
    """autonomous=True: an explore mission will drive itself (2026-11-23 fix 3: honest message)."""
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
            loaded, detail = load_master_graph()
            if autonomous:
                base = "Mapping started. I'll drive myself slowly and stop for anything in my way."
            else:
                base = ("Mapping started. I won't drive myself, so steer me around slowly with the app or "
                        "gamepad, then say save the map.")
            if loaded:
                return True, base + " Loaded the existing master map so new scans will densify it."
            return True, base + " " + detail
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

# 2026-11-17 map rotation: when the SD card is low or full, overwrite (remove) the OLDEST
# auto-named maps (explore_YYYYMMDD_HHMMSS / map_YYYYMMDD_HHMMSS) so a new save never fails
# for space. Maps you named yourself are never removed. Only runs as part of a save.
MAP_MIN_FREE_MB = float(E("MAP_MIN_FREE_MB", "200"))
MAP_MAX_KEEP = int(E("MAP_MAX_KEEP", "5"))
_AUTO_MAP_RX = re.compile(r"^(?:explore|map)_\d{8}_\d{6}(?:_\d{6})?$")
def _backup_name_rx(master=None):
    return re.compile(r"^" + re.escape(master or MASTER_MAP_NAME) + r"_bak_\d{8}_\d{6}(?:_\d+)?$")
def _free_mb(path):
    st = os.statvfs(path); return st.f_bavail * st.f_frsize / 1e6
def _map_exts():
    return MAP_EXTS
def _is_protected_stem(stem, keep=()):
    if stem in keep or stem == MASTER_MAP_NAME:
        return True
    if _backup_name_rx().match(stem):
        return True
    return False
def _auto_maps(keep=()):
    stems = {}
    for fn in os.listdir(MAPS):
        stem, ext = os.path.splitext(fn)
        if ext in GRID_EXTS + GRAPH_EXTS and _AUTO_MAP_RX.match(stem) and stem not in keep:
            stems[stem] = max(stems.get(stem, 0), os.path.getmtime(os.path.join(MAPS, fn)))
    return [s for s, _ in sorted(stems.items(), key=lambda kv: kv[1])]
def _remove_stem(stem, dirs=None):
    """Remove yaml/pgm/posegraph/data for stem from MAPS and the container-shared maps dir."""
    removed_any = False
    for d in (dirs or (MAPS, os.path.join(HOST_DIR, "maps"))):
        for ext in MAP_EXTS:
            p = os.path.join(d, stem + ext)
            try:
                if os.path.exists(p):
                    os.remove(p); removed_any = True
            except OSError as ex:
                _log(f"could not remove {p}: {ex}")
    return removed_any
def make_room(keep=(), force=False, need_mb=None):
    """Remove oldest auto-named maps until free space >= need_mb. force=True removes at least one (disk-full retry)."""
    need = MAP_MIN_FREE_MB if need_mb is None else need_mb
    removed = []
    for stem in _auto_maps(keep):
        if not force and _free_mb(MAPS) >= need:
            break
        _remove_stem(stem)
        removed.append(stem); force = False
        _log(f"map rotation: removed oldest map {stem} (free {_free_mb(MAPS):.0f} MB, want {need:.0f} MB)")
    if force or _free_mb(MAPS) < need:
        # If still tight, drop oldest master rollback backups (never the live master itself).
        for stem in list_master_backups():  # oldest first
            if stem in keep:
                continue
            if not force and _free_mb(MAPS) >= need:
                break
            _remove_stem(stem)
            removed.append(stem); force = False
            _log(f"map rotation: removed oldest master backup {stem} (free {_free_mb(MAPS):.0f} MB)")
    return removed
def purge_old_maps(max_keep=None, keep=()):
    """Keep only the `max_keep` most recent auto-named exploration maps; delete all older ones.

    The live master map and all custom-named maps are never purged. Timestamped master
    rollback backups are rotated separately by purge_master_backups().
    """
    limit = MAP_MAX_KEEP if max_keep is None else int(max_keep)
    if limit < 1:
        limit = 1
    stems = {}
    if os.path.isdir(MAPS):
        for fn in os.listdir(MAPS):
            stem, ext = os.path.splitext(fn)
            if ext in GRID_EXTS + GRAPH_EXTS and _AUTO_MAP_RX.match(stem):
                stems[stem] = max(stems.get(stem, 0.0), os.path.getmtime(os.path.join(MAPS, fn)))
    # Sort newest first; keep `limit` newest plus any explicitly protected `keep` stems
    ordered = [s for s, _ in sorted(stems.items(), key=lambda kv: (kv[1], kv[0]), reverse=True)]
    keep_set = set(keep)
    kept = []
    for s in ordered:
        if s in keep_set or len(kept) < limit:
            kept.append(s)
    kept_set = set(kept)
    purged = []
    for s in ordered:
        if s in kept_set:
            continue
        _remove_stem(s)
        purged.append(s)
        _log(f"map auto-purge: removed old exploration map {s} (policy: keep latest {limit})")
    # Also bound master rollback backups; never touch master itself or custom names.
    purged.extend(purge_master_backups(max_keep=limit))
    return purged

def list_master_backups():
    """Return master rollback stems oldest-first."""
    rx = _backup_name_rx()
    stems = {}
    if not os.path.isdir(MAPS):
        return []
    for fn in os.listdir(MAPS):
        stem, ext = os.path.splitext(fn)
        if ext in MAP_EXTS and rx.match(stem):
            stems[stem] = max(stems.get(stem, 0.0), os.path.getmtime(os.path.join(MAPS, fn)))
    return [s for s, _ in sorted(stems.items(), key=lambda kv: (kv[1], kv[0]))]

def purge_master_backups(max_keep=None):
    """Keep only the newest `max_keep` timestamped master rollback backups."""
    limit = MAP_MAX_KEEP if max_keep is None else int(max_keep)
    if limit < 1:
        limit = 1
    ordered = list_master_backups()  # oldest first
    if len(ordered) <= limit:
        return []
    drop = ordered[:len(ordered) - limit]
    for stem in drop:
        _remove_stem(stem)
        _log(f"master backup rotation: removed {stem} (policy: keep latest {limit})")
    return drop

def _write_room(path, data, keep, removed):
    import errno
    for _ in range(4):
        try:
            atomic_write(path, data); return
        except OSError as ex:
            if ex.errno != errno.ENOSPC: raise
            got = make_room(keep=keep, force=True)
            if not got: raise
            removed.extend(got)
    atomic_write(path, data)

# ---------------------------------------------------------------------------
# Master pose-graph densification helpers
# ---------------------------------------------------------------------------
def resolve_save_name(name):
    """Return (disk_name, is_custom). Auto/explore/empty names densify into the master map."""
    cleaned = clean_name(name)
    if cleaned == MASTER_MAP_NAME or _AUTO_MAP_RX.match(cleaned):
        return MASTER_MAP_NAME, False
    return cleaned, True

def master_graph_exists(directory=None):
    d = directory or MAPS
    return (os.path.exists(os.path.join(d, MASTER_MAP_NAME + ".posegraph"))
            and os.path.exists(os.path.join(d, MASTER_MAP_NAME + ".data")))

def _copy_exts(stem, src_dir, dst_dir, exts, keep=(), removed=None):
    os.makedirs(dst_dir, exist_ok=True)
    copied = []
    for ext in exts:
        src = os.path.join(src_dir, stem + ext)
        if not os.path.exists(src):
            continue
        dst = os.path.join(dst_dir, stem + ext)
        data = open(src, "rb").read()
        if removed is None:
            atomic_write(dst, data)
        else:
            _write_room(dst, data, keep, removed)
        copied.append(dst)
    return copied

def stage_master_graph():
    """Copy persistent master.posegraph/.data from MAPS into the container-shared folder."""
    _setup()
    if not master_graph_exists(MAPS):
        return False
    os.makedirs(os.path.join(HOST_DIR, "maps"), exist_ok=True)
    for ext in GRAPH_EXTS:
        src = os.path.join(MAPS, MASTER_MAP_NAME + ext)
        dst = os.path.join(HOST_DIR, "maps", MASTER_MAP_NAME + ext)
        atomic_write(dst, open(src, "rb").read())
    return True

def _svc_ok(r, expect_result=False):
    if r is None:
        return False
    out = ((r.stdout or "") + (r.stderr or ""))
    low = out.lower()
    if r.returncode != 0:
        return False
    if "not available" in low or "does not exist" in low or "failed to write" in low:
        return False
    if expect_result:
        if re.search(r"result\s*[=:]\s*255\b", out):
            return False
        if re.search(r"result\s*[=:]\s*0\b", out):
            return True
        # Humble may print RESULT_SUCCESS; treat rc=0 without a 255 as success.
    return True

def _wait_for_service(name, timeout_s=15.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            r = _ros2_call(["ros2", "service", "list"], timeout=10)
        except Exception as ex:
            _log(f"service list failed while waiting for {name}: {ex}")
            r = None
        if r is not None and r.returncode == 0 and name in (r.stdout or ""):
            return True
        time.sleep(0.5)
    return False

def serialize_map_graph(stem):
    """Call /slam_toolbox/serialize_map and persist stem.posegraph + stem.data to MAPS.

    filename is the path with no extension, as required by SerializePoseGraph.srv.
    Returns (ok, detail). Never raises.
    """
    try:
        _setup()
        os.makedirs(os.path.join(HOST_DIR, "maps"), exist_ok=True)
        cont_stem = f"{CONT_DIR}/maps/{stem}"
        if not _wait_for_service(SERIALIZE_SRV, timeout_s=8.0):
            _log(f"serialize_map: {SERIALIZE_SRV} not listed yet, trying anyway")
        payload = '{filename: "%s"}' % cont_stem
        r = _ros2_call(["ros2", "service", "call", SERIALIZE_SRV, SERIALIZE_TYPE, payload], timeout=60)
        out = ((r.stdout or "") + (r.stderr or "")) if r else ""
        # slam_toolbox may take a moment to flush the two files
        host_dir = os.path.join(HOST_DIR, "maps")
        deadline = time.time() + 8
        while time.time() < deadline:
            if all(os.path.exists(os.path.join(host_dir, stem + ext)) and
                   os.path.getsize(os.path.join(host_dir, stem + ext)) > 0
                   for ext in GRAPH_EXTS):
                break
            time.sleep(0.2)
        if not _svc_ok(r, expect_result=True) and not all(
                os.path.exists(os.path.join(host_dir, stem + ext)) for ext in GRAPH_EXTS):
            _log(f"serialize_map failed rc={getattr(r, 'returncode', None)}: {out[-400:]}")
            return False, out[-400:]
        _copy_exts(stem, host_dir, MAPS, GRAPH_EXTS)
        _log(f"serialize_map wrote {stem}.posegraph + {stem}.data")
        return True, "serialized"
    except Exception as ex:
        _log(f"serialize_map error: {ex}")
        return False, str(ex)

def load_master_graph():
    """If a persistent master pose-graph exists, stage it and deserialize into slam_toolbox.

    Uses match_type=START_AT_FIRST_NODE (1). On failure, logs a warning and leaves
    slam_toolbox running with a fresh graph. Returns (loaded: bool, detail: str).
    """
    if not master_graph_exists(MAPS):
        _log("no prior master pose-graph; starting a fresh master map")
        return False, "Started a fresh master map."
    try:
        if not stage_master_graph():
            _log("warning: could not stage master pose-graph; continuing with a fresh graph")
            return False, "Started a fresh master map (could not stage the prior graph)."
        if not _wait_for_service(DESERIALIZE_SRV, timeout_s=15.0):
            _log(f"warning: {DESERIALIZE_SRV} did not appear; continuing with a fresh graph")
            return False, "Started a fresh master map (deserialize service was not ready)."
        cont_stem = f"{CONT_DIR}/maps/{MASTER_MAP_NAME}"
        # DeserializePoseGraph.srv: filename, match_type=1 (START_AT_FIRST_NODE), initial_pose unused.
        payload = ('{filename: "%s", match_type: %d, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}'
                   % (cont_stem, MATCH_START_AT_FIRST_NODE))
        r = _ros2_call(["ros2", "service", "call", DESERIALIZE_SRV, DESERIALIZE_TYPE, payload], timeout=60)
        out = ((r.stdout or "") + (r.stderr or "")) if r else ""
        if not _svc_ok(r, expect_result=False):
            _log(f"warning: deserialize_map failed, continuing with a fresh graph: {out[-400:]}")
            return False, "Started a fresh master map (the prior graph could not be loaded)."
        _log(f"loaded prior master pose-graph via {DESERIALIZE_SRV} "
             f"(filename={cont_stem}, match_type={MATCH_START_AT_FIRST_NODE} START_AT_FIRST_NODE)")
        return True, "Loaded the existing master map."
    except Exception as ex:
        _log(f"warning: deserialize_map error, continuing with a fresh graph: {ex}")
        return False, "Started a fresh master map (the prior graph could not be loaded)."

def backup_master():
    """Copy current master.* to a timestamped rollback set, then rotate to MAP_MAX_KEEP."""
    if not os.path.isdir(MAPS):
        return None
    present = [ext for ext in MAP_EXTS if os.path.exists(os.path.join(MAPS, MASTER_MAP_NAME + ext))]
    if not present:
        return None
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bak = f"{MASTER_MAP_NAME}_bak_{stamp}"
    n = 2
    while any(os.path.exists(os.path.join(MAPS, bak + ext)) for ext in MAP_EXTS):
        bak = f"{MASTER_MAP_NAME}_bak_{stamp}_{n}"; n += 1
    for ext in present:
        src = os.path.join(MAPS, MASTER_MAP_NAME + ext)
        dst = os.path.join(MAPS, bak + ext)
        try:
            atomic_write(dst, open(src, "rb").read())
        except OSError as ex:
            _log(f"master backup could not write {dst}: {ex}")
        # Mirror into the shared folder too, so a rollback is visible alongside the live map.
        host_dst = os.path.join(HOST_DIR, "maps", bak + ext)
        try:
            os.makedirs(os.path.join(HOST_DIR, "maps"), exist_ok=True)
            atomic_write(host_dst, open(src, "rb").read())
        except OSError:
            pass
    _log(f"backed up master map to {bak} before overwrite")
    purge_master_backups()
    return bak

# ---------------------------------------------------------------------------
# Occupancy-grid densify (rosbridge fallback when serialize_map isn't available)
# ---------------------------------------------------------------------------
def _parse_map_yaml(text):
    meta = dict(re.findall(r"^(\w+):\s*(.+)$", text, re.M))
    res = float(meta.get("resolution", 0.05))
    origin_raw = meta.get("origin", "[0.0, 0.0, 0.0]")
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", origin_raw)
    origin = [float(nums[0]) if len(nums) > 0 else 0.0,
              float(nums[1]) if len(nums) > 1 else 0.0,
              float(nums[2]) if len(nums) > 2 else 0.0]
    return meta, res, origin

def _read_pgm(path):
    with open(path, "rb") as f:
        raw = f.read()
    if not raw.startswith(b"P5"):
        raise ValueError(f"{path} is not a binary PGM (P5)")
    # Skip magic, comments, then w h, then maxval, then one newline and the pixels.
    i = 0
    parts = []
    n = len(raw)
    while i < n and len(parts) < 4:
        if raw[i:i+1] in (b" ", b"\t", b"\r", b"\n"):
            i += 1; continue
        if raw[i:i+1] == b"#":
            nl = raw.find(b"\n", i)
            i = n if nl < 0 else nl + 1
            continue
        j = i
        while j < n and raw[j:j+1] not in (b" ", b"\t", b"\r", b"\n"):
            j += 1
        parts.append(raw[i:j]); i = j
    if len(parts) < 4:
        raise ValueError(f"{path} has a truncated PGM header")
    w, h, maxval = int(parts[1]), int(parts[2]), int(parts[3])
    if i < n and raw[i:i+1] in (b" ", b"\t", b"\r", b"\n"):
        # consume the single whitespace that terminates maxval
        i += 1
    pixels = raw[i:i + w * h]
    if len(pixels) < w * h:
        pixels = pixels + bytes([PGM_UNKNOWN]) * (w * h - len(pixels))
    return w, h, maxval, pixels

def _write_pgm_bytes(w, h, pixels):
    return b"P5\n%d %d\n255\n" % (w, h) + bytes(pixels)

def _pgm_is_unknown(v):
    return v == PGM_UNKNOWN

def _pgm_is_occupied(v):
    # ROS map_saver: 0 = occupied; also treat very dark cells as occupied.
    return v <= 50

def merge_cell(old, new):
    """Densify one occupancy cell: keep known values when the new scan is unknown;
    occupied is a stronger observation than free; otherwise prefer the new scan."""
    if _pgm_is_unknown(new):
        return old
    if _pgm_is_unknown(old):
        return new
    if _pgm_is_occupied(old) or _pgm_is_occupied(new):
        return PGM_OCCUPIED if (_pgm_is_occupied(old) or _pgm_is_occupied(new)) else new
    return new

def _format_map_yaml(image_name, res, origin, meta=None):
    mode = (meta or {}).get("mode", "trinary")
    negate = (meta or {}).get("negate", "0")
    occ = (meta or {}).get("occupied_thresh", "0.65")
    free = (meta or {}).get("free_thresh", "0.25")
    ox, oy, oyaw = origin[0], origin[1], origin[2] if len(origin) > 2 else 0.0
    return (f"image: {image_name}\nmode: {mode}\nresolution: {res}\n"
            f"origin: [{ox}, {oy}, {oyaw}]\nnegate: {negate}\n"
            f"occupied_thresh: {occ}\nfree_thresh: {free}\n")

def merge_occupancy_maps(old_yaml, old_pgm, new_yaml, new_pgm, image_name=None):
    """Align two ROS map_saver grids by origin/resolution and densify occupancy cells.

    If resolution is incompatible, the new map is returned unchanged (and a note is logged).
    Returns dict with yaml text, pgm bytes, width, height, resolution, origin, merged (bool).
    """
    image_name = image_name or (MASTER_MAP_NAME + ".pgm")
    ny_meta, nres, norigin = _parse_map_yaml(open(new_yaml).read())
    nw, nh, _, npix = _read_pgm(new_pgm)
    if not (os.path.exists(old_yaml) and os.path.exists(old_pgm)):
        yaml_text = _format_map_yaml(image_name, nres, norigin, ny_meta)
        return {"yaml": yaml_text, "pgm": _write_pgm_bytes(nw, nh, npix),
                "width": nw, "height": nh, "resolution": nres, "origin": norigin, "merged": False}
    oy_meta, ores, oorigin = _parse_map_yaml(open(old_yaml).read())
    ow, oh, _, opix = _read_pgm(old_pgm)
    if abs(ores - nres) > 1e-6:
        _log(f"occupancy merge skipped: resolution mismatch old={ores} new={nres}")
        yaml_text = _format_map_yaml(image_name, nres, norigin, ny_meta)
        return {"yaml": yaml_text, "pgm": _write_pgm_bytes(nw, nh, npix),
                "width": nw, "height": nh, "resolution": nres, "origin": norigin, "merged": False}
    res = nres
    min_x = min(oorigin[0], norigin[0])
    min_y = min(oorigin[1], norigin[1])
    max_x = max(oorigin[0] + ow * res, norigin[0] + nw * res)
    max_y = max(oorigin[1] + oh * res, norigin[1] + nh * res)
    cw = max(1, int(round((max_x - min_x) / res)))
    ch = max(1, int(round((max_y - min_y) / res)))
    canvas = bytearray([PGM_UNKNOWN]) * (cw * ch)
    yaw = norigin[2] if len(norigin) > 2 else 0.0

    def _blit(pixels, w, h, origin, into_existing=False):
        ox = int(round((origin[0] - min_x) / res))
        oy = int(round((origin[1] - min_y) / res))  # cells up from canvas bottom
        for r in range(h):
            src_gy = (h - 1 - r)  # row from bottom in source
            for c in range(w):
                dx, dy = c + ox, src_gy + oy
                if dx < 0 or dy < 0 or dx >= cw or dy >= ch:
                    continue
                dr = ch - 1 - dy
                di = dr * cw + dx
                val = pixels[r * w + c]
                if into_existing:
                    canvas[di] = merge_cell(canvas[di], val)
                else:
                    canvas[di] = val

    _blit(opix, ow, oh, oorigin, into_existing=False)
    _blit(npix, nw, nh, norigin, into_existing=True)
    origin = [min_x, min_y, yaw]
    yaml_text = _format_map_yaml(image_name, res, origin, ny_meta)
    return {"yaml": yaml_text, "pgm": _write_pgm_bytes(cw, ch, canvas),
            "width": cw, "height": ch, "resolution": res, "origin": origin, "merged": True}

def _write_map_pair(stem, yaml_text, pgm_bytes, keep, removed):
    """Atomically write stem.yaml/.pgm into MAPS and HOST_DIR/maps with a relative image: path."""
    yaml_text = re.sub(r"^image:.*$", f"image: {stem}.pgm", yaml_text, flags=re.M)
    if not yaml_text.endswith("\n"):
        yaml_text += "\n"
    for d in (MAPS, os.path.join(HOST_DIR, "maps")):
        os.makedirs(d, exist_ok=True)
        _write_room(os.path.join(d, stem + ".pgm"), pgm_bytes, keep, removed)
        _write_room(os.path.join(d, stem + ".yaml"), yaml_text, keep, removed)
    return os.path.join(MAPS, stem + ".yaml"), os.path.join(MAPS, stem + ".pgm")

def _pgm_dims(path):
    w, h, _, _ = _read_pgm(path)
    return w, h

def _fallback_save_map(name, maps_dir):
    import gb_robot
    return gb_robot.save_map(name, maps_dir)

def _info_from_pair(name, dst_y, dst_p, yaml_text=None):
    y = yaml_text if yaml_text is not None else open(dst_y).read()
    meta, res, origin = _parse_map_yaml(y)
    w, h = _pgm_dims(dst_p)
    return {"name": name, "yaml": dst_y, "pgm": dst_p, "width": w, "height": h,
            "resolution": res, "origin": origin}

def save(name=None, stop_after=True, store=None):
    """Save the live map, densifying into the master map unless a custom name is given.

    Auto/exploration names (None, explore_*, map_*) write master.yaml/.pgm plus a serialized
    master.posegraph/.data, after a bounded timestamped rollback of any previous master.
    Custom names (e.g. living_room) are written to their own files and never auto-purged.
    Both the primary map_saver_cli path and the rosbridge /map fallback serialize the
    pose-graph while slam_toolbox is still running (before stop_after).
    Returns (ok, message, info).
    """
    _setup()
    if not running():
        return False, "Mapping isn't running, so there's no map to save. Say start mapping first.", None
    dest, is_custom = resolve_save_name(name)
    bak = None
    if not is_custom:
        bak = backup_master()
    removed = make_room(keep=(dest, MASTER_MAP_NAME))
    target = f"{CONT_DIR}/maps/{dest}"
    r = _dx([f"{CONT_DIR}/rosrun.sh", "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", target,
             "--ros-args", "-p", "map_subscribe_transient_local:=true", "-p", "save_map_timeout:=10.0"], timeout=40)
    src_y, src_p = os.path.join(HOST_DIR, "maps", dest + ".yaml"), os.path.join(HOST_DIR, "maps", dest + ".pgm")
    used_fallback = False
    merged = False
    serialized = False
    if not (os.path.exists(src_y) and os.path.exists(src_p) and os.path.getsize(src_p) > 0):
        _log(f"map_saver_cli failed rc={r.returncode}: {(r.stdout + r.stderr)[-300:]}")
        try:
            # Fallback: grab /map over rosbridge. serialize_map first (while slam is still up)
            # so the growing pose-graph is persisted even if map_saver_cli failed.
            serialized, _ser_detail = serialize_map_graph(dest)
            stem, res, w, h, origin = _fallback_save_map(dest, MAPS)
            used_fallback = True
            fb_y, fb_p = stem + ".yaml", stem + ".pgm"
            prior_y = os.path.join(MAPS, dest + ".yaml")
            prior_p = os.path.join(MAPS, dest + ".pgm")
            # If we just backed up master, the live dest files may already have been copied
            # to dest_bak_*; backup_master does not delete master, so prior still exists.
            # Prefer the rollback as the "old" grid if we are about to overwrite dest.
            old_y, old_p = prior_y, prior_p
            if bak:
                cand_y, cand_p = os.path.join(MAPS, bak + ".yaml"), os.path.join(MAPS, bak + ".pgm")
                if os.path.exists(cand_y) and os.path.exists(cand_p):
                    old_y, old_p = cand_y, cand_p
            # When serialize_map worked, /map already reflects the deserialized+densified graph
            # — use it as-is (after rewriting the image: line). When serialize was unavailable
            # and a prior master grid exists, merge occupancy cells conservatively.
            if (not serialized) and (not is_custom) and os.path.exists(old_y) and os.path.exists(old_p):
                result = merge_occupancy_maps(old_y, old_p, fb_y, fb_p, image_name=dest + ".pgm")
                merged = result["merged"]
                dst_y, dst_p = _write_map_pair(dest, result["yaml"], result["pgm"], (dest, MASTER_MAP_NAME), removed)
                w, h, res, origin = result["width"], result["height"], result["resolution"], result["origin"]
            else:
                y = open(fb_y).read()
                dst_y, dst_p = _write_map_pair(dest, y, open(fb_p, "rb").read(), (dest, MASTER_MAP_NAME), removed)
                w, h = _pgm_dims(dst_p)
            # Drop the throwaway timestamped file gb_robot.save_map created so we don't
            # leave a disconnected layered map next to master / the custom name.
            fb_stem = os.path.basename(stem)
            if fb_stem != dest:
                _remove_stem(fb_stem, dirs=(MAPS,))
            info = {"name": dest, "yaml": dst_y, "pgm": dst_p, "width": w, "height": h,
                    "resolution": float(res), "origin": origin, "fallback": True,
                    "serialized": serialized, "merged": merged, "backup": bak,
                    "custom": is_custom}
            if store is not None:
                store.add_map(dest, dst_y, dst_p, info["resolution"], w, h, info["origin"])
            _log(f"saved via rosbridge fallback {dest} {w}x{h} serialized={serialized} merged={merged}")
            purged = purge_old_maps(keep=(dest, MASTER_MAP_NAME))
            label = dest.replace("_", " ")
            msg = f"Map saved as {label}, {w} by {h} cells, on my SD card."
            if not is_custom:
                msg += " Densified into the growing master map."
            if merged:
                msg += " Merged with the previous master occupancy grid."
            if stop_after:
                ok, m = stop(); msg += " Mapping stopped." if ok else " " + m
            return True, msg, info
        except Exception as ex:
            _log(f"rosbridge fallback failed: {ex}")
        return False, "Saving the map failed, no map has been received yet. Drive a little and try again.", None
    # Primary path: map_saver_cli wrote HOST_DIR/maps/<dest>.yaml/.pgm. Serialize the live
    # pose-graph before stop_after so the next start() can reload and densify it.
    serialized, _ser_detail = serialize_map_graph(dest)
    y = open(src_y).read()
    dst_y, dst_p = _write_map_pair(dest, y, open(src_p, "rb").read(), (dest, MASTER_MAP_NAME), removed)
    info = _info_from_pair(dest, dst_y, dst_p, yaml_text=open(dst_y).read())
    info.update({"serialized": serialized, "merged": False, "backup": bak,
                 "fallback": False, "custom": is_custom})
    if store is not None:
        store.add_map(dest, dst_y, dst_p, info["resolution"], info["width"], info["height"], info["origin"])
    _log(f"saved {dst_y} {dst_p} {info['width']}x{info['height']} serialized={serialized}")
    purged = purge_old_maps(keep=(dest, MASTER_MAP_NAME))
    msg = f"Map saved as {dest.replace('_', ' ')}, {info['width']} by {info['height']} cells, on my SD card."
    if not is_custom:
        msg += " Densified into the growing master map."
    if removed:
        msg += f" The card was low, so I overwrote the {len(removed)} oldest saved map" + ("s." if len(removed) > 1 else ".")
    if stop_after:
        ok, m = stop(); msg += " Mapping stopped." if ok else " " + m
    return True, msg, info
