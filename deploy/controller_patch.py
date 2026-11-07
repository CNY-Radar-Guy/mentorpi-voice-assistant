"""Exact patch for the incident snapshot of ros_robot_controller_node.py.

The installed source is deploy/vendor/ros_robot_controller_node.py, sha256
455e6362c71a2d8ba7dacf7db3a81d42183f371b7a04b6364d360fc37fb339d9, taken from
/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py.

The running package is not a site-packages copy. colcon's egg-link points
at build/ros_robot_controller, and that package directory is a symlink to
the source tree. Python loads the source file. The installer hashes that
regular file, writes it once, and does not replace the symlink or the
console script. A missing site-packages path is recorded and is not a
failure. An unknown hash writes nothing. A hash that matches neither the
snapshot nor the patched file is left untouched.

Applying the patch does not restart the controller. Rollback does not copy
the backup back: the original set_motor_state accepts every nonzero command.
"""
import hashlib
import os
import shutil

ORIGINAL_SHA256 = "455e6362c71a2d8ba7dacf7db3a81d42183f371b7a04b6364d360fc37fb339d9"
VENDOR_NAME = "ros_robot_controller_node.py"
WS_DEFAULT = "/home/ubuntu/ros2_ws"
SRC_REL = (
    "src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py"
)
EGG_REL = (
    "install/ros_robot_controller/lib/python3.10/site-packages/"
    "ros-robot-controller.egg-link"
)
# This path is not how the running package is loaded. Absence is not a failure.
SITE_REL = (
    "install/ros_robot_controller/lib/python3.10/site-packages/"
    "ros_robot_controller/ros_robot_controller_node.py"
)
# Console script. It imports the module. It is not the module and is never written.
CONSOLE_REL = "install/ros_robot_controller/lib/ros_robot_controller/ros_robot_controller"
CONTAINER_SRC = WS_DEFAULT + "/" + SRC_REL
CONTAINER_INSTALL = WS_DEFAULT + "/" + SITE_REL
CANDIDATES = (CONTAINER_SRC,)
MISSING_FROM_ARCHIVE = (
    "/home/ubuntu/software/chassis_adjustment/robot_correction_factors.yaml",
    "ros_robot_controller/setup.py",
)

_OLD_IMPORT = "import os\n"
_NEW_IMPORT = "import os\nimport sys\n"

_OLD_INIT = """        # Initialize motor speed(初始化电机速度)
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.clock = self.get_clock()
"""
_NEW_INIT = """        # Initialize motor speed(初始化电机速度)
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        self._gb_install_hook()

        self.clock = self.get_clock()
"""

_OLD_MOTOR = """    def set_motor_state(self, msg):
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)
"""

_NEW_MOTOR = '''    def _gb_install_hook(self):
        """Load the admission modules and arm the controller-side zero timer.

        ~/set_motor stays subscribed. /gb/set_motor_admitted is extra.
        The timer calls Board.set_motor_speed with zeros; it dies if this
        process dies. Another process can still open /dev/rrc.
        """
        sys.path.insert(0, os.environ.get("GB_HOOK_DIR", "/home/ubuntu/shared/gb_safety/gb_hook"))
        import gb_actuation
        import gb_intent
        import gb_motor_cap
        self._gb_actuation = gb_actuation
        self._gb_intent = gb_intent
        self._gb_cap = gb_motor_cap
        self._gb = gb_actuation.ControllerHook()
        self.create_subscription(MotorsState, "/gb/set_motor_admitted", self._gb_admitted, 10)
        self.create_timer(0.1, self._gb_expire)

    def set_motor_state(self, msg):
        """Raw topic. Zeros pass. Nonzero is replaced with zeros."""
        pairs = [(i.id, i.rps) for i in msg.data]
        speeds, _decision = self._gb_actuation.board_command(
            pairs, None, time.monotonic(), None, self._gb_actuation.maint_held(), None, self._gb)
        if speeds is not None:
            self.board.set_motor_speed([[i, r] for i, r in speeds])

    def _gb_admitted(self, msg):
        """Recheck a public-topic command against the lease, envelope, and cap.

        Follow-me is not a bounded_look lease, so it is not admitted here.
        Straight commands use the installed left/right correction before the
        tank map. enforced_on_robot stays false until this process is the one
        running on the robot. A crash of this process does not zero the board.
        """
        pairs = [(i.id, i.rps) for i in msg.data]
        now = time.monotonic()
        lease = self._gb_intent.read_lease()
        envelope = self._gb_intent.read_envelope(now)
        if envelope:
            left, right = self._gb_cap.load_track_correction()
            lin, ang = self._gb_cap.correct_tank_twist(envelope["lin"], envelope["ang"], left, right)
            envelope = dict(envelope, lin=lin, ang=ang)
        claim = self._gb_intent.claim_from(lease, envelope, now)
        speeds, decision = self._gb.set_motor_state(
            pairs, now, lease, maint=self._gb_actuation.maint_held(), claim=claim)
        decision["vendor_callback"] = True
        decision["enforced_on_robot"] = False
        decision["serial_bypass"] = True
        decision["controller_crash_zeros_board"] = False
        decision["follow_admitted"] = False
        self.board.set_motor_speed([[i, r] for i, r in speeds])

    def _gb_expire(self):
        speeds, _decision = self._gb.idle_tick(time.monotonic())
        if speeds is not None:
            self.board.set_motor_speed([[i, r] for i, r in speeds])
'''


def vendor_path(root=None):
    root = root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(root, "deploy", "vendor", VENDOR_NAME)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def read_vendor():
    with open(vendor_path(), "rb") as handle:
        return handle.read()


def patch_source(text):
    """Return the patched source. Refuse anything that is not the snapshot."""
    if isinstance(text, bytes):
        text = text.decode("utf-8")
    if "_gb_install_hook" in text and _OLD_MOTOR not in text:
        return text
    if _OLD_IMPORT not in text or _OLD_INIT not in text or _OLD_MOTOR not in text:
        raise ValueError("controller source is not the incident snapshot")
    if text.count(_OLD_MOTOR) != 1 or text.count(_OLD_INIT) != 1:
        raise ValueError("controller anchors are not unique")
    out = text.replace(_OLD_IMPORT, _NEW_IMPORT, 1)
    out = out.replace(_OLD_INIT, _NEW_INIT, 1)
    out = out.replace(_OLD_MOTOR, _NEW_MOTOR, 1)
    return out


def patched_bytes():
    return patch_source(read_vendor()).encode("utf-8")


def patched_sha256():
    return sha256_bytes(patched_bytes())


def _egg_target(egg_path):
    try:
        lines = open(egg_path).read().splitlines()
    except OSError:
        return None
    for line in lines:
        line = line.strip()
        if line and not line.startswith("#") and line != ".":
            return line if os.path.isabs(line) else os.path.join(os.path.dirname(egg_path), line)
    return None


def resolve_layout(ws):
    """Unique regular files Python can load, plus optional paths that are absent.

    The observed colcon layout is an egg-link to build/<pkg>, whose package
    directory is a symlink to src. Those paths are one inode. The site-packages
    module path is recorded as absent and does not fail the plan. A symlink is
    never the write path: the write path is the regular file it resolves to.
    """
    ws = os.path.abspath(ws)
    named = []
    absent = []
    src = os.path.join(ws, SRC_REL)
    if os.path.lexists(src):
        named.append(src)
    else:
        absent.append(src)
    egg = os.path.join(ws, EGG_REL)
    if os.path.isfile(egg) and not os.path.islink(egg):
        root = _egg_target(egg)
        if root:
            through = os.path.join(root, "ros_robot_controller", VENDOR_NAME)
            if os.path.lexists(through):
                named.append(through)
            else:
                absent.append(through)
    site = os.path.join(ws, SITE_REL)
    if os.path.lexists(site):
        named.append(site)
    else:
        absent.append(site)
    groups = {}
    for path in named:
        real = os.path.realpath(path)
        slot = groups.setdefault(real, {"aliases": [], "symlink_aliases": []})
        if path not in slot["aliases"]:
            slot["aliases"].append(path)
        if os.path.islink(path) or path != real:
            slot["symlink_aliases"].append(path)
    return groups, absent


def install_layout(ws, apply=False, backup_dir=None):
    """Hash every resolved file first. Any unknown hash writes nothing.

    apply True still does not restart a process. Writing the source changes
    what the next import loads. The process that already imported the module
    keeps the old code.
    """
    groups, absent = resolve_layout(ws)
    result = {
        "ok": False, "changed": False, "restart": False, "clear_maint": False,
        "writes": [], "absent": absent, "files": [], "backup": None,
        "reason": "no_loaded_module",
    }
    if not groups:
        return result
    patched = patched_sha256()
    for real in sorted(groups):
        info = groups[real]
        if os.path.islink(real) or not os.path.isfile(real):
            result["reason"] = "target_not_regular"
            result["path"] = real
            return result
        with open(real, "rb") as handle:
            digest = sha256_bytes(handle.read())
        if digest == patched:
            kind = "patched"
        elif digest == ORIGINAL_SHA256:
            kind = "original"
        else:
            kind = "unknown"
        result["files"].append({
            "realpath": real, "digest": digest, "kind": kind, "aliases": info["aliases"],
        })
    if any(item["kind"] == "unknown" for item in result["files"]):
        result["reason"] = "hash_mismatch"
        result["found"] = [item["digest"] for item in result["files"] if item["kind"] == "unknown"]
        return result
    originals = [item for item in result["files"] if item["kind"] == "original"]
    result["ok"] = True
    result["reason"] = "already_patched" if not originals else "ready"
    if not apply or not originals:
        return result
    backup_dir = backup_dir or os.path.join(os.path.abspath(ws), ".gb-controller-backup")
    for item in originals:
        one = apply_file(item["realpath"], apply=True, backup_dir=backup_dir)
        if not one.get("ok"):
            result.update(ok=False, reason=one.get("reason") or "write_failed", backup=one.get("backup"))
            return result
        with open(item["realpath"], "rb") as handle:
            after = sha256_bytes(handle.read())
        if after != patched or os.path.islink(item["realpath"]):
            result.update(ok=False, reason="write_failed")
            return result
        result["writes"].append(item["realpath"])
        result["backup"] = one.get("backup")
    result["changed"] = bool(result["writes"])
    result["reason"] = "patched"
    return result


def apply_file(path, apply=False, backup_dir=None):
    """Patch one regular file. apply False changes nothing. A restart is never requested."""
    result = {"path": path, "changed": False, "restart": False, "backup": None,
              "clear_maint": False, "power_switch": "whole robot; no separate motor switch"}
    if os.path.islink(path):
        result.update(ok=False, reason="refusing_to_replace_symlink")
        return result
    try:
        with open(path, "rb") as handle:
            current = handle.read()
    except OSError as ex:
        result.update(ok=False, reason="unreadable", detail=str(ex))
        return result
    digest = sha256_bytes(current)
    patched = patched_sha256()
    if digest == patched:
        result.update(ok=True, reason="already_patched")
        return result
    if digest != ORIGINAL_SHA256:
        result.update(ok=False, reason="hash_mismatch", found=digest, expected=ORIGINAL_SHA256)
        return result
    result.update(ok=True, reason="dry_run_original_matches" if not apply else "patched")
    if not apply:
        return result
    backup_dir = backup_dir or os.path.dirname(path)
    os.makedirs(backup_dir, exist_ok=True)
    backup = os.path.join(backup_dir, os.path.basename(path) + ".gb-orig")
    if not os.path.exists(backup):
        shutil.copyfile(path, backup)
    elif sha256_bytes(open(backup, "rb").read()) != ORIGINAL_SHA256:
        result.update(ok=False, reason="backup_is_not_the_snapshot", changed=False)
        return result
    tmp = path + ".gb-tmp"
    with open(tmp, "wb") as handle:
        handle.write(patched_bytes())
    os.replace(tmp, path)
    result.update(changed=True, backup=backup, reason="patched")
    return result


def rollback_file(path, backup_path=None):
    """Keep the patched callback. Do not restore the original nonzero writer."""
    del backup_path
    try:
        digest = sha256_bytes(open(path, "rb").read())
    except OSError as ex:
        return {"ok": False, "restored": False, "nonzero_denied": True, "detail": str(ex),
                "reason": "unreadable"}
    if digest != patched_sha256():
        return {"ok": False, "restored": False, "nonzero_denied": True,
                "reason": "refusing to restore the original set_motor_state; this file is not the patched callback"}
    return {"ok": True, "restored": False, "nonzero_denied": True, "restart": False, "clear_maint": False,
            "reason": "backup kept for audit; original set_motor_state is not restored"}
