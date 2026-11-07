"""Exact patch for the incident snapshot of ros_robot_controller_node.py.

The installed source is deploy/vendor/ros_robot_controller_node.py, sha256
455e6362c71a2d8ba7dacf7db3a81d42183f371b7a04b6364d360fc37fb339d9, taken from
/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py.

setup.py was not in the incident archive, so this does not rebuild the
package. It rewrites that .py, and the same bytes under the install tree
when they still match. A hash that matches neither the snapshot nor the
patched file is left untouched.

Applying the patch does not restart the controller. Rollback does not copy
the backup back: the original set_motor_state accepts every nonzero command.
"""
import hashlib
import os
import shutil

ORIGINAL_SHA256 = "455e6362c71a2d8ba7dacf7db3a81d42183f371b7a04b6364d360fc37fb339d9"
VENDOR_NAME = "ros_robot_controller_node.py"
CONTAINER_SRC = (
    "/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/"
    "ros_robot_controller/ros_robot_controller_node.py"
)
CONTAINER_INSTALL = (
    "/home/ubuntu/ros2_ws/install/ros_robot_controller/lib/python3.10/"
    "site-packages/ros_robot_controller/ros_robot_controller_node.py"
)
CANDIDATES = (CONTAINER_SRC, CONTAINER_INSTALL)
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


def apply_file(path, apply=False, backup_dir=None):
    """Patch one file. apply False changes nothing. A restart is never requested."""
    result = {"path": path, "changed": False, "restart": False, "backup": None,
              "clear_maint": False, "power_switch": "whole robot; no separate motor switch"}
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
