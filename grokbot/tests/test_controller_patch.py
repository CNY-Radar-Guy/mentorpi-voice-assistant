"""The patched vendor callback, executed. No robot and no restart.

Helper decisions are not enough: this loads the incident
ros_robot_controller_node.py, applies the exact patch, constructs
RosRobotController with a fake board, and calls set_motor_state,
_gb_admitted, and the timer callback.
"""
import os, shutil, subprocess, sys, types
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "grokbot"))
os.environ["GB_HOOK_DIR"] = os.path.join(ROOT, "grokbot")
os.environ["MACHINE_TYPE"] = "MentorPi_Tank"

import gb_actuation
import gb_bounded
import gb_intent
import gb_motor_cap as cap
from deploy.controller_patch import (ORIGINAL_SHA256, apply_file, patch_source,
                                     patched_sha256, read_vendor, rollback_file,
                                     sha256_bytes, vendor_path)

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

original = read_vendor()
check("the vendored node is the incident snapshot", sha256_bytes(original) == ORIGINAL_SHA256)
patched = patch_source(original.decode("utf-8"))
check("the patch replaces set_motor_state", "def _gb_install_hook" in patched and "def _gb_expire" in patched
      and "data.extend([[i.id, i.rps]])" not in patched)
check("raw set_motor stays subscribed", "create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)" in patched)
digest = patched_sha256()
again = patch_source(patched)
check("patching the patched source is idempotent", again == patched and sha256_bytes(again.encode("utf-8")) == digest)

work = os.path.join(os.environ["GB_SHARED_DIR"], "controller")
os.makedirs(work, exist_ok=True)
live = os.path.join(work, "ros_robot_controller_node.py")
with open(live, "wb") as handle:
    handle.write(original)
dry = apply_file(live, apply=False, backup_dir=work)
check("a matching snapshot is recognized and not written", dry["ok"] and dry["changed"] is False and dry["restart"] is False
      and sha256_bytes(open(live, "rb").read()) == ORIGINAL_SHA256)
done = apply_file(live, apply=True, backup_dir=work)
check("apply writes the patched callback and keeps a snapshot backup",
      done["changed"] and sha256_bytes(open(live, "rb").read()) == digest
      and sha256_bytes(open(done["backup"], "rb").read()) == ORIGINAL_SHA256)
second = apply_file(live, apply=True, backup_dir=work)
check("a second apply does not rewrite", second["ok"] and second["reason"] == "already_patched" and second["changed"] is False)
rolled = rollback_file(live, done["backup"])
check("rollback does not restore the original nonzero writer",
      rolled["restored"] is False and rolled["nonzero_denied"] is True
      and sha256_bytes(open(live, "rb").read()) == digest)
alien = os.path.join(work, "other.py")
with open(alien, "w") as handle:
    handle.write("def set_motor_state(self, msg):\n    pass\n")
refused = apply_file(alien, apply=True, backup_dir=work)
check("a different file is left untouched", refused["ok"] is False and refused["reason"] == "hash_mismatch"
      and open(alien).read().startswith("def set_motor_state"))

class Node(object):
    def __init__(self, name):
        self.subs = []
        self.timers = []
    def create_subscription(self, _typ, topic, cb, _qos):
        self.subs.append((topic, cb))
    def create_publisher(self, *_a, **_k):
        return object()
    def create_service(self, *_a, **_k):
        return object()
    def create_timer(self, period, cb):
        self.timers.append((period, cb))
    def declare_parameter(self, *_a, **_k):
        return None
    def get_parameter(self, _name):
        return types.SimpleNamespace(value="imu_link")
    def get_logger(self):
        return self
    def info(self, *_a):
        pass
    def error(self, *_a):
        pass
    def warn(self, *_a):
        pass
    def get_clock(self):
        return self

class Board(object):
    def __init__(self):
        self.writes = []
    def set_motor_speed(self, data):
        self.writes.append(data)
    def enable_reception(self, *_a):
        pass
    def set_motor_type(self, *_a):
        pass
    def set_battery_level(self, *_a):
        pass
    def pwm_servo_set_offset(self, *_a):
        pass

def lazy(name):
    if name in sys.modules and name in ("yaml",):
        return sys.modules[name]
    module = types.ModuleType(name)
    sys.modules[name] = module
    return module

rclpy = lazy("rclpy")
rclpy.init = lambda *_a, **_k: None
rclpy.shutdown = lambda *_a, **_k: None
rclpy_node = lazy("rclpy.node")
rclpy_node.Node = Node
sdk = lazy("ros_robot_controller.ros_robot_controller_sdk")
sdk.Board = Board
sdk.PacketReportKeyEvents = type("PacketReportKeyEvents", (), {})
lazy("ros_robot_controller")
for name in ("std_srvs", "std_srvs.srv", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg",
             "ros_robot_controller_msgs", "ros_robot_controller_msgs.srv", "ros_robot_controller_msgs.msg"):
    lazy(name)
std_srvs = sys.modules["std_srvs.srv"]
std_srvs.Trigger = type("Trigger", (), {})
sensor = sys.modules["sensor_msgs.msg"]
for name in ("Imu", "Joy"):
    setattr(sensor, name, type(name, (), {}))
std_msgs = sys.modules["std_msgs.msg"]
for name in ("UInt16", "Bool"):
    setattr(std_msgs, name, type(name, (), {}))
msgs = sys.modules["ros_robot_controller_msgs.msg"]
for name in ("ButtonState", "BuzzerState", "MotorsState", "BusServoState", "LedState",
             "SetBusServoState", "ServosPosition", "SetPWMServoState", "Sbus", "OLEDState",
             "RGBStates", "PWMServoState"):
    setattr(msgs, name, type(name, (), {}))
srvs = sys.modules["ros_robot_controller_msgs.srv"]
for name in ("GetBusServoState", "GetPWMServoState"):
    setattr(srvs, name, type(name, (), {}))

class _Thread(object):
    def __init__(self, *a, **k):
        pass
    def start(self):
        pass
sys.modules["threading"].Thread = _Thread
sys.modules["signal"].signal = lambda *_a, **_k: None

clock = {"t": 5000.0}
ns = {}
exec(compile(patched, vendor_path(), "exec"), ns)
ns["time"].monotonic = lambda: clock["t"]
node = ns["RosRobotController"]("ros_robot_controller")
check("the controller timer is armed at 0.1 s", node.timers and node.timers[0][0] == 0.1 and node.timers[0][1] == node._gb_expire)
topics = [topic for topic, _cb in node.subs]
check("raw and admitted subscriptions are both present", "~/set_motor" in topics and "/gb/set_motor_admitted" in topics)

class Item(object):
    def __init__(self, mid, rps):
        self.id = mid
        self.rps = rps

class Msg(object):
    def __init__(self, pairs):
        self.data = [Item(i, r) for i, r in pairs]

node.set_motor_state(Msg([(1, 0.4), (2, 0.4), (3, -0.4), (4, -0.4)]))
flat = [r for _i, r in node.board.writes[-1]]
check("raw nonzero is not written to the board", all(r == 0.0 for r in flat) and 0.4 not in flat)
node.set_motor_state(Msg([(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)]))
check("raw zeros reach the board", all(r == 0.0 for _i, r in node.board.writes[-1]))

gb_intent._BOOK["lease"] = None
lease = gb_intent.issue_lease("look around", 5000.0, 45)
motors = cap.tank_rps_map(0.05, 0.0)
gb_intent.write_envelope(lease["id"], 0.05, 0.0, 5000.0)
clock["t"] = 5000.2
node._gb_admitted(Msg(motors))
written = node.board.writes[-1]
check("an admitted matching command is what the board receives",
      [(i, r) for i, r in written] == [(i, r) for i, r in motors])
follow = gb_intent.open_lease("follow me", 5000.0, 45)
follow["action"] = "follow"
follow["mono_deadline"] = 5045.0
gb_intent.write_lease(follow)
gb_intent.write_envelope(follow["id"], 0.05, 0.0, 5000.2)
node._gb_admitted(Msg(motors))
check("follow is not admitted by the callback", all(r == 0.0 for _i, r in node.board.writes[-1]))
gb_intent.write_lease(lease)
gb_intent.write_envelope(lease["id"], 0.05, 0.0, 5000.2)
clock["t"] = 5000.3
node._gb_admitted(Msg(motors))
check("the admitted command was nonzero before the timer", any(abs(r) > 0 for _i, r in node.board.writes[-1]))
clock["t"] = 5000.9
node._gb_expire()
check("the controller timer writes zeros without gb-actuation",
      all(r == 0.0 for _i, r in node.board.writes[-1]))

yaml_path = os.path.join(work, "robot_correction_factors.yaml")
with open(yaml_path, "w") as handle:
    handle.write("left_correction_factor: 1.0\nright_correction_factor: 1.02\n")
os.environ["GB_CORRECTION_YAML"] = yaml_path
left, right = cap.load_track_correction(yaml_path)
lin, ang = cap.correct_tank_twist(0.1, 0.0, left, right)
check("straight correction matches the publisher formula", abs(ang - (0.1 * 0.02 * 5.50)) < 1e-12 and lin == 0.1)
corrected = cap.tank_rps_map(lin, ang)
plain = cap.tank_rps_map(0.1, 0.0)
gb_intent.write_envelope(lease["id"], 0.1, 0.0, 5010.0)
clock["t"] = 5010.1
node._gb_admitted(Msg(corrected))
check("corrected motors pass the callback", [(i, r) for i, r in node.board.writes[-1]] == [(i, r) for i, r in corrected])
node._gb_admitted(Msg(plain))
check("uncorrected motors are rejected when the factors are not 1",
      all(r == 0.0 for _i, r in node.board.writes[-1]))
corner = cap.tank_rps_map(0.2, 0.5)
gb_intent.write_envelope(lease["id"], 0.2, 0.5, 5010.2)
clock["t"] = 5010.3
node._gb_admitted(Msg(corner))
capped = [r for _i, r in node.board.writes[-1]]
check("the callback clamps to the provisional cap",
      any(abs(r) > cap.half_rps() for _i, r in corner) and all(abs(r) <= cap.half_rps() + 1e-9 for r in capped)
      and any(abs(r) > 0 for r in capped))

look = gb_bounded.BoundedLook()
look.begin(1000.0, (0.0, 0.0, 0.0), 45, source="/odom", frame="odom", child="base_footprint", lidar_yaw=0.0)
early = look.step(1044.0, 2.0, 2.0, 2.0, (0.1, 0.0, 0.0), 0.1, 0.1)
late = gb_bounded.BoundedLook()
late.begin(1000.0, (0.0, 0.0, 0.0), 45, source="/odom", frame="odom", child="base_footprint", lidar_yaw=0.0)
ended = late.step(1045.0, 2.0, 2.0, 2.0, (0.1, 0.0, 0.0), 0.1, 0.1)
gate = open(os.path.join(ROOT, "grokbot", "gb_safety", "gb_gate.py")).read()
check("the look clock is monotonic in the gate",
      "self.look.begin(self.look_mono0" in gate and "self.look.step(time.monotonic()" in gate
      and early.get("allow") is True and ended.get("phase", ended.get("reason")) != "bounded_out")
check("follow remains outside this lease", gb_actuation.claim_ok(
    {"lease_id": "x", "source": "voice", "action": "follow", "lin": 0.0, "ang": 0.0},
    {"id": "x", "source": "voice", "action": "follow", "revoked": False}) is False)

proc = subprocess.run(["sh", os.path.join(ROOT, "deploy", "controller_patch_install.sh")],
                      cwd="/tmp", env=dict(os.environ, GB_CONTROLLER_PATCH_APPLY="0"),
                      capture_output=True, text=True)
check("the installer dry run does not apply or restart",
      proc.returncode == 0 and "No file written" in proc.stdout and "ros_robot_controller was not restarted" not in proc.stdout
      and ORIGINAL_SHA256 in proc.stdout, proc.stdout[-400:] + proc.stderr[-400:])

print("test_controller_patch: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
