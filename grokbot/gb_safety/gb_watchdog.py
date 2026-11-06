#!/usr/bin/env python3
"""gb_watchdog.py - independent motion watchdog (own systemd unit gb-watchdog, Restart=always; runs in the container).

Watches every velocity topic the Hiwonder controller listens to (/controller/cmd_vel, /cmd_vel, /app/cmd_vel), the motor
topic itself (/ros_robot_controller/set_motor) and the gate heartbeat (/gb/gate_status). Sends ZERO (never anything else):
  * ESTOP file present                          -> zero Twist + zero MotorsState at 10 Hz while latched
  * current command non-zero but not refreshed for WD_STALE_S (2 s) -> zero burst ("stale" = STM32 would hold it)
  * gate heartbeat stale (>1 s) while the gate was moving        -> zero burst
  * motors non-zero (set_motor) with no non-zero cmd_vel for 2 s -> zero burst (catches a latched board)
  * idle keep-alive: while nothing non-zero for >2 s, a zero MotorsState once per WD_IDLE_ZERO_S (1 s)
Direct controller stop: zero MotorsState on /ros_robot_controller/set_motor = Hiwonder SDK Board.set_motor_speed([[1,0]..[4,0]]).
If the ros_robot_controller node is gone (no subscriber on set_motor for 1 s) and a stop is needed, it writes the SDK motor
packet (func 3, sub-cmd 0x01, 4 x <id, 0.0f>) straight to /dev/rrc (port is free when the node is dead).
WD_DRY=1 -> publishes to /gb/wd_test_* topics instead and never opens the serial port (unit tests).
"""
import json, os, struct, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

E = os.environ.get
DRY = E("WD_DRY", "0") == "1"
WATCH = [t for t in E("WD_WATCH", "/controller/cmd_vel,/cmd_vel,/app/cmd_vel").split(",") if t]
ZERO_TWIST_TOPIC = E("WD_ZERO_TOPIC", "/gb/wd_test_cmd" if DRY else "/controller/cmd_vel")
MOTOR_TOPIC = E("WD_MOTOR_TOPIC", "/ros_robot_controller/set_motor")
MOTOR_OUT = E("WD_MOTOR_OUT", "/gb/wd_test_motor" if DRY else MOTOR_TOPIC)
GATE_STATUS = E("WD_GATE_STATUS", "/gb/gate_status")
ESTOP = E("GB_ESTOP_FILE", "/home/ubuntu/shared/gb_safety/ESTOP")
MAINT = E("GB_MAINT_FILE", "/home/ubuntu/shared/gb_safety/MAINT")
STALE = float(E("WD_STALE_S", "2.0"))
IDLE_ZERO = float(E("WD_IDLE_ZERO_S", "1.0"))
LOGF = E("WD_LOG", "/home/ubuntu/shared/gb_safety/watchdog%s.log" % ("_dry" if DRY else ""))
SERIAL = E("WD_SERIAL", "/dev/rrc")

try:
    from ros_robot_controller_msgs.msg import MotorsState, MotorState
except Exception:
    MotorsState = MotorState = None

def _now_s():
    t = time.time(); return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)

def _crc8(data):
    try:
        from ros_robot_controller.ros_robot_controller_sdk import checksum_crc8
        return checksum_crc8(bytes(data))
    except Exception:
        return None

class WD(Node):
    def __init__(self):
        super().__init__("gb_watchdog_dry" if DRY else "gb_watchdog")
        os.makedirs(os.path.dirname(LOGF), exist_ok=True)
        self.logf = open(LOGF, "a", buffering=1)
        self.zt = self.create_publisher(Twist, ZERO_TWIST_TOPIC, 10)
        self.zm = self.create_publisher(MotorsState, MOTOR_OUT, 10) if MotorsState else None
        self.cur = {}                                  # topic -> (nonzero?, t_last_msg, lin, ang)
        for t in WATCH:
            self.create_subscription(Twist, t, lambda m, t=t: self.on_twist(m, t), 20)
        if MotorsState:
            self.create_subscription(MotorsState, MOTOR_TOPIC, self.on_motor, 20)
        self.create_subscription(String, GATE_STATUS, self.on_gate, 10)
        self.motor_nz = False; self.motor_t = 0.0; self.last_nz_cmd = 0.0
        self.gate = None; self.gate_t = 0.0; self.gate_moving_t = 0.0
        self.last_idle_zero = 0.0; self.last_estop_zero = 0.0; self.burst_until = 0.0; self.burst_why = ""
        self.ctrl_missing_since = None; self.n_zero = 0; self.n_serial = 0
        self.estop = False
        self.own_zero_t = 0.0
        self.create_timer(0.1, self.tick)
        self.log("watchdog up dry=%s watch=%s motor=%s zero->%s,%s stale=%.1fs estop=%s"
                 % (DRY, WATCH, MOTOR_TOPIC, ZERO_TWIST_TOPIC, MOTOR_OUT, STALE, ESTOP))

    def log(self, msg):
        line = "%s [watchdog] %s" % (_now_s(), msg)
        try: self.logf.write(line + "\n")
        except Exception: pass
        self.get_logger().info(msg)

    def on_twist(self, m, t):
        nz = abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4 or abs(m.linear.y) > 1e-4
        now = time.time()
        prev = self.cur.get(t)
        if nz and (prev is None or not prev[0]):
            self.log("nonzero on %s lin=%+.3f ang=%+.3f" % (t, m.linear.x, m.angular.z))
        if nz: self.last_nz_cmd = now
        self.cur[t] = (nz, now, m.linear.x, m.angular.z)

    def on_motor(self, m):
        nz = any(abs(x.rps) > 1e-3 for x in m.data)
        if nz != self.motor_nz:
            self.log("motors %s" % ("NONZERO " + str([round(x.rps, 2) for x in m.data]) if nz else "zero"))
        self.motor_nz = nz; self.motor_t = time.time()

    def on_gate(self, m):
        try: self.gate = json.loads(m.data)
        except Exception: return
        self.gate_t = time.time()
        if self.gate.get("mode") not in (None, "idle"): self.gate_moving_t = self.gate_t

    def _ack_maint(self):
        path = E("WD_STATE", "/home/ubuntu/shared/gb_safety/watchdog_state.json")
        body = {"t": time.time(), "maint": True, "zeroing": True, "release": E("GB_RELEASE", ""), "why": "maintenance"}
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(body, f)
            os.replace(tmp, path)
        except OSError:
            pass

    def zero(self, why, serial_ok=True):
        tw = Twist(); self.zt.publish(tw)
        if self.zm:
            ms = MotorsState(); ms.data = [MotorState(id=i, rps=0.0) for i in (1, 2, 3, 4)]
            self.zm.publish(ms)
        self.n_zero += 1; self.own_zero_t = time.time()
        if serial_ok and not DRY and self.controller_dead():
            self.serial_zero(why)

    def controller_dead(self):
        try: n = self.count_subscribers(MOTOR_TOPIC)
        except Exception: n = 1
        now = time.time()
        if n == 0:
            if self.ctrl_missing_since is None: self.ctrl_missing_since = now
            return now - self.ctrl_missing_since > 1.0
        self.ctrl_missing_since = None; return False

    def serial_zero(self, why):
        data = [0x01, 4]
        for i in range(4): data.extend(struct.pack("<Bf", i, 0.0))
        buf = [0xAA, 0x55, 3, len(data)] + list(data)
        c = _crc8(buf[2:])
        if c is None:
            self.log("serial zero skipped: SDK crc8 unavailable"); return
        buf.append(c)
        try:
            import serial
            p = serial.Serial(SERIAL, 1000000, timeout=0.2); p.rts = False; p.dtr = False
            p.write(bytes(buf)); p.flush(); p.close(); self.n_serial += 1
            self.log("controller node absent -> wrote SDK motor-zero packet to %s (%s)" % (SERIAL, why))
        except Exception as ex:
            self.log("serial zero failed: %s" % ex)

    def start_burst(self, why, secs=2.0):
        now = time.time()
        if now > self.burst_until:
            self.log("ZERO BURST: %s" % why)
        self.burst_until = now + secs; self.burst_why = why

    def tick(self):
        now = time.time()
        if os.path.exists(MAINT):
            self.zero("maintenance")
            self._ack_maint()
            return
        e = os.path.exists(ESTOP)
        if e != self.estop:
            self.log("ESTOP %s" % ("LATCHED -> zero at 10 Hz" if e else "cleared"))
            self.estop = e
        if e:
            self.zero("estop"); return
        # stale non-zero command on any controller input (ignore our own zeros)
        for t, (nz, tl, lin, ang) in list(self.cur.items()):
            if nz and now - tl > STALE:
                self.start_burst("last command on %s was non-zero (lin=%+.3f ang=%+.3f) %.1fs ago with no refresh"
                                 % (t, lin, ang, now - tl))
                self.cur[t] = (False, now, 0.0, 0.0)
        # gate died while moving
        if self.gate_moving_t and now - self.gate_t > 1.0 and self.gate_t >= self.gate_moving_t - 0.2:
            self.start_burst("gate heartbeat stale %.1fs while it was moving" % (now - self.gate_t)); self.gate_moving_t = 0.0
        # motors still turning with no fresh non-zero command
        if self.motor_nz and now - self.last_nz_cmd > STALE and now - self.motor_t < 5.0:
            self.start_burst("motors non-zero with no non-zero cmd_vel for %.1fs" % (now - self.last_nz_cmd)); self.motor_nz = False
        if now < self.burst_until:
            self.zero(self.burst_why); return
        # idle keep-alive zero straight to the motor topic
        any_nz = any(v[0] for v in self.cur.values()) or (now - self.last_nz_cmd < STALE)
        if not any_nz and now - self.last_idle_zero >= IDLE_ZERO:
            if self.zm:
                ms = MotorsState(); ms.data = [MotorState(id=i, rps=0.0) for i in (1, 2, 3, 4)]; self.zm.publish(ms)
            self.last_idle_zero = now

def main():
    rclpy.init()
    w = WD()
    try:
        rclpy.spin(w)
    except (KeyboardInterrupt, Exception) as ex:
        try: w.log("watchdog exiting (%s) -> final zero" % type(ex).__name__); w.zero("exit", serial_ok=False)
        except Exception: pass
    finally:
        try: w.destroy_node(); rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
