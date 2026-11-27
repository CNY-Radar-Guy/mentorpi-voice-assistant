"""gb_armguard.py - container backstop for the wake-phrase arming (2026-11-25). Runs as systemd gb-armguard.
Disarms at start (boot default = disarmed). While DISARMED, any non-zero Twist on /gb/cmd_in or any non-stop
request on /gb/motion_req is answered at once with a gate stop request ({"op":"stop","src":"armguard"}) - a normal
gate stop (no fault). Deletes ARMED when ESTOP/MAINT appear. Publishes /gb/arm_state (JSON) at 1 Hz. Never touches
gb_gate, its thresholds or any latch."""
import json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gb_arm
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ArmGuard(Node):
    def __init__(self):
        super().__init__("gb_armguard")
        gb_arm.disarm("gb-armguard start (boot default = disarmed)", actor="safety")
        self.req = self.create_publisher(String, "/gb/motion_req", 10)
        self.st = self.create_publisher(String, "/gb/arm_state", 10)
        self.create_subscription(Twist, "/gb/cmd_in", self.on_twist, 20)
        self.create_subscription(String, "/gb/motion_req", self.on_req, 20)
        self.create_timer(1.0, self.tick)
        self.last_stop = 0.0; self.blocked = 0
        self.get_logger().info(f"armguard up: flag {gb_arm.path()} armed={gb_arm.is_armed()}")

    def _stop(self, why):
        self.blocked += 1
        if time.time() - self.last_stop > 0.1:
            self.last_stop = time.time()
            self.req.publish(String(data=json.dumps({"op": "stop", "src": "armguard", "id": "arm%x" % int(time.time() * 1000)})))
            self.get_logger().warn(f"DISARMED: motion refused -> gate stop ({why})")
            gb_arm._log(f"backstop stop: {why}")

    def on_twist(self, m):
        if (m.linear.x or m.angular.z or m.linear.y) and not gb_arm.is_armed():
            self._stop(f"/gb/cmd_in lin={m.linear.x:+.2f} ang={m.angular.z:+.2f}")

    def on_req(self, m):
        try: d = json.loads(m.data)
        except Exception: return
        if d.get("src") == "armguard" or d.get("op") == "stop": return
        if (d.get("lin") or d.get("ang") or d.get("op") not in (None, "stop", "status", "noop", "ping")) and not gb_arm.is_armed():
            self._stop(f"/gb/motion_req op={d.get('op')} src={d.get('src')}")

    def tick(self):
        if os.path.exists(gb_arm.path()) and gb_arm._latched():
            gb_arm.disarm("ESTOP/MAINT latched", actor="safety")
        s = gb_arm.state(); s["blocked"] = self.blocked; s["t"] = time.time()
        self.st.publish(String(data=json.dumps(s)))

def main():
    rclpy.init(); n = ArmGuard()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass

if __name__ == "__main__":
    main()
