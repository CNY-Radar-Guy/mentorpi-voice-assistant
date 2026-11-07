#!/usr/bin/env python3
"""Subscribe to every ROS set_motor writer and publish the admitted command.

The controller stops hearing the raw topic only after its subscription is
remapped to /gb/set_motor_admitted. Until that remap, this process does not
stand in front of the board. Direct /dev/rrc writes are never seen.
"""
import json, os, time

import gb_actuation
import gb_intent
import gb_motor_cap as cap


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from ros_robot_controller_msgs.msg import MotorState, MotorsState

    rclpy.init()
    node = Node("gb_actuation")
    out = node.create_publisher(MotorsState, "/gb/set_motor_admitted", 10)
    remapped = os.environ.get("GB_CONTROLLER_REMAPPED", "0") == "1"
    lease = {"current": None}

    def on_motor(msg):
        pairs = []
        for item in msg.data:
            pairs.append((item.id, item.rps))
        speeds, decision = gb_actuation.admit(
            pairs, time.time(), lease["current"], maint=gb_actuation.maint_held())
        decision["controller_remapped"] = remapped
        decision["enforced_on_robot"] = bool(remapped)
        decision["serial_bypass"] = True
        decision["hardware_top_speed_verified"] = False
        if decision.get("repair"):
            gb_intent.write_status({
                "actuation": decision,
                "repair": decision["repair"],
                "interlock": False,
                "half_rps": cap.half_rps(),
            })
        admitted = MotorsState()
        admitted.data = [MotorState(id=i, rps=float(r)) for i, r in speeds]
        if remapped:
            out.publish(admitted)

    node.create_subscription(MotorsState, "/ros_robot_controller/set_motor", on_motor, QoSProfile(depth=20))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
