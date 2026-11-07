#!/usr/bin/env python3
"""Partial intent observer. Subscribes and writes a report. Publishes nothing.

This is not a motor interlock. lidar_app, line_following, object_tracking,
hand_gesture, odom_publisher, rosbridge, and gb_guard can still command the
controller. Do not start this unit by clearing MAINT.
"""
import json, os, time

import gb_intent


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    rclpy.init()
    node = Node("gb_intent")
    seen = []

    def on_twist(msg, topic):
        nz = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 1e-6
        seen.append({"topic": topic, "node": None, "nz": nz, "intent_id": None})
        if len(seen) > 200:
            del seen[:-200]

    def on_req(msg):
        try:
            body = json.loads(msg.data)
        except ValueError:
            body = {}
        seen.append({
            "topic": "/gb/motion_req",
            "node": body.get("src"),
            "nz": body.get("op") not in (None, "stop", "noop"),
            "intent_id": body.get("intent_id"),
        })

    qos = QoSProfile(depth=20)
    for topic in ("/controller/cmd_vel", "/cmd_vel", "/app/cmd_vel", "/gb/cmd_in"):
        node.create_subscription(Twist, topic, lambda m, t=topic: on_twist(m, t), qos)
    node.create_subscription(String, "/gb/motion_req", on_req, 10)

    def tick():
        report = gb_intent.observe(seen, [])
        report["pid"] = os.getpid()
        gb_intent.write_status(report)

    node.create_timer(2.0, tick)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
