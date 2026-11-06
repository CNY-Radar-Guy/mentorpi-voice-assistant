#!/usr/bin/env python3
"""Publish zero Twist on /controller/cmd_vel for a few seconds, then exit.

Run inside the MentorPi container with the ROS environment sourced.
This process never publishes a non-zero command and never deletes an ESTOP file.
"""
import sys, time

TOPIC = "/controller/cmd_vel"


def seconds_arg(argv):
    seconds = 4.0
    args = argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--seconds" and i + 1 < len(args):
            seconds = float(args[i + 1])
            i += 2
        else:
            i += 1
    if seconds <= 0 or seconds > 40:
        raise SystemExit("zero hold must be between 0 and 40 seconds")
    return seconds


def main(argv):
    duration = seconds_arg(argv)
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()
    node = rclpy.create_node("gb_zero_hold")
    pub = node.create_publisher(Twist, TOPIC, 10)
    end = time.time() + duration
    msg = Twist()
    try:
        while time.time() < end:
            pub.publish(msg)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
