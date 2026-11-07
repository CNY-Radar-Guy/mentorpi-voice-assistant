#!/usr/bin/env python3
"""ROS admission node. Importing this file does not load the ROS client.

The host interpreter has no rclpy and no ros_robot_controller_msgs. systemd
must start this through gb_safety_run.sh, which runs it in the container
with rosrun.sh. A flat /home/pi/grokbot path is not a release.

lease["current"] is loaded from the lease file on every motor message.
An active explore lease does not by itself admit a raw command: the gate
envelope has to name the same id and the rps have to match that twist.

/gb/set_motor_admitted is a public topic. Any node can publish it.
GB_CONTROLLER_REMAPPED is not read and is not proof. The controller callback
has to recheck, and it has to keep the raw set_motor subscription so the
watchdog can still write zeros. This process dying cannot zero a board
that kept the last speed; that timeout lives in the controller hook.
"""
import os
import time

import gb_actuation
import gb_intent
import gb_motor_cap as cap


def bind_lease(slot, path=None):
    """Assign the lease the voice process wrote. A missing file stays None."""
    slot["current"] = gb_intent.read_lease(path)
    return slot["current"]


def on_motor_pairs(pairs, now, slot, maint=False):
    bind_lease(slot)
    envelope = gb_intent.read_envelope(now)
    claim = gb_intent.claim_from(slot.get("current"), envelope, now)
    speeds, decision = gb_actuation.admit(
        pairs, now, slot.get("current"), maint=maint, claim=claim)
    decision["enforced_on_robot"] = False
    decision["public_topic_bypass"] = True
    decision["gb_controller_remapped_is_proof"] = False
    decision["lease_loaded"] = bool(slot.get("current"))
    decision["lease_id"] = (slot.get("current") or {}).get("id")
    decision["container_maint"] = os.environ.get("GB_MAINT_FILE") or ""
    decision["hardware_top_speed_verified"] = False
    decision["serial_bypass"] = True
    return speeds, decision


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from ros_robot_controller_msgs.msg import MotorState, MotorsState

    rclpy.init()
    node = Node("gb_actuation")
    # Publishing here is not enforcement. The controller must recheck.
    out = node.create_publisher(MotorsState, gb_actuation.ADMITTED_TOPIC, 10)
    slot = {"current": None}

    def on_motor(msg):
        pairs = [(item.id, item.rps) for item in msg.data]
        speeds, decision = on_motor_pairs(
            pairs, time.monotonic(), slot, maint=gb_actuation.maint_held())
        try:
            gb_intent.write_status({
                "actuation": decision,
                "repair": decision.get("repair"),
                "interlock": False,
                "half_rps": cap.half_rps(),
                "enforced_on_robot": False,
            })
        except OSError:
            pass
        admitted = MotorsState()
        admitted.data = [MotorState(id=i, rps=float(r)) for i, r in speeds]
        out.publish(admitted)

    node.create_subscription(
        MotorsState, gb_actuation.RAW_MOTOR, on_motor, QoSProfile(depth=20))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
