"""Not a safety boundary. Do not launch this file.

The earlier draft remapped ~/set_motor to only /gb/set_motor_admitted.
That drops the watchdog's direct zero on /ros_robot_controller/set_motor,
and any ROS node can publish the admitted topic. GB_CONTROLLER_REMAPPED
is not proof that a callback checked anything.

A later supervised controller change has to keep the raw subscription for
zeros and recheck admitted commands inside set_motor_state. This file
raises instead of installing the unsafe remap.
"""
UNSAFE_REMAP = ("set_motor", "/gb/set_motor_admitted")
RAW_ZERO_TOPIC = "/ros_robot_controller/set_motor"


def generate_launch_description():
    raise RuntimeError(
        "refusing to remap set_motor onto /gb/set_motor_admitted; "
        "that drops watchdog zeros and any node can publish the public topic"
    )
