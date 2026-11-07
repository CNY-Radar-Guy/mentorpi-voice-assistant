"""Controller launch with the motor subscription moved behind admission.

The vendor node subscribes to ~/set_motor. This file remaps that name to
/gb/set_motor_admitted. gb-actuation must be running or the board receives
nothing. Direct serial writes are not remapped. Do not use this file to
clear MAINT or to stop the watchdog.
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    imu_frame = LaunchConfiguration("imu_frame", default="imu_link")
    return LaunchDescription([
        DeclareLaunchArgument("imu_frame", default_value=imu_frame),
        Node(
            package="ros_robot_controller",
            executable="ros_robot_controller",
            output="screen",
            parameters=[{"imu_frame": imu_frame}],
            remappings=[("set_motor", "/gb/set_motor_admitted")],
        ),
    ])
