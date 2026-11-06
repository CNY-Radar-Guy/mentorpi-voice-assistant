#!/usr/bin/env python3
"""8080 coexistence decision. Nothing here binds a port or restarts bringup.

Live facts, 2026-11-06 evening, from the parent (not guessed):
  * web_video_server is started by: ros2 launch bringup bringup.launch.py
  * GET /snapshot?topic=/ascamera/camera_publisher/rgb0/image
    returned HTTP 200, a JPEG, 73327 bytes, 640x400
  * the stock root page links stream_viewer and snapshot for depth, rgb, ir,
    and hand_trajectory
  * the stock server has no proxy and no redirect
  * port 8081 was free

This repository does not contain bringup.launch.py. The ROS parameter on
web_video_server itself is `port`. Whether bringup exposes that as a launch
argument that can change the video server without restarting the lidar,
camera, and controller drivers is not in this tree, so this plan does not
invent an argument name and does not apply a change.

An overlay on 8081 does not fulfill a phone page on 8080.
"""

LAUNCH = "ros2 launch bringup bringup.launch.py"
SNAPSHOT_TOPIC = "/ascamera/camera_publisher/rgb0/image"
SNAPSHOT_URL = "http://127.0.0.1:8080/snapshot?topic=" + SNAPSHOT_TOPIC
ROS_PARAM = "port"

REMAINING = (
    "Parent reads bringup.launch.py inside the MentorPi image and names a launch argument that sets only the web_video_server ROS parameter `port`.",
    "That argument must move the video server to an internal port such as 8082 without restarting the lidar, camera, or controller drivers. Restarting all of `ros2 launch bringup bringup.launch.py` is not an acceptable migration.",
    "This repository does not contain bringup.launch.py, so no argument name is invented here and no launch file is edited.",
    "After a reviewed move, GET http://127.0.0.1:8082/snapshot?topic=/ascamera/camera_publisher/rgb0/image must return a JPEG before anything binds 8080.",
    "Only then may gb-front bind 8080 and proxy /snapshot?topic= to that upstream and proxy the phone root to the 8081 overlay. Until that review, 8080 stays unmet.",
    "Port 8081 is free. Starting the overlay there leaves the stock server on 8080 and does not make the phone root ready.",
)

INVERSE_ROLLBACK = (
    "Stop and disable gb-front and gb-stream only. Do not docker-kill MentorPi and do not stop the lidar, camera, or controller drivers.",
    "Restore the bringup launch file from the backup the parent took before any port edit. If no edit was made, there is nothing to restore.",
    "Return web_video_server to port 8080 with the same reviewed argument. Do not restart the rest of bringup to do it.",
    "Confirm GET http://127.0.0.1:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image returns a JPEG (FFD8). The last confirmed response was HTTP 200, 73327 bytes, 640x400.",
    "Confirm gb-front is not bound to 8080 and that the stock root still serves its stream_viewer and snapshot links.",
    "Do not delete data/ESTOP. This rollback does not clear a stop latch and does not start a follow.",
)


def migration_decision(launch_arg_reviewed=False, drivers_preserved=False, jpeg_on_8082=False):
    """Always reports the phone root unmet. A reviewed argument is not an applied move."""
    reviewed = bool(launch_arg_reviewed and drivers_preserved and jpeg_on_8082)
    return {
        "phone_url_8080": False,
        "phone_ready": False,
        "unmet_8080": True,
        "server_moved": False,
        "overlay_8081_fulfills_8080": False,
        "apply": False,
        "reviewed_argument_present": reviewed,
        "launch": LAUNCH,
        "ros_parameter": ROS_PARAM,
        "snapshot_url": SNAPSHOT_URL,
        "remaining": list(REMAINING),
        "inverse_rollback": list(INVERSE_ROLLBACK),
    }


def main(argv):
    del argv
    decision = migration_decision()
    print("8080 unmet. web_video_server was not moved. The phone page is not ready.")
    print("launch: " + decision["launch"])
    print("stock snapshot still required: " + decision["snapshot_url"])
    print("remaining:")
    for line in decision["remaining"]:
        print(" - " + line)
    print("inverse rollback, if a reviewed move is later applied:")
    for line in decision["inverse_rollback"]:
        print(" - " + line)
    return 0


if __name__ == "__main__":
    sys_exit = __import__("sys").exit
    sys_exit(main(__import__("sys").argv))
