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

The live bringup.launch.py (source and install identical, 25 Jun 2025) starts
the server with no arguments and no parameters:

    Node(package='web_video_server', executable='web_video_server', output='screen')

Package 1.0.0 main() creates node web_video_server and a second node
_web_video_server. The constructor calls get_parameter("port") once on that
private node and, when the parameter is unset, uses 8080 and address 0.0.0.0,
then starts the socket. There is no declare_parameter and no parameter
callback. ros2 param get /web_video_server port is unset. ros2 param set does
not change the bind. A second process would also default to 8080, so this
plan does not start one. Snapshots stay on the existing server.

An overlay on 8081 does not fulfill a phone page on 8080.
"""

LAUNCH = "ros2 launch bringup bringup.launch.py"
SNAPSHOT_TOPIC = "/ascamera/camera_publisher/rgb0/image"
SNAPSHOT_URL = "http://127.0.0.1:8080/snapshot?topic=" + SNAPSHOT_TOPIC
ROS_PARAM = "port"
PORT_MUTABLE = False
PROBE = (
    "ros2 param list /web_video_server",
    "curl -sI http://127.0.0.1:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image",
)

REMAINING = (
    "The stock node is already bound to 8080. ros2 param set does not move it, and this plan does not restart bringup.",
    "A second web_video_server would also use port 8080. Do not start one. The read-only probes are ros2 param list /web_video_server and a snapshot GET that does not bind a port.",
    "Snapshots stay on http://127.0.0.1:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image. That URL already returned a JPEG.",
    "8080 stays unmet for the phone page. gb-front and gb-stream stay inactive. An overlay on 8081 does not fulfill the phone root.",
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
        "port_mutable": PORT_MUTABLE,
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
