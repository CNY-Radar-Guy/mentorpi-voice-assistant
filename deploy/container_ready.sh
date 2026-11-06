#!/bin/bash
# Readiness commands for the parent on the Pi. This script does not restart
# services and does not publish velocity. When docker or the container is
# absent it prints the commands and exits 0 so a workstation dry run is not a
# false failure.
set -uo pipefail
C=${WALLY_CONTAINER:-MentorPi}
cat << 'EOF'
Parent checks, inside MentorPi, after the release is staged. No motion.
1. source /opt/ros/humble/setup.bash
2. python3 -c "import rclpy; from nav2_msgs.action import NavigateToPose; from sensor_msgs.msg import Image, CameraInfo, LaserScan"
3. test -f /home/ubuntu/shared/gb_safety/yolov8n.onnx
4. python3 -c "import mediapipe" || echo "hand landmarker not installed; pointing stays unavailable"
5. ros2 param list /web_video_server
6. curl -fsS -o /dev/null -w "%{http_code}\n" "http://127.0.0.1:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image"
Hand model, only if pointing is wanted, from the container:
  python3 -m pip install --user 'mediapipe==0.10.18'
  mkdir -p /home/ubuntu/shared/gb_safety/models
  curl -fsSL -o /home/ubuntu/shared/gb_safety/models/hand_landmarker.task \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
Do not start a second web_video_server. Port 8080 stays with the stock node.
EOF
if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not on this machine. The checks above were not executed."
  exit 0
fi
if [ "$(docker inspect -f '{{.State.Running}}' "$C" 2>/dev/null || true)" != "true" ]; then
  echo "container $C is not running here. The checks above were not executed."
  exit 0
fi
docker exec -u ubuntu "$C" bash -lc 'source /opt/ros/humble/setup.bash; python3 -c "import rclpy; from nav2_msgs.action import NavigateToPose"'
echo "container imports succeeded"
exit 0
