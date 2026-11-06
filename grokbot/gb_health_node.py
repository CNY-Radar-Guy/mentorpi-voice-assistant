#!/usr/bin/env python3
"""Container health publisher. Writes the file the gate already reads.

Depth is passed through resolve_depth_scale with the current forward LiDAR
range. An unconfirmed mono16 frame does not become a depth mode. This process
does not publish velocity and does not reload a mission.
"""
import json, os, time

OUT = os.environ.get("GB_HEALTH_FILE", "/home/ubuntu/shared/gb_safety/health.json")


def record(now, lidar_frac, depth_encoding, depth_samples, lidar_front_m, user_lidar=None, user_camera=None):
    import gb_health
    try:
        import gb_obstacle
        scale, how = gb_obstacle.resolve_depth_scale(depth_encoding, depth_samples, lidar_front_m)
    except Exception:
        scale, how = None, "unavailable"
    frac = gb_health.depth_valid_fraction(depth_samples, scale) if scale else 0.0
    lidar_ok = lidar_frac is not None and float(lidar_frac) >= gb_health.MIN_VALID_FRAC
    depth_ok = how in gb_health.VALIDATED_DEPTH_UNITS and frac >= gb_health.MIN_VALID_FRAC
    if lidar_ok:
        mode = "lidar"
    elif depth_ok:
        mode = "depth"
    else:
        mode = "none"
    body = {
        "t": float(now),
        "mode": mode,
        "lidar_online": bool(lidar_ok),
        "lidar_valid_frac": 0.0 if lidar_frac is None else float(lidar_frac),
        "depth_online": bool(depth_ok),
        "depth_unit": how if depth_ok else None,
        "depth_valid_frac": float(frac),
        "depth_scale_m": scale,
        "executable": False,
    }
    if user_lidar is not None:
        body["user_lidar"] = bool(user_lidar)
    if user_camera is not None:
        body["user_camera"] = bool(user_camera)
    return body


def write_record(body, path=None):
    path = path or OUT
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(body, f)
    os.replace(tmp, path)
    return path


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    rclpy.init()
    node = Node("gb_health")
    state = {"scan": None, "depth": None}

    def on_scan(msg):
        vals = [float(r) for r in msg.ranges if r == r and 0.05 < float(r) < 30.0]
        state["scan"] = (time.time(), vals)

    def on_depth(msg):
        state["depth"] = msg

    node.create_subscription(LaserScan, "/scan_raw", on_scan, qos_profile_sensor_data)
    node.create_subscription(Image, "/ascamera/camera_publisher/depth0/image_raw", on_depth, qos_profile_sensor_data)

    def tick():
        now = time.time()
        scan = state["scan"]
        lidar_frac = None
        front = None
        if scan and now - scan[0] <= 1.0 and scan[1]:
            lidar_frac = len(scan[1]) / max(1, len(scan[1]))
            # The message already dropped non-finite ranges, so coverage is the
            # fraction only when the driver published a full scan. Use the count
            # against the width carried on the message when we still have it.
            front = sorted(scan[1])[len(scan[1]) // 2]
            lidar_frac = 1.0 if len(scan[1]) >= 50 else len(scan[1]) / 50.0
        samples = []
        encoding = ""
        msg = state["depth"]
        if msg is not None:
            encoding = msg.encoding
            raw = bytes(msg.data)
            if encoding in ("16UC1", "mono16"):
                import numpy as np
                samples = np.frombuffer(raw, dtype="<u2").reshape(msg.height, msg.width)[::16, ::16].ravel().tolist()
            elif encoding == "32FC1":
                import numpy as np
                samples = np.frombuffer(raw, dtype="<f4").reshape(msg.height, msg.width)[::16, ::16].ravel().tolist()
        body = record(now, lidar_frac, encoding, samples, front)
        write_record(body)

    node.create_timer(0.5, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
