#!/usr/bin/env python3
"""Container health publisher. Writes the file the gate already reads.

Depth is passed through resolve_depth_scale with the current forward LiDAR
range. An unconfirmed mono16 frame does not become a depth mode. This process
does not publish velocity and does not reload a mission.
"""
import json, os, time

OUT = os.environ.get("GB_HEALTH_FILE", "/home/ubuntu/shared/gb_safety/health.json")


def record(now, lidar_frac, depth_encoding, depth_samples, lidar_front_m, user_lidar=None, user_camera=None,
           topic_online=None, nonzero_frac=None, forward_raw=None, tf_fresh=False, camera_ahead_m=None):
    import gb_health
    try:
        import gb_obstacle
        scale, how = gb_obstacle.resolve_depth_scale(
            depth_encoding, None if forward_raw is not None else depth_samples, lidar_front_m,
            forward_raw=forward_raw, camera_ahead_m=camera_ahead_m, tf_fresh=tf_fresh)
    except Exception:
        scale, how = None, "unavailable"
    frac = gb_health.depth_valid_fraction(depth_samples if forward_raw is None else [forward_raw], scale) if scale else 0.0
    lidar_ok = lidar_frac is not None and float(lidar_frac) >= gb_health.MIN_VALID_FRAC
    metric = how in gb_health.VALIDATED_DEPTH_UNITS and frac >= gb_health.MIN_VALID_FRAC
    topic = bool(depth_samples) if topic_online is None else bool(topic_online)
    if lidar_ok:
        mode = "lidar"
    elif metric:
        mode = "depth"
    else:
        mode = "none"
    try:
        nz = float(nonzero_frac) if nonzero_frac is not None else 0.0
    except (TypeError, ValueError):
        nz = 0.0
    body = {
        "t": float(now),
        "mode": mode,
        "lidar_online": bool(lidar_ok),
        "lidar_valid_frac": 0.0 if lidar_frac is None else float(lidar_frac),
        "depth_topic_online": topic,
        "depth_metric_valid": bool(metric),
        "depth_online": bool(metric),
        "depth_unit": how if metric else None,
        "depth_scale_reason": how,
        "depth_valid_frac": float(frac) if metric else 0.0,
        "depth_nonzero_frac": nz if topic else 0.0,
        "depth_scale_m": scale if metric else None,
        "tf_applied": bool(tf_fresh),
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
        import gb_obstacle
        vals = [float(r) for r in msg.ranges if r == r and 0.05 < float(r) < 30.0]
        forward = gb_obstacle.lidar_forward_m(msg.ranges, msg.angle_min, msg.angle_increment)
        state["scan"] = (time.time(), vals, forward)

    def on_depth(msg):
        state["depth"] = (time.time(), msg)

    node.create_subscription(LaserScan, "/scan_raw", on_scan, qos_profile_sensor_data)
    node.create_subscription(Image, "/ascamera/camera_publisher/depth0/image_raw", on_depth, qos_profile_sensor_data)

    def tick():
        now = time.time()
        scan = state["scan"]
        lidar_frac = None
        front = None
        front = None
        if scan and now - scan[0] <= 1.0 and scan[1]:
            lidar_frac = 1.0 if len(scan[1]) >= 50 else len(scan[1]) / 50.0
            front = scan[2]
        samples = []
        encoding = ""
        topic = False
        nonzero = 0.0
        forward_raw = None
        held = state["depth"]
        msg = held[1] if held and now - held[0] <= 1.5 else None
        if msg is not None:
            encoding = msg.encoding
            import gb_obstacle
            img = gb_obstacle.decode_depth(encoding, bytes(msg.data), msg.height, msg.width, msg.step, msg.is_bigendian)
            if img is not None:
                topic = True
                small = img[::16, ::16]
                nonzero = float((small > 0).mean()) if small.size else 0.0
                forward_raw = gb_obstacle.center_raw(img, (img.shape[1] - 1) / 2.0, (img.shape[0] - 1) / 2.0)
                samples = [forward_raw] if forward_raw is not None else []
        body = record(now, lidar_frac, encoding, samples, front, topic_online=topic, nonzero_frac=nonzero,
                      forward_raw=forward_raw, tf_fresh=False)
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
