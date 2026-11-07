"""Metric person range and pointing geometry. No ROS and no invented targets.

A box becomes a range only with a depth scale, camera intrinsics, and enough
valid pixels. A name, including "John", never selects a person. A pointing
target is the pixel where the wrist-to-index ray leaves the hand, sampled in
the depth image. Missing landmarks produce no target.
"""
import math

MIN_PIXELS = 20
PERSON_CONF = 0.75


def choose_person(people, point=None):
    """people: [{conf, box:[x1,y1,x2,y2]}]. Returns (person, ambiguous).

    One confident person is enough. Several require a pointing pixel inside
    exactly one box. Identity text is not an input.
    """
    usable = []
    for person in people or []:
        try:
            conf = float(person.get("conf") or 0)
            box = person.get("box")
            if conf >= PERSON_CONF and box and len(box) == 4:
                usable.append(person)
        except (TypeError, ValueError):
            continue
    if point is not None and usable:
        hits = [p for p in usable if _inside(point, p["box"])]
        if len(hits) == 1:
            return hits[0], False
        return None, True
    if len(usable) == 1:
        return usable[0], False
    if len(usable) > 1:
        return None, True
    return None, False


def _inside(point, box):
    try:
        u, v = float(point[0]), float(point[1])
        x1, y1, x2, y2 = [float(x) for x in box]
    except (TypeError, ValueError, IndexError):
        return False
    return x1 <= u <= x2 and y1 <= v <= y2


def box_range(depth, box, scale_m, fx, fy, cx, cy, min_pixels=MIN_PIXELS):
    """Median metres inside the box. Returns None when the unit or the sample count fails."""
    try:
        scale = float(scale_m)
        fx, fy, cx, cy = float(fx), float(fy), float(cx), float(cy)
    except (TypeError, ValueError):
        return None
    if scale <= 0 or fx <= 0 or fy <= 0 or depth is None:
        return None
    try:
        height = len(depth)
        width = len(depth[0]) if height else 0
        x1, y1, x2, y2 = [int(float(v)) for v in box]
    except (TypeError, ValueError, IndexError):
        return None
    x1 = max(0, min(width - 1, x1)); x2 = max(0, min(width, x2))
    y1 = max(0, min(height - 1, y1)); y2 = max(0, min(height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    vals = []
    for row in depth[y1:y2]:
        for raw in row[x1:x2]:
            try:
                metres = float(raw) * scale
            except (TypeError, ValueError):
                continue
            if 0.15 <= metres <= 6.0:
                vals.append(metres)
    if len(vals) < int(min_pixels):
        return None
    vals.sort()
    dist = vals[len(vals) // 2]
    u = (x1 + x2) / 2.0
    bearing = math.atan2((u - cx) * dist / fx, dist)
    return {"depth_m": dist, "valid_pixels": len(vals), "bearing": bearing}


def pointing_pixel(landmarks, width, height):
    """Extend wrist (0) through index tip (8). Landmarks are normalized 0..1."""
    try:
        wrist = landmarks[0]
        tip = landmarks[8]
        x0, y0 = float(wrist["x"]) * width, float(wrist["y"]) * height
        x1, y1 = float(tip["x"]) * width, float(tip["y"]) * height
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy)
    if norm < 5.0:
        return None
    # One hand-length past the fingertip, then clipped to the image.
    u = x1 + dx
    v = y1 + dy
    if u < 0 or v < 0 or u >= width or v >= height:
        u = min(max(u, 0), width - 1)
        v = min(max(v, 0), height - 1)
    return u, v


def range_at_pixel(depth, u, v, scale_m, fx, fy, cx, cy, window=4):
    box = [u - window, v - window, u + window, v + window]
    return box_range(depth, box, scale_m, fx, fy, cx, cy, min_pixels=8)


def quat_rotate(q, vector):
    x, y, z, w = [float(v) for v in q]
    vx, vy, vz = [float(v) for v in vector]
    ix = w * vx + y * vz - z * vy
    iy = w * vy + z * vx - x * vz
    iz = w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz
    return (
        ix * w + iw * -x + iy * -z - iz * -y,
        iy * w + iw * -y + iz * -x - ix * -z,
        iz * w + iw * -z + ix * -y - iy * -x,
    )


def optical_to_point(u, v, metres, fx, fy, cx, cy):
    return ((float(u) - float(cx)) * metres / float(fx), (float(v) - float(cy)) * metres / float(fy), float(metres))


def apply_tf(point, rotation_xyzw, translation):
    rotated = quat_rotate(rotation_xyzw, point)
    return tuple(rotated[i] + float(translation[i]) for i in range(3))


def track_record(now, depth_m, bearing, conf, unit, valid_pixels, source, map_xy=None, ambiguous=False):
    """The file the follow loop reads. No field is filled in from a name or a guess."""
    if ambiguous or depth_m is None or unit not in ("crosscheck_mm", "crosscheck_m", "32FC1_metres", "lidar"):
        return {"t": float(now), "person": {"t": float(now), "ambiguous": True}}
    person = {
        "t": float(now),
        "depth_m": float(depth_m),
        "bearing": float(bearing or 0.0),
        "conf": float(conf),
        "source": source,
        "unit": unit,
        "valid_pixels": int(valid_pixels),
        "fresh": True,
        "ambiguous": False,
        "provenance": "yolov8n+depth",
    }
    if map_xy and len(map_xy) >= 2:
        person["x"] = float(map_xy[0])
        person["y"] = float(map_xy[1])
    return {"t": float(now), "person": person}


def sighting_record(now, people, range_obs=None):
    """Detection without inventing a range.

    people are {conf, box}. range_obs may carry a depth measurement only when
    its unit is already validated, or a lidar range only when extrinsics_validated
    is true. Otherwise the person is recorded with a reason and no distance.
    """
    now = float(now)
    people = [p for p in (people or []) if isinstance(p, dict)]
    if not people:
        return {"t": now, "detect_ok": True, "person": None, "reason": "no_person"}
    if len(people) > 1:
        return {"t": now, "detect_ok": True, "person": {"t": now, "ambiguous": True,
                "conf": max(float(p.get("conf") or 0) for p in people)}}
    one = people[0]
    person = {"t": now, "conf": float(one.get("conf") or 0), "box": list(one.get("box") or []),
              "ambiguous": False, "name": None}
    obs = range_obs or {}
    source = obs.get("source")
    if source == "depth" and obs.get("depth_m") is not None and obs.get("unit") in (
            "crosscheck_mm", "crosscheck_m", "32FC1_metres"):
        person.update(depth_m=float(obs["depth_m"]), source="depth", unit=obs["unit"],
                      bearing=float(obs.get("bearing") or 0.0))
    elif source == "lidar" and obs.get("extrinsics_validated") is True and obs.get("depth_m") is not None:
        person.update(depth_m=float(obs["depth_m"]), source="lidar", unit="lidar",
                      bearing=float(obs.get("bearing") or 0.0))
        if obs.get("clearance_m") is not None:
            person["clearance_m"] = float(obs["clearance_m"])
    else:
        person["reason"] = "depth_unit_unconfirmed"
        person["lidar_reason"] = "extrinsics_not_validated"
    return {"t": now, "detect_ok": True, "person": person}
