"""Metric person range and pointing geometry. No ROS and no invented targets.

A box becomes a range only with a depth scale, camera intrinsics, and enough
valid pixels. A name, including "John", never selects a person. A pointing
target is the pixel where the wrist-to-index ray leaves the hand, sampled in
the depth image. Missing landmarks produce no target.
"""
import json, math, os

MIN_PIXELS = 20
PERSON_CONF = 0.75
OBSERVATION_FRESH_S = 0.5
CAPTURE_FUTURE_S = 0.05
CAPTURE_DOMAIN_S = 60.0
PAIR_MAX_S = 0.05
MM_PER_COUNT = 0.001
FEET_M = 2.1336
FEET_CLOSER_THAN_TORSO_M = 0.0762
TORSO_REFERENCE_M = FEET_M + FEET_CLOSER_THAN_TORSO_M
FEW_PERCENT = 0.03


def keep_latest(previous, arrived):
    """The acquisition slot holds one frame. A new arrival replaces it."""
    return arrived if arrived is not None else previous


def capture_time(stamp_s, receive_t, future_s=CAPTURE_FUTURE_S, domain_s=CAPTURE_DOMAIN_S):
    """Wall-clock capture from a ROS header. Receipt time is not a substitute.

    Zero, pre-unix, future, and other-clock stamps fail closed.
    A stamp 0.35 s behind receipt is still this clock.
    """
    try:
        stamp = float(stamp_s)
        receive = float(receive_t)
    except (TypeError, ValueError):
        return None, "capture_stamp_missing"
    if stamp != stamp or receive != receive or stamp <= 0 or stamp < 1e9:
        return None, "capture_stamp_zero"
    age = receive - stamp
    if age < -float(future_s):
        return None, "capture_stamp_future"
    if age > float(domain_s):
        return None, "capture_stamp_clock"
    return stamp, "ok"


def frames_paired(rgb_t, depth_t, max_delta=PAIR_MAX_S):
    """True when two capture stamps are the same exposure, within max_delta."""
    try:
        return abs(float(rgb_t) - float(depth_t)) <= float(max_delta)
    except (TypeError, ValueError):
        return False


def fresh_capture(capture_t, now, limit=OBSERVATION_FRESH_S):
    """Age of the capture. A later write time cannot make it fresh."""
    try:
        age = float(now) - float(capture_t)
    except (TypeError, ValueError):
        return False
    return 0 <= age <= float(limit)


def stamp_observation(body, capture_t, receive_t, write_t, infer_s=None):
    """Publish capture as t. write_t and receive_t stay diagnostic fields."""
    body = dict(body or {})
    person = dict(body["person"]) if isinstance(body.get("person"), dict) else None
    try:
        receive = float(receive_t)
        written = float(write_t)
    except (TypeError, ValueError):
        receive, written = None, None
    body["time_base"] = "capture"
    body["receive_t"] = receive
    body["write_t"] = written
    cap = None
    try:
        if capture_t is not None:
            cap = float(capture_t)
    except (TypeError, ValueError):
        cap = None
    if cap is None:
        body.pop("t", None)
        body["capture_ok"] = False
        if person is not None:
            person.pop("t", None)
            person["time_base"] = "capture"
            body["person"] = person
        return body
    body["t"] = cap
    body["capture_ok"] = True
    if infer_s is not None:
        try:
            body["infer_s"] = float(infer_s)
        except (TypeError, ValueError):
            pass
    body["latency_s"] = None if written is None else written - cap
    if person is not None:
        person["t"] = cap
        person["time_base"] = "capture"
        person["receive_t"] = receive
        person["write_t"] = written
        if body.get("latency_s") is not None:
            person["latency_s"] = body["latency_s"]
        body["person"] = person
    return body


def _rel_error(raw, reference):
    metres = float(raw) * MM_PER_COUNT
    return (metres - float(reference)) / float(reference)


def assess_depth_record(body):
    """Nominal 0.001 m per count, or a reason. The scale is not fitted to the tape.

    The authorised reference is 7 ft to the feet plus 3 in (0.0762 m) because
    the feet were closer than the torso sample: 2.2098 m. LiDAR extrinsics are
    not required. A health flag or a person-track file is not this record.
    """
    if not isinstance(body, dict):
        return {"ok": False, "reason": "no_scale_record", "errors": []}
    if "fit_factor" in body or "multiplier" in body:
        return {"ok": False, "reason": "fit_factor_not_allowed", "errors": []}
    try:
        if float(body.get("scale_m_per_count")) != MM_PER_COUNT:
            return {"ok": False, "reason": "scale_not_nominal_mm", "errors": []}
    except (TypeError, ValueError):
        return {"ok": False, "reason": "scale_not_nominal_mm", "errors": []}
    prov = body.get("provenance") if isinstance(body.get("provenance"), dict) else {}
    try:
        tape_ok = prov.get("tape") == "7ft_to_feet"
        offset = float(prov.get("feet_closer_than_torso_m"))
        reference = float(prov.get("reference_m"))
    except (TypeError, ValueError):
        return {"ok": False, "reason": "provenance_incomplete", "errors": []}
    if not tape_ok or prov.get("landmark") != "torso":
        return {"ok": False, "reason": "provenance_incomplete", "errors": []}
    if abs(offset - FEET_CLOSER_THAN_TORSO_M) > 1e-9:
        return {"ok": False, "reason": "body_offset_not_3in", "errors": []}
    if abs(reference - TORSO_REFERENCE_M) > 1e-9 or abs((FEET_M + offset) - reference) > 1e-9:
        return {"ok": False, "reason": "reference_not_corrected", "errors": []}
    cam = body.get("camera") if isinstance(body.get("camera"), dict) else {}
    encoding = str(cam.get("encoding") or "")
    if cam.get("align_mode") is not True or cam.get("depth_correction") is not True:
        return {"ok": False, "reason": "alignment_not_logged", "errors": []}
    if encoding not in ("mono16", "16UC1"):
        return {"ok": False, "reason": "encoding_not_mono16", "errors": []}
    try:
        if int(cam.get("width")) != 640 or int(cam.get("height")) != 400:
            return {"ok": False, "reason": "size_not_640x400", "errors": []}
    except (TypeError, ValueError):
        return {"ok": False, "reason": "size_not_640x400", "errors": []}
    if cam.get("rgb_frame_id") not in (None, "rgb_camera_link"):
        return {"ok": False, "reason": "camera_identity", "errors": []}
    if cam.get("depth_frame_id") not in (None, "depth_camera_link"):
        return {"ok": False, "reason": "camera_identity", "errors": []}
    samples = body.get("samples")
    if not isinstance(samples, list) or len(samples) != 3:
        return {"ok": False, "reason": "need_three_paired_samples", "errors": []}
    errors = []
    captures = []
    for sample in samples:
        if not isinstance(sample, dict) or sample.get("people") != 1:
            return {"ok": False, "reason": "not_one_person", "errors": errors}
        try:
            conf = float(sample.get("conf"))
            raw = float(sample.get("raw_median"))
            rgb_t = float(sample.get("rgb_capture_t"))
            depth_t = float(sample.get("depth_capture_t"))
        except (TypeError, ValueError):
            return {"ok": False, "reason": "sample_incomplete", "errors": errors}
        if conf < PERSON_CONF:
            return {"ok": False, "reason": "confidence_below_0.75", "errors": errors}
        if raw <= 0 or rgb_t < 1e9 or depth_t < 1e9 or not frames_paired(rgb_t, depth_t):
            return {"ok": False, "reason": "sample_stamps_not_paired", "errors": errors}
        err = _rel_error(raw, reference)
        if abs(err) > FEW_PERCENT:
            return {"ok": False, "reason": "outside_few_percent", "errors": errors + [err]}
        if "error" in sample:
            try:
                if abs(float(sample["error"]) - err) > 1e-6:
                    return {"ok": False, "reason": "error_not_preserved", "errors": errors}
            except (TypeError, ValueError):
                return {"ok": False, "reason": "error_not_preserved", "errors": errors}
        errors.append(err)
        captures.append(rgb_t)
    if max(captures) - min(captures) > OBSERVATION_FRESH_S:
        return {"ok": False, "reason": "samples_not_one_set", "errors": errors}
    return {"ok": True, "scale": MM_PER_COUNT, "unit": "documented_mm", "reason": "documented_mm", "errors": errors}


def load_depth_record(path=None):
    if path is None:
        import gb_paths
        path = gb_paths.depth_scale_record_path()
    try:
        with open(path) as f:
            body = json.load(f)
    except (OSError, ValueError):
        return None
    return body if isinstance(body, dict) else None


def validated_depth_scale(encoding, path=None):
    """(0.001, documented_mm) only when the logged record passes. Otherwise None."""
    if str(encoding or "") not in ("mono16", "16UC1"):
        return None
    found = assess_depth_record(load_depth_record(path))
    if not found.get("ok"):
        return None
    return found["scale"], found["unit"]


def json_number(value):
    """Python int or float. A numpy float32 is not accepted by json.dump."""
    item = getattr(value, "item", None)
    if callable(item) and not isinstance(value, (str, bytes, dict, list, tuple)):
        value = item()
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return int(value)
    return float(value)


def people_from_dets(dets):
    """Model rows (label, conf, x1, y1, x2, y2) as plain numbers. No range is added."""
    out = []
    for row in dets or []:
        try:
            out.append({
                "label": row[0],
                "conf": json_number(row[1]),
                "box": [json_number(row[i]) for i in (2, 3, 4, 5)],
            })
        except (TypeError, ValueError, IndexError):
            continue
    return out


def dump_json(path, body):
    """Atomic JSON write. A numpy scalar cannot leave the previous file in place."""
    def _default(obj):
        item = getattr(obj, "item", None)
        if callable(item):
            return json_number(obj)
        raise TypeError("not JSON serializable: %s" % type(obj).__name__)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(body, f, default=_default)
    os.replace(tmp, path)


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
    if ambiguous or depth_m is None or unit not in ("crosscheck_mm", "crosscheck_m", "32FC1_metres", "lidar", "documented_mm"):
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


def bearing_from_box(box, fx, cx):
    """Camera-frame yaw of the box centre. None without intrinsics."""
    import math
    try:
        u = (float(box[0]) + float(box[2])) / 2.0
        focal = float(fx)
        if focal == 0:
            return None
        return math.atan2(u - float(cx), focal)
    except (TypeError, ValueError, IndexError):
        return None


def range_for_detection(box, fx, cx, beams, calibration, beam_receive_age, depth_obs=None):
    """Validated depth, otherwise a calibrated lidar beam at the box bearing.

    A missing calibration returns None. The forward-corridor minimum is not used.
    beam_receive_age is wall-clock age; a header stamp cannot make a stale scan fresh.
    """
    if isinstance(depth_obs, dict) and depth_obs.get("depth_m") is not None and depth_obs.get("unit") in (
            "crosscheck_mm", "crosscheck_m", "32FC1_metres"):
        return {"source": "depth", "depth_m": float(depth_obs["depth_m"]), "unit": depth_obs["unit"],
                "bearing": float(depth_obs.get("bearing") or 0.0), "extrinsics_validated": False}
    try:
        age = float(beam_receive_age)
    except (TypeError, ValueError):
        return None
    if age < -0.1 or age > 0.5:
        return None
    bearing = bearing_from_box(box, fx, cx)
    if bearing is None:
        return None
    import gb_obstacle
    rng = gb_obstacle.lidar_person_range(beams, bearing, calibration)
    if rng is None:
        return None
    return {"source": "lidar", "depth_m": float(rng), "unit": "lidar", "bearing": bearing,
            "extrinsics_validated": True, "clearance_m": float(rng)}


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
    try:
        conf = json_number(one.get("conf") or 0)
        box = [json_number(v) for v in list(one.get("box") or [])]
    except (TypeError, ValueError):
        return {"t": now, "detect_ok": True, "person": None, "reason": "detection_not_numeric"}
    person = {"t": now, "conf": conf, "box": box, "ambiguous": False, "name": None}
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
        if obs.get("scale_reason"):
            person["scale_reason"] = obs.get("scale_reason")
    return {"t": now, "detect_ok": True, "person": person}
