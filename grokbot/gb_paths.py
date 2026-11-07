"""Canonical runtime paths. Code may live in a release directory; data does not.

GB_DATA, GB_DIR, GB_MODELS, and GB_SHARED_ESTOP are read here. A module that
builds a path from dirname(__file__) would write stops into the release tree,
which the gate never copies.
"""
import os


def data_dir():
    explicit = os.environ.get("GB_DATA")
    if explicit:
        return os.path.expanduser(explicit)
    root = os.environ.get("GB_DIR")
    if root:
        return os.path.join(os.path.expanduser(root), "data")
    return "/home/pi/grokbot/data"


def models_dir():
    explicit = os.environ.get("GB_MODELS")
    if explicit:
        return os.path.expanduser(explicit)
    return "/home/pi/grokbot/models"


def host_estop():
    explicit = os.environ.get("GB_ESTOP_HOST")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.join(data_dir(), "ESTOP")


def shared_estop():
    """Host path of the bind the container reads as /home/ubuntu/shared/gb_safety/ESTOP."""
    return os.path.expanduser(os.environ.get("GB_SHARED_ESTOP") or "/home/pi/docker/tmp/gb_safety/ESTOP")


def speaking_flag():
    return os.path.join(data_dir(), "speaking.json")


def shared_dir():
    """Directory both the host voice and the container tracker use for state.

    On the host this is the directory of the shared ESTOP file. In the container
    the launcher sets GB_SHARED_DIR to /home/ubuntu/shared/gb_safety. Those are
    the same bind.
    """
    explicit = os.environ.get("GB_SHARED_DIR")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.dirname(shared_estop())


def demand_path():
    explicit = os.environ.get("GB_DEMAND_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.join(shared_dir(), "demand.json")


def extrinsics_path():
    """Read-only camera-to-lidar calibration. This tree never writes it."""
    explicit = os.environ.get("GB_EXTRINSICS_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.join(shared_dir(), "lidar_extrinsics.json")


def depth_scale_record_path():
    """Logged millimetre proof. Absent until the runtime writes the real samples."""
    explicit = os.environ.get("GB_DEPTH_SCALE_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.join(shared_dir(), "depth_scale_record.json")


def person_track_path():
    explicit = os.environ.get("PERSON_TRACK_FILE")
    if explicit:
        return os.path.expanduser(explicit)
    return os.path.join(shared_dir(), "person_track.json")
