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
