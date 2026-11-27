"""Point every test at a private directory before a live Pi path can be opened."""
import os, tempfile

def install():
    root = tempfile.mkdtemp(prefix="gbtest_")
    os.makedirs(os.path.join(root, "data"), exist_ok=True)
    os.environ["GB_DATA"] = os.path.join(root, "data")
    os.environ["GB_SHARED_DIR"] = root
    os.environ["GB_DEMAND_FILE"] = os.path.join(root, "demand.json")
    os.environ["GB_SHARED_ESTOP"] = os.path.join(root, "ESTOP")
    os.environ["GB_MISSION_STATUS"] = os.path.join(root, "mission.json")
    os.environ["GB_HEALTH_FILE"] = os.path.join(root, "health.json")
    os.environ["HEALTH_FILE"] = os.environ["GB_HEALTH_FILE"]
    os.environ["WD_STATE_HOST"] = os.path.join(root, "watchdog_state.json")
    os.environ["GUARD_CAPS"] = os.path.join(root, "caps.json")
    os.environ["GB_DEPTH_SCALE_FILE"] = os.path.join(root, "missing-depth-scale.json")
    os.environ["GB_TEACH_DB"] = os.path.join(root, "teach.db")
    os.environ["GB_INTENT_FILE"] = os.path.join(root, "intent_status.json")
    return root
