"""Hand landmarks for pointing. The geometry lives in gb_metric.

MediaPipe is not imported at module import time. If it or the documented
task file is absent, landmarks() returns no points and the mission does not
invent a target. The parent installs them inside the container; this file
does not download anything.
"""
import os

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
INSTALL = (
    "python3 -m pip install --user 'mediapipe==0.10.18'",
    "mkdir -p /home/ubuntu/shared/gb_safety/models",
    "curl -fsSL -o /home/ubuntu/shared/gb_safety/models/hand_landmarker.task " + MODEL_URL,
)


def model_path():
    return os.environ.get("GB_HAND_MODEL", "/home/ubuntu/shared/gb_safety/models/hand_landmarker.task")


def available():
    if not os.path.isfile(model_path()):
        return False
    try:
        import mediapipe  # noqa: F401
    except Exception:
        return False
    return True


def landmarks(image_rgb):
    """Return a list of {x,y} normalized landmarks, or (None, reason)."""
    if not available():
        return None, "hand_landmarker_missing"
    try:
        import mediapipe as mp
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path()),
            num_hands=1)
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as landmarker:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            result = landmarker.detect(mp_image)
        if not result.hand_landmarks:
            return None, "no_hand"
        hand = result.hand_landmarks[0]
        return [{"x": p.x, "y": p.y} for p in hand], None
    except Exception as ex:
        return None, "hand_landmarker_failed:%s" % type(ex).__name__
