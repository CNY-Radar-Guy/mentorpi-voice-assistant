"""gb_vision.py - tiny local image fingerprint (MobileNetV2 global-pool embedding, 1280-d, ONNX on CPU)."""
import io, os
import numpy as np

MODEL = os.path.expanduser(os.environ.get("EMBED_MODEL", "~/grokbot/models/mnv2_embed.onnx"))
CROP = float(os.environ.get("POINT_CROP", "0.6"))     # central fraction of the frame treated as "what I'm pointing at"
_sess = None

def _session():
    global _sess
    if _sess is None:
        import onnxruntime as ort
        o = ort.SessionOptions(); o.intra_op_num_threads = 2
        _sess = ort.InferenceSession(MODEL, o, providers=["CPUExecutionProvider"])
    return _sess

def crop_center(jpeg, frac=CROP):
    from PIL import Image
    im = Image.open(io.BytesIO(jpeg)).convert("RGB"); w, h = im.size
    cw, ch = int(w * frac), int(h * frac)
    return im.crop(((w - cw) // 2, (h - ch) // 2, (w + cw) // 2, (h + ch) // 2))

def embed(jpeg):
    im = crop_center(jpeg).resize((224, 224))
    x = (np.asarray(im, np.float32) / 255.0 - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
    x = x.transpose(2, 0, 1)[None].astype(np.float32)
    v = _session().run(None, {"input": x})[0][0].astype(np.float32)
    return v / (np.linalg.norm(v) + 1e-9)

def from_blob(blob):
    v = np.frombuffer(blob, dtype=np.float16).astype(np.float32)
    return v / (np.linalg.norm(v) + 1e-9)

def jpeg_for_api(jpeg, max_side=1280, quality=85):
    from PIL import Image
    im = Image.open(io.BytesIO(jpeg)).convert("RGB"); im.thumbnail((max_side, max_side))
    b = io.BytesIO(); im.save(b, "JPEG", quality=quality); return b.getvalue()
