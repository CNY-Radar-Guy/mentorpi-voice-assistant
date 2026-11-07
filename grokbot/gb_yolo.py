"""YOLOv8 raw head for OpenCV DNN. The tracker and the frame test use this.

The graph is static. The blob size has to be the size the file was exported
with. Scores are the network's class scores. Nothing rescales them.
"""
import numpy as np


def detect(net, img, size, names, conf=0.45, iou=0.45):
    """img is BGR uint8, which is what cv2.imread and the camera path both are."""
    import cv2
    h, w = img.shape[:2]
    size = int(size)
    if size <= 0:
        return []
    blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (size, size), swapRB=True, crop=False)
    net.setInput(blob)
    out = net.forward()
    arr = out[0] if isinstance(out, (list, tuple)) else out
    if arr.ndim != 3:
        return []
    if arr.shape[1] == 84:
        pred = arr[0].T
    elif arr.shape[2] == 84:
        pred = arr[0]
    else:
        return []
    boxes = pred[:, :4]
    scores = pred[:, 4:]
    ci = scores.argmax(1)
    cf = scores[np.arange(len(scores)), ci]
    keep = cf >= float(conf)
    boxes, ci, cf = boxes[keep], ci[keep], cf[keep]
    if len(boxes) == 0:
        return []
    sx, sy = w / float(size), h / float(size)
    xy = np.stack([(boxes[:, 0] - boxes[:, 2] / 2) * sx,
                   (boxes[:, 1] - boxes[:, 3] / 2) * sy,
                   boxes[:, 2] * sx, boxes[:, 3] * sy], 1)
    picked = np.array(cv2.dnn.NMSBoxes(xy.tolist(), cf.tolist(), float(conf), float(iou))).flatten()
    res = []
    for i in picked:
        x1, y1, bw, bh = [float(v) for v in xy[i]]
        label = names[int(ci[i])] if int(ci[i]) < len(names) else str(int(ci[i]))
        res.append((label, float(cf[i]), x1, y1, x1 + bw, y1 + bh))
    return res
