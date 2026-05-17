import numpy as np
from rpms.config import LENS_POSITION, CONF_THRESHOLD, FRAME_W, FRAME_H


def init_camera():
    from picamera2 import Picamera2
    picam = Picamera2()
    cfg = picam.create_preview_configuration(
        main={'size': (FRAME_W, FRAME_H), 'format': 'RGB888'}
    )
    picam.configure(cfg)
    # Manual focus locked at ~1.5m: LensPosition=0.67 diopters → 1/0.67≈1.5m
    picam.set_controls({'AfMode': 0, 'LensPosition': LENS_POSITION})
    picam.start()
    return picam


def init_model():
    from ultralytics import YOLO
    return YOLO('yolov8n_ncnn_model', task='detect')


def capture_frame(picam) -> np.ndarray:
    return picam.capture_array()


def detect_person(frame: np.ndarray, model) -> tuple[int, float] | tuple[None, None]:
    """Return (centroid_x, bbox_area) for the largest detected person.

    Selects by largest bounding box area (proxy for closest person).
    Returns (None, None) when no person is detected.
    """
    results = model(frame, classes=[0], conf=CONF_THRESHOLD, verbose=False)
    boxes = results[0].boxes.xyxy.cpu().numpy()  # shape (N, 4) xyxy

    if len(boxes) == 0:
        return None, None

    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    best = boxes[np.argmax(areas)]
    cx = int((best[0] + best[2]) / 2)
    return cx, float(np.max(areas))
