import numpy as np
import pytest
from unittest.mock import MagicMock


def _make_model_mock(boxes_xyxy: np.ndarray):
    """Return a mock YOLO model that yields the given boxes."""
    model = MagicMock()
    result = MagicMock()
    result.boxes.xyxy.cpu.return_value.numpy.return_value = boxes_xyxy
    model.return_value = [result]
    return model


class TestDetectPerson:
    def test_no_detections_returns_none(self, blank_frame):
        from rpms.vision import detect_person
        model = _make_model_mock(np.empty((0, 4)))
        cx, area = detect_person(blank_frame, model)
        assert cx is None
        assert area is None

    def test_single_box_centroid(self, blank_frame):
        from rpms.vision import detect_person
        # box: x1=100, y1=50, x2=300, y2=450 → cx=200, area=200*400=80000
        boxes = np.array([[100, 50, 300, 450]], dtype=float)
        model = _make_model_mock(boxes)
        cx, area = detect_person(blank_frame, model)
        assert cx == 200
        assert area == pytest.approx(80000.0)

    def test_picks_largest_box(self, blank_frame):
        from rpms.vision import detect_person
        boxes = np.array([
            [0,   0,  50,  50],   # small: area=2500,  cx=25
            [100, 0, 500, 400],   # large: area=160000, cx=300
        ], dtype=float)
        model = _make_model_mock(boxes)
        cx, area = detect_person(blank_frame, model)
        assert cx == 300
        assert area == pytest.approx(160000.0)

    def test_centroid_x_is_horizontal_midpoint(self, blank_frame):
        from rpms.vision import detect_person
        boxes = np.array([[200, 0, 440, 480]], dtype=float)
        model = _make_model_mock(boxes)
        cx, area = detect_person(blank_frame, model)
        assert cx == 320  # (200+440)//2

    def test_model_called_with_correct_args(self, blank_frame):
        from rpms.vision import detect_person
        from rpms.config import CONF_THRESHOLD
        model = _make_model_mock(np.empty((0, 4)))
        detect_person(blank_frame, model)
        model.assert_called_once_with(
            blank_frame,
            classes=[0],
            conf=CONF_THRESHOLD,
            verbose=False,
        )
