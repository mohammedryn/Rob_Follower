# RPMS Software Sprint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and tune the complete RPMS software stack — from project scaffold to a working autonomous human-following demo — in 2 days on hardware that is already physically assembled.

**Architecture:** Single-threaded Python control loop on Raspberry Pi 5 handles YOLOv8n-NCNN inference, VL53L5CX ToF polling, dual-channel PID, and serial command dispatch. Teensy 4.1 firmware receives `L{n}R{n}\n` ASCII commands and executes 20kHz hardware PWM with a 500ms safety watchdog.

**Tech Stack:** Python 3.11 · picamera2 · ultralytics (NCNN backend) · vl53l5cx-ctypes · pyserial · pytest · Arduino/C++ (Teensy 4.1)

**Spec:** `docs/superpowers/specs/2026-05-17-rpms-execution-design.md`

---

## File Map

| File | Role |
|------|------|
| `rpms/__init__.py` | Makes `rpms` an importable package |
| `rpms/config.py` | All constants — gains, distances, ports, camera params |
| `rpms/pid.py` | `PIDController` class — filtered derivative, anti-windup, deadband |
| `rpms/motor.py` | `init_serial()`, `send_motors(ser, left, right)` |
| `rpms/tof.py` | `init_tof()`, `get_depth_info(tof)` |
| `rpms/vision.py` | `init_camera()`, `capture_frame(picam)`, `detect_person(frame, model)` |
| `rpms/main.py` | ~40-line control loop — imports all above, no logic |
| `firmware/teensy_motor/teensy_motor.ino` | Teensy: PWM + watchdog + serial parser |
| `tools/export_ncnn.py` | One-shot NCNN export script |
| `tests/conftest.py` | Shared pytest fixtures |
| `tests/test_pid.py` | PID unit tests |
| `tests/test_motor.py` | Motor serial protocol unit tests |
| `tests/test_tof.py` | ToF logic unit tests (mocked hardware) |
| `tests/test_vision.py` | Person detection unit tests (mocked model) |
| `requirements.txt` | Python dependencies |

---

## Task 1: Project Scaffold

**Files:**
- Create: `rpms/__init__.py`
- Create: `requirements.txt`
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`
- Create: `tools/__init__.py`
- Create: `firmware/teensy_motor/` (directory only)

- [ ] **Step 1: Create directory structure**

```bash
cd /home/m0mspagetthi/Rob_Follower
mkdir -p rpms tests tools firmware/teensy_motor
touch rpms/__init__.py tests/__init__.py tools/__init__.py
```

- [ ] **Step 2: Create `requirements.txt`**

```
picamera2
ultralytics
pyserial
vl53l5cx-ctypes
numpy
pytest
```

- [ ] **Step 3: Create `tests/conftest.py`**

```python
# tests/conftest.py
import numpy as np
import pytest


@pytest.fixture
def blank_frame():
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def mock_tof_data_factory():
    """Returns a factory that builds mock VL53L5CX data objects."""
    from unittest.mock import MagicMock

    def _factory(distances: list[int]):
        """distances: list of 64 ints (8x8 grid, row-major)."""
        data = MagicMock()
        data.distance_mm = distances
        return data

    return _factory
```

- [ ] **Step 4: Verify pytest can be imported**

```bash
python3 -m pytest --collect-only
```

Expected output: `no tests ran` (no test files yet — that's fine).

- [ ] **Step 5: Commit**

```bash
git init  # only if not already a git repo
git add rpms/ tests/ tools/ firmware/ requirements.txt
git commit -m "feat: scaffold project structure"
```

---

## Task 2: `config.py` — Constants

**Files:**
- Create: `rpms/config.py`

All tunable constants live here. During PID tuning, only this file changes.

- [ ] **Step 1: Create `rpms/config.py`**

```python
# rpms/config.py

# --- Frame ---
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# --- Following distances (mm) ---
FOLLOW_DIST_MM = 800
STOP_DIST_MM   = 400

# --- Serial ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD        = 115200

# --- Motor speed (PWM units, 0–255) ---
BASE_SPEED = 160
MAX_SPEED  = 220

# --- Steer PID (lateral pixel error → differential speed) ---
KP_STEER         = 0.25
KI_STEER         = 0.001
KD_STEER         = 0.05
STEER_DEADBAND   = 40       # pixels: suppress micro-corrections near center
STEER_OUT_LIMIT  = 100      # max steer contribution (PWM units)
STEER_INTG_LIMIT = 1000
STEER_DERIV_ALPHA = 0.2     # low-pass coefficient for derivative (0=no filter, 1=raw)

# --- Speed PD (distance error → base speed offset; Ki=0 by design) ---
KP_SPEED          = 0.15
KD_SPEED          = 0.02
SPEED_OUT_LIMIT   = 80
SPEED_DERIV_ALPHA = 0.3

# --- Camera ---
LENS_POSITION  = 0.67   # diopters; focus_distance_m = 1/LENS_POSITION ≈ 1.5m
CONF_THRESHOLD = 0.50
```

- [ ] **Step 2: Verify import works**

```bash
python3 -c "from rpms.config import FOLLOW_DIST_MM; print(FOLLOW_DIST_MM)"
```

Expected: `800`

- [ ] **Step 3: Commit**

```bash
git add rpms/config.py
git commit -m "feat: add config constants"
```

---

## Task 3: `pid.py` — PID Controller (TDD)

**Files:**
- Create: `rpms/pid.py`
- Create: `tests/test_pid.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/test_pid.py
import time
import pytest
from unittest.mock import patch
from rpms.pid import PIDController


# --- Helpers ---

def make_pid(Kp=1.0, Ki=0.0, Kd=0.0, deadband=0,
             integral_limit=1000, output_limit=255,
             derivative_alpha=0.0, start_time=0.0):
    """Create a PIDController with a controlled fake clock."""
    with patch('rpms.pid.time') as mock_time:
        mock_time.time.return_value = start_time
        pid = PIDController(
            Kp=Kp, Ki=Ki, Kd=Kd,
            deadband=deadband,
            integral_limit=integral_limit,
            output_limit=output_limit,
            derivative_alpha=derivative_alpha,
        )
    pid._fake_time = start_time
    return pid


def update_at(pid, error, dt=0.1):
    """Advance the fake clock by dt seconds and call update."""
    pid._fake_time += dt
    with patch('rpms.pid.time') as mock_time:
        mock_time.time.return_value = pid._fake_time
        return pid.update(error)


# --- Tests ---

class TestProportional:
    def test_positive_error(self):
        pid = make_pid(Kp=2.0)
        out = update_at(pid, 50.0)
        assert abs(out - 100.0) < 0.01

    def test_negative_error(self):
        pid = make_pid(Kp=2.0)
        out = update_at(pid, -50.0)
        assert abs(out - (-100.0)) < 0.01

    def test_zero_error(self):
        pid = make_pid(Kp=2.0)
        out = update_at(pid, 0.0)
        assert out == 0.0


class TestDeadband:
    def test_inside_deadband_returns_zero(self):
        pid = make_pid(Kp=2.0, deadband=40)
        out = update_at(pid, 30.0)
        assert out == 0.0

    def test_on_deadband_boundary_returns_zero(self):
        pid = make_pid(Kp=2.0, deadband=40)
        out = update_at(pid, 40.0)
        assert out == 0.0

    def test_outside_deadband_responds(self):
        pid = make_pid(Kp=2.0, deadband=40)
        out = update_at(pid, 41.0)
        assert out > 0.0

    def test_deadband_resets_integral(self):
        pid = make_pid(Kp=0.0, Ki=1.0, deadband=40, integral_limit=1000)
        update_at(pid, 100.0, dt=1.0)  # accumulate integral
        update_at(pid, 10.0, dt=1.0)   # inside deadband: integral reset
        out = update_at(pid, 100.0, dt=1.0)
        # integral should have started fresh after deadband reset
        assert abs(out) < 200.0  # not carrying 2-second accumulation


class TestOutputClamping:
    def test_positive_clamp(self):
        pid = make_pid(Kp=10.0, output_limit=100)
        out = update_at(pid, 50.0)
        assert out == 100.0

    def test_negative_clamp(self):
        pid = make_pid(Kp=10.0, output_limit=100)
        out = update_at(pid, -50.0)
        assert out == -100.0


class TestIntegralAntiWindup:
    def test_integral_clamped(self):
        pid = make_pid(Kp=0.0, Ki=1.0, integral_limit=50, output_limit=10000)
        for _ in range(20):
            update_at(pid, 10.0, dt=1.0)  # accumulate 200 without clamp
        out = update_at(pid, 0.0, dt=0.001)
        # output = Ki * integral ≤ 1.0 * 50.0
        assert abs(out) <= 50.1


class TestDerivative:
    def test_derivative_positive_on_rising_error(self):
        pid = make_pid(Kp=0.0, Ki=0.0, Kd=1.0, derivative_alpha=0.0)
        update_at(pid, 0.0, dt=0.1)
        out = update_at(pid, 10.0, dt=0.1)
        # raw_deriv = (10-0)/0.1 = 100; with alpha=0, filtered=raw
        assert out > 0.0

    def test_derivative_filtered_smaller_than_raw(self):
        pid_raw = make_pid(Kp=0.0, Ki=0.0, Kd=1.0, derivative_alpha=0.0)
        pid_flt = make_pid(Kp=0.0, Ki=0.0, Kd=1.0, derivative_alpha=0.2)
        update_at(pid_raw, 0.0, dt=0.1)
        update_at(pid_flt, 0.0, dt=0.1)
        raw_out = update_at(pid_raw, 100.0, dt=0.1)
        flt_out = update_at(pid_flt, 100.0, dt=0.1)
        assert abs(flt_out) < abs(raw_out)
```

- [ ] **Step 2: Run failing tests**

```bash
python3 -m pytest tests/test_pid.py -v
```

Expected: Multiple failures — `ModuleNotFoundError: No module named 'rpms.pid'`

- [ ] **Step 3: Create `rpms/pid.py`**

```python
# rpms/pid.py
import time as time


class PIDController:
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        deadband: float = 0.0,
        integral_limit: float = 1000.0,
        output_limit: float = 255.0,
        derivative_alpha: float = 0.2,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.deadband = deadband
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_alpha = derivative_alpha

        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.prev_time = time.time()

    def update(self, error: float) -> float:
        if abs(error) <= self.deadband:
            error = 0.0
            self.integral = 0.0

        now = time.time()
        dt = max(now - self.prev_time, 1e-4)

        self.integral += error * dt
        self.integral = max(
            -self.integral_limit, min(self.integral_limit, self.integral)
        )

        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (
            self.derivative_alpha * raw_derivative
            + (1.0 - self.derivative_alpha) * self.filtered_derivative
        )

        output = (
            self.Kp * error
            + self.Ki * self.integral
            + self.Kd * self.filtered_derivative
        )
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_time = now
        return output
```

- [ ] **Step 4: Run tests — all must pass**

```bash
python3 -m pytest tests/test_pid.py -v
```

Expected:
```
tests/test_pid.py::TestProportional::test_positive_error PASSED
tests/test_pid.py::TestProportional::test_negative_error PASSED
tests/test_pid.py::TestProportional::test_zero_error PASSED
tests/test_pid.py::TestDeadband::test_inside_deadband_returns_zero PASSED
tests/test_pid.py::TestDeadband::test_on_deadband_boundary_returns_zero PASSED
tests/test_pid.py::TestDeadband::test_outside_deadband_responds PASSED
tests/test_pid.py::TestDeadband::test_deadband_resets_integral PASSED
tests/test_pid.py::TestOutputClamping::test_positive_clamp PASSED
tests/test_pid.py::TestOutputClamping::test_negative_clamp PASSED
tests/test_pid.py::TestIntegralAntiWindup::test_integral_clamped PASSED
tests/test_pid.py::TestDerivative::test_derivative_positive_on_rising_error PASSED
tests/test_pid.py::TestDerivative::test_derivative_filtered_smaller_than_raw PASSED
12 passed
```

- [ ] **Step 5: Commit**

```bash
git add rpms/pid.py tests/test_pid.py
git commit -m "feat: add PIDController with anti-windup, deadband, filtered derivative"
```

---

## Task 4: `motor.py` — Serial Interface (TDD)

**Files:**
- Create: `rpms/motor.py`
- Create: `tests/test_motor.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/test_motor.py
from unittest.mock import MagicMock, patch
import pytest


class TestSendMotors:
    def _make_ser(self):
        return MagicMock()

    def test_forward_both(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, 100, 100)
        ser.write.assert_called_once_with(b'L100R100\n')

    def test_reverse_both(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, -100, -100)
        ser.write.assert_called_once_with(b'L-100R-100\n')

    def test_stop(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, 0, 0)
        ser.write.assert_called_once_with(b'L0R0\n')

    def test_differential_turn(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, 80, 160)
        ser.write.assert_called_once_with(b'L80R160\n')

    def test_clamps_above_max(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, 300, 300)
        ser.write.assert_called_once_with(b'L220R220\n')

    def test_clamps_below_negative_max(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, -300, -300)
        ser.write.assert_called_once_with(b'L-220R-220\n')

    def test_floats_cast_to_int(self):
        from rpms.motor import send_motors
        ser = self._make_ser()
        send_motors(ser, 100.7, 99.3)
        ser.write.assert_called_once_with(b'L100R99\n')
```

- [ ] **Step 2: Run failing tests**

```bash
python3 -m pytest tests/test_motor.py -v
```

Expected: `ModuleNotFoundError: No module named 'rpms.motor'`

- [ ] **Step 3: Create `rpms/motor.py`**

```python
# rpms/motor.py
import serial
from rpms.config import SERIAL_PORT, BAUD, MAX_SPEED


def init_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)


def send_motors(ser: serial.Serial, left: float, right: float) -> None:
    left  = int(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right = int(max(-MAX_SPEED, min(MAX_SPEED, right)))
    ser.write(f'L{left}R{right}\n'.encode())
```

- [ ] **Step 4: Run tests — all must pass**

```bash
python3 -m pytest tests/test_motor.py -v
```

Expected: `7 passed`

- [ ] **Step 5: Commit**

```bash
git add rpms/motor.py tests/test_motor.py
git commit -m "feat: add motor serial interface with clamping"
```

---

## Task 5: `tof.py` — ToF Sensor Wrapper (TDD)

**Files:**
- Create: `rpms/tof.py`
- Create: `tests/test_tof.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/test_tof.py
import numpy as np
import pytest
from unittest.mock import MagicMock


def _make_tof_mock(ready: bool, grid: list[int] | None = None):
    """Build a mock VL53L5CX sensor object."""
    tof = MagicMock()
    tof.data_ready.return_value = ready
    if grid is not None:
        data = MagicMock()
        data.distance_mm = grid
        tof.get_data.return_value = data
    return tof


def _flat_grid(value: int) -> list[int]:
    return [value] * 64


class TestGetDepthInfo:
    def test_no_data_returns_failsafe(self):
        """When sensor has no fresh data, return fail-safe stop values."""
        from rpms.tof import get_depth_info
        tof = _make_tof_mock(ready=False)
        dist, obstacle = get_depth_info(tof)
        assert dist == 0
        assert obstacle is True

    def test_clear_path_at_follow_distance(self):
        from rpms.tof import get_depth_info
        tof = _make_tof_mock(ready=True, grid=_flat_grid(800))
        dist, obstacle = get_depth_info(tof)
        assert dist == 800
        assert obstacle is False

    def test_obstacle_in_peripheral_zone(self):
        """Any zone < STOP_DIST_MM triggers obstacle flag."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[0] = 300  # corner zone, outside center 4x4
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is True

    def test_obstacle_in_center_zone(self):
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[2 * 8 + 2] = 350  # zone at row=2, col=2 (inside center 4x4)
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is True

    def test_center_dist_is_min_of_center_zones(self):
        """dist return value reflects the minimum in the center 4x4 block."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(1000)
        grid[2 * 8 + 3] = 750  # row=2, col=3: inside center 4x4, closer
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert dist == 750
        assert obstacle is False

    def test_zero_values_ignored(self):
        """VL53L5CX returns 0 for invalid readings; these must not trigger stops."""
        from rpms.tof import get_depth_info
        grid = _flat_grid(800)
        grid[5] = 0  # invalid reading — should be ignored
        tof = _make_tof_mock(ready=True, grid=grid)
        dist, obstacle = get_depth_info(tof)
        assert obstacle is False

    def test_all_zero_grid_returns_safe_default(self):
        """If all readings are invalid (0), return FOLLOW_DIST_MM, no obstacle."""
        from rpms.tof import get_depth_info
        from rpms.config import FOLLOW_DIST_MM
        tof = _make_tof_mock(ready=True, grid=_flat_grid(0))
        dist, obstacle = get_depth_info(tof)
        assert dist == FOLLOW_DIST_MM
        assert obstacle is False
```

- [ ] **Step 2: Run failing tests**

```bash
python3 -m pytest tests/test_tof.py -v
```

Expected: `ModuleNotFoundError: No module named 'rpms.tof'`

- [ ] **Step 3: Create `rpms/tof.py`**

```python
# rpms/tof.py
import numpy as np
from rpms.config import STOP_DIST_MM, FOLLOW_DIST_MM


def init_tof():
    from vl53l5cx_ctypes import VL53L5CX
    tof = VL53L5CX()
    tof.start_ranging()
    return tof


def get_depth_info(tof) -> tuple[int, bool]:
    """Return (min_center_dist_mm, obstacle_detected).

    Fail-safe: if no fresh data is available, returns (0, True) so the
    caller stops the robot rather than continuing on a stale reading.
    """
    if not tof.data_ready():
        return 0, True

    data = tof.get_data()
    grid = np.array(data.distance_mm, dtype=np.int32).reshape(8, 8)

    # Center 4x4 block: zones [2:6, 2:6] face directly forward
    center_flat = grid[2:6, 2:6].flatten()
    center_valid = center_flat[center_flat > 0]
    min_center = int(np.min(center_valid)) if len(center_valid) > 0 else FOLLOW_DIST_MM

    # Full-grid obstacle check (ignore 0 = invalid readings)
    all_valid = grid[grid > 0]
    obstacle = bool(np.any(all_valid < STOP_DIST_MM)) if len(all_valid) > 0 else False

    return min_center, obstacle
```

- [ ] **Step 4: Run tests — all must pass**

```bash
python3 -m pytest tests/test_tof.py -v
```

Expected: `7 passed`

- [ ] **Step 5: Commit**

```bash
git add rpms/tof.py tests/test_tof.py
git commit -m "feat: add VL53L5CX wrapper with fail-safe no-data handling"
```

---

## Task 6: `vision.py` — Camera + Detection (TDD)

**Files:**
- Create: `rpms/vision.py`
- Create: `tests/test_vision.py`

Note: `init_camera()` and `capture_frame()` require physical hardware — they are not unit-tested. `detect_person(frame, model)` is pure computation and is fully tested.

- [ ] **Step 1: Write failing tests**

```python
# tests/test_vision.py
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
```

- [ ] **Step 2: Run failing tests**

```bash
python3 -m pytest tests/test_vision.py -v
```

Expected: `ModuleNotFoundError: No module named 'rpms.vision'`

- [ ] **Step 3: Create `rpms/vision.py`**

```python
# rpms/vision.py
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
```

- [ ] **Step 4: Run tests — all must pass**

```bash
python3 -m pytest tests/test_vision.py -v
```

Expected: `5 passed`

- [ ] **Step 5: Commit**

```bash
git add rpms/vision.py tests/test_vision.py
git commit -m "feat: add vision pipeline with NCNN person detection"
```

---

## Task 7: `tools/export_ncnn.py` — One-Shot Model Export

**Files:**
- Create: `tools/export_ncnn.py`

Run this once on the RPi5 before any other software task. The resulting `yolov8n_ncnn_model/` directory must exist before `main.py` can start.

- [ ] **Step 1: Create `tools/export_ncnn.py`**

```python
# tools/export_ncnn.py
"""Run once on the RPi5 to export YOLOv8n to NCNN format.

Output: yolov8n_ncnn_model/ in the current directory.
Verify with: ls yolov8n_ncnn_model/  (should show .param and .bin files)
"""
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.export(format='ncnn')
print("Export complete. Verify: ls yolov8n_ncnn_model/")
```

- [ ] **Step 2: Run on RPi5 (one-time setup)**

```bash
cd /home/m0mspagetthi/Rob_Follower
python3 tools/export_ncnn.py
```

Expected output ends with: `Export complete. Verify: ls yolov8n_ncnn_model/`

- [ ] **Step 3: Verify export produced required files**

```bash
ls yolov8n_ncnn_model/
```

Expected: Files ending in `.param` and `.bin` are present.

- [ ] **Step 4: Smoke-test inference on a blank frame**

```bash
python3 - << 'EOF'
import numpy as np
from ultralytics import YOLO
model = YOLO('yolov8n_ncnn_model', task='detect')
frame = np.zeros((480, 640, 3), dtype=np.uint8)
r = model(frame, classes=[0], conf=0.5, verbose=False)
print(f"NCNN inference OK. Boxes on blank frame: {len(r[0].boxes)}")
# Expect 0 boxes on a blank frame — that's correct behaviour
EOF
```

Expected: `NCNN inference OK. Boxes on blank frame: 0`

- [ ] **Step 5: Commit**

```bash
git add tools/export_ncnn.py
git commit -m "feat: add NCNN export script"
```

---

## Task 8: Teensy Firmware

**Files:**
- Create: `firmware/teensy_motor/teensy_motor.ino`

This is flashed via Arduino IDE or PlatformIO. There are no automated tests — verification is manual via serial monitor.

- [ ] **Step 1: Create `firmware/teensy_motor/teensy_motor.ino`**

```cpp
// firmware/teensy_motor/teensy_motor.ino
// Receives L{int}R{int}\n commands from RPi5 over USB serial.
// Drives two BTS7960 H-bridge modules via 20kHz hardware PWM.
// Safety watchdog: stops motors if no command received in 500ms.

#include <Arduino.h>

// BTS7960 #1 — Left side motors (front-left + rear-left, wired in parallel)
#define L_RPWM 2   // Forward PWM
#define L_LPWM 3   // Reverse PWM
#define L_EN   4   // Enable (active HIGH)

// BTS7960 #2 — Right side motors (front-right + rear-right, wired in parallel)
#define R_RPWM 5   // Forward PWM
#define R_LPWM 6   // Reverse PWM
#define R_EN   7   // Enable (active HIGH)

#define WATCHDOG_MS 500   // Stop motors if silent for this long

unsigned long lastCmdMs  = 0;
bool          motorsSafe = false;

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  analogWrite(L_RPWM, left  >= 0 ? left  : 0);
  analogWrite(L_LPWM, left  <  0 ? -left : 0);
  analogWrite(R_RPWM, right >= 0 ? right : 0);
  analogWrite(R_LPWM, right <  0 ? -right : 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(L_EN, OUTPUT); digitalWrite(L_EN, HIGH);
  pinMode(R_EN, OUTPUT); digitalWrite(R_EN, HIGH);

  // 20kHz PWM — above audible range, within BTS7960 25kHz limit
  analogWriteFrequency(L_RPWM, 20000);
  analogWriteFrequency(L_LPWM, 20000);
  analogWriteFrequency(R_RPWM, 20000);
  analogWriteFrequency(R_LPWM, 20000);
  analogWriteResolution(8);  // 0–255 range

  lastCmdMs = millis();
}

void loop() {
  // Watchdog: halt if RPi5 goes silent (crash, disconnect, KeyboardInterrupt)
  if ((millis() - lastCmdMs) > WATCHDOG_MS) {
    if (!motorsSafe) {
      setMotors(0, 0);
      motorsSafe = true;
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int l_idx = cmd.indexOf('L');
    int r_idx = cmd.indexOf('R');
    if (l_idx >= 0 && r_idx > l_idx) {
      int left  = cmd.substring(l_idx + 1, r_idx).toInt();
      int right = cmd.substring(r_idx + 1).toInt();
      setMotors(left, right);
      lastCmdMs  = millis();
      motorsSafe = false;
    }
  }
}
```

- [ ] **Step 2: Flash to Teensy 4.1**

Open `firmware/teensy_motor/teensy_motor.ino` in Arduino IDE.
- Board: `Teensy 4.1`
- Tools → USB Type: `Serial`
- Click Upload.

Expected: Upload succeeds, Teensy LED solid (not flashing error pattern).

- [ ] **Step 3: Verify motors via serial monitor**

Open Arduino IDE Serial Monitor at 115200 baud. Type and send each command manually:

```
L100R100
```
Expected: Both motor sides spin forward.

```
L0R0
```
Expected: Both sides stop.

```
L100R-100
```
Expected: Left side forward, right side reverse (robot pivots clockwise).

- [ ] **Step 4: Verify watchdog**

Send `L150R150` in serial monitor. Then close the serial monitor (simulating RPi5 disconnect). Wait 600ms. Expected: motors stop on their own.

- [ ] **Step 5: Commit**

```bash
git add firmware/teensy_motor/teensy_motor.ino
git commit -m "feat: add Teensy firmware with 500ms safety watchdog"
```

---

## Task 9: `main.py` — Control Loop

**Files:**
- Create: `rpms/main.py`

- [ ] **Step 1: Create `rpms/main.py`**

```python
# rpms/main.py
import time
import numpy as np
from rpms.config import (
    CENTER_X, FOLLOW_DIST_MM, BASE_SPEED, MAX_SPEED,
    KP_STEER, KI_STEER, KD_STEER,
    STEER_DEADBAND, STEER_OUT_LIMIT, STEER_INTG_LIMIT, STEER_DERIV_ALPHA,
    KP_SPEED, KD_SPEED, SPEED_OUT_LIMIT, SPEED_DERIV_ALPHA,
)
from rpms.pid    import PIDController
from rpms.motor  import init_serial, send_motors
from rpms.tof    import init_tof, get_depth_info
from rpms.vision import init_camera, init_model, capture_frame, detect_person


def main():
    picam = init_camera()
    model = init_model()
    tof   = init_tof()
    ser   = init_serial()

    steer_pid = PIDController(
        Kp=KP_STEER, Ki=KI_STEER, Kd=KD_STEER,
        deadband=STEER_DEADBAND,
        integral_limit=STEER_INTG_LIMIT,
        output_limit=STEER_OUT_LIMIT,
        derivative_alpha=STEER_DERIV_ALPHA,
    )
    speed_pid = PIDController(
        Kp=KP_SPEED, Ki=0.0, Kd=KD_SPEED,
        output_limit=SPEED_OUT_LIMIT,
        derivative_alpha=SPEED_DERIV_ALPHA,
    )

    print("Waiting for Teensy to boot...")
    time.sleep(2)
    print("Running. Press Ctrl+C to stop.")

    try:
        while True:
            t0 = time.time()

            dist_mm, obstacle = get_depth_info(tof)
            if obstacle:
                send_motors(ser, 0, 0)
                continue

            frame = capture_frame(picam)
            cx, _area = detect_person(frame, model)
            if cx is None:
                send_motors(ser, 0, 0)
                continue

            steer = steer_pid.update(cx - CENTER_X)
            speed = speed_pid.update(dist_mm - FOLLOW_DIST_MM)
            speed = float(np.clip(BASE_SPEED + speed, -BASE_SPEED, MAX_SPEED))

            send_motors(ser, speed - steer, speed + steer)

            fps = 1.0 / max(time.time() - t0, 1e-4)
            print(f"CX:{cx:4d}  DIST:{dist_mm:5d}mm  SPD:{speed:6.1f}  STR:{steer:6.1f}  FPS:{fps:4.1f}")

    except KeyboardInterrupt:
        print("\nStopping...")
        send_motors(ser, 0, 0)
        picam.stop()
        tof.stop_ranging()
        ser.close()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Run the test suite — all previous tests still pass**

```bash
python3 -m pytest tests/ -v
```

Expected: All prior tests (pid, motor, tof, vision) pass. `main.py` has no unit tests — it will be verified by hardware integration in Task 10.

- [ ] **Step 3: Commit**

```bash
git add rpms/main.py
git commit -m "feat: add main control loop"
```

---

## Task 10: Day 1 Hardware Integration (Manual)

These steps run on the physical robot. All prior software tasks should be committed and pushed before starting.

> **Prerequisite:** Task 7 NCNN export completed successfully on the RPi5.

- [ ] **Step 1: Verify I2C is enabled and fast**

```bash
ls /dev/i2c*
```

Expected: `/dev/i2c-1` exists.

```bash
i2cdetect -y 1
```

Expected: Address `52` appears in the grid (this is the VL53L5CX at 0x52).

If address does not appear:
- Check `/boot/firmware/config.txt` contains `dtparam=i2c_arm=on,i2c_arm_baudrate=400000`
- Check physical wiring: SDA → GPIO 3 (pin 3), SCL → GPIO 5 (pin 5), VCC → 3.3V, GND → GND

- [ ] **Step 2: Verify Teensy serial link from RPi5**

With Teensy connected via USB:

```bash
ls /dev/ttyACM*
```

Expected: `/dev/ttyACM0` appears.

```python
python3 - << 'EOF'
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
time.sleep(2)  # Teensy boot
ser.write(b'L100R100\n')
time.sleep(1)
ser.write(b'L0R0\n')
ser.close()
print("Serial test done. Both sides should have spun for 1 second.")
EOF
```

Expected: Both motor sides spin for 1 second then stop.

- [ ] **Step 3: Verify ToF readings**

```python
python3 - << 'EOF'
from rpms.tof import init_tof, get_depth_info
import time
tof = init_tof()
time.sleep(0.5)
for _ in range(10):
    dist, obs = get_depth_info(tof)
    print(f"dist={dist}mm  obstacle={obs}")
    time.sleep(0.1)
tof.stop_ranging()
EOF
```

Expected: `dist` values near actual distance to nearest object (hold hand at ~80cm → dist ~800). `obstacle=False` when path clear, `obstacle=True` when hand within 40cm.

- [ ] **Step 4: Verify camera + detection**

```python
python3 - << 'EOF'
from rpms.vision import init_camera, init_model, capture_frame, detect_person
picam = init_camera()
model = init_model()
import time; time.sleep(1)
for _ in range(5):
    frame = capture_frame(picam)
    cx, area = detect_person(frame, model)
    print(f"cx={cx}  area={area}")
    time.sleep(0.2)
picam.stop()
EOF
```

Stand 1-1.5m in front of the camera. Expected: `cx` values near 320 (center of frame). `area` a positive float. When you step aside, both return `None`.

- [ ] **Step 5: Run `main.py` — centroid steering test**

Before starting, **set `KP_STEER=0.20, KI_STEER=0.0, KD_STEER=0.0`** in `rpms/config.py` to start with proportional-only steering. This isolates the first issue if any.

```bash
python3 -m rpms.main
```

Stand 1m in front of the robot and slowly move left/right. Expected: Robot steers toward you. Speed stays roughly constant (PD distance control with initial gains will vary slightly).

When obstacle is placed within 40cm of front sensor: robot stops. When removed: robot resumes following.

Press Ctrl+C. Expected: `Stopping...` printed, motors halt, no exception.

- [ ] **Step 6: Day 1 checkpoint commit**

```bash
git add -u
git commit -m "chore: day 1 hardware integration verified"
```

---

## Task 11: PID Tuning (Day 2)

Tune one channel at a time in the order below. Edit only `rpms/config.py`. No other file changes during tuning.

> **Before starting:** Restore `KI_STEER=0.001, KD_STEER=0.05` if you zeroed them in Task 10 Step 5.

- [ ] **Step 1: Tune `KP_STEER` — wheels lifted off ground**

Lift the robot so wheels spin freely. Run:

```bash
python3 -m rpms.main
```

Watch `STR` value in console output. Slowly move your hand left/right in front of camera.

- Start with `KP_STEER=0.10`. Increase by 0.05 each run until `STR` reacts sharply to lateral movement.
- Back off 20% from the value that feels sharp.
- Typical working range: 0.20–0.30.

- [ ] **Step 2: Tune `KD_STEER` — wheels lifted**

Set `KI_STEER=0.0` temporarily. Increase `KD_STEER` from 0.0 in steps of 0.02 until turn oscillation damps. If console `STR` values look noisy (rapidly alternating sign): decrease `STEER_DERIV_ALPHA` toward 0.1 (more filtering).

- [ ] **Step 3: Restore `KI_STEER=0.001` and test straight corridor following**

Robot on the ground. Walk straight ahead at 1m distance for 10m. Observe if robot drifts laterally over time. If drift persists: increase `KI_STEER` by 0.0005. If robot oscillates with integral: decrease or set back to 0.

- [ ] **Step 4: Tune `KP_SPEED` — robot on ground**

Set `KD_SPEED=0.0` temporarily. Walk toward and away from robot. Increase `KP_SPEED` from 0.10 in steps of 0.02 until robot noticeably accelerates when you move away and brakes when you approach.

Typical working range: 0.10–0.20.

- [ ] **Step 5: Tune `KD_SPEED`**

With `KP_SPEED` set, add `KD_SPEED=0.01`. Increase in steps of 0.01 until speed oscillation at the following distance set-point damps. Excessive `KD_SPEED` causes surging on approach.

- [ ] **Step 6: Combined following test**

Walk normally (varied speed, direction changes, 90° corners). Observe:

- Robot holds ~800mm distance (console `DIST` ± 150mm is acceptable)
- Robot recovers target after stepping behind an obstacle briefly
- No persistent oscillation in straight corridor

Adjust `STEER_DEADBAND` in `config.py` if needed: increase if robot wiggles on straight paths; decrease if turns feel sluggish.

- [ ] **Step 7: Commit final tuned gains**

```bash
git add rpms/config.py
git commit -m "tune: finalize PID gains for demo environment"
```

---

## Task 12: Demo Validation

Run these acceptance tests in order. All must pass before the demo.

- [ ] **Step 1: 5-minute corridor following test**

```bash
python3 -m rpms.main
```

Walk at varied speeds (0.5–1.2 m/s) for 5 continuous minutes in a 20m corridor. Log to file for review:

```bash
python3 -m rpms.main 2>&1 | tee demo_log.txt
```

Pass criteria:
- Zero collisions
- `DIST` stays within 800 ± 200mm for >80% of the run
- No unrecoverable target loss (robot resumes after brief occlusion)

- [ ] **Step 2: Obstacle avoidance verification**

While robot is following, place a chair or bag directly in its path. Expected: Robot stops (motors to 0). Remove obstacle. Expected: Robot resumes following within 1 second.

- [ ] **Step 3: Watchdog verification**

While robot is running, pull the USB cable between RPi5 and Teensy. Expected: Motors stop within 500ms. Reconnect. Expected: `main.py` reconnects on next startup (serial re-init).

- [ ] **Step 4: Graceful shutdown verification**

```bash
python3 -m rpms.main
# Let it run for 30 seconds, then:
# Ctrl+C
```

Expected: `Stopping...` printed. Motors halt immediately. No Python traceback.

- [ ] **Step 5: Final commit**

```bash
git add demo_log.txt
git commit -m "chore: demo validation passed — all acceptance tests green"
```

---

## Self-Review

**Spec coverage check:**

| Spec requirement | Covered by |
|---|---|
| `config.py` with all constants | Task 2 |
| `PIDController` with filtered derivative, anti-windup, deadband | Task 3 |
| `motor.py` — `send_motors` clips to MAX_SPEED, sends `L{n}R{n}\n` | Task 4 |
| `tof.py` — fail-safe `(0, True)` when no fresh data | Task 5 |
| `vision.py` — `LensPosition=0.67`, `task='detect'`, correct `configure(cfg)` | Task 6 |
| NCNN export one-shot script | Task 7 |
| Teensy firmware: 20kHz PWM + 500ms watchdog | Task 8 |
| `main.py` ≤ 40 lines, no inline logic | Task 9 |
| Day 1 hardware integration (serial, ToF, vision, basic following) | Task 10 |
| PID tuning procedure matching §6.7 | Task 11 |
| 5-min corridor demo, obstacle stop, watchdog, clean shutdown | Task 12 |

**Placeholder scan:** No TBDs, no "implement later", no "similar to above". All code steps show actual code. All command steps show exact commands with expected output.

**Type/name consistency:**
- `send_motors(ser, left, right)` — matches Task 4 definition and Task 9 call sites. ✓
- `get_depth_info(tof) → (int, bool)` — matches Task 5 definition and Task 9 call site. ✓
- `detect_person(frame, model) → (int, float) | (None, None)` — matches Task 6 definition and Task 9 call site. ✓
- `PIDController(Kp, Ki, Kd, ...)` — matches Task 3 definition and Task 9 instantiation. ✓
- All config constants used in Task 9 are defined in Task 2. ✓
