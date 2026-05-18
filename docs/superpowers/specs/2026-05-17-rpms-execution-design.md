# RPMS Software Execution Design
**Date:** 2026-05-17
**Project:** Robotic Payload Mobility System — Software Sprint
**Deadline:** 2 days from start
**Hardware state:** Fully assembled (chassis, motors, BTS7960, Teensy 4.1, RPi5, Camera Module 3, VL53L5CX)
**Developer:** Solo

---

## Context

The RPMS hardware is assembled and verified (motors spin, wiring complete). No software exists yet. This document specifies the complete software design for a 2-day sprint to achieve a working autonomous human-following demo:

**Success criterion:** Robot follows a designated person in a 20m indoor corridor for 5 continuous minutes, maintaining ~800mm following distance, without collision.

---

## Architecture

Single-threaded, two-tier:

```
[Camera Module 3] ─── MIPI CSI ──────────────────────────────┐
[VL53L5CX]       ─── I2C 400kHz ─────────────────────────────┤
                                                              ↓
                                              [RPi5: Python main loop]
                                            vision → tof → PID → serial
                                                              │
                                              USB serial: L{n}R{n}\n
                                                              ↓
                                              [Teensy 4.1: C++ firmware]
                                            parse → setMotors → 20kHz PWM
                                                              │
                                                 [BTS7960 x2 → JGB37 x4]
```

**RPi5 owns:** frame capture, YOLO inference, ToF polling, both PID channels, motor command serialization.

**Teensy owns:** serial command parsing, hardware PWM generation, 500ms safety watchdog.

**Protocol:** `L{int}R{int}\n` where int ∈ [-255, 255]. Positive = forward, negative = reverse.

**Why single-threaded:** With a 2-day deadline and 12-15 FPS inference already adequate for walking speed, threading complexity is not justified. The simplest architecture that meets the success criterion wins.

---

## File Structure

```
Rob_Follower/
├── firmware/
│   └── teensy_motor/
│       └── teensy_motor.ino       # Teensy: PWM executor + 500ms watchdog
├── rpms/
│   ├── config.py                  # ALL tunable constants (gains, distances, ports)
│   ├── pid.py                     # PIDController class
│   ├── tof.py                     # VL53L5CX wrapper
│   ├── vision.py                  # Camera + YOLO person detection
│   ├── motor.py                   # Serial interface to Teensy
│   └── main.py                    # Control loop (imports above, ~40 lines)
├── tools/
│   └── export_ncnn.py             # One-shot NCNN export (run once)
├── docs/
│   └── superpowers/specs/
│       └── 2026-05-17-rpms-execution-design.md   # This file
├── RPMS_Project_Report.md
└── RPMS_Project_Report.docx
```

**Principle:** Each file has exactly one responsibility. During PID tuning, only `config.py` changes. During debugging, the failing module is immediately obvious. `main.py` stays short — it is the authoritative description of the control loop, readable in one screen.

---

## Module Contracts

### `config.py`

Single source of truth for every constant in the system. No other file defines magic numbers.

```python
# Frame
FRAME_W, FRAME_H = 640, 480

# Following
FOLLOW_DIST_MM = 800
STOP_DIST_MM   = 400

# Serial
SERIAL_PORT = '/dev/ttyACM0'
BAUD        = 115200

# Motor
BASE_SPEED = 160
MAX_SPEED  = 220

# PID — steer channel (steering: lateral pixel error → differential speed)
KP_STEER = 0.25
KI_STEER = 0.001
KD_STEER = 0.05
STEER_DEADBAND   = 40      # pixels around center before steering engages
STEER_OUT_LIMIT  = 100
STEER_INTG_LIMIT = 1000
STEER_DERIV_ALPHA = 0.2    # low-pass filter on derivative (0=no filter, 1=raw)

# PID — speed channel (PD only: Ki=0, windup-free for distance control)
KP_SPEED = 0.15
KD_SPEED = 0.02
SPEED_OUT_LIMIT   = 80
SPEED_DERIV_ALPHA = 0.3

# Camera
LENS_POSITION = 0.67   # diopters; focus_distance_m = 1/LENS_POSITION ≈ 1.5m
CONF_THRESHOLD = 0.50
```

### `pid.py`

`PIDController(Kp, Ki, Kd, deadband, integral_limit, output_limit, derivative_alpha)`

- `update(error) → float`: Computes PID output for the given error signal.
- Deadband: if `|error| < deadband`, error and integral are zeroed (suppresses micro-oscillation).
- Integral clamp: anti-windup via `max(-integral_limit, min(integral_limit, integral))`.
- Derivative: low-pass filtered via `alpha * raw_deriv + (1-alpha) * prev_filtered_deriv`. Attenuates pixel-level noise from bounding box jitter.
- Speed channel is instantiated with `Ki=0.0` — operates as PD. Integral action causes surge on target re-acquisition and is not needed at this set-point.

### `vision.py`

**`init_camera() → Picamera2`**
- Creates preview config at 640x480 RGB888.
- Sets `AfMode=0` (manual), `LensPosition=0.67` (≈1.5m focus).
- Starts capture. Returns configured `Picamera2` instance.

**`detect_person(picam2) → (cx: int, area: float) | (None, None)`**
- Captures one frame via `picam2.capture_array()`.
- Runs `model(frame, classes=[0], conf=CONF_THRESHOLD, verbose=False)`.
- If no detections: returns `(None, None)`.
- Selects largest bounding box by area (proxy for closest person).
- Returns centroid x-coordinate and bounding box area.

**Known limitation:** Largest-bbox selection is fragile in multi-person scenes. A closer bystander will capture control. Acceptable for demo in a controlled corridor.

**Bug fixes applied vs. user's original snippet:**
1. `cfg = picam2.create_preview_configuration(...)` then `picam2.configure(cfg)` — not `picam2.configure("preview")`.
2. `YOLO("yolov8n_ncnn_model", task='detect')` — `task=` required for NCNN models.
3. `LensPosition=0.67` not `3.0` (3.0 diopters = 33cm focus, not 1.5m).

### `tof.py`

**`init_tof() → VL53L5CX`**
- Initialises sensor, calls `start_ranging()`. Returns sensor handle.

**`get_depth_info(tof) → (dist_mm: int, obstacle: bool)`**
- Non-blocking: if `not tof.data_ready()`, returns `(0, True)` — **fail-safe stop**, not `(FOLLOW_DIST_MM, False)`.
  - Rationale: returning 800mm when sensor has no data tells the robot "you're at perfect distance, no obstacles" — which is false and unsafe. Returning `(0, True)` stops the robot until fresh data arrives.
- Parses 8x8 grid. Center 4x4 zones (`grid[2:6, 2:6]`) → `min_center` for speed control.
- Full grid any-zone check → `obstacle` flag for emergency stop.

### `motor.py`

**`init_serial() → serial.Serial`**
- Opens `SERIAL_PORT` at `BAUD`, `timeout=0.05`. Returns handle.

**`send_motors(ser, left, right)`**
- Clips both to `[-MAX_SPEED, MAX_SPEED]`.
- Sends `f'L{int(left)}R{int(right)}\n'.encode()`.

### `main.py`

40-line control loop. Imports all modules. No logic that belongs in a module.

```
init camera, tof, serial, both PIDs
sleep 2s (Teensy boot)

loop:
  dist_mm, obstacle = get_depth_info(tof)
  if obstacle: send_motors(0, 0); continue

  cx, area = detect_person(picam)
  if cx is None: send_motors(0, 0); continue

  steer = steer_pid.update(cx - CENTER_X)
  speed = speed_pid.update(dist_mm - FOLLOW_DIST_MM)
  speed = clip(BASE_SPEED + speed, -BASE_SPEED, MAX_SPEED)

  send_motors(speed - steer, speed + steer)

on KeyboardInterrupt: send_motors(0, 0), cleanup
```

### `teensy_motor.ino`

- 20kHz hardware PWM on all 4 channels (FlexPWM, analogWriteFrequency).
- Parses `L{n}R{n}\n` via `Serial.readStringUntil('\n')` + `indexOf`.
- `setMotors(int left, int right)`: drives L_RPWM/L_LPWM and R_RPWM/R_LPWM.
- **500ms watchdog:** `if (millis() - lastCmdTime > 500) { setMotors(0,0); }` — halts if RPi5 stops sending.

---

## 2-Day Execution Timeline

### Day 1 — Moving robot, seeing person, obstacle stop

| Block | Time | Task | Exit condition |
|-------|------|------|----------------|
| D1-1 | 45 min | RPi5 OS setup: `apt install`, `pip3 install picamera2 ultralytics pyserial vl53l5cx-ctypes`, enable I2C, set 400kHz in `/boot/firmware/config.txt`, reboot | All imports succeed in Python |
| D1-2 | 30 min | NCNN smoke test: run `tools/export_ncnn.py`, run single inference, print box count | `yolov8n_ncnn_model/` exists; inference returns ≥1 box on a test frame with a person |
| D1-3 | 45 min | Flash `teensy_motor.ino` via Arduino IDE / PlatformIO | `L100R100\n` via serial monitor → both motor sides spin forward |
| D1-4 | 30 min | Serial link test: Python sends `L100R100\n`, `L0R0\n` | Robot drives forward then stops on command |
| D1-5 | 60 min | Write `vision.py`, `motor.py`, `config.py`, minimal `main.py` (centroid steering only, constant speed) | Robot steers toward person; no ToF yet |
| D1-6 | 90 min | Write `tof.py`, verify raw readings | `get_depth_info()` returns ~800mm when hand at arm's length; ~300mm when hand is close |
| D1-7 | 45 min | Wire ToF into `main.py` | Emergency stop triggers when obstacle enters 40cm; resumes when clear |

**Day 1 done:** Robot follows person left/right at constant speed. Stops for obstacles.

---

### Day 2 — Full PID, tuning, demo

| Block | Time | Task | Exit condition |
|-------|------|------|----------------|
| D2-1 | 45 min | Add `pid.py`, wire both PID channels into `main.py` | Speed varies with distance; steering is PID not raw proportional |
| D2-2 | 60 min | Steer PID tuning (§6.7 procedure: Kp only first, then Ki, then Kd) | No oscillation in straight corridor; smooth 90° corner tracking |
| D2-3 | 45 min | Speed PD tuning | Robot holds 800mm ± 150mm at walking speed (~1 m/s) |
| D2-4 | 30 min | Combined following test run | 5-min corridor walk, zero collisions, no target switch |
| D2-5 | 60 min | Buffer — fix whatever broke | Exists because something always breaks |
| D2-6 | 30 min | Demo prep: clean startup script, verify watchdog kills motors on Ctrl+C | Robot halts safely on script exit |

**Day 2 done:** Full demo-ready autonomous following.

---

## Key Risks

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| VL53L5CX I2C setup fails / library compile error | Medium | D1-6 is allocated 90 min. If it runs long, cut D1-7 — centroid-only demo still shows following. |
| NCNN export produces wrong model / slow inference | Low | D1-2 smoke test catches this before any other code exists. Fallback: use PyTorch backend at ~5 FPS — worse but functional. |
| Teensy serial parse drops partial frames | Low | 50ms read timeout on RPi5 side + watchdog on Teensy side handles gracefully. |
| PID steer oscillates badly | Medium | Follow §6.7 in order: Kp-only first. Never tune Kd before Kp is stable. |
| Person detection <10 FPS | Low | Verify `model.info()` confirms NCNN backend loaded. Common mistake: forgetting to export, YOLO silently falls back to PyTorch. |
| Largest-bbox target switch in multi-person corridor | High during demo | Run demo in controlled single-person corridor. This is a known limitation, not a bug. |

---

## PID Tuning Reference

Start with all gains at zero except Kp_steer. Follow this order — skipping steps causes coupled oscillation that is hard to diagnose:

1. **Kp_steer only.** Lift robot, move a person left/right in front of camera. Increase until steer output reacts sharply. Back off 20%.
2. **Ki_steer = 0.001.** Test in corridor. Increase only if robot drifts laterally at steady state.
3. **Kd_steer.** Increase until oscillation on sharp turns is damped. If noisy: decrease `STEER_DERIV_ALPHA` (more filtering).
4. **Kp_speed.** Enable speed channel. Increase until robot accelerates smoothly when target moves away and brakes approaching.
5. **Full combined test.** Adjust `STEER_DEADBAND` if oscillates on straight path (increase) or sluggish on turns (decrease).

All constants in `config.py`. No code changes needed during tuning.

---

## Setup Commands (Day 1 Sequence)

```bash
# 1. System packages
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-opencv -y

# 2. Python packages
pip3 install picamera2 ultralytics pyserial vl53l5cx-ctypes

# 3. Enable I2C
sudo raspi-config  # Interface Options → I2C → Enable

# 4. Set I2C to 400kHz — add this line to /boot/firmware/config.txt:
# dtparam=i2c_arm=on,i2c_arm_baudrate=400000
sudo reboot

# 5. NCNN export (run once from project root)
python3 tools/export_ncnn.py
# Verify: ls yolov8n_ncnn_model/ should show .param and .bin files

# 6. Smoke test inference
python3 -c "
from ultralytics import YOLO
import numpy as np
model = YOLO('yolov8n_ncnn_model', task='detect')
frame = np.zeros((480, 640, 3), dtype=np.uint8)
r = model(frame, classes=[0], conf=0.5, verbose=False)
print('NCNN OK, boxes:', len(r[0].boxes))
"
```

---

## Definition of Done

The project is complete when:

- [ ] `python3 rpms/main.py` starts without errors on the RPi5
- [ ] Robot steers toward a walking person at 0.5–3m range
- [ ] Robot maintains ~800mm following distance (±200mm acceptable)
- [ ] Emergency stop triggers reliably when obstacle enters 40cm
- [ ] Robot halts safely on Ctrl+C (watchdog confirmed)
- [ ] 5-minute corridor following test: zero collisions, no unintended target switch

---

*Spec written from RPMS_Project_Report.md (2026-05-17 revision with all bug fixes applied).*
