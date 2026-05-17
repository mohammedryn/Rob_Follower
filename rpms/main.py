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
