import serial
from rpms.config import SERIAL_PORT, BAUD, MAX_SPEED


def init_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)


def send_motors(ser: serial.Serial, left: float, right: float) -> None:
    left  = int(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right = int(max(-MAX_SPEED, min(MAX_SPEED, right)))
    ser.write(f'L{left}R{right}\n'.encode())
