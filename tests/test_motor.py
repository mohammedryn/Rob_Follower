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
