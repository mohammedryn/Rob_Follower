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
