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
        # alpha=0 → no filter (pass raw through); alpha→1 → heavy smoothing
        self.filtered_derivative = (
            (1.0 - self.derivative_alpha) * raw_derivative
            + self.derivative_alpha * self.filtered_derivative
        )

        output = self.Kp * error + self.Ki * self.integral
        output = max(-self.output_limit, min(self.output_limit, output))
        output += self.Kd * self.filtered_derivative

        self.prev_error = error
        self.prev_time = now
        return output
