"""Simple PID controller for longitudinal control."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: Optional[float] = None
    output_limit: Optional[float] = None


class PID:
    def __init__(self, config: PIDConfig):
        self.cfg = config
        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        if self.cfg.integral_limit is not None:
            limit = abs(self.cfg.integral_limit)
            self.integral = max(-limit, min(limit, self.integral))

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (
            self.cfg.kp * error
            + self.cfg.ki * self.integral
            + self.cfg.kd * derivative
        )

        if self.cfg.output_limit is not None:
            limit = abs(self.cfg.output_limit)
            output = max(-limit, min(limit, output))

        return output
