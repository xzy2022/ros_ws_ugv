#!/usr/bin/env python3
"""
阿克曼转向几何辅助工具，以可复用类的形式实现。
"""
import math
from typing import Tuple


class AckermannKinematics:
    def __init__(self, wheel_base: float, front_tread: float, rear_tread: float, max_inside_angle: float):
        # 轴距
        self.wheel_base = wheel_base
        # 前后轮距
        self.front_tread = front_tread
        self.rear_tread = rear_tread
        # 最大内侧轮转向角（弧度）
        self.max_inside_angle = max_inside_angle

        self._validate_params()
        self.max_ideal_steer = self._compute_max_ideal_steer()

    def _validate_params(self):
        for name, val in [
            ("wheel_base", self.wheel_base),
            ("front_tread", self.front_tread),
            ("rear_tread", self.rear_tread),
            ("max_inside_angle", self.max_inside_angle),
        ]:
            if val <= 0:
                raise ValueError(f"{name} must be > 0, got {val}")

    def _compute_max_ideal_steer(self) -> float:
        # tan(maxsteerInside) = wheelbase / radius
        r_max = self.wheel_base / math.tan(self.max_inside_angle)
        r_ideal = r_max + (self.front_tread / 2.0)
        return math.atan2(self.wheel_base, r_ideal)

    def clamp_ideal_steer(self, steer: float) -> float:
        """将理想转向角限制在可行范围内。"""
        return max(-self.max_ideal_steer, min(self.max_ideal_steer, steer))

    def wheel_commands(self, ideal_steer: float, rear_wheel_speed: float) -> Tuple[float, float, float, float]:
        """
        给定期望的中间轮胎转向角和后轮角速度，（自行车模型的前轮转向角和后轮角速度）
        计算每个后轮的速度和前轮的转向角。

        返回 (rear_left_speed, rear_right_speed, steer_left_angle, steer_right_angle)。
        """
        if ideal_steer == 0 or self.wheel_base == 0:
            return rear_wheel_speed, rear_wheel_speed, ideal_steer, ideal_steer

        # 计算转弯半径
        r = self.wheel_base / math.fabs(math.tan(ideal_steer))
        sign = math.copysign(1.0, ideal_steer)

        rL_rear = r - (sign * (self.rear_tread / 2.0))
        rR_rear = r + (sign * (self.rear_tread / 2.0))
        rL_front = r - (sign * (self.front_tread / 2.0))
        rR_front = r + (sign * (self.front_tread / 2.0))

        rear_right_speed = rear_wheel_speed * rR_rear / r
        rear_left_speed = rear_wheel_speed * rL_rear / r

        steer_left_angle = math.atan2(self.wheel_base, rL_front) * sign
        steer_right_angle = math.atan2(self.wheel_base, rR_front) * sign

        return rear_left_speed, rear_right_speed, steer_left_angle, steer_right_angle