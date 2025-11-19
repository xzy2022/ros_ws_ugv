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
        self.rear_radius = 0.3  # 默认后轮半径，单位米

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


    def bicycle_model(self, V, Omega):
        """
        运动原语-->自行车模型
        input:
            V: 线速度
            Omega: 角速度
        return: 自行车模型的前轮偏转角度和后轮角速度
        """
        # 若速度过小，则直接视为原地不动
        if math.isclose(V, 0.0, abs_tol=1e-6):
            return 0.0, 0.0

        # 转弯半径 R
        R = V / Omega

        # 自行车模型的几何关系：tan(delta) = L * Omega / V
        ideal_steer = math.atan2(self.wheel_base, R)
        ideal_steer = self.clamp_ideal_steer(ideal_steer)

        # 这里返回的后轮“角速度”保持与输入 V 同尺度，外部如需转为轮角速度可再除以轮半径
        rear_wheel_w = V / self.rear_radius

        return ideal_steer, rear_wheel_w

    def wheel_commands(self, ideal_steer: float, rear_wheel_w: float) -> Tuple[float, float, float, float]:
        """
        自行车模型-->四轮状态
        input:
            ideal_steer: 自行车模型的前轮偏转角度
            rear_wheel_w: 自行车模型的后轮角速度

        return: 两个前轮的偏转角和两个后轮的角速度

        返回 (rear_left_speed, rear_right_speed, steer_left_angle, steer_right_angle)。
        """
        ideal_steer = self.clamp_ideal_steer(ideal_steer)

        if ideal_steer == 0 or self.wheel_base == 0:
            return rear_wheel_w, rear_wheel_w, ideal_steer, ideal_steer

        # 计算转弯半径
        R = self.wheel_base / math.fabs(math.tan(ideal_steer))
        sign = math.copysign(1.0, ideal_steer)

        # 计算各轮转弯半径
        rL_rear = R - (sign * (self.rear_tread / 2.0))
        rR_rear = R + (sign * (self.rear_tread / 2.0))
        rL_front = R - (sign * (self.front_tread / 2.0))
        rR_front = R + (sign * (self.front_tread / 2.0))

        # 计算两个后轮的角速度。注意角速度与线速度成正比，而这里的后轮线速度与转弯半径成正比，因此角速度与转弯半径成正比。
        rear_right_speed = rear_wheel_w * rR_rear / R
        rear_left_speed = rear_wheel_w * rL_rear / R

        # 计算两个前轮的转向角
        steer_left_angle = math.atan2(self.wheel_base, rL_front) * sign
        steer_right_angle = math.atan2(self.wheel_base, rR_front) * sign

        return rear_left_speed, rear_right_speed, steer_left_angle, steer_right_angle
