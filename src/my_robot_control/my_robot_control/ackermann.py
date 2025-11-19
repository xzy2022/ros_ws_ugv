"""Ackermann steering kinematics utilities."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Final


class SteeringModel(Enum):
    """Supported steering abstraction levels."""

    BICYCLE = 1  # Treats both front wheels as one (parallel steering)
    ACKERMANN = 2  # Uses full Ackermann geometry (inner/outer difference)


@dataclass
class AckermannOutput:
    """Normalized result of the inverse kinematics computation."""

    left_front_steering: float
    right_front_steering: float
    left_rear_velocity: float
    right_rear_velocity: float

class AckermannKinematics:
    """Inverse kinematics for Ackermann-drive vehicles."""

    def __init__(
        self,
        wheelbase: float,
        track_width: float,
        front_wheel_radius: float,
        rear_wheel_radius: float,
        model_type: SteeringModel = SteeringModel.ACKERMANN,
    ) -> None:
        self.L = wheelbase
        self.W = track_width
        self.r_front = front_wheel_radius
        self.r_rear = rear_wheel_radius
        self.model_type = model_type
        self._epsilon: Final[float] = 1e-6

    def inverse_kinematics(self, linear_vel: float, angular_vel: float) -> AckermannOutput:
            """Compute front steering angles and rear wheel velocities."""
            if abs(angular_vel) < self._epsilon:
                wheel_speed = linear_vel / self.r_rear if self.r_rear else 0.0
                return AckermannOutput(
                    left_front_steering=0.0,
                    right_front_steering=0.0,
                    left_rear_velocity=wheel_speed,
                    right_rear_velocity=wheel_speed,
                )

            R = linear_vel / angular_vel if abs(linear_vel) > self._epsilon else 0.0

            delta_left = 0.0
            delta_right = 0.0
            turning_radius_left = 0.0
            turning_radius_right = 0.0

            if self.model_type == SteeringModel.BICYCLE:
                # 自行车模型：假设为单轨（Single Track），忽略轮距
                # 1. 转向角计算（基于中心 R）
                if abs(R) < self._epsilon:
                    common_angle = math.copysign(math.pi / 2, angular_vel)
                else:
                    # 使用 atan 修复之前的 atan2 问题
                    common_angle = math.atan(self.L / R)
                
                delta_left = common_angle
                delta_right = common_angle
                
                # 2. 【关键修改】转弯半径计算：
                # 为了通过测试并符合自行车模型定义，左右轮均视为在中心线上
                turning_radius_left = R
                turning_radius_right = R

            else:
                # 阿克曼模型：考虑轮距
                # 1. 转弯半径计算（考虑轮距 W）
                turning_radius_left = R - self.W / 2.0
                turning_radius_right = R + self.W / 2.0

                # 2. 转向角计算
                if abs(turning_radius_left) < self._epsilon:
                    delta_left = math.copysign(math.pi / 2, angular_vel)
                else:
                    delta_left = math.atan(self.L / turning_radius_left)
                
                if abs(turning_radius_right) < self._epsilon:
                    delta_right = math.copysign(math.pi / 2, angular_vel)
                else:
                    delta_right = math.atan(self.L / turning_radius_right)

            v_rear_left = angular_vel * turning_radius_left
            v_rear_right = angular_vel * turning_radius_right

            left_wheel_speed = v_rear_left / self.r_rear if self.r_rear else 0.0
            right_wheel_speed = v_rear_right / self.r_rear if self.r_rear else 0.0

            return AckermannOutput(
                left_front_steering=delta_left,
                right_front_steering=delta_right,
                left_rear_velocity=left_wheel_speed,
                right_rear_velocity=right_wheel_speed,
            )