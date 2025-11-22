"""
Lateral controllers (pure logic, no ROS).
横向控制器算法库（纯逻辑实现，不依赖 ROS）。
包含纯跟踪（Pure Pursuit）和 Stanley 两种常见的路径跟踪算法。
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple, Union


@dataclass
class ControlCommand:
    """基础控制命令数据结构"""
    steering_angle: float  # 转向角/前轮偏转角 (弧度 radians)
    velocity: float        # 线速度 (米/秒 m/s)


@dataclass
class TrackerOutput:
    """
    跟踪器详细输出
    对应阿克曼底盘所需的控制量
    """
    steering_angle: float    # delta: 前轮偏转角 (rad)
    angular_velocity: float  # omega: 车辆角速度 (rad/s)


@dataclass
class VehicleState:
    """车辆状态描述"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0            # 航向角 (弧度)
    velocity: float = 0.0       # 当前线速度 (m/s)


@dataclass
class PathPoint:
    """路径点定义"""
    x: float
    y: float
    yaw: float = 0.0            # 该点的期望航向 (rad)
    velocity: float = 0.0       # 该点的期望速度 (m/s)


class LateralController(ABC):
    """横向控制器抽象基类"""
    @abstractmethod
    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        """计算转向角 (radians)"""
        raise NotImplementedError


def _normalize_angle(angle: float) -> float:
    """
    角度归一化
    将角度限制在 [-pi, pi] 范围内，防止角度累积过大导致计算错误。
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _find_lookahead_point(path: Sequence[PathPoint], lookahead: float) -> Optional[PathPoint]:
    """
    查找局部路径中的预瞄点。
    假设：path 已经在车辆坐标系下（车辆位于原点 (0,0)）。
    逻辑：返回第一个距离原点超过 lookahead 距离的点。
    """
    for p in path:
        if math.hypot(p.x, p.y) >= lookahead:
            return p
    return path[-1] if path else None


def _find_nearest_point(path: Sequence[PathPoint]) -> Optional[PathPoint]:
    """
    查找离车辆最近的路径点。
    假设：车辆位于原点 (0,0)，path 已经在车辆坐标系下。
    """
    if not path:
        return None
    # 使用欧几里得距离平方最小化来寻找最近点
    return min(path, key=lambda p: p.x * p.x + p.y * p.y)


class PurePursuit(LateralController):
    """
    纯跟踪算法 (Pure Pursuit) 实现类
    基于几何关系的路径跟踪算法。
    """
    def __init__(self, wheelbase: float, ld_gain: float, min_ld: float):
        self.L = wheelbase      # 车辆轴距 (m)
        self.k = ld_gain        # 预瞄距离增益系数 (lookahead = k * v + min_ld)
        self.min_ld = min_ld    # 最小预瞄距离 (m)

    def compute(
        self,
        state: Optional[VehicleState] = None,
        path: Optional[Sequence[PathPoint]] = None,
        *,
        v_target: Optional[float] = None,
        lateral_error: Optional[float] = None,
        lookahead_dist: Optional[float] = None,
    ) -> Union[float, TrackerOutput]:
        result_type: Union[float, TrackerOutput]
        """
        双重用途计算函数：
        1. 传统模式 (Legacy): compute(state, path) -> 返回 steering angle (float)
           用于处理局部坐标系下的路径点列表。
        
        2. 参数化模式 (PPT signature): compute(v_target=..., lateral_error=..., lookahead_dist=...)
           -> 返回 TrackerOutput (包含 delta 和 omega)
           直接基于计算好的横向误差和预瞄距离进行控制律计算。
        """
        # === 模式 2: 直接参数计算模式 (对应之前的 ROS Node 逻辑) ===
        if v_target is not None and lateral_error is not None and lookahead_dist is not None:
            return self.compute_command(v_target, lateral_error, lookahead_dist)

        # === 模式 1: 传统状态+路径模式 ===
        if not path:
            return 0.0

        # 动态计算预瞄距离：Ld = k * v + min_Ld
        lookahead = max(self.min_ld, self.k * max(state.velocity, 0.0) if state else self.min_ld)
        
        # 在局部路径中寻找预瞄点
        target = _find_lookahead_point(path, lookahead)
        if target is None:
            return 0.0

        # 计算目标点相对于车头的角度 alpha
        alpha = math.atan2(target.y, target.x)
        
        # 纯跟踪核心公式：delta = arctan( 2*L*sin(alpha) / Ld )
        # 这里的 2*sin(alpha)/Ld 其实等价于 curvature (曲率)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        return _normalize_angle(delta)

    # === 便利 API：匹配实验原理的核心算法 ===
    def compute_command(self, v_target: float, lateral_error: float, lookahead_dist: float) -> TrackerOutput:
        """
        根据预先计算好的几何参数计算转向角和角速度。
        
        参数:
            v_target: 期望线速度 (m/s)
            lateral_error: 横向误差 y_d (车辆坐标系下目标点的 y 坐标)
            lookahead_dist: 预瞄距离 L_d
        """
        if abs(lookahead_dist) < 1e-6:
            return TrackerOutput(0.0, 0.0)

        # 1. 计算圆弧曲率 (Curvature)
        # 公式推导: R = L_d^2 / (2 * y_d)  =>  kappa = 1/R = 2 * y_d / L_d^2
        curvature = (2.0 * lateral_error) / (lookahead_dist ** 2)
        
        # 2. 计算前轮偏转角 delta
        # 公式: delta = arctan(L * kappa) = arctan(2 * L * y_d / L_d^2)
        delta = math.atan(self.L * curvature)
        
        # 3. 计算车辆角速度 omega (基于阿克曼运动学)
        # 公式: omega = (v / L) * tan(delta)
        # 代入 tan(delta) 后简化可得: omega = 2 * v * y_d / L_d^2
        omega = (v_target / self.L) * math.tan(delta) if abs(self.L) > 1e-6 else 0.0
        
        return TrackerOutput(steering_angle=_normalize_angle(delta), angular_velocity=omega)

    @staticmethod
    def find_lookahead_point(path_points: Sequence[Tuple[float, float, float]], current_x: float, current_y: float, lookahead_dist: float) -> Optional[Tuple[float, float, float]]:
        """
        简单的几何搜索工具方法（静态方法）。
        功能：在全局路径点中，找到第一个距离当前位置 (current_x, current_y) 大于等于 lookahead_dist 的点。
        
        返回: (x, y, v) 形式的元组，即目标预瞄点。
        """
        for p in path_points:
            dx = p[0] - current_x
            dy = p[1] - current_y
            # math.hypot(dx, dy) 计算欧几里得距离 sqrt(dx^2 + dy^2)
            if math.hypot(dx, dy) >= lookahead_dist:
                return p
        # 如果所有点都在预瞄距离内，则返回最后一个点作为目标
        return path_points[-1] if path_points else None


import math
import numpy as np
from typing import Sequence, Tuple

class Stanley(LateralController):
    """
    Stanley 路径跟踪控制器
    
    修正点：
    适配输入状态为 base_link (基座) 而非后轴中心的情况。
    """
    def __init__(self, k_gain: float, k_soft: float = 0.0, base_to_front_dist: float = 0.923):
        """
        :param k_gain: 横向误差增益 k
        :param k_soft: 速度软化参数
        :param base_to_front_dist: 【关键参数】从输入的车辆坐标系原点(base_link)到前轮中心的纵向距离。
                                   根据你的 URDF，此处应设为 0.923。
        """
        self.k = k_gain
        self.k_soft = k_soft
        # 这里 L 不再单纯指轴距，而是指“感知中心到控制中心”的距离
        # base_link --> 前轴中心的距离
        self.L_offset = base_to_front_dist 

    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        """
        计算 Stanley 转向角
        
        :param state: 车辆状态 (来自 base_link 的位姿: x, y, yaw, velocity)
        :param path: 规划路径点列表 (全局坐标系)
        :return: 前轮转向角 delta (rad)
        """
        if not path:
            return 0.0

        # ---------------------------------------------------------
        # 1. [修正逻辑] 推算前轴中心位置 (Front Axle Position)
        # ---------------------------------------------------------
        # 既然 state 是 base_link 的位姿，
        # 前轴位置 = base_link位置 + (base_link到前轴的距离) * 方向向量
        fx = state.x + self.L_offset * math.cos(state.yaw)
        fy = state.y + self.L_offset * math.sin(state.yaw)

        # ---------------------------------------------------------
        # 2. 寻找离“前轴”最近的路径点
        # ---------------------------------------------------------
        nearest_idx, nearest_point = self._find_nearest_point_to_front_axle(fx, fy, path)
        
        # ---------------------------------------------------------
        # 3. 计算横向误差 (Cross-track Error)
        # ---------------------------------------------------------
        # 使用欧几里得距离计算绝对误差
        dx = fx - nearest_point.x
        dy = fy - nearest_point.y
        absolute_error = math.hypot(dx, dy)

        # 判定误差符号 (使用几何投影法)
        # 将误差向量投影到路径点的局部坐标系y轴上
        path_yaw = nearest_point.yaw
        local_y = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)

        # 算法中约定路在车左为正，路在车右为负，与上式结果相反
        local_y = - local_y  
        
        # 确定带符号的横向误差
        current_cross_track_error = absolute_error * (1.0 if local_y > 0 else -1.0)

        # ---------------------------------------------------------
        # 4. 计算航向误差 (Heading Error)
        # ---------------------------------------------------------
        heading_error = _normalize_angle(path_yaw - state.yaw)

        # ---------------------------------------------------------
        # 5. 计算最终控制律
        # ---------------------------------------------------------
        vel = max(state.velocity, 0.01) 
        
        # 非线性反馈项
        cross_track_steering = math.atan2(self.k * current_cross_track_error, vel + self.k_soft)
        
        delta = heading_error + cross_track_steering
        return _normalize_angle(delta)

    def _find_nearest_point_to_front_axle(self, fx: float, fy: float, path: Sequence[PathPoint]) -> Tuple[int, PathPoint]:
        """寻找离前轴坐标 (fx, fy) 最近的路径点"""
        min_dist_sq = float('inf')
        nearest_idx = -1
        
        for i, p in enumerate(path):
            d_sq = (fx - p.x)**2 + (fy - p.y)**2
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i
        
        return nearest_idx, path[nearest_idx]

def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def _normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle