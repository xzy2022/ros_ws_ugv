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


class Stanley(LateralController):
    """
    Stanley 控制器实现类
    基于前轮反馈控制的路径跟踪算法，考虑了横向误差和航向误差。
    """
    def __init__(self, k_gain: float, k_soft: float):
        self.k = k_gain      # 横向误差增益 (Gain)
        self.k_soft = k_soft # 软化参数，防止低速时分母过小导致转向剧烈抖动

    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        """
        计算 Stanley 转向角。
        注意：此处的 path 预期已经转换到了车辆坐标系下 (Vehicle Frame)。
        """
        # 找到最近的路径点（即前轴中心在路径上的投影点）
        nearest = _find_nearest_point(path)
        if nearest is None:
            return 0.0

        # 1. 横向误差 (Cross-track Error)
        # 在车辆坐标系下，最近点的 y 坐标即为横向误差
        cross_track_error = nearest.y
        
        # 2. 航向误差 (Heading Error)
        # 路径点的航向减去车辆当前的航向
        heading_error = _normalize_angle(nearest.yaw - state.yaw)
        
        # 获取当前速度，确保不为负
        vel = max(state.velocity, 0.0)

        # 3. 组合控制律
        # delta = 航向误差 + arctan( k * 横向误差 / (速度 + 软化系数) )
        delta = heading_error + math.atan2(self.k * cross_track_error, vel + self.k_soft)
        
        return _normalize_angle(delta)