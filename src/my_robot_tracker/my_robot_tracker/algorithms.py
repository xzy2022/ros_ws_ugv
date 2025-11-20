"""Lateral controllers (pure logic, no ROS)."""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple, Union


@dataclass
class ControlCommand:
    steering_angle: float  # radians
    velocity: float        # m/s


@dataclass
class TrackerOutput:
    steering_angle: float  # delta (rad)
    angular_velocity: float  # omega (rad/s)


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0            # heading in radians
    velocity: float = 0.0       # m/s


@dataclass
class PathPoint:
    x: float
    y: float
    yaw: float = 0.0            # heading of the path at this point (rad)
    velocity: float = 0.0       # desired speed at this point (m/s)


class LateralController(ABC):
    @abstractmethod
    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        """Return steering angle in radians."""
        raise NotImplementedError


def _normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _find_lookahead_point(path: Sequence[PathPoint], lookahead: float) -> Optional[PathPoint]:
    """Return the first point whose distance from origin exceeds lookahead."""
    for p in path:
        if math.hypot(p.x, p.y) >= lookahead:
            return p
    return path[-1] if path else None


def _find_nearest_point(path: Sequence[PathPoint]) -> Optional[PathPoint]:
    """Assumes vehicle at origin, path already in vehicle frame."""
    if not path:
        return None
    return min(path, key=lambda p: p.x * p.x + p.y * p.y)


class PurePursuit(LateralController):
    def __init__(self, wheelbase: float, ld_gain: float, min_ld: float):
        self.L = wheelbase
        self.k = ld_gain
        self.min_ld = min_ld

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
        Dual-use compute:
        - Legacy signature: compute(state, path) -> steering angle
        - PPT signature: compute(v_target=?, lateral_error=?, lookahead_dist=?)
          -> TrackerOutput (delta, omega)
        """
        # PPT-style direct computation branch
        if v_target is not None and lateral_error is not None and lookahead_dist is not None:
            return self.compute_command(v_target, lateral_error, lookahead_dist)

        # Legacy branch: state + path -> steering
        if not path:
            return 0.0

        lookahead = max(self.min_ld, self.k * max(state.velocity, 0.0) if state else self.min_ld)
        target = _find_lookahead_point(path, lookahead)
        if target is None:
            return 0.0

        alpha = math.atan2(target.y, target.x)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        return _normalize_angle(delta)

    # === Convenience API matching PPT签名 (基于横向误差与预瞄距离) ===
    def compute_command(self, v_target: float, lateral_error: float, lookahead_dist: float) -> TrackerOutput:
        """Compute steering and yaw rate using precomputed lookahead geometry."""
        if abs(lookahead_dist) < 1e-6:
            return TrackerOutput(0.0, 0.0)

        curvature = (2.0 * lateral_error) / (lookahead_dist ** 2)
        delta = math.atan(self.L * curvature)
        omega = (v_target / self.L) * math.tan(delta) if abs(self.L) > 1e-6 else 0.0
        return TrackerOutput(steering_angle=_normalize_angle(delta), angular_velocity=omega)

    @staticmethod
    def find_lookahead_point(path_points: Sequence[Tuple[float, float, float]], current_x: float, current_y: float, lookahead_dist: float) -> Optional[Tuple[float, float, float]]:
        """简单几何搜索：返回第一个距离 >= lookahead 的全局坐标点 (x, y, v)."""
        for p in path_points:
            dx = p[0] - current_x
            dy = p[1] - current_y
            if math.hypot(dx, dy) >= lookahead_dist:
                return p
        return path_points[-1] if path_points else None


class Stanley(LateralController):
    def __init__(self, k_gain: float, k_soft: float):
        self.k = k_gain
        self.k_soft = k_soft

    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        nearest = _find_nearest_point(path)
        if nearest is None:
            return 0.0

        # With path already in vehicle frame, cross-track error is simply lateral offset.
        cross_track_error = nearest.y
        heading_error = _normalize_angle(nearest.yaw - state.yaw)
        vel = max(state.velocity, 0.0)

        delta = heading_error + math.atan2(self.k * cross_track_error, vel + self.k_soft)
        return _normalize_angle(delta)
