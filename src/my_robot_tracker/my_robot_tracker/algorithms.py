"""Lateral controllers (pure logic, no ROS)."""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


@dataclass
class ControlCommand:
    steering_angle: float  # radians
    velocity: float        # m/s


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

    def compute(self, state: VehicleState, path: Sequence[PathPoint]) -> float:
        if not path:
            return 0.0

        lookahead_dist = max(self.min_ld, self.k * max(state.velocity, 0.0))
        target = _find_lookahead_point(path, lookahead_dist)
        if target is None:
            return 0.0

        alpha = math.atan2(target.y, target.x)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead_dist)
        return _normalize_angle(delta)


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
