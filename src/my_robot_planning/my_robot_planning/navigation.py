"""Navigation utilities: path loading, deceleration, and KDTree queries."""

from dataclasses import dataclass
import csv
import math
from typing import List, Optional

import numpy as np
from scipy.spatial import KDTree


@dataclass
class PathPoint:
    x: float
    y: float
    yaw: float
    velocity: float = 0.0


class GlobalPathManager:
    def __init__(self, max_decel: float = 1.0):
        self.max_decel = max_decel

    def load_from_csv(self, file_path: str, target_velocity: float) -> List[PathPoint]:
        points: List[PathPoint] = []
        with open(file_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append(
                    PathPoint(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row.get("yaw", 0.0)),
                        velocity=target_velocity,
                    )
                )
        return self._apply_deceleration(points)

    def _distance(self, p1: PathPoint, p2: PathPoint) -> float:
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def _apply_deceleration(self, points: List[PathPoint]) -> List[PathPoint]:
        if not points:
            return []

        points[-1].velocity = 0.0
        for i in range(len(points) - 2, -1, -1):
            dist = self._distance(points[i], points[i + 1])
            limit_vel = math.sqrt(max(0.0, points[i + 1].velocity ** 2 + 2 * self.max_decel * dist))
            points[i].velocity = min(limit_vel, points[i].velocity)
        return points


class LocalPathPlanner:
    def __init__(self, points: List[PathPoint]):
        self.points = points
        self.coords = np.array([[p.x, p.y] for p in points]) if points else np.empty((0, 2))
        self.tree: Optional[KDTree] = KDTree(self.coords) if len(self.coords) else None

    def get_closest_index(self, x: float, y: float) -> int:
        if self.tree is None:
            return 0
        _, idx = self.tree.query([x, y], k=1)
        return int(idx)

    def extract_local_path(self, start_idx: int, length: int) -> List[PathPoint]:
        end_idx = min(start_idx + length, len(self.points))
        return self.points[start_idx:end_idx]

