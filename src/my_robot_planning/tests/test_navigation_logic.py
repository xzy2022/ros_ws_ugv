#!/usr/bin/env python3
import unittest
import math
import csv
import os
import sys
from pathlib import Path

# 确保可以直接运行本测试文件而无需安装包
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from my_robot_planning.navigation import GlobalPathManager, LocalPathPlanner, PathPoint


class TestNavigationLogic(unittest.TestCase):
    def setUp(self):
        # 创建一个临时的 CSV 文件用于测试
        self.csv_filename = "test_path.csv"
        with open(self.csv_filename, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw"])
            # 创建一条直线路径: (0,0) -> (10,0)
            for i in range(11):
                writer.writerow([float(i), 0.0, 0.0])

    def tearDown(self):
        # 测试结束后删除临时文件
        if os.path.exists(self.csv_filename):
            os.remove(self.csv_filename)

    def test_global_path_loading_and_deceleration(self):
        """测试加载 CSV 和末端减速逻辑"""
        target_vel = 5.0
        max_decel = 1.0
        manager = GlobalPathManager(max_decel=max_decel)

        points = manager.load_from_csv(self.csv_filename, target_velocity=target_vel)

        self.assertEqual(len(points), 11)
        self.assertEqual(points[0].x, 0.0)
        self.assertEqual(points[-1].x, 10.0)

        # 验证末端速度是否为 0
        self.assertAlmostEqual(points[-1].velocity, 0.0)

        # 验证倒数第二个点的速度是否符合物理公式 v^2 = u^2 + 2as
        # u=0, a=1, s=1 (距离) -> v = sqrt(0 + 2*1*1) = 1.414
        expected_vel_minus_1 = math.sqrt(2 * max_decel * 1.0)
        self.assertAlmostEqual(points[-2].velocity, expected_vel_minus_1, places=3)

    def test_local_planner_query(self):
        """测试 KDTree 查询和路径截取"""
        points = [PathPoint(x=i, y=0.0, yaw=0.0, velocity=1.0) for i in range(100)]
        planner = LocalPathPlanner(points)

        # 1. 测试寻找最近点
        # 机器人在 (10.1, 0.5)，最近点应该是 index 10 (x=10, y=0)
        idx = planner.get_closest_index(10.1, 0.5)
        self.assertEqual(idx, 10)

        # 2. 测试路径截取 (Local Path Extraction)
        # 从 index 10 开始，截取 20 个点
        local_path = planner.extract_local_path(idx, length=20)
        self.assertEqual(len(local_path), 20)
        self.assertEqual(local_path[0].x, 10.0)
        self.assertEqual(local_path[-1].x, 29.0)

    def test_boundary_conditions(self):
        """测试边界情况"""
        # 靠近终点时截取
        points = [PathPoint(x=i, y=0, yaw=0, velocity=1) for i in range(10)]
        planner = LocalPathPlanner(points)

        # 在 index 8 处截取 5 个点，但只剩 2 个点了
        local_path = planner.extract_local_path(8, 5)
        self.assertEqual(len(local_path), 2)  # index 8 and 9

    def test_empty_csv_handling(self):
        """测试空文件或无数据的情况"""
        empty_csv = "empty.csv"
        with open(empty_csv, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw"]) # 只有表头
        
        try:
            manager = GlobalPathManager()
            points = manager.load_from_csv(empty_csv, target_velocity=5.0)
            self.assertEqual(len(points), 0)
            self.assertIsInstance(points, list)
        finally:
            if os.path.exists(empty_csv):
                os.remove(empty_csv)

    def test_single_point_path(self):
        """测试只有 1 个点的路径"""
        single_pt_csv = "single.csv"
        with open(single_pt_csv, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw"])
            writer.writerow([0.0, 0.0, 0.0])

        try:
            manager = GlobalPathManager()
            points = manager.load_from_csv(single_pt_csv, target_velocity=5.0)
            self.assertEqual(len(points), 1)
            # 只有一个点时，它既是起点也是终点，速度应设为 0
            self.assertEqual(points[0].velocity, 0.0)
        finally:
            if os.path.exists(single_pt_csv):
                os.remove(single_pt_csv)

    def test_insufficient_deceleration_distance(self):
        """测试路径太短无法达到目标速度的情况"""
        # 只有两个点，距离 2.0 米
        # max_decel = 1.0
        # 根据 v^2 = 2as => v = sqrt(2*1*2) = 2.0 m/s
        # 如果 target_velocity 设为 10.0，起点的速度应该被 clamp 到 2.0，而不是 10.0
        
        short_path_csv = "short.csv"
        with open(short_path_csv, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw"])
            writer.writerow([0.0, 0.0, 0.0])
            writer.writerow([2.0, 0.0, 0.0])

        try:
            target_vel = 10.0
            max_decel = 1.0
            manager = GlobalPathManager(max_decel=max_decel)
            points = manager.load_from_csv(short_path_csv, target_velocity=target_vel)

            self.assertEqual(len(points), 2)
            self.assertEqual(points[-1].velocity, 0.0)
            # 验证起点速度是否被物理限制覆盖
            self.assertAlmostEqual(points[0].velocity, 2.0, places=3)
            self.assertNotEqual(points[0].velocity, target_vel)
        finally:
            if os.path.exists(short_path_csv):
                os.remove(short_path_csv)

    def test_missing_optional_columns(self):
        """测试 CSV 缺少可选列 (yaw) 的情况"""
        no_yaw_csv = "no_yaw.csv"
        with open(no_yaw_csv, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"]) # 只有 x, y
            writer.writerow([1.0, 2.0])

        try:
            manager = GlobalPathManager()
            points = manager.load_from_csv(no_yaw_csv, target_velocity=1.0)
            self.assertEqual(points[0].x, 1.0)
            self.assertEqual(points[0].y, 2.0)
            self.assertEqual(points[0].yaw, 0.0) # 默认值应生效
        finally:
            if os.path.exists(no_yaw_csv):
                os.remove(no_yaw_csv)

    def test_empty_planner_initialization(self):
        """测试空列表初始化规划器的鲁棒性"""
        planner = LocalPathPlanner([])
        
        # 验证 tree 是否为 None
        self.assertIsNone(planner.tree)
        
        # 验证查询是否安全返回 0
        idx = planner.get_closest_index(5.0, 5.0)
        self.assertEqual(idx, 0)
        
        # 验证截取是否返回空列表
        path = planner.extract_local_path(0, 10)
        self.assertEqual(path, [])

if __name__ == "__main__":
    unittest.main()
