import unittest
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from my_robot_tracker.algorithms import PurePursuit


class TestPurePursuitLogic(unittest.TestCase):
    def setUp(self):
        # 初始化：设轴距 L=1.0，方便口算验证
        self.wheelbase = 1.0
        self.controller = PurePursuit(wheelbase=self.wheelbase, ld_gain=0.0, min_ld=0.0)

    def test_straight_line(self):
        """场景1：目标点在正前方 (无横向误差)"""
        output = self.controller.compute(v_target=1.0, lateral_error=0.0, lookahead_dist=2.0)
        self.assertAlmostEqual(output.steering_angle, 0.0)
        self.assertAlmostEqual(output.angular_velocity, 0.0)

    def test_left_turn_geometry(self):
        """场景2：典型左转几何验证 (与 PPT 公式对齐)"""
        L_d = 2.0
        y_d = 1.0
        target_v = 2.0

        output = self.controller.compute(v_target=target_v, lateral_error=y_d, lookahead_dist=L_d)

        expected_delta = math.atan(0.5)
        self.assertAlmostEqual(output.steering_angle, expected_delta, places=4)
        self.assertAlmostEqual(output.angular_velocity, 1.0, places=4)

    def test_right_turn(self):
        """场景3：右转 (y_d 为负)"""
        output = self.controller.compute(v_target=1.0, lateral_error=-1.0, lookahead_dist=2.0)
        self.assertLess(output.steering_angle, 0.0)
        self.assertLess(output.angular_velocity, 0.0)

    def test_zero_speed(self):
        """场景4：速度为0"""
        output = self.controller.compute(v_target=0.0, lateral_error=1.0, lookahead_dist=2.0)
        self.assertNotEqual(output.steering_angle, 0.0)
        self.assertAlmostEqual(output.angular_velocity, 0.0)


if __name__ == "__main__":
    unittest.main()
