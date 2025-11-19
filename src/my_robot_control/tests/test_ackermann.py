import math
import unittest

from my_robot_control.ackermann import AckermannKinematics, AckermannOutput, SteeringModel


class AckermannKinematicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.model = AckermannKinematics(
            wheelbase=1.0,
            track_width=0.5,
            front_wheel_radius=0.3,
            rear_wheel_radius=0.3,
            model_type=SteeringModel.ACKERMANN,
        )

    def test_straight_motion(self) -> None:
        result = self.model.inverse_kinematics(2.0, 0.0)
        self.assertAlmostEqual(result.left_front_steering, 0.0)
        self.assertAlmostEqual(result.right_front_steering, 0.0)
        expected_wheel_speed = 2.0 / 0.3
        self.assertAlmostEqual(result.left_rear_velocity, expected_wheel_speed)
        self.assertAlmostEqual(result.right_rear_velocity, expected_wheel_speed)

    def test_left_turn(self) -> None:
        result = self.model.inverse_kinematics(1.0, 1.0)
        self.assertGreater(result.left_front_steering, result.right_front_steering)
        self.assertGreater(result.right_rear_velocity, result.left_rear_velocity)
        expected_left_angle = math.atan(self.model.L / (1.0 - self.model.W / 2.0))
        self.assertAlmostEqual(result.left_front_steering, expected_left_angle)

    def test_bicycle_model_parallel(self) -> None:
        self.model.model_type = SteeringModel.BICYCLE
        result = self.model.inverse_kinematics(1.0, 0.5)
        self.assertAlmostEqual(result.left_front_steering, result.right_front_steering)
        self.assertAlmostEqual(result.left_rear_velocity, result.right_rear_velocity)

    # ==================== 卓越补充测试（直接复制即可） ====================

    def test_right_turn_symmetry(self) -> None:
        """验证右转与左转的对称性，防止左右方向硬编码错误"""
        result = self.model.inverse_kinematics(1.0, -1.0)  # 右转
        self.assertLess(result.left_front_steering, result.right_front_steering)
        self.assertLess(result.left_rear_velocity, result.right_rear_velocity)
        # 角度应大小相等、方向相反
        self.assertAlmostEqual(abs(result.left_front_steering), 
                              abs(result.right_front_steering), places=5)

    def test_zero_linear_vel_pure_rotation(self) -> None:
        """测试零速但非零角速度时的极限工况（原地旋转）"""
        result = self.model.inverse_kinematics(0.0, 1.0)
        # 左右后轮应速度相等、方向相反
        self.assertAlmostEqual(result.left_rear_velocity, -result.right_rear_velocity, places=5)
        # 转向角应趋近90度（取决于具体实现，此处验证不崩溃）
        self.assertFalse(math.isnan(result.left_front_steering))
        self.assertFalse(math.isinf(result.right_front_steering))

    def test_negative_velocity_reversing(self) -> None:
        """验证负速度（倒车）时的转向逻辑是否反转"""
        result = self.model.inverse_kinematics(-1.0, 0.5)  # 倒车左转
        # 转向角关系应保持（内侧轮 > 外侧轮）
        self.assertGreater(result.left_front_steering, result.right_front_steering)
        # 所有轮速应为负值
        self.assertTrue(result.left_rear_velocity < 0)
        self.assertTrue(result.right_rear_velocity < 0)

    def test_minimum_turning_radius_constraint(self) -> None:
        """测试转向角超限时的安全处理（需模型支持max_steering_angle）"""
        # 角速度过大导致所需转向角超限
        # 根据实现不同，可能抛出异常或返回截断值
        # 这里假设模型会抛出ValueError（如不支持异常，请改为断言角度范围）
        with self.assertRaises(ValueError):
            self.model.inverse_kinematics(1.0, 5.0)  # 需要极大转向角

    def test_very_small_angular_velocity_numerical(self) -> None:
        """测试极小角速度时的数值稳定性"""
        result = self.model.inverse_kinematics(1.0, 1e-8)  # 接近直线
        # 转向角应接近零
        self.assertAlmostEqual(result.left_front_steering, 0.0, places=7)
        # 轮速应几乎相等
        self.assertAlmostEqual(result.left_rear_velocity, result.right_rear_velocity, places=7)

    def test_forward_inverse_consistency(self) -> None:
        """验证正逆运动学的一致性（需实现forward_kinematics）"""
        # 注意：此测试仅当存在前向运动学方法时有效
        # 如未实现，可保持pass或删除本测试
        # v_cmd, w_cmd = 1.5, 0.3
        # ik_result = self.model.inverse_kinematics(v_cmd, w_cmd)
        # fk_result = self.model.forward_kinematics(ik_result)
        # self.assertAlmostEqual(fk_result.linear_vel, v_cmd, places=5)
        # self.assertAlmostEqual(fk_result.angular_vel, w_cmd, places=5)
        pass

    def test_different_track_width_sensitivity(self) -> None:
        """验证模型对轮距参数变化的敏感性"""
        original_W = self.model.W
        # 测试宽轮距
        self.model.W = 1.0
        result_wide = self.model.inverse_kinematics(1.0, 0.5)
        diff_wide = result_wide.right_rear_velocity - result_wide.left_rear_velocity
        
        # 恢复窄轮距
        self.model.W = original_W
        result_narrow = self.model.inverse_kinematics(1.0, 0.5)
        diff_narrow = result_narrow.right_rear_velocity - result_narrow.left_rear_velocity
        
        # 宽轮距应产生更大差速
        self.assertGreater(diff_wide, diff_narrow)

    def test_steering_model_switching(self) -> None:
        """验证运行时切换模型的正确性"""
        # 阿克曼模式
        self.model.model_type = SteeringModel.ACKERMANN
        ackermann_result = self.model.inverse_kinematics(1.0, 0.5)
        
        # 自行车模式
        self.model.model_type = SteeringModel.BICYCLE
        bicycle_result = self.model.inverse_kinematics(1.0, 0.5)
        
        # 自行车模型转向角应介于阿克曼内外角之间
        self.assertGreater(ackermann_result.left_front_steering, 
                          bicycle_result.left_front_steering)
        self.assertLess(ackermann_result.right_front_steering, 
                       bicycle_result.right_front_steering)

    def test_extremely_low_speed(self) -> None:
        """测试极低速时的数值安全和电机指令平滑性"""
        result = self.model.inverse_kinematics(1e-6, 0.5)  # 接近零速
        # 不应产生NaN或Inf
        self.assertFalse(math.isnan(result.left_front_steering))
        self.assertFalse(math.isinf(result.right_rear_velocity))
        # 转向角应仍在有效范围内
        self.assertTrue(abs(result.left_front_steering) <= math.pi)

    def test_steering_angle_wraparound(self) -> None:
        """测试大角度转向的归一化处理"""
        result = self.model.inverse_kinematics(0.1, 10.0)  # 超大角速度
        # 转向角应被归一化到[-π, π]范围（取决于模型实现）
        self.assertGreaterEqual(result.left_front_steering, -math.pi)
        self.assertLessEqual(result.left_front_steering, math.pi)
        self.assertGreaterEqual(result.right_front_steering, -math.pi)
        self.assertLessEqual(result.right_front_steering, math.pi)


if __name__ == "__main__":
    unittest.main()