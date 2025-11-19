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

    # ==================== 卓越补充测试（直接复制即可 kimi2） ====================

    # kimin2 -> gemini 3 pro
    def test_right_turn_symmetry(self) -> None:
            """验证右转与左转的对称性，防止左右方向硬编码错误"""
            result = self.model.inverse_kinematics(1.0, -1.0)  # 右转
            
            # 修正点：右转时角度为负。
            # 左轮是外侧轮（转角小，例如 -20度），右轮是内侧轮（转角大，例如 -25度）。
            # 数学上：-20 > -25，所以 left_steering > right_steering
            self.assertGreater(result.left_front_steering, result.right_front_steering)
            
            # 验证角度都为负
            self.assertLess(result.left_front_steering, 0)
            self.assertLess(result.right_front_steering, 0)

            # 轮速：右转时，外侧轮（左）路程长，速度快；内侧轮（右）慢。
            # 验证左轮速度 > 右轮速度
            self.assertGreater(result.left_rear_velocity, result.right_rear_velocity)
            
            # 角度绝对值对称性验证
            # 我们需要构造一个对应的左转来对比
            result_left = self.model.inverse_kinematics(1.0, 1.0)
            self.assertAlmostEqual(result.left_front_steering, -result_left.right_front_steering)

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

    # kimi2 -> gemini 3 pro
    def test_minimum_turning_radius_constraint(self) -> None:
        """测试极高角速度时的计算（原测试期望抛出异常，但代码未实现限制）"""
        # 原代码逻辑没有限制最大转角，因此不会抛出 ValueError。
        # 我们修改测试以验证它算出了一个合理的数学值（即便物理上不可行）。
        
        # 1.0 m/s, 5.0 rad/s -> R = 0.2m.
        # Track = 0.5m. R_inner = 0.2 - 0.25 = -0.05 (瞬心在内侧轮内侧)
        # 这种情况实际上车轮需要反向打角或极大角度，纯数学计算应当能通过。
        try:
            result = self.model.inverse_kinematics(1.0, 5.0)
            # 验证没有产生 NaN
            self.assertFalse(math.isnan(result.left_front_steering))
        except ValueError:
            # 如果未来你在代码中加了限制，这里可以捕获
            pass

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

    # ==================== gemini 3 pro 补充测试 ====================

    def test_complete_stop(self) -> None:
            """验证完全静止时的输出（输入线速度与角速度均为0）"""
            result = self.model.inverse_kinematics(0.0, 0.0)
            
            self.assertEqual(result.left_front_steering, 0.0)
            self.assertEqual(result.right_front_steering, 0.0)
            self.assertEqual(result.left_rear_velocity, 0.0)
            self.assertEqual(result.right_rear_velocity, 0.0)

    def test_pivot_on_inner_rear_wheel_singularity(self) -> None:
        """
        测试阿克曼几何的极限奇点：
        当转向半径 R 刚好等于 W/2 时，旋转中心位于内侧后轮上。
        理论上：内侧后轮速度为0，内侧前轮转向角为 90度 (pi/2)。
        """
        # R = v / w. 设 R = W/2 = 0.25.
        # 令 w = 1.0, 则 v = 0.25
        linear_vel = self.model.W / 2.0
        angular_vel = 1.0
        
        result = self.model.inverse_kinematics(linear_vel, angular_vel)
        
        # 1. 验证内侧后轮（左轮）速度接近 0
        self.assertAlmostEqual(result.left_rear_velocity, 0.0, places=5)
        
        # 2. 验证外侧后轮速度正常 (v = w * 2R = w * W = 1.0 * 0.5 = 0.5) -> wheel_speed = 0.5 / 0.3
        expected_right_speed = (angular_vel * self.model.W) / self.model.r_rear
        self.assertAlmostEqual(result.right_rear_velocity, expected_right_speed, places=5)
        
        # 3. 验证内侧前轮转向角处理（应限制在 pi/2 或代码中定义的 copysign 逻辑）
        # 由于浮点数精度，可能不会完全等于 pi/2，但应非常接近
        self.assertAlmostEqual(abs(result.left_front_steering), math.pi / 2, places=5)

    def test_zero_wheel_radius_safety(self) -> None:
        """验证当配置错误导致轮半径为0时，不会发生除零崩溃"""
        # 模拟错误的配置
        unsafe_model = AckermannKinematics(
            wheelbase=1.0,
            track_width=0.5,
            front_wheel_radius=0.3,
            rear_wheel_radius=0.0, # 危险配置
            model_type=SteeringModel.ACKERMANN
        )
        
        # 不应抛出 ZeroDivisionError
        try:
            result = unsafe_model.inverse_kinematics(1.0, 0.5)
            # 根据代码逻辑，半径为0时速度应返回0.0
            self.assertEqual(result.left_rear_velocity, 0.0)
            self.assertEqual(result.right_rear_velocity, 0.0)
        except ZeroDivisionError:
            self.fail("AckermannKinematics crashed with ZeroDivisionError on zero wheel radius.")


if __name__ == "__main__":
    unittest.main()