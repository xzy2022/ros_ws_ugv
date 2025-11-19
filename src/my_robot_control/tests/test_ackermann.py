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


if __name__ == "__main__":
    unittest.main()
