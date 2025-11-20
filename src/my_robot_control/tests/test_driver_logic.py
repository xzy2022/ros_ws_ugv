#!/usr/bin/env python3
import math
import time
import unittest

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TestAckermannDriverLogic(unittest.TestCase):
    """Validates that the driver publishes correct wheel commands for cmd_vel."""

    def setUp(self) -> None:
        if not rospy.core.is_initialized():
            rospy.init_node("test_ackermann_driver_logic", anonymous=True)

        self.received_left = None
        self.received_right = None
        self.timeout = 5.0

        self.sub_left = rospy.Subscriber(
            "/smart/rear_left_velocity_controller/command",
            Float64,
            self._left_callback,
        )
        self.sub_right = rospy.Subscriber(
            "/smart/rear_right_velocity_controller/command",
            Float64,
            self._right_callback,
        )

    def tearDown(self) -> None:
        self.sub_left.unregister()
        self.sub_right.unregister()

    def _left_callback(self, msg: Float64) -> None:
        self.received_left = msg.data

    def _right_callback(self, msg: Float64) -> None:
        self.received_right = msg.data

    def test_cmd_translation(self) -> None:
        """Sending cmd_vel results in expected wheel angular velocity."""
        pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)

        # Allow publisher/subscribers to connect
        rospy.sleep(0.5)

        cmd = Twist()
        cmd.linear.x = 1.5
        cmd.angular.z = 0.0
        pub.publish(cmd)

        end_time = time.time() + self.timeout
        while time.time() < end_time and (self.received_left is None or self.received_right is None):
            rospy.sleep(0.1)

        self.assertIsNotNone(self.received_left, "Left wheel did not receive a command")
        self.assertIsNotNone(self.received_right, "Right wheel did not receive a command")

        wheel_radius = rospy.get_param("/smart/vehicle_physics/wheel_radius", 0.3)
        expected_speed = cmd.linear.x / wheel_radius

        self.assertAlmostEqual(self.received_left, expected_speed, delta=0.5)
        self.assertAlmostEqual(self.received_right, expected_speed, delta=0.5)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("my_robot_control", "test_driver_logic", TestAckermannDriverLogic)
