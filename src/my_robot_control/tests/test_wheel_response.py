#!/usr/bin/env python3
import time
import unittest

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

"""
    测试四个轮子的控制响应性能，包括前轮的转向位置控制和后轮的速度控制。
    从而分析是否要为前轮增加控制器PID，或者使用什么控制器

"""

class JointStateMonitor:
    """Tracks latest joint positions and velocities for assertions."""

    def __init__(self, topic: str):
        self.latest_positions = {}
        self.latest_velocities = {}
        self.received_first_msg = False
        self.sub = rospy.Subscriber(topic, JointState, self.callback)

    def callback(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            self.latest_positions[name] = msg.position[i]
            self.latest_velocities[name] = msg.velocity[i]
        self.received_first_msg = True

    def get_state(self, joint_name: str):
        if joint_name not in self.latest_positions:
            return None, None
        return self.latest_positions[joint_name], self.latest_velocities[joint_name]


class TestWheelResponse(unittest.TestCase):
    def setUp(self) -> None:
        if not rospy.core.is_initialized():
            rospy.init_node("test_wheel_response", anonymous=True)

        self.robot_ns = "/smart"
        self.monitor = JointStateMonitor(f"{self.robot_ns}/joint_states")

        timeout = time.time() + 10.0
        while not self.monitor.received_first_msg and time.time() < timeout:
            rospy.sleep(0.1)
        if not self.monitor.received_first_msg:
            self.fail("Could not receive joint_states; is Gazebo/controllers running?")

        self.pub_steer_l = rospy.Publisher(
            f"{self.robot_ns}/front_left_steering_position_controller/command",
            Float64,
            queue_size=1,
        )
        self.pub_steer_r = rospy.Publisher(
            f"{self.robot_ns}/front_right_steering_position_controller/command",
            Float64,
            queue_size=1,
        )
        self.pub_vel_l = rospy.Publisher(
            f"{self.robot_ns}/rear_left_velocity_controller/command", Float64, queue_size=1
        )
        self.pub_vel_r = rospy.Publisher(
            f"{self.robot_ns}/rear_right_velocity_controller/command", Float64, queue_size=1
        )

        rospy.sleep(1.0)

    def test_steering_response_effort(self) -> None:
        joint_l = "front_left_steering_joint"
        joint_r = "front_right_steering_joint"

        test_cases = [
            (0.0, 0.05, 2.0),
            (0.3, 0.05, 2.0),
            (-0.3, 0.05, 2.0),
            (0.5, 0.10, 2.0),
        ]

        for target_angle, tolerance, wait_time in test_cases:
            self.pub_steer_l.publish(Float64(target_angle))
            self.pub_steer_r.publish(Float64(target_angle))
            rospy.sleep(wait_time)

            real_pos_l, _ = self.monitor.get_state(joint_l)
            real_pos_r, _ = self.monitor.get_state(joint_r)

            if real_pos_l is None or real_pos_r is None:
                self.fail(f"Joint names {joint_l}/{joint_r} not found in joint_states")

            error_l = abs(real_pos_l - target_angle)
            error_r = abs(real_pos_r - target_angle)
            max_err = max(error_l, error_r)
            rospy.loginfo(
                "Steer target %.3f | actual L %.3f R %.3f | max_err %.4f",
                target_angle,
                real_pos_l,
                real_pos_r,
                max_err,
            )

            self.assertLess(error_l, tolerance, f"Left steering error {error_l:.3f} exceeds {tolerance}")
            self.assertLess(error_r, tolerance, f"Right steering error {error_r:.3f} exceeds {tolerance}")

    def test_rear_velocity_response_god_mode(self) -> None:
        joint_l = "rear_left_wheel_joint"
        joint_r = "rear_right_wheel_joint"

        test_cases = [
            (0.0, 0.001, 1.0),
            (5.0, 0.001, 1.0),
            (10.0, 0.001, 1.0),
            (-5.0, 0.001, 1.0),
        ]

        for target_vel, tolerance, wait_time in test_cases:
            self.pub_vel_l.publish(Float64(target_vel))
            self.pub_vel_r.publish(Float64(target_vel))
            rospy.sleep(wait_time)

            _, real_vel_l = self.monitor.get_state(joint_l)
            _, real_vel_r = self.monitor.get_state(joint_r)

            if real_vel_l is None or real_vel_r is None:
                self.fail("Rear wheel joints not found in joint_states")

            error_l = abs(real_vel_l - target_vel)
            error_r = abs(real_vel_r - target_vel)
            max_err = max(error_l, error_r)
            rospy.loginfo(
                "Rear target %.2f | actual L %.3f R %.3f | max_err %.4f",
                target_vel,
                real_vel_l,
                real_vel_r,
                max_err,
            )

            self.assertLess(error_l, tolerance, f"Left wheel velocity error {error_l:.3f} exceeds {tolerance}")
            self.assertLess(error_r, tolerance, f"Right wheel velocity error {error_r:.3f} exceeds {tolerance}")


if __name__ == "__main__":
    import rostest

    rostest.rosrun("my_robot_control", "test_wheel_response", TestWheelResponse)
