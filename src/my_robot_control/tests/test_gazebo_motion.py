#!/usr/bin/env python3
import time
import unittest

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestGazeboMotion(unittest.TestCase):
    """End-to-end test: ensure robot moves in Gazebo when commanded."""

    def setUp(self) -> None:
        if not rospy.core.is_initialized():
            rospy.init_node("test_gazebo_motion", anonymous=True)

    def test_robot_moves_forward(self) -> None:
        topic = "/smart/ground_truth/state"
        try:
            rospy.wait_for_message(topic, Odometry, timeout=30.0)
        except rospy.ROSException as exc:
            self.fail(f"Did not receive ground truth data: {exc}")

        initial = rospy.wait_for_message(topic, Odometry)
        start_x = initial.pose.pose.position.x

        pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)
        rospy.sleep(0.5)

        cmd = Twist()
        cmd.linear.x = 2.0

        duration = 2.0
        end = time.time() + duration
        rate = rospy.Rate(20)
        while time.time() < end and not rospy.is_shutdown():
            pub.publish(cmd)
            rate.sleep()

        pub.publish(Twist())

        final = rospy.wait_for_message(topic, Odometry, timeout=10.0)
        displacement = final.pose.pose.position.x - start_x
        rospy.loginfo("Displacement measured: %.3f m", displacement)
        self.assertGreater(displacement, 0.5, "Robot did not move forward in simulation")


if __name__ == "__main__":
    import rostest

    rostest.rosrun("my_robot_control", "test_gazebo_motion", TestGazeboMotion)
