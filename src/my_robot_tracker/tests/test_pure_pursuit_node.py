#!/usr/bin/env python3
import unittest
import time

import rospy
from geometry_msgs.msg import Twist
from my_robot_msgs.msg import Lane, Waypoint


class TestPurePursuitNode(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_pp_node_client")
        self.received_cmd = None
        self.path_pub = rospy.Publisher("final_waypoints", Lane, queue_size=1, latch=True)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

    def cmd_cb(self, msg):
        self.received_cmd = msg

    def test_tracking_behavior(self):
        """测试：当收到路径和TF时，节点是否发布了正确的 cmd_vel"""
        time.sleep(2.0)

        lane = Lane()
        lane.header.frame_id = "map"
        lane.header.stamp = rospy.Time.now()

        wp = Waypoint()
        wp.pose.pose.position.x = 2.0
        wp.pose.pose.position.y = 1.0
        wp.twist.twist.linear.x = 1.0
        lane.waypoints.append(wp)

        self.path_pub.publish(lane)

        timeout = time.time() + 2.0
        while self.received_cmd is None and time.time() < timeout:
            time.sleep(0.1)

        self.assertIsNotNone(self.received_cmd, "超时：未收到 /cmd_vel 指令")
        self.assertGreater(self.received_cmd.angular.z, 0.0, "应执行左转，但收到右转或直线指令")
        self.assertAlmostEqual(self.received_cmd.linear.x, 1.0, delta=0.01)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("my_robot_tracker", "test_pure_pursuit_node", TestPurePursuitNode)
