#!/usr/bin/env python3
import time
import unittest

import rospy
from nav_msgs.msg import Odometry
from my_robot_msgs.msg import Lane


class TestNavigationStack(unittest.TestCase):
    def setUp(self):
        self.node_name = "test_navigation_stack"
        if not rospy.core.is_initialized():
            rospy.init_node(self.node_name, anonymous=True)

        # 1. 订阅全局路径 (验证 GlobalLoader)
        self.global_lane = None
        rospy.Subscriber("/base_waypoints", Lane, self.global_cb, queue_size=1)

        # 2. 订阅局部路径 (验证 LocalPlanner)
        self.local_lane = None
        rospy.Subscriber("/final_waypoints", Lane, self.local_cb, queue_size=1)

        # 3. 准备发布假的里程计数据 (模拟机器人移动)
        self.odom_pub = rospy.Publisher("/smart/ground_truth/state", Odometry, queue_size=1)

        # 等待节点启动和话题连接
        rospy.sleep(2.0)

    def global_cb(self, msg):
        self.global_lane = msg

    def local_cb(self, msg):
        self.local_lane = msg

    def test_1_global_path_loading(self):
        """测试 Part 2: 全局路径加载器是否工作正常"""
        timeout = time.time() + 5.0
        while self.global_lane is None and time.time() < timeout:
            rospy.sleep(0.1)

        self.assertIsNotNone(self.global_lane, "Global path was not published!")

        # 验证 CSV 加载的数据量 (CSV有11个点)
        self.assertEqual(len(self.global_lane.waypoints), 11, "Should load 11 waypoints from csv")

        # 验证减速逻辑: 最后一个点速度必须为 0
        last_wp = self.global_lane.waypoints[-1]
        self.assertAlmostEqual(
            last_wp.twist.twist.linear.x, 0.0, places=3, msg="Last waypoint velocity should be 0.0"
        )

        # 验证起始速度: 应该接近设定的 10km/h (2.77 m/s)
        first_wp = self.global_lane.waypoints[0]
        self.assertAlmostEqual(
            first_wp.twist.twist.linear.x,
            10.0 / 3.6,
            places=1,
            msg="Start velocity incorrect",
        )

    def test_2_local_planner_logic(self):
        """测试 Part 3 & 4: 局部规划器是否能根据 Odometry 截取路径"""

        # 确保全局路径已经加载
        if self.global_lane is None:
            rospy.wait_for_message("/base_waypoints", Lane, timeout=5.0)

        # === 场景 A: 机器人位于起点 (0,0) ===
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        self.odom_pub.publish(odom)

        # 等待局部规划器响应
        self.local_lane = None
        for _ in range(10):
            self.odom_pub.publish(odom)
            rospy.sleep(0.1)
            if self.local_lane is not None:
                break

        self.assertIsNotNone(self.local_lane, "Local planner did not publish /final_waypoints")

        # 验证: 在 (0,0) 时，局部路径应该从 index 0 开始
        # 假设 lookahead_wps = 5 (在 .test 文件里设置)
        self.assertEqual(len(self.local_lane.waypoints), 5, "Local path length mismatch")
        self.assertAlmostEqual(self.local_lane.waypoints[0].pose.pose.position.x, 0.0)

        # === 场景 B: 机器人向前移动到 (3.1, 0.1) ===
        self.local_lane = None  # 清空之前的接收
        odom.pose.pose.position.x = 3.1
        odom.pose.pose.position.y = 0.1

        start_wait = time.time()
        while time.time() - start_wait < 2.0:
            self.odom_pub.publish(odom)
            rospy.sleep(0.1)
            if self.local_lane is not None and self.local_lane.waypoints[0].pose.pose.position.x > 0.1:
                break

        self.assertIsNotNone(self.local_lane, "Local planner did not publish after movement")

        first_local_x = self.local_lane.waypoints[0].pose.pose.position.x
        self.assertAlmostEqual(
            first_local_x,
            3.0,
            delta=0.1,
            msg=f"Expected closest point x=3.0, got {first_local_x}",
        )

        # 期望: x=3,4,5,6,7
        last_local_x = self.local_lane.waypoints[-1].pose.pose.position.x
        self.assertAlmostEqual(last_local_x, 7.0, delta=0.1)

    def test_3_end_of_path_behavior(self):
        """测试终点附近剩余点数不足 lookahead 时的行为"""
        if self.global_lane is None:
            rospy.wait_for_message("/base_waypoints", Lane, timeout=5.0)

        odom = Odometry()
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = 9.8  # 接近终点 x=10.0
        odom.pose.pose.position.y = 0.0

        self.local_lane = None
        for _ in range(10):
            self.odom_pub.publish(odom)
            rospy.sleep(0.1)
            if self.local_lane is not None:
                break

        self.assertIsNotNone(self.local_lane, "Planner failed near end of path")

        count = len(self.local_lane.waypoints)
        self.assertTrue(0 < count <= 5, f"Expected 1-5 points near end, got {count}")

        last_x = self.local_lane.waypoints[-1].pose.pose.position.x
        self.assertAlmostEqual(last_x, 10.0, delta=0.05)

    def test_4_timestamp_and_frame_id_check(self):
        """测试局部规划器输出的时间戳新鲜且坐标系匹配"""
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = 5.0
        odom.pose.pose.position.y = 0.0

        self.local_lane = None
        start = time.time()
        while time.time() - start < 2.0:
            self.odom_pub.publish(odom)
            rospy.sleep(0.1)
            if self.local_lane is not None:
                break

        self.assertIsNotNone(self.local_lane, "Local planner did not publish for timestamp check")

        diff = (rospy.Time.now() - self.local_lane.header.stamp).to_sec()
        self.assertLess(diff, 0.5, f"Planner is publishing stale data (age={diff:.3f}s)")
        self.assertEqual(
            self.local_lane.header.frame_id,
            odom.header.frame_id,
            "Frame ID mismatch between odometry and local path",
        )


if __name__ == "__main__":
    import rostest

    rostest.rosrun("my_robot_planning", "test_navigation_stack", TestNavigationStack)
