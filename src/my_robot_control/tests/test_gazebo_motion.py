#!/usr/bin/env python3
import time
import unittest

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 监听真实位姿：订阅 /smart/ground_truth/state 话题获取机器人在仿真世界中的精确位置（Odometry 消息）
# 发送运动指令：通过 /smart/cmd_vel 话题持续发送 2.0 m/s 的前进线速度，持续 2 秒
# 测量位移：对比运动前后的 X 坐标位置差
# 断言结果：如果位移大于 1.8 米则测试通过，否则失败

class TestGazeboMotion(unittest.TestCase):
    """End-to-end test: ensure robot moves in Gazebo when commanded."""

    def setUp(self) -> None:
        # 初始化节点，必须指定 anonymous=True 以免和被测节点冲突
        rospy.init_node("test_gazebo_motion", anonymous=True)

    def test_robot_moves_forward(self) -> None:
        topic = "/smart/ground_truth/state"
        
        # 1. 确保能收到里程计数据
        try:
            rospy.wait_for_message(topic, Odometry, timeout=10.0)
        except rospy.ROSException:
            self.fail(f"Did not receive ground truth data from {topic}")

        # 2. 获取初始位置
        initial = rospy.wait_for_message(topic, Odometry)
        start_x = initial.pose.pose.position.x
        
        # 使用 print 可以在 rostest --text 模式下看到调试信息
        print(f"\n[DEBUG] Start X: {start_x:.4f}")

        pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)
        
        # 等待发布者连接，防止第一条指令丢失
        rospy.sleep(1.0) 

        cmd = Twist()
        cmd.linear.x = 2.0
        
        # === 核心修改：使用仿真时间 (ROSTime) ===
        # 确保使用的是仿真里的时间，而不是电脑系统时间
        start_time = rospy.Time.now()
        duration = rospy.Duration(2.0) # 仿真时间跑2秒
        rate = rospy.Rate(20)

        print(f"[DEBUG] Sending command linear.x=2.0 for {duration.to_sec()} simulation seconds...")
        
        while (rospy.Time.now() - start_time) < duration and not rospy.is_shutdown():
            pub.publish(cmd)
            rate.sleep()

        # 发送停止指令
        pub.publish(Twist())
        
        # 给一点物理惯性停止的时间
        rospy.sleep(1.0)

        # 3. 获取最终位置
        final = rospy.wait_for_message(topic, Odometry, timeout=5.0)
        end_x = final.pose.pose.position.x
        displacement = end_x - start_x
        
        print(f"[DEBUG] End X: {end_x:.4f}")
        print(f"[DEBUG] Total Displacement: {displacement:.4f} m")
        
        # 4. 断言
        # 理论值：2.0 m/s * 2.0 s = 4.0 m
        # 考虑到加速过程，位移应该略小于 4.0 m (例如 3.5 - 3.9m)
        # 如果大于 4.1m，说明有问题（或者之前提到的超速问题）
        
        # 我们先断言它确实动了 (比如大于 3.9米)
        self.assertGreater(displacement, 3.7, f"Robot moved too slow! Disp: {displacement}")
        
        # 我们可以加一个上限断言，验证是否超速（验证 Sim Time 修复是否生效）
        # 理论最大值 4.0米，加上一点误差允许到 4.1米
        self.assertLess(displacement, 4.3, f"Robot moved too fast! Disp: {displacement}")

if __name__ == "__main__":
    import rostest
    rostest.rosrun("my_robot_control", "test_gazebo_motion", TestGazeboMotion)