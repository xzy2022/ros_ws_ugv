#!/usr/bin/env python3
import unittest
import rospy
import time
import statistics
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TestGazeboVelocityProfile(unittest.TestCase):
    """
    Deep diagnosis test: Samples velocity over time to analyze 
    acceleration, steady-state speed, and braking performance.
    """

    def setUp(self) -> None:
        rospy.init_node("test_gazebo_velocity_profile", anonymous=True)
        self.odom_topic = "/smart/ground_truth/state"
        self.latest_odom = None
        
        # 订阅里程计更新
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # 等待第一帧数据确保连接
        try:
            rospy.wait_for_message(self.odom_topic, Odometry, timeout=5.0)
        except rospy.ROSException:
            self.fail("Failed to connect to ground truth topic!")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def get_current_speed(self) -> float:
        if self.latest_odom:
            # 直接读取仿真器计算的线速度 (Vx)
            return self.latest_odom.twist.twist.linear.x
        return 0.0

    def get_current_x(self) -> float:
        if self.latest_odom:
            return self.latest_odom.pose.pose.position.x
        return 0.0

    def test_velocity_profile(self) -> None:
        pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)
        rospy.sleep(1.0) # 等待发布者连接

        print("\n" + "="*30)
        print("[DEBUG] Starting Velocity Profile Test")
        print("="*30)

        start_x = self.get_current_x()
        print(f"[INIT] Start Position X: {start_x:.4f} m")

        # === 阶段 1: 运动 (0s -> 2s) ===
        cmd = Twist()
        cmd.linear.x = 2.0
        
        start_time = rospy.Time.now()
        run_duration = rospy.Duration(5.0)
        rate = rospy.Rate(10) # 10Hz采样率 (每0.1s一次)

        velocities_during_motion = []
        
        print(f"[PHASE 1] Commanding 5.0 m/s for {run_duration.to_sec()}s...")
        
        while (rospy.Time.now() - start_time) < run_duration and not rospy.is_shutdown():
            pub.publish(cmd)
            
            # 采样当前速度
            current_v = self.get_current_speed()
            velocities_during_motion.append(current_v)
            
            # 打印实时速度，方便你直接在控制台看
            print(f"  -> T+{(rospy.Time.now() - start_time).to_sec():.2f}s | Vel: {current_v:.4f} m/s")
            
            rate.sleep()

        # === 阶段 2: 制动 (5s -> 8s) ===
        print("[PHASE 2] Commanding STOP (0.0 m/s)...")
        stop_cmd = Twist()
        pub.publish(stop_cmd) # 发送第一次停止
        
        stop_start_time = rospy.Time.now()
        stop_duration = rospy.Duration(3.0)
        
        velocities_during_stop = []

        while (rospy.Time.now() - stop_start_time) < stop_duration and not rospy.is_shutdown():
            pub.publish(stop_cmd) # 持续发送停止指令防止超时保护
            
            current_v = self.get_current_speed()
            velocities_during_stop.append(current_v)
            
            print(f"  -> T+{(rospy.Time.now() - start_time).to_sec():.2f}s | Vel: {current_v:.4f} m/s")
            
            rate.sleep()

        # === 分析结果 ===
        end_x = self.get_current_x()
        total_displacement = end_x - start_x
        
        # 计算加速完成后的平均速度（去掉前0.5秒的加速期）
        steady_state_samples = velocities_during_motion[5:] 
        avg_speed = statistics.mean(steady_state_samples) if steady_state_samples else 0.0
        max_speed = max(velocities_during_motion)
        final_speed = velocities_during_stop[-1]

        print("="*30)
        print(f"[RESULT] Total Displacement: {total_displacement:.4f} m")
        print(f"[RESULT] Average Speed (Steady): {avg_speed:.4f} m/s")
        print(f"[RESULT] Max Speed Reached:     {max_speed:.4f} m/s")
        print(f"[RESULT] Final Speed (at 4s):   {final_speed:.4f} m/s")
        print("="*30)

        # === 断言诊断 ===
        
        # 1. 检查是否停下来了
        self.assertLess(abs(final_speed), 0.2, 
            f"Braking failed! Robot is still moving at {final_speed:.2f} m/s")

        # 2. 检查速度是否严重超标 (诊断 23m 问题)
        # 如果你设定2m/s，它跑到了10m/s，这里会直接报错并告诉你当前跑了多少
        if avg_speed > 3.0:
            self.fail(f"CRITICAL: Overspeed detected! Target: 2.0 m/s, Actual Avg: {avg_speed:.2f} m/s. Check Controller Config!")
        
        # 3. 正常的速度检查
        self.assertAlmostEqual(avg_speed, 2.0, delta=0.5, 
            msg=f"Speed incorrect. Target 2.0, Got {avg_speed:.2f}")

if __name__ == "__main__":
    import rostest
    rostest.rosrun("my_robot_control", "test_gazebo_velocity_profile", TestGazeboVelocityProfile)
