#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# 【关键】这里直接引用你写好的库，就像引用 numpy 一样自然
# 确保你的 setup.py 配置正确
from my_robot_control.ackermann import AckermannKinematics, SteeringModel

class AckermannDriverNode:
    def __init__(self):
        rospy.init_node('ackermann_driver', anonymous=False)

        # 1. 从参数服务器加载车辆物理参数 (在 global_params.yaml 中定义)
        # 这样你修改车的大小不用改代码
        wheelbase = rospy.get_param("~wheelbase", 0.3)    # L
        track_width = rospy.get_param("~track_width", 0.2) # W
        wheel_radius = rospy.get_param("~wheel_radius", 0.05)
        
        # 2. 获取控制器的话题名称前缀
        # 假设你在 yaml 里定义了控制器的名字，比如 rear_left_velocity_controller
        controller_ns = rospy.get_param("~controller_namespace", "") # e.g., "smart"
        
        # 3. 初始化你的纯逻辑算法库
        self.kinematics = AckermannKinematics(
            wheelbase=wheelbase,
            track_width=track_width,
            front_wheel_radius=wheel_radius,
            rear_wheel_radius=wheel_radius, # 假设前后轮半径相同
            model_type=SteeringModel.ACKERMANN
        )

        # 4. 设置发布者 (连接到 Gazebo 的 ros_control 插件)
        # 注意：ros_control 默认的 topic 格式是: <controller_name>/command
        # 这里我们需要构建 4 个发布者
        self.pub_lf_steering = rospy.Publisher(
            'front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rf_steering = rospy.Publisher(
            'front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_lr_vel = rospy.Publisher(
            'rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_rr_vel = rospy.Publisher(
            'rear_right_velocity_controller/command', Float64, queue_size=1)

        # 5. 订阅 cmd_vel
        cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.sub_cmd = rospy.Subscriber(cmd_topic, Twist, self.cmd_callback, queue_size=1)

        # [可选] 安全看门狗：如果一段时间没收到 cmd_vel，就停车
        self.last_cmd_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.1), self.watchdog_callback)
        
        rospy.loginfo(f"Ackermann Driver Started. L={wheelbase}, W={track_width}")

    def cmd_callback(self, msg: Twist):
        """收到速度指令，调用算法库进行解算"""
        self.last_cmd_time = rospy.Time.now()
        
        v = msg.linear.x
        w = msg.angular.z

        # --- 核心逻辑调用 ---
        # 这里体现了 Logic 与 Communication 的完美分离
        output = self.kinematics.inverse_kinematics(v, w)
        # ------------------

        # 发布给 Gazebo
        self.pub_lf_steering.publish(output.left_front_steering)
        self.pub_rf_steering.publish(output.right_front_steering)
        self.pub_lr_vel.publish(output.left_rear_velocity)
        self.pub_rr_vel.publish(output.right_rear_velocity)

    def watchdog_callback(self, event):
        """如果在 0.5秒内没有收到新指令，强制停车（安全机制）"""
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 0.5:
            self.pub_lf_steering.publish(0.0)
            self.pub_rf_steering.publish(0.0)
            self.pub_lr_vel.publish(0.0)
            self.pub_rr_vel.publish(0.0)

if __name__ == '__main__':
    try:
        node = AckermannDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass