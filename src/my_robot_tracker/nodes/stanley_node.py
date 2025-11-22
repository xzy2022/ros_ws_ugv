#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # [新增] 引入里程计消息类型
from tf.transformations import euler_from_quaternion
from my_robot_msgs.msg import Lane

# 引入上一轮定义的辅助类 (如果放在单独文件请 import)
from my_robot_tracker.algorithms import Stanley, VehicleState, PathPoint

class StanleyNode:
    def __init__(self):
        rospy.init_node("stanley_controller_node")
        
        # --- 参数设置 ---
        self.robot_ns = self._detect_robot_ns()
        self.wheelbase = self._get_param_with_fallback("wheelbase", 1.868)
        self.k_gain = self._get_param_with_fallback("stanley_k", 0.1)
        self.k_soft = self._get_param_with_fallback("stanley_k_soft", 1.0)

        # --- 话题名称 ---
        self.path_topic = self._resolve_topic("final_waypoints", "final_waypoints")
        self.cmd_vel_topic = "/smart/cmd_vel"
        # [新增] 真值话题名称
        self.ground_truth_topic = "/smart/ground_truth/state"

        # --- 算法实例 ---
        self.controller = Stanley(
            k_gain=self.k_gain,
            k_soft=self.k_soft,
            base_to_front_dist=0.923 # 根据 URDF 设置 base_link 到前轮中心的距离
        )

        # --- 状态变量缓存 ---
        self.latest_path = []  
        self.current_state = None # [新增] 用于存储最新的 VehicleState

        # --- 通信接口 ---
        # 1. 路径订阅
        rospy.Subscriber(self.path_topic, Lane, self.path_callback, queue_size=1)
        
        # 2. [新增] 真值/里程计订阅
        # 根据你提供的 YAML，该话题包含 Pose 和 Twist，属于 Odometry 类型
        rospy.Subscriber(self.ground_truth_topic, Odometry, self.odom_callback, queue_size=1)
        
        # 3. 指令发布
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        # --- 定时器 ---
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def odom_callback(self, msg: Odometry):
        """
        [新增] 处理真值/里程计消息
        从 /smart/ground_truth/state 中提取位置、姿态和速度
        """
        # 1. 提取位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 2. 提取姿态 (四元数 -> 欧拉角)
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # 3. 提取速度
        # 注意：Ground Truth 通常包含 twist，这比假设路径速度要准确得多
        velocity = msg.twist.twist.linear.x
        
        # 4. 更新全局状态缓存
        # 这里的 x, y 通常是车辆参考点（可能是后轴也可能是重心，取决于仿真模型设定）
        # Stanley 算法内部会自动将其转换到前轴
        self.current_state = VehicleState(
            x=x,
            y=y,
            yaw=yaw,
            velocity=velocity
        )

    def path_callback(self, msg: Lane):
        # (保持原有逻辑不变)
        path_data = []
        for wp in msg.waypoints:
            x = wp.pose.pose.position.x
            y = wp.pose.pose.position.y
            v = wp.twist.twist.linear.x
            
            q = wp.pose.pose.orientation
            _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

            path_data.append(PathPoint(x=x, y=y, yaw=yaw, velocity=v))
        self.latest_path = path_data

    def control_loop(self, _event):
        """
        主循环：使用缓存的 Odom 状态和 Path 进行计算
        """
        # 安全检查：必须同时拥有路径和车辆状态才能计算
        if not self.latest_path or self.current_state is None:
            # 可选：如果太久没收到 odom，可以发布停车指令
            return

        # === 核心修改区域 Start ===
        
        # 之前是 lookup_transform，现在直接使用 self.current_state
        # 因为 self.current_state 在 odom_callback 中已经被高频更新了
        vehicle_state = self.current_state

        # === 核心修改区域 End ===

        # 1. 计算 Stanley 转向角 (radians)
        steering_angle = self.controller.compute(vehicle_state, self.latest_path)

        # 2. 构造控制指令
        cmd = Twist()
        
        # 线速度：依然使用路径点的期望速度 (前馈)
        # 或者，如果你想做闭环纵向控制，可以用 (target_v - vehicle_state.velocity) 做 PID
        cmd.linear.x = self.latest_path[0].velocity 
        
        # 限制转向角幅度
        max_steer = 0.785 # 45 deg
        steering_angle = max(min(steering_angle, max_steer), -max_steer)

        # 角速度转换: omega = v / L * tan(delta)
        # 注意：这里用的是“当前指令线速度”来计算所需的角速度
        # print(f"self.wheelbase: {self.wheelbase}")
        if abs(self.wheelbase) > 1e-3:
            cmd.angular.z = (cmd.linear.x / self.wheelbase) * math.tan(steering_angle)
        else:
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ... (辅助函数 _detect_robot_ns, _resolve_topic 等保持不变) ...
    def _detect_robot_ns(self) -> str:
        ns = rospy.get_namespace().strip("/")
        if ns: return ns
        return rospy.get_param("~robot_name", rospy.get_param("robot_name", ""))

    def _get_param_with_fallback(self, name: str, default):
        private_name = "~" + name
        if rospy.has_param(private_name): return rospy.get_param(private_name)
        candidate_keys = []
        if self.robot_ns: candidate_keys.append(f"/{self.robot_ns}/{name}")
        candidate_keys.append("/" + name if not name.startswith("/") else name)
        candidate_keys.append(name)
        for key in candidate_keys:
            if rospy.has_param(key): return rospy.get_param(key)
        return default

    def _resolve_topic(self, topic_key: str, default: str) -> str:
        topic_name = self._get_param_with_fallback(f"topics/{topic_key}", default)
        if isinstance(topic_name, str) and topic_name.startswith("/"):
            topic_name = topic_name.lstrip("/")
        return topic_name

def main():
    try:
        StanleyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()