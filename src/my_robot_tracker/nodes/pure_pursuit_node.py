#!/usr/bin/env python3
"""
ROS 节点封装：纯跟踪 (Pure Pursuit) 横向控制算法
注意：本节点不包含纵向 PID 控制，仅直接下发路径点的目标速度。
"""

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from my_robot_msgs.msg import Lane
from my_robot_tracker.algorithms import PurePursuit
# 显式导入 tf2_geometry_msgs 是必要的，
# 否则 tf_buffer.transform() 无法自动处理 PointStamped 类型的消息转换
import tf2_geometry_msgs  # noqa: F401

class PurePursuitNode:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node("pure_pursuit_node")

        # 2. 确定参数上下文 (主要用于多机器人场景下的命名空间处理)
        self.robot_ns = self._detect_robot_ns()

        # 3. 加载参数
        # 采用级联回退机制：私有参数(~) -> 全局/命名空间参数 -> 默认值
        
        # 车辆轴距 (Wheelbase)，默认为 0.335m
        self.wheelbase = self._get_param_with_fallback(
            "wheelbase",
            self._get_param_with_fallback("vehicle_physics/wheelbase", 0.335),
        )
        
        # 预瞄距离参数
        self.lookahead_dist = self._get_param_with_fallback("lookahead_distance", 1.0)
        self.min_lookahead = self._get_param_with_fallback("min_lookahead_distance", self.lookahead_dist)
        self.ld_gain = self._get_param_with_fallback("lookahead_gain", 0.0) # 速度增益系数
        
        # 坐标系定义
        self.map_frame = self._get_param_with_fallback("map_frame", "world")      # 全局地图坐标系
        self.base_frame = self._get_param_with_fallback("base_frame", "base_link") # 机器人基座坐标系

        # 4. 解析话题名称 (Topic Resolution)
        # 允许通过配置文件重映射话题名，强制使用相对名称以适应命名空间
        self.path_topic = self._resolve_topic("final_waypoints", "final_waypoints")
        self.cmd_vel_topic = "/smart/cmd_vel"

        # 5. 初始化控制器算法实例
        self.controller = PurePursuit(
            wheelbase=self.wheelbase, 
            ld_gain=self.ld_gain, 
            min_ld=self.min_lookahead
        )

        # 6. 初始化 TF (坐标变换) 监听器
        # Buffer 用于缓存坐标关系，Listener 用于从 ROS 系统中订阅 TF 消息
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 7. 配置 IO (订阅者和发布者)
        rospy.Subscriber(self.path_topic, Lane, self.path_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        # 8. 初始化控制循环定时器
        self.latest_path = []  # 存储格式：list of (x, y, v)
        # 以 20Hz (0.05s) 的频率运行 control_loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def path_callback(self, msg: Lane):
        """
        回调函数：接收局部路径或全局路径消息
        将 ROS 消息转换为简单的 Python 列表以便处理 (x, y, velocity)
        """
        path_data = []
        for wp in msg.waypoints:
            path_data.append(
                (
                    wp.pose.pose.position.x,
                    wp.pose.pose.position.y,
                    wp.twist.twist.linear.x,
                )
            )
        self.latest_path = path_data

    def control_loop(self, _event):
        """
        主控制循环：执行定位、搜索预瞄点、坐标转换和指令计算
        """
        # 如果还没有收到路径，直接跳过
        if not self.latest_path:
            return

        # === 第一步：获取机器人当前在地图中的位置 ===
        try:
            # 查询 Map -> Base_link 的变换
            # 这告诉我们机器人在地图坐标系下的位置 (curr_x, curr_y)
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0))
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # 如果 TF 树断裂或数据延迟，打印警告并停车
            rospy.logwarn_throttle(1.0, "TF Transform failure - waiting for buffers to fill")
            self.publish_stop()
            return

        # === 第二步：在全局路径上搜索预瞄点 ===
        # 利用算法类的静态方法，在路径列表中找到距离当前位置 >= lookahead 的点
        target = self.controller.find_lookahead_point(self.latest_path, curr_x, curr_y, self.lookahead_dist)
        
        if target is None:
            self.publish_stop()
            return

        target_x, target_y, target_v = target

        # === 第三步：将目标点转换回机器人坐标系 (Base Frame) ===
        # 这一步至关重要：
        # 纯跟踪算法需要在车辆坐标系下计算横向误差 (lateral error)。
        # 在车辆坐标系中，目标点的 Y 坐标 就是横向误差。
        
        # 1. 构造一个全局坐标系下的点对象
        global_point = PointStamped()
        global_point.header.frame_id = self.map_frame
        global_point.header.stamp = rospy.Time(0) # 使用最新可用的变换
        global_point.point.x = target_x
        global_point.point.y = target_y

        try:
            # 2. 执行坐标转换：Map Frame -> Base Frame
            local_point = self.tf_buffer.transform(global_point, self.base_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn_throttle(1.0, "TF Transform failure during point transform")
            self.publish_stop()
            return

        # 提取横向误差 (在 base_link 坐标系中，Y 轴通常指向左侧)
        lateral_error = local_point.point.y 

        # === 第四步：计算控制指令 ===
        # 调用算法核心 (模式2：参数透传模式)，直接计算所需的转向角和角速度
        output = self.controller.compute_command(
            v_target=target_v,
            lateral_error=lateral_error,
            lookahead_dist=self.lookahead_dist,
        )

        # === 第五步：发布速度指令 ===
        cmd = Twist()
        cmd.linear.x = target_v          # 直接透传目标速度 (无 PID)
        cmd.angular.z = output.angular_velocity # 设定计算出的角速度
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        """安全停车辅助函数"""
        self.cmd_pub.publish(Twist())

    def _detect_robot_ns(self) -> str:
        """
        自动检测机器人命名空间。
        尝试顺序：ROS 命名空间 -> 私有参数 'robot_name' -> 全局参数 'robot_name'
        """
        ns = rospy.get_namespace().strip("/")
        if ns:
            return ns
        return rospy.get_param("~robot_name", rospy.get_param("robot_name", ""))

    def _get_param_with_fallback(self, name: str, default):
        """
        具有优先级的参数加载器。
        优先级：
        1. 私有参数 (~param_name)
        2. 命名空间参数 (/robot_ns/param_name)
        3. 绝对/全局参数 (/param_name 或 param_name)
        4. 默认值 (default)
        """
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)

        candidate_keys = []
        if self.robot_ns:
            candidate_keys.append(f"/{self.robot_ns}/{name}")
        candidate_keys.append("/" + name if not name.startswith("/") else name)
        candidate_keys.append(name)

        for key in candidate_keys:
            if rospy.has_param(key):
                return rospy.get_param(key)
        return default

    def _resolve_topic(self, topic_key: str, default: str) -> str:
        """
        从参数服务器读取话题配置，并强制处理为相对路径。
        
        如果配置中写了 "/final_waypoints" (绝对路径)，会强制去掉 "/" 变为 "final_waypoints"，
        这样 ROS 才能正确地把它挂载到当前节点的命名空间下 (例如 /robot1/final_waypoints)。
        """
        topic_name = self._get_param_with_fallback(f"topics/{topic_key}", default)
        if isinstance(topic_name, str) and topic_name.startswith("/"):
            rospy.logwarn_once("Topic '%s' supplied as absolute name '%s'; stripping leading '/'.", topic_key, topic_name)
            topic_name = topic_name.lstrip("/")
        return topic_name


def main():
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()