#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Global Path Logger (全局路径记录器)
功能：订阅全局路径话题 (Lane 消息)，将其保存为 TXT 文件。
特性：因为全局路径通常是静态的，该节点采用“写一次”策略 (Write-Once)，
      即记录成功后自动停止监听，避免重复写入。
"""

import os
from pathlib import Path
from datetime import datetime
from typing import Optional

import rospy
import rospkg
from tf.transformations import euler_from_quaternion

# 引入自定义消息类型
from my_robot_msgs.msg import Lane

class GlobalPathLogger:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("global_path_logger")

        # 1. 获取命名空间设置
        # robot_ns: 自动检测到的机器人命名空间 (例如 "smart")
        # use_robot_ns: 参数决定是否在话题前自动添加命名空间前缀
        self.robot_ns = self._detect_robot_ns()
        self.use_robot_ns = rospy.get_param("~use_robot_namespace", True)
        
        # 2. 初始化状态标志
        # self._has_logged: 用于确保路径只被记录一次，避免重复写入文件
        self._has_logged = False

        # 3. 准备输出文件路径
        # 如果未指定 output_file，则根据 output_dir 和当前时间戳生成文件名
        self.output_path = self._prepare_output_path()

        # 4. 解析订阅话题
        # 默认订阅 "/base_waypoints"，会根据 use_robot_namespace 决定是否变成 "/smart/base_waypoints"
        self.path_topic = self._resolve_topic("base_waypoints", "/base_waypoints")
        
        # 5. 创建订阅者
        # Lane 消息通常包含大量点，且通常由发布者开启 latch (锁存)，因此 queue_size=1 足够
        self.sub = rospy.Subscriber(self.path_topic, Lane, self.path_callback, queue_size=1)
        
        rospy.loginfo(f"全局路径记录器已就绪。正在监听话题: '{self.path_topic}'")
        rospy.loginfo(f"等待写入文件: '{self.output_path}'")

    def path_callback(self, msg: Lane):
        """
        回调函数：处理接收到的 Lane 消息。
        逻辑：收到消息 -> 解析数据 -> 写入 TXT -> 取消订阅。
        """
        # 如果已经记录过，直接忽略后续消息
        if self._has_logged:
            return

        # 检查消息是否为空
        if not msg.waypoints:
            rospy.logwarn("收到空的路径消息 (Waypoints 列表为空)，跳过记录。")
            return

        rospy.loginfo(f"收到全局路径，包含 {len(msg.waypoints)} 个路径点。正在写入文件...")

        try:
            # 确保目标文件夹存在，如果不存在则创建
            Path(self.output_path).parent.mkdir(parents=True, exist_ok=True)

            # 使用 "w" 模式打开文件 (覆盖写入/新建)
            with open(self.output_path, "w") as f:
                # 写入表头注释
                # id: 序号
                # x, y, z: 位置坐标
                # yaw: 航向角 (弧度)
                # target_linear_v: 目标线速度 (m/s)
                f.write("# id x y z yaw target_linear_v\n")

                for i, wp in enumerate(msg.waypoints):
                    # --- 1. 提取位置信息 ---
                    pos = wp.pose.pose.position
                    
                    # --- 2. 提取姿态并转换为欧拉角 (Yaw) ---
                    ori = wp.pose.pose.orientation
                    
                    # 防御性编程：防止四元数全0 (非法的四元数) 导致转换函数报错
                    if ori.w == 0 and ori.x == 0 and ori.y == 0 and ori.z == 0:
                        yaw = 0.0
                    else:
                        # euler_from_quaternion 返回 (roll, pitch, yaw)
                        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]

                    # --- 3. 提取目标速度 ---
                    # 假设速度信息存储在 twist.linear.x 中
                    target_v = wp.twist.twist.linear.x

                    # --- 4. 格式化并写入 ---
                    # 保留6位小数
                    line = f"{i} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} {yaw:.6f} {target_v:.6f}\n"
                    f.write(line)
            
            rospy.loginfo(f"成功！全局路径已保存至: {self.output_path}")
            
            # 标记为已完成
            self._has_logged = True
            
            # 取消订阅，释放资源，不再处理后续消息
            self.sub.unregister()
            rospy.loginfo("任务完成，已停止订阅话题。")

        except Exception as e:
            rospy.logerr(f"写入文件失败: {e}")

    def _detect_robot_ns(self) -> str:
        """
        探测机器人的命名空间。
        优先级：ROS 命名空间 > 'robot_name' 参数
        """
        ns = rospy.get_namespace().strip("/")
        if ns:
            return ns
        # 如果节点本身没有命名空间，尝试读取 robot_name 参数
        return rospy.get_param("~robot_name", rospy.get_param("robot_name", ""))

    def _get_param_with_fallback(self, name: str, default):
        """
        按优先级查找参数。
        优先级：私有参数 (~name) -> 带命名空间的参数 (/ns/name) -> 全局参数 (/name) -> 默认值
        """
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)

        candidate_keys = []
        if self.robot_ns:
            candidate_keys.append(f"/{self.robot_ns}/{name}")
        
        # 处理传入的 name 可能自带 '/' 的情况
        clean_name = name if not name.startswith("/") else name
        candidate_keys.append("/" + clean_name) # 尝试绝对路径
        candidate_keys.append(name) # 尝试相对路径

        for key in candidate_keys:
            if rospy.has_param(key):
                return rospy.get_param(key)
        return default

    def _resolve_topic(self, topic_key: str, default: str) -> str:
        """
        解析最终订阅的话题名称。
        逻辑：
        1. 从参数服务器查找话题名称配置。
        2. 如果配置了 use_robot_namespace=True，则在话题前加上机器人命名空间。
        """
        # 获取话题的基础名称 (例如 "base_waypoints")
        topic_name = self._get_param_with_fallback(f"topics/{topic_key}", default)
        
        # 如果是绝对路径 (以 / 开头)，先去掉 / 以便后续处理
        if isinstance(topic_name, str) and topic_name.startswith("/"):
            topic_name = topic_name.lstrip("/")

        # 如果启用了命名空间，且话题名尚未包含该命名空间，则拼接
        if self.robot_ns and self.use_robot_ns and not topic_name.startswith(f"{self.robot_ns}/"):
            topic_name = f"{self.robot_ns}/{topic_name}"
        
        # 确保最终话题名以 / 开头 (ROS 规范建议绝对路径)
        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name
            
        return topic_name

    def _prepare_output_path(self) -> str:
        """
        生成日志文件的完整路径。
        逻辑：优先使用 output_file 参数，否则使用 output_dir + 时间戳自动生成。
        """
        explicit_file = rospy.get_param("~output_file", None)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        
        if explicit_file:
            path_obj = Path(explicit_file).expanduser()
        else:
            default_dir = rospy.get_param("~output_dir", rospkg.get_ros_home())
            # 文件名格式: global_path_年月日-时分秒.txt
            filename = f"global_path_{timestamp}.txt"
            path_obj = Path(default_dir).expanduser() / filename

        return str(path_obj)


def main():
    try:
        GlobalPathLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()