#!/usr/bin/env python3
"""Record ground-truth odometry into a plain text log."""

import os
from pathlib import Path
from datetime import datetime
from typing import Optional

import rospy
import rospkg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class GroundTruthLogger:
    def __init__(self):
        rospy.init_node("ground_truth_logger")

        self.robot_ns = self._detect_robot_ns()
        self.use_robot_ns = rospy.get_param("~use_robot_namespace", True)

        self.output_path = self._prepare_output_path()

        self.odom_topic = self._resolve_topic("ground_truth", "ground_truth/state")
        self.file_handle = open(self.output_path, "a", buffering=1)
        
        # [修改 1] 更新表头，增加 linear_v (线速度) 和 angular_v (角速度)
        if self.file_handle.tell() == 0:
            self.file_handle.write("# stamp x y z yaw linear_v angular_v\n")

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        rospy.loginfo("Ground-truth logger subscribing to '%s', writing to '%s'", self.odom_topic, self.output_path)

        rospy.on_shutdown(self._on_shutdown)

    def odom_callback(self, msg: Odometry):
        # 1. 提取位姿
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

        # [修改 2] 提取速度信息
        # linear.x 代表车辆前进的线速度 (m/s)
        # angular.z 代表车辆转弯的角速度 (rad/s)
        linear_v = msg.twist.twist.linear.x
        angular_v = msg.twist.twist.angular.z

        # [修改 3] 将速度数据写入日志行
        line = f"{stamp.to_sec():.6f} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} {yaw:.6f} {linear_v:.6f} {angular_v:.6f}\n"
        self.file_handle.write(line)

    def _on_shutdown(self):
        try:
            if self.file_handle:
                self.file_handle.flush()
                self.file_handle.close()
        except Exception:
            pass

    def _detect_robot_ns(self) -> str:
        """Derive robot namespace from current node namespace or parameters."""
        ns = rospy.get_namespace().strip("/")
        if ns:
            return ns
        return rospy.get_param("~robot_name", rospy.get_param("robot_name", ""))

    def _get_param_with_fallback(self, name: str, default):
        """Resolve parameter with priority: private -> /<robot_ns>/name -> /name -> name -> default."""
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
        """Read topic name from topics dictionary and prefix robot namespace when configured."""
        topic_name = self._get_param_with_fallback(f"topics/{topic_key}", default)
        if isinstance(topic_name, str) and topic_name.startswith("/"):
            rospy.logwarn_once("Topic '%s' supplied as absolute name '%s'; stripping leading '/'.", topic_key, topic_name)
            topic_name = topic_name.lstrip("/")

        if self.robot_ns and self.use_robot_ns and not topic_name.startswith(f"{self.robot_ns}/"):
            topic_name = f"{self.robot_ns}/{topic_name}"
        return topic_name

    def _prepare_output_path(self) -> str:
        """Determine unique log file path per run."""
        # If user provides an explicit file, honor it. Otherwise, create timestamped file under output_dir.
        explicit_file = rospy.get_param("~output_file", None)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        if explicit_file:
            path_obj = Path(explicit_file).expanduser()
        else:
            default_dir = rospy.get_param("~output_dir", rospkg.get_ros_home())
            filename = f"ground_truth_pose_{timestamp}.txt"
            path_obj = Path(default_dir).expanduser() / filename

        path_obj.parent.mkdir(parents=True, exist_ok=True)
        return str(path_obj)


def main():
    GroundTruthLogger()
    rospy.spin()


if __name__ == "__main__":
    main()