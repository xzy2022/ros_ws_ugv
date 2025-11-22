#!/usr/bin/env python3
import os
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from my_robot_msgs.msg import Lane, Waypoint
from my_robot_planning.navigation import GlobalPathManager

# /base_path (类型: nav_msgs/Path)
# 只包含一系列的位姿 (Pose)，即位置 $(x, y, z)$ 和姿态 (四元数 orientation)。
# 特点：它通常用于在 RViz 中画出一条绿色的线，让人直观地看到路径长什么样。它不包含速度信息。


# /base_waypoints (类型: my_robot_msgs/Lane)
# 主要用途：控制与规划。
# 特点：它不仅包含了位置和姿态（Pose），还包含了每一个点的目标速度 (Twist)。这是给自动驾驶控制器（如 Pure Pursuit 或 MPC）使用的核心数据。

class GlobalPathLoaderNode:
    def __init__(self):
        rospy.init_node("global_path_loader")

        self.csv_path = rospy.get_param("~csv_path", "")
        self.velocity_kmph = rospy.get_param("~velocity_kmph", 15.0)
        self.max_decel = rospy.get_param("~max_decel", 1.0)

        if not self.csv_path:
            rospy.logfatal("csv_path parameter is required")
            raise rospy.ROSInitException("Missing csv_path parameter")
        if not os.path.isabs(self.csv_path) and "$(find" in self.csv_path:
            pass
        elif not os.path.exists(self.csv_path):
            rospy.logfatal("CSV path does not exist: %s", self.csv_path)
            raise rospy.ROSInitException("CSV path missing")

        self.manager = GlobalPathManager(max_decel=self.max_decel)

        self.lane_pub = rospy.Publisher("/base_waypoints", Lane, queue_size=1, latch=True)
        self.viz_pub = rospy.Publisher("/base_path", Path, queue_size=1, latch=True)

        self.load_and_publish()
        rospy.spin()

    def load_and_publish(self):
        target_vel_mps = self.velocity_kmph / 3.6
        try:
            points = self.manager.load_from_csv(self.csv_path, target_vel_mps)
        except Exception as exc:
            rospy.logerr("Failed to load path: %s", exc)
            return

        lane_msg = Lane()
        lane_msg.header.frame_id = "world"
        lane_msg.header.stamp = rospy.Time.now()

        path_msg = Path()
        path_msg.header = lane_msg.header

        for p in points:
            wp = Waypoint()
            wp.pose.header = lane_msg.header
            wp.pose.pose.position.x = p.x
            wp.pose.pose.position.y = p.y
            q = tft.quaternion_from_euler(0, 0, p.yaw)
            wp.pose.pose.orientation = Quaternion(*q)
            wp.twist.header = lane_msg.header
            wp.twist.twist.linear.x = p.velocity
            wp.forward = True
            lane_msg.waypoints.append(wp)

            ps = PoseStamped()
            ps.header = lane_msg.header
            ps.pose = wp.pose.pose
            path_msg.poses.append(ps)

        rospy.loginfo("Loaded and published %d waypoints", len(lane_msg.waypoints))
        self.lane_pub.publish(lane_msg)
        self.viz_pub.publish(path_msg)


def main():
    try:
        GlobalPathLoaderNode()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
