#!/usr/bin/env python3
"""ROS node wrapper for Pure Pursuit lateral tracking (no PID)."""

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from my_robot_msgs.msg import Lane
from my_robot_tracker.algorithms import PurePursuit
import tf2_geometry_msgs  # noqa: F401 (needed for transform)


class PurePursuitNode:
    def __init__(self):
        rospy.init_node("pure_pursuit_node")

        # Parameters
        self.wheelbase = rospy.get_param("~wheelbase", 0.335)
        self.lookahead_dist = rospy.get_param("~lookahead_distance", 1.0)
        self.map_frame = rospy.get_param("~map_frame", "world")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        # Controller
        self.controller = PurePursuit(wheelbase=self.wheelbase, ld_gain=0.0, min_ld=self.lookahead_dist)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # IO
        rospy.Subscriber("/final_waypoints", Lane, self.path_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.latest_path = []  # list of tuples (x, y, v)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20 Hz

    def path_callback(self, msg: Lane):
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
        if not self.latest_path:
            return

        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0))
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn_throttle(1.0, "TF Transform failure - waiting for buffers to fill")
            self.publish_stop()
            return

        target = self.controller.find_lookahead_point(self.latest_path, curr_x, curr_y, self.lookahead_dist)
        if target is None:
            self.publish_stop()
            return

        target_x, target_y, target_v = target

        # Transform target point into base frame
        global_point = PointStamped()
        global_point.header.frame_id = self.map_frame
        global_point.header.stamp = rospy.Time(0)
        global_point.point.x = target_x
        global_point.point.y = target_y

        try:
            local_point = self.tf_buffer.transform(global_point, self.base_frame)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn_throttle(1.0, "TF Transform failure during point transform")
            self.publish_stop()
            return

        lateral_error = local_point.point.y  # base_link y-axis is left
        output = self.controller.compute_command(
            v_target=target_v,
            lateral_error=lateral_error,
            lookahead_dist=self.lookahead_dist,
        )

        cmd = Twist()
        cmd.linear.x = target_v
        cmd.angular.z = output.angular_velocity
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main():
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
