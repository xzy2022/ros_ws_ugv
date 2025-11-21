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

        # Parameter context
        self.robot_ns = self._detect_robot_ns()

        # Parameters: private -> global (robot_ns) -> default
        self.wheelbase = self._get_param_with_fallback(
            "wheelbase",
            self._get_param_with_fallback("vehicle_physics/wheelbase", 0.335),
        )
        self.lookahead_dist = self._get_param_with_fallback("lookahead_distance", 1.0)
        self.min_lookahead = self._get_param_with_fallback("min_lookahead_distance", self.lookahead_dist)
        self.ld_gain = self._get_param_with_fallback("lookahead_gain", 0.0)
        self.map_frame = self._get_param_with_fallback("map_frame", "world")
        self.base_frame = self._get_param_with_fallback("base_frame", "base_link")

        # Topic resolution via central dictionary
        self.path_topic = self._resolve_topic("final_waypoints", "final_waypoints")
        self.cmd_vel_topic = self._resolve_topic("cmd_vel", "cmd_vel")

        # Controller
        self.controller = PurePursuit(wheelbase=self.wheelbase, ld_gain=self.ld_gain, min_ld=self.min_lookahead)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # IO
        rospy.Subscriber(self.path_topic, Lane, self.path_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

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
        rospy.logdebug("Publishing cmd to %s: linear_x=%.2f angular_z=%.2f", self.cmd_vel_topic, cmd.linear.x, cmd.angular.z)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _detect_robot_ns(self) -> str:
        """Pick robot namespace from node ns or common params."""
        ns = rospy.get_namespace().strip("/")
        if ns:
            return ns
        return rospy.get_param("~robot_name", rospy.get_param("robot_name", ""))

    def _get_param_with_fallback(self, name: str, default):
        """Resolve parameter with priority: private -> robot namespace -> global -> default."""
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
        """Read topic name from topics dictionary and enforce relative naming."""
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
