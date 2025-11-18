#!/usr/bin/env python3

import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from styx_msgs.msg import Lane


class StanleyController:
    def __init__(self):
        rospy.init_node("stanley_controller", log_level=rospy.DEBUG)

        self.k_gain = rospy.get_param("~k_gain", 0.8)
        self.softening_gain = rospy.get_param("~softening_gain", 0.1)
        self.stop_distance = rospy.get_param("~stop_distance", 0.5)
        self.wheel_base = rospy.get_param("~wheel_base", 1.868)

        rospy.Subscriber("/smart/rear_pose", PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber("/smart/velocity", TwistStamped, self.vel_cb, queue_size=1)
        rospy.Subscriber("/final_waypoints", Lane, self.lane_cb, queue_size=1)

        self.twist_pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)

        self.current_pose = None
        self.current_velocity = None
        self.current_waypoints = None

        self._loop()

    def _loop(self):
        rate = rospy.Rate(20)
        rospy.logwarn("stanley controller starts")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_velocity and self.current_waypoints:
                cmd = self.calculate_twist_command()
                self.twist_pub.publish(cmd)
            rate.sleep()

    def pose_cb(self, data: PoseStamped):
        self.current_pose = data

    def vel_cb(self, data: TwistStamped):
        self.current_velocity = data

    def lane_cb(self, data: Lane):
        self.current_waypoints = data

    def calculate_twist_command(self) -> Twist:
        waypoints = self.current_waypoints.waypoints
        if len(waypoints) < 2:
            return Twist()

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        quat = (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)

        nearest_idx, _ = self._nearest_waypoint_index(current_x, current_y, waypoints)
        goal_distance = self._distance_to_last(current_x, current_y, waypoints)

        if goal_distance < self.stop_distance:
            rospy.logdebug("Stanley: stop because distance to goal is below threshold")
            return Twist()

        path_yaw = self._path_heading(nearest_idx, waypoints)
        heading_error = self._normalize_angle(path_yaw - yaw)

        cross_track_error = self._cross_track_error(
            current_x, current_y, waypoints[nearest_idx].pose.pose.position, path_yaw
        )

        current_speed = self.current_velocity.twist.linear.x
        safe_speed = max(abs(current_speed), 0.1)
        steering_correction = math.atan2(self.k_gain * cross_track_error, safe_speed + self.softening_gain)
        steering_angle = heading_error + steering_correction

        target_speed = waypoints[min(nearest_idx + 1, len(waypoints) - 1)].twist.twist.linear.x

        twist_cmd = Twist()
        twist_cmd.linear.x = target_speed
        twist_cmd.angular.z = steering_angle
        return twist_cmd

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _nearest_waypoint_index(self, x: float, y: float, waypoints):
        closest_idx = 0
        closest_dist = float("inf")
        for i, wp in enumerate(waypoints):
            wx = wp.pose.pose.position.x
            wy = wp.pose.pose.position.y
            dist = math.hypot(wx - x, wy - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        return closest_idx, closest_dist

    def _path_heading(self, idx: int, waypoints) -> float:
        if idx >= len(waypoints) - 1:
            ref_prev = idx - 1 if idx > 0 else idx
            ref_next = idx
        else:
            ref_prev = idx
            ref_next = idx + 1

        prev_wp = waypoints[ref_prev].pose.pose.position
        next_wp = waypoints[ref_next].pose.pose.position
        dx = next_wp.x - prev_wp.x
        dy = next_wp.y - prev_wp.y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    @staticmethod
    def _cross_track_error(x: float, y: float, waypoint_pos, path_yaw: float) -> float:
        dx = x - waypoint_pos.x
        dy = y - waypoint_pos.y
        return -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

    @staticmethod
    def _distance_to_last(x: float, y: float, waypoints) -> float:
        goal = waypoints[-1].pose.pose.position
        return math.hypot(goal.x - x, goal.y - y)


if __name__ == "__main__":
    try:
        StanleyController()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start stanley controller node.")
