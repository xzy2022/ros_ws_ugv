#!/usr/bin/env python3

"""
Publish a constant cmd_vel to drive the vehicle in a circle, record poses,
and save trajectory + plot for radius verification.
"""

import os
import math
import time
from collections import deque

import rospy
from geometry_msgs.msg import Twist, PoseStamped
import tf.transformations as tft

class CircleCmdVelTest:
    def __init__(self):
        rospy.init_node("cmdvel_circle_test")

        self.linear_speed = rospy.get_param("~linear_speed", 2.0)
        self.radius = max(rospy.get_param("~radius", 5.0), 0.0)
        self.duration = rospy.get_param("~duration", 20.0)
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = rospy.get_param(
            "~output_dir", os.path.join(base_dir, "results")
        )
        os.makedirs(self.output_dir, exist_ok=True)

        self.angular_speed = 0.0 if self.radius == 0 else self.linear_speed / self.radius
        self.sign = 1.0 if self.angular_speed >= 0 else -1.0

        self.pose_buffer = deque()
        self.start_time = None
        self.start_pose = None

        self.pose_sub = rospy.Subscriber(
            "/smart/center_pose", PoseStamped, self.pose_cb, queue_size=10
        )
        self.cmd_pub = rospy.Publisher("/smart/cmd_vel", Twist, queue_size=1)

    def pose_cb(self, msg):
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            self.start_pose = msg
        t_sec = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0.0
        yaw = self._quat_to_yaw(msg.pose.orientation)
        self.pose_buffer.append((t_sec, msg.pose.position.x, msg.pose.position.y, yaw))

    def publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(20.0)
        end_time = time.time() + self.duration
        while not rospy.is_shutdown() and time.time() < end_time:
            self.publish_cmd(self.linear_speed, self.angular_speed)
            rate.sleep()

        # stop vehicle
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            rate.sleep()

        if len(self.pose_buffer) < 2:
            rospy.logwarn("Not enough pose samples collected to evaluate trajectory.")
            return

        txt_path = self._save_results()
        rospy.loginfo("Trajectory saved to %s", txt_path)

    def _quat_to_yaw(self, q):
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tft.euler_from_quaternion(quat)
        return yaw

    def _compute_expected_center(self):
        if not self.start_pose or self.radius <= 0:
            return None
        yaw0 = self._quat_to_yaw(self.start_pose.pose.orientation)
        x0 = self.start_pose.pose.position.x
        y0 = self.start_pose.pose.position.y
        dx = -math.sin(yaw0) * self.radius * self.sign
        dy = math.cos(yaw0) * self.radius * self.sign
        return x0 + dx, y0 + dy

    def _save_results(self):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        txt_path = os.path.join(self.output_dir, f"circle_{timestamp}.txt")

        with open(txt_path, "w") as f:
            f.write("# linear_speed: %.3f m/s\n" % self.linear_speed)
            f.write("# radius_cmd: %.3f m\n" % self.radius)
            f.write("# angular_speed_cmd: %.3f rad/s\n" % self.angular_speed)
            f.write("# t[s]\tx[m]\ty[m]\tyaw[rad]\n")
            for t_sec, x, y, yaw in self.pose_buffer:
                f.write(f"{t_sec:.3f}\t{x:.6f}\t{y:.6f}\t{yaw:.6f}\n")

        xs = [p[1] for p in self.pose_buffer]
        ys = [p[2] for p in self.pose_buffer]

        center = self._compute_expected_center()
        if center:
            cx, cy = center
            radii = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
            est_radius = sum(radii) / len(radii)
        else:
            est_radius = 0.0

        # Log an inline summary so users can compare quickly; plotting happens offline.
        rospy.loginfo("Cmd radius=%.2f m, estimated radius=%.2f m from %d samples",
                      self.radius, est_radius, len(xs))

        return txt_path


if __name__ == "__main__":
    tester = CircleCmdVelTest()
    tester.run()
