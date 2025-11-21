#!/usr/bin/env python3
"""Broadcast TF from ground truth odometry."""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class GroundTruthTFBroadcaster:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/smart/ground_truth/state")
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)


    def odom_cb(self, msg: Odometry):
        # --- 添加这行调试信息 ---
        rospy.loginfo_throttle(1.0, f"Received Odom: {msg.header.seq}, sending TF...") 

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp or rospy.Time.now()
        tf_msg.header.frame_id = msg.header.frame_id or self.parent_frame
        tf_msg.child_frame_id = msg.child_frame_id or self.child_frame

        tf_msg.transform.translation = msg.pose.pose.position
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(tf_msg)


def main():
    rospy.init_node("ground_truth_tf_broadcaster", anonymous=False)
    GroundTruthTFBroadcaster()
    rospy.spin()


if __name__ == "__main__":
    main()
