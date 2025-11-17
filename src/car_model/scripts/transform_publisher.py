#!/usr/bin/env python3  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class transform_publisher():
    def __init__(self):
        rospy.init_node('transform_publisher')

        self.br = tf.TransformBroadcaster()

        # 记录上一次发布 TF 的时间
        self.last_tf_stamp = rospy.Time(0)
        # 目标 TF 频率，比如 50 Hz
        self.max_tf_rate_hz = 50.0
        self.min_tf_dt = rospy.Duration(1.0 / self.max_tf_rate_hz)

        rospy.Subscriber('/smart/center_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.spin()

    def pose_cb(self, msg):
        pose = msg.pose.position
        orientation = msg.pose.orientation

        # 1. 取时间戳：优先用消息 header
        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()

        # 2. 限频 + 确保时间单调递增
        #   如果时间没有走过一个 min_tf_dt，就先不发新的 TF
        if (stamp - self.last_tf_stamp) < self.min_tf_dt:
            return

        self.last_tf_stamp = stamp

        # 3. 父坐标系，从 header.frame_id 里读（去掉前导 /），默认 world
        parent_frame = msg.header.frame_id.lstrip('/') if msg.header.frame_id else 'world'

        self.br.sendTransform(
            (pose.x, pose.y, pose.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            stamp,
            'base_link',
            parent_frame
        )

if __name__ == "__main__":
    try:
        transform_publisher()
    except:
        rospy.logwarn("cannot start transform publisher")
