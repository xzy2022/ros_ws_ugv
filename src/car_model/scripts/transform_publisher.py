#!/usr/bin/env python3  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class transform_publisher():
	def __init__(self):
		rospy.init_node('transform_publisher')
		# Keep a single broadcaster to avoid re‑creating it on every callback
		self.br = tf.TransformBroadcaster()

		rospy.Subscriber('/smart/center_pose', PoseStamped, self.pose_cb, queue_size = 1)
		rospy.spin()

	def pose_cb(self, msg):

		# pose = msg.pose.position
		# orientation = msg.pose.orientation
		# br = tf.TransformBroadcaster()
		# br.sendTransform((pose.x, pose.y, pose.z),
		# 					(orientation.x, orientation.y, orientation.z, orientation.w),
		# 					rospy.Time.now(),
		# 					'base_link', 'world')
		
		# AI 给出的改进版本

		pose = msg.pose.position
		orientation = msg.pose.orientation

		# 如果 header 中没有时间戳，则使用当前时间，否则使用 header 中的时间戳
		stamp = msg.header.stamp if msg.header.stamp and not msg.header.stamp.is_zero() else rospy.Time.now()
		# 从 header 中获取父坐标系名称，去掉前导斜杠，如果没有则默认为 'world'
		parent_frame = msg.header.frame_id.lstrip('/') if msg.header.frame_id else 'world'
		self.br.sendTransform((pose.x, pose.y, pose.z),
						(orientation.x, orientation.y, orientation.z, orientation.w),
						stamp, 'base_link', parent_frame)



if __name__ == "__main__":
	try:
		transform_publisher()
	except:
		rospy.logwarn("cannot start transform publisher")