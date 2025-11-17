#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Clock

def clock_cb(msg):
    sim_time_from_clock = msg.clock.to_sec()
    sim_time_from_now = rospy.Time.now().to_sec()

    rospy.loginfo("clock: %.9f, rospy.Time.now(): %.9f",
                  sim_time_from_clock, sim_time_from_now)

if __name__ == '__main__':
    rospy.init_node('sim_time_debug')

    # 确认已经设置了 use_sim_time
    # use_sim_time = rospy.get_param('/use_sim_time', False)
    # rospy.loginfo("/use_sim_time = %s", use_sim_time)

    rospy.Subscriber('/clock', Clock, clock_cb)

    rospy.spin()
