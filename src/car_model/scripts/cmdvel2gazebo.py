#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

from ackermann_utils import AckermannKinematics

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)

        # 车辆几何和动力学参数（可通过启动文件中的私有参数覆盖）
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.3)
        self.L = rospy.get_param('~wheel_base', 1.868)  # 轴距（米）
        self.T_front = rospy.get_param('~front_tread', 1.284)
        self.T_rear = rospy.get_param('~rear_tread', 1.284)
        self.maxsteerInside = rospy.get_param('~max_steer_inside', 0.6)
        self.timeout = rospy.Duration.from_sec(rospy.get_param('~cmd_timeout', 0.2))
        self.loop_rate = rospy.get_param('~publish_rate', 10.0)
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/smart/cmd_vel')

        rospy.Subscriber(cmd_vel_topic, Twist, self.callback, queue_size=1)

        self.pub_steerL = rospy.Publisher(
            '/smart/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher(
            '/smart/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher(
            '/smart/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher(
            '/smart/rear_right_velocity_controller/command', Float64, queue_size=1)

        # 初始速度和轮胎角度为0
        self.rear_wheel_speed = 0
        self.ideal_steer = 0
        self.lastMsg = rospy.Time.now()

        # Ackermann 几何计算类
        self.ackermann = AckermannKinematics(self.L, self.T_front, self.T_rear, self.maxsteerInside)
        # 理想中间轮胎的最大转向角（基于内侧轮胎极限）
        self.maxsteer = self.ackermann.max_ideal_steer

        # 循环
        rate = rospy.Rate(self.loop_rate)  # 以配置的赫兹频率运行
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        

    def callback(self, data):
        # w = v / r
        if self.wheel_radius <= 0:
            rospy.logwarn_once("wheel_radius must be > 0; ignoring speed command")
            self.rear_wheel_speed = 0
        else:
            self.rear_wheel_speed = data.linear.x / self.wheel_radius
        # 限制理想转向角，使阿克曼转向达到最大值
        self.ideal_steer = self.ackermann.clamp_ideal_steer(data.angular.z)
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # 现在这些值已发布，我们
        # 重置速度，以便如果在下一个时间步收不到新的
        # 命令就会超时；注意
        # 轮胎角度不会改变
        # 注意：我们仅在配置的超时时间后才将轮速设为0
        if (rospy.Time.now() - self.lastMsg) > self.timeout:
            self.rear_wheel_speed = 0
            msgRear = Float64()
            msgRear.data = self.rear_wheel_speed
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)

            return

        # self.ideal_steer是阿克曼模型中虚拟前轮的弧度转向角增量
        rearL, rearR, steerL, steerR = self.ackermann.wheel_commands(self.ideal_steer, self.rear_wheel_speed)

        msgRearL = Float64()
        msgRearL.data = rearL
        msgRearR = Float64()
        msgRearR.data = rearR
        self.pub_rearL.publish(msgRearL)
        self.pub_rearR.publish(msgRearR)

        msgSteerL = Float64()
        msgSteerL.data = steerL
        msgSteerR = Float64()
        msgSteerR.data = steerR
        self.pub_steerL.publish(msgSteerL)
        self.pub_steerR.publish(msgSteerR)


if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass
