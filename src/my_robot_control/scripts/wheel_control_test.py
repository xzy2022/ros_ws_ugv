#!/usr/bin/env python3
"""Simple wheel-level control test node.

Publishes directly to the wheel velocity and steering controllers based on
mode parameters. Currently supports only straight driving with 1 rad/s rear
wheel speed and zero front steering.
"""
import rospy
from std_msgs.msg import Float64


class WheelControlTest:
    def __init__(self):
        rospy.init_node("wheel_control_test")

        self.robot_name = rospy.get_param("~robot_name", "smart")
        self.mode = rospy.get_param("~mode", "straight")
        self.speed = rospy.get_param("~speed", 1.0)
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)

        ns = f"/{self.robot_name}"
        self.pub_rr = rospy.Publisher(
            f"{ns}/rear_right_velocity_controller/command", Float64, queue_size=10
        )
        self.pub_rl = rospy.Publisher(
            f"{ns}/rear_left_velocity_controller/command", Float64, queue_size=10
        )
        self.pub_fr = rospy.Publisher(
            f"{ns}/front_right_steering_position_controller/command",
            Float64,
            queue_size=10,
        )
        self.pub_fl = rospy.Publisher(
            f"{ns}/front_left_steering_position_controller/command",
            Float64,
            queue_size=10,
        )

    def _publish_straight(self):
        cmd_speed = Float64(self.speed)
        zero = Float64(0.0)
        self.pub_rr.publish(cmd_speed)
        self.pub_rl.publish(cmd_speed)
        self.pub_fr.publish(zero)
        self.pub_fl.publish(zero)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo(
            "wheel_control_test running with mode=%s speed=%.2f rad/s",
            self.mode,
            self.speed,
        )
        while not rospy.is_shutdown():
            if self.mode == "straight":
                self._publish_straight()
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "Unknown mode '%s'. Holding zero commands.",
                    self.mode,
                )
                self.pub_rr.publish(Float64(0.0))
                self.pub_rl.publish(Float64(0.0))
                self.pub_fr.publish(Float64(0.0))
                self.pub_fl.publish(Float64(0.0))
            rate.sleep()


if __name__ == "__main__":
    WheelControlTest().run()
