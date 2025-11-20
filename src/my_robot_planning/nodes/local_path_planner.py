#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from my_robot_msgs.msg import Lane, Waypoint
from my_robot_planning.navigation import LocalPathPlanner, PathPoint


class LocalPlannerNode:
    def __init__(self):
        rospy.init_node("local_path_planner")

        self.lookahead_wps = rospy.get_param("~lookahead_wps", 20)
        self.frequency = rospy.get_param("~frequency", 20.0)

        self.planner_logic = None
        self.global_waypoints = []
        self.current_pose = None

        rospy.Subscriber("/smart/ground_truth/state", Odometry, self.pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self.waypoints_cb)

        self.local_lane_pub = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        self.local_viz_pub = rospy.Publisher("/final_path", Path, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.control_loop)

    def waypoints_cb(self, msg: Lane):
        if self.planner_logic is not None:
            return
        self.global_waypoints = msg.waypoints
        points = [
            PathPoint(
                x=wp.pose.pose.position.x,
                y=wp.pose.pose.position.y,
                yaw=0.0,
                velocity=wp.twist.twist.linear.x,
            )
            for wp in msg.waypoints
        ]
        self.planner_logic = LocalPathPlanner(points)
        rospy.loginfo("Global path received: %d waypoints", len(points))

    def pose_cb(self, msg):
        if isinstance(msg, Odometry):
            self.current_pose = msg.pose.pose
        elif isinstance(msg, PoseStamped):
            self.current_pose = msg.pose

    def control_loop(self, event):
        if self.current_pose is None or self.planner_logic is None:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        closest_idx = self.planner_logic.get_closest_index(x, y)
        local_points = self.planner_logic.extract_local_path(closest_idx, self.lookahead_wps)

        lane = Lane()
        lane.header.frame_id = "world"
        lane.header.stamp = rospy.Time.now()

        path_msg = Path()
        path_msg.header = lane.header

        for i, p in enumerate(local_points):
            global_idx = closest_idx + i
            if global_idx < len(self.global_waypoints):
                wp = self.global_waypoints[global_idx]
            else:
                wp = Waypoint()
                wp.pose.pose.position.x = p.x
                wp.pose.pose.position.y = p.y
                wp.twist.twist.linear.x = p.velocity
                wp.forward = True
            lane.waypoints.append(wp)

            ps = PoseStamped()
            ps.header = lane.header
            ps.pose = wp.pose.pose
            path_msg.poses.append(ps)

        self.local_lane_pub.publish(lane)
        self.local_viz_pub.publish(path_msg)


def main():
    try:
        LocalPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
