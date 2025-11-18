#!/usr/bin/env python3

import os
import math
import time
import subprocess
from datetime import datetime
from typing import Optional

import rospy
import rosgraph
import tf
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane


class ExperimentRunner:
    def __init__(self):
        rospy.init_node("experiment_runner", log_level=rospy.DEBUG)

        self.algorithm = rospy.get_param("~algorithm", "stanley")  # stanley or pure_persuit
        self.duration = rospy.get_param("~duration", 60.0)
        self.require_rviz = rospy.get_param("~require_rviz", False)
        self.start_controller = rospy.get_param("~start_controller", True)
        self.run_collection = rospy.get_param("~run_collection", True)
        self.run_metrics = rospy.get_param("~run_metrics", True)
        self.run_plots = rospy.get_param("~run_plots", True)
        self.run_index = rospy.get_param("~run_index", True)
        self.existing_data_path = rospy.get_param("~existing_data_path", "")
        results_root = rospy.get_param("~results_root", "")

        # paths
        self.base_dir = results_root if results_root else os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"
        )
        if self.run_collection:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.run_dir = os.path.join(self.base_dir, timestamp)
            os.makedirs(self.run_dir, exist_ok=True)
            self.data_path = os.path.join(self.run_dir, "data.txt")
        else:
            if not self.existing_data_path:
                rospy.logerr("run_collection is false but no existing_data_path provided")
                raise SystemExit(1)
            self.data_path = self.existing_data_path
            self.run_dir = os.path.dirname(self.data_path)
        os.makedirs(self.run_dir, exist_ok=True)
        self.summary_path = os.path.join(self.run_dir, "summary.txt")
        self.index_path = os.path.join(self.base_dir, "summary.txt")

        # state
        self.pose: Optional[PoseStamped] = None
        self.velocity: Optional[TwistStamped] = None
        self.waypoints: Optional[Lane] = None
        self.target_speed: float = 0.0

        rospy.Subscriber("/smart/rear_pose", PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber("/smart/velocity", TwistStamped, self.vel_cb, queue_size=1)
        rospy.Subscriber("/final_waypoints", Lane, self.waypoints_cb, queue_size=1)

        self.controller_process: Optional[subprocess.Popen] = None

        if self.run_collection:
            if self._check_prerequisites():
                if self.start_controller:
                    self._start_controller()
                self._run_experiment()
                if self.start_controller:
                    self._stop_controller()
            else:
                rospy.logerr("Prerequisite check failed, experiment not started")
                return
        if self.run_metrics or self.run_plots or self.run_index:
            self._analyze_and_plot()

    def _start_controller(self):
        launch_map = {
            "stanley": ["roslaunch", "stanley_controller", "stanley_controller.launch"],
            "pure_persuit": ["roslaunch", "pure_persuit", "pure_persuit.launch"],
        }
        cmd = launch_map.get(self.algorithm)
        if cmd:
            rospy.logwarn(f"Starting controller: {self.algorithm}")
            self.controller_process = subprocess.Popen(cmd)
        else:
            rospy.logwarn(f"No controller started, unknown algorithm: {self.algorithm}")
            return
        time.sleep(1.0)

    def _stop_controller(self):
        if self.controller_process and self.controller_process.poll() is None:
            rospy.logwarn("Stopping controller process")
            self.controller_process.terminate()
            try:
                self.controller_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Controller did not exit, killing")
                self.controller_process.kill()

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def vel_cb(self, msg: TwistStamped):
        self.velocity = msg

    def waypoints_cb(self, msg: Lane):
        self.waypoints = msg
        if msg.waypoints:
            self.target_speed = msg.waypoints[0].twist.twist.linear.x

    def _check_prerequisites(self) -> bool:
        """
        Check if gazebo, model, paths, and (optionally) rviz are available before the experiment.
        """
        ready = True
        attempts = 10
        delay = 1.0

        for attempt in range(attempts):
            topics = rospy.get_published_topics()
            topic_names = {t[0] for t in topics}

            master = rosgraph.Master(rospy.get_name())
            try:
                state = master.getSystemState()
                publishers = state[0]
                nodes_with_publishers = {n for _, nodelist in publishers for n in nodelist}
            except Exception as exc:  # pragma: no cover
                rospy.logwarn(f"Failed to read ROS master state: {exc}")
                nodes_with_publishers = set()

            gazebo_ok = any(t.startswith("/gazebo/") for t in topic_names) or "/gazebo" in topic_names
            model_ok = all(t in topic_names for t in ["/smart/rear_pose", "/smart/velocity"])
            global_path_ok = "/base_waypoints" in topic_names or "/lane" in topic_names
            local_path_ok = "/final_waypoints" in topic_names
            if self.require_rviz:
                rviz_ok = any("rviz" in n for n in nodes_with_publishers)
            else:
                rviz_ok = True

            rviz_status = "YES" if rviz_ok else ("SKIPPED" if not self.require_rviz else "NO")
            rospy.logwarn(f"Prereq check {attempt+1}/{attempts}: "
                          f"Gazebo={'YES' if gazebo_ok else 'NO'}, "
                          f"CarModel={'YES' if model_ok else 'NO'}, "
                          f"GlobalPath={'YES' if global_path_ok else 'NO'}, "
                          f"LocalPath={'YES' if local_path_ok else 'NO'}, "
                          f"RViz={rviz_status}")

            if gazebo_ok and model_ok and global_path_ok and local_path_ok and rviz_ok:
                return True

            rospy.sleep(delay)

        return False

    def _run_experiment(self):
        rospy.logwarn(f"Experiment running for {self.duration:.1f} seconds using {self.algorithm}")
        rate = rospy.Rate(20)
        start = rospy.Time.now().to_sec()
        with open(self.data_path, "w") as f:
            f.write("# stamp target_x target_y actual_x actual_y heading speed target_speed cte heading_err\n")
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                if (now - start) > self.duration:
                    break
                if self.pose and self.velocity and self.waypoints and self.waypoints.waypoints:
                    entry = self._collect_sample()
                    if entry:
                        f.write(" ".join(f"{v:.6f}" for v in entry) + "\n")
                rate.sleep()
        rospy.logwarn("Experiment data collection finished")

    def _collect_sample(self):
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        quat = (
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)

        nearest_idx, _ = self._nearest_waypoint(px, py, self.waypoints.waypoints)
        target_wp = self.waypoints.waypoints[nearest_idx]
        target_x = target_wp.pose.pose.position.x
        target_y = target_wp.pose.pose.position.y
        path_yaw = self._path_heading(nearest_idx, self.waypoints.waypoints)
        heading_err = self._normalize_angle(path_yaw - yaw)
        cte = self._cross_track_error(px, py, target_x, target_y, path_yaw)

        stamp = self.pose.header.stamp.to_sec() if self.pose.header.stamp else rospy.Time.now().to_sec()
        speed = self.velocity.twist.linear.x
        target_speed = target_wp.twist.twist.linear.x
        return [stamp, target_x, target_y, px, py, yaw, speed, target_speed, cte, heading_err]

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _cross_track_error(px, py, tx, ty, path_yaw):
        dx = px - tx
        dy = py - ty
        return -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

    @staticmethod
    def _path_heading(idx, waypoints):
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
    def _nearest_waypoint(x, y, waypoints):
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

    def _analyze_and_plot(self):
        if not os.path.exists(self.data_path) or os.path.getsize(self.data_path) == 0:
            rospy.logwarn(f"No data logged at {self.data_path}, skipping analysis")
            return
        data = np.loadtxt(self.data_path, comments="#")
        if data.size == 0:
            rospy.logwarn("No data logged, skipping analysis")
            return
        stamp = data[:, 0]
        target_xy = data[:, 1:3]
        actual_xy = data[:, 3:5]
        heading = data[:, 5]
        speed = data[:, 6]
        target_speed = data[:, 7]
        cte = data[:, 8]
        heading_err = data[:, 9]

        rms_cte = float(np.sqrt(np.mean(cte ** 2)))
        mean_abs_cte = float(np.mean(np.abs(cte)))
        rms_heading = float(np.sqrt(np.mean(heading_err ** 2)))
        speed_rmse = float(np.sqrt(np.mean((speed - target_speed) ** 2)))
        total_path = float(np.sum(np.linalg.norm(np.diff(actual_xy, axis=0), axis=1)))
        goal_err = float(np.linalg.norm(actual_xy[-1] - target_xy[-1]))

        if self.run_metrics:
            lines = [
                f"algorithm: {self.algorithm}",
                f"samples: {len(data)}",
                f"duration: {stamp[-1] - stamp[0]:.2f}s",
                f"rms_cte: {rms_cte:.4f}",
                f"mean_abs_cte: {mean_abs_cte:.4f}",
                f"rms_heading_error: {rms_heading:.4f}",
                f"speed_rmse: {speed_rmse:.4f}",
                f"actual_path_length: {total_path:.2f} m",
                f"goal_position_error: {goal_err:.4f} m",
            ]
            with open(self.summary_path, "w") as f:
                f.write("\n".join(lines))
            rospy.logwarn(f"Metrics written to {self.summary_path}")
            if self.run_index:
                with open(self.index_path, "a") as f:
                    f.write(f"{datetime.now().isoformat()} {self.algorithm} {self.summary_path}\n")
                rospy.logwarn(f"Run indexed in {self.index_path}")

        if self.run_plots:
            self._plot_paths(target_xy, actual_xy)
            self._plot_series(stamp, cte, "Cross-track error (m)", "cte.png")
            heading_err_deg = np.degrees(heading_err)
            self._plot_series(stamp, heading_err_deg, "Heading error (deg)", "heading_error.png")
            self._plot_speed(stamp, speed, target_speed)
            rospy.logwarn(f"Plots written to {self.run_dir}")

        rospy.logwarn(f"Analysis complete for data: {self.data_path}")

    def _plot_paths(self, target_xy, actual_xy):
        plt.figure()
        plt.plot(target_xy[:, 0], target_xy[:, 1], label="target path", linestyle="--")
        plt.plot(actual_xy[:, 0], actual_xy[:, 1], label="actual path")
        plt.axis("equal")
        plt.legend()
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("Path tracking")
        plt.grid(True)
        plt.tight_layout()
        path = os.path.join(self.run_dir, "path.png")
        plt.savefig(path)
        plt.close()

    def _plot_series(self, stamp, series, ylabel, filename):
        plt.figure()
        plt.plot(stamp, series)
        plt.xlabel("time (s)")
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.tight_layout()
        path = os.path.join(self.run_dir, filename)
        plt.savefig(path)
        plt.close()

    def _plot_speed(self, stamp, speed, target_speed):
        plt.figure()
        plt.plot(stamp, speed, label="actual")
        plt.plot(stamp, target_speed, label="target", linestyle="--")
        plt.xlabel("time (s)")
        plt.ylabel("speed (m/s)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        path = os.path.join(self.run_dir, "speed.png")
        plt.savefig(path)
        plt.close()


if __name__ == "__main__":
    try:
        ExperimentRunner()
    except rospy.ROSInterruptException:
        pass
