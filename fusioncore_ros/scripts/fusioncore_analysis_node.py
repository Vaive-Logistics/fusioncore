#!/usr/bin/env python3
"""Record FusionCore inputs/outputs and save diagnostic plots on shutdown."""

import csv
import math
import os
import signal
import time
from datetime import datetime
from threading import Lock

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from fusioncore_ros.msg import FilterHealth


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def diagonal_covariance(covariance, indexes):
    values = []
    for index in indexes:
        value = covariance[index]
        values.append(math.sqrt(value) if value >= 0.0 else float("nan"))
    return values


def yaw_from_quaternion(quaternion):
    numerator = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    denominator = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(numerator, denominator)


def unwrap_angles(values):
    if not values:
        return []
    return np.unwrap(np.asarray(values, dtype=float)).tolist()


def wrap_angles(values):
    values = np.asarray(values, dtype=float)
    return np.arctan2(np.sin(values), np.cos(values))


class FusioncoreAnalysisNode(Node):
    def __init__(self):
        super().__init__("fusioncore_analysis")
        self.declare_parameter("output_dir", "~/fusioncore_analysis")
        self.declare_parameter("idle_timeout_sec", 5.0)
        self.declare_parameter("save_period_sec", 0.0)

        self.output_root = os.path.abspath(os.path.expanduser(
            self.get_parameter("output_dir").value))
        self.idle_timeout = float(self.get_parameter("idle_timeout_sec").value)
        save_period = float(self.get_parameter("save_period_sec").value)
        self.lock = Lock()
        self.last_message_wall_time = time.monotonic()
        self.saved = False
        self.shutdown_requested = False
        self.gnss_origin = None

        self.imu = []
        self.gnss = []
        self.pose = []
        self.odom = []
        self.fake_imu = []
        self.imu_bias = []

        self.create_subscription(Imu, "/imu/data", self.imu_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/fix", self.gnss_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/fusion/pose",
                                 self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/fusion/odom", self.odom_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, "/ona2/fake_imu", self.fake_imu_callback,
                     qos_profile_sensor_data)
        self.create_subscription(FilterHealth, "/fusion/debug/filter_health",
                     self.filter_health_callback, qos_profile_sensor_data)
        self.create_timer(
            0.5, self._watch_for_end_of_bag,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        if save_period > 0.0:
            self.create_timer(save_period, lambda: self.save_plots("periodic"))
        self.get_logger().info(
            "Recording /imu/data, /fix, /fusion/pose, /fusion/odom, "
            "/ona2/fake_imu and /fusion/debug/filter_health")

    def _touch(self):
        self.last_message_wall_time = time.monotonic()

    def imu_callback(self, message):
        with self.lock:
            self.imu.append([
                stamp_seconds(message),
                message.angular_velocity.x, message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x, message.linear_acceleration.y,
                message.linear_acceleration.z,
                *diagonal_covariance(message.angular_velocity_covariance, [0, 4, 8]),
                *diagonal_covariance(message.linear_acceleration_covariance, [0, 4, 8]),
                *diagonal_covariance(message.orientation_covariance, [0, 4, 8]),
            ])
            self._touch()

    def gnss_callback(self, message):
        if message.status.status < 0 or not math.isfinite(message.latitude):
            return
        latitude = math.radians(message.latitude)
        longitude = math.radians(message.longitude)
        with self.lock:
            if self.gnss_origin is None:
                self.gnss_origin = (latitude, longitude)
            origin_latitude, origin_longitude = self.gnss_origin
            earth_radius = 6378137.0
            east = (longitude - origin_longitude) * math.cos(origin_latitude) * earth_radius
            north = (latitude - origin_latitude) * earth_radius
            sigma_x, sigma_y, sigma_z = diagonal_covariance(
                message.position_covariance, [0, 4, 8])
            self.gnss.append([
                stamp_seconds(message), east, north, message.altitude,
                sigma_x, sigma_y, sigma_z, int(message.status.status),
            ])
            self._touch()

    def pose_callback(self, message):
        pose = message.pose
        with self.lock:
            self.pose.append(self._pose_row(message.header.stamp, pose.pose, pose.covariance))
            self._touch()

    def odom_callback(self, message):
        with self.lock:
            self.odom.append([
                *self._pose_row(message.header.stamp, message.pose.pose,
                                message.pose.covariance),
                message.twist.twist.linear.x, message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.x, message.twist.twist.angular.y,
                message.twist.twist.angular.z,
                *diagonal_covariance(message.twist.covariance, [0, 7, 14, 21, 28, 35]),
            ])
            self._touch()

    def fake_imu_callback(self, message):
        with self.lock:
            self.fake_imu.append([
                stamp_seconds(message),
                yaw_from_quaternion(message.orientation),
                *diagonal_covariance(message.orientation_covariance, [0, 4, 8]),
            ])
            self._touch()

    def filter_health_callback(self, message):
        with self.lock:
            self.imu_bias.append([
                stamp_seconds(message),
                message.gyro_bias_x, message.gyro_bias_y, message.gyro_bias_z,
                message.accel_bias_x, message.accel_bias_y, message.accel_bias_z,
            ])
            self._touch()

    @staticmethod
    def _pose_row(stamp, pose, covariance):
        return [
            stamp.sec + stamp.nanosec * 1e-9,
            pose.position.x, pose.position.y, pose.position.z,
            yaw_from_quaternion(pose.orientation),
            *diagonal_covariance(covariance, [0, 7, 14, 21, 28, 35]),
        ]

    def _watch_for_end_of_bag(self):
        if self.idle_timeout <= 0.0 or self.saved:
            return
        now = time.monotonic()
        with self.lock:
            has_data = bool(self.imu or self.gnss or self.pose or self.odom or
                            self.fake_imu or self.imu_bias)
        if has_data and now - self.last_message_wall_time >= self.idle_timeout:
            self.save_plots("bag_idle")
            self.shutdown_requested = True
            rclpy.shutdown()

    def save_plots(self, reason):
        with self.lock:
            if self.saved or not (self.imu or self.gnss or self.pose or self.odom or
                                  self.fake_imu or self.imu_bias):
                return
            imu = list(self.imu)
            gnss = list(self.gnss)
            pose = list(self.pose)
            odom = list(self.odom)
            fake_imu = list(self.fake_imu)
            imu_bias = list(self.imu_bias)
            self.saved = True

        folder = os.path.join(self.output_root, datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(folder, exist_ok=True)
        self._write_csv(os.path.join(folder, "imu.csv"), imu,
                        ["t", "wx", "wy", "wz", "ax", "ay", "az", "sigma_wx",
                         "sigma_wy", "sigma_wz", "sigma_ax", "sigma_ay", "sigma_az",
                         "sigma_roll", "sigma_pitch", "sigma_yaw"])
        self._write_csv(os.path.join(folder, "gnss.csv"), gnss,
                        ["t", "east_m", "north_m", "altitude_m", "sigma_x_m",
                         "sigma_y_m", "sigma_z_m", "status"])
        self._write_csv(os.path.join(folder, "fusion_pose.csv"), pose,
                        ["t", "x_m", "y_m", "z_m", "yaw_rad", "sigma_x_m", "sigma_y_m",
                         "sigma_z_m", "sigma_roll_rad", "sigma_pitch_rad", "sigma_yaw_rad"])
        self._write_csv(os.path.join(folder, "fusion_odom.csv"), odom,
                        ["t", "x_m", "y_m", "z_m", "yaw_rad", "sigma_x_m", "sigma_y_m",
                         "sigma_z_m", "sigma_roll_rad", "sigma_pitch_rad", "sigma_yaw_rad",
                         "vx", "vy", "vz", "wx", "wy", "wz", "sigma_vx", "sigma_vy",
                         "sigma_vz", "sigma_wx", "sigma_wy", "sigma_wz"])
        self._write_csv(os.path.join(folder, "fake_imu.csv"), fake_imu,
                ["t", "yaw_rad", "sigma_roll_rad", "sigma_pitch_rad",
                 "sigma_yaw_rad"])
        self._write_csv(os.path.join(folder, "imu_bias.csv"), imu_bias,
                ["t", "gyro_bias_x_rad_s", "gyro_bias_y_rad_s",
                 "gyro_bias_z_rad_s", "accel_bias_x_m_s2",
                 "accel_bias_y_m_s2", "accel_bias_z_m_s2"])
        self._plot_imu(folder, imu)
        self._plot_gnss(folder, gnss)
        self._plot_pose(folder, pose, odom)
        self._plot_odom(folder, odom)
        self._plot_comparison(folder, gnss, pose, odom)
        self._plot_yaw_comparison(folder, pose, odom, fake_imu)
        self._plot_imu_bias(folder, imu_bias)
        self.get_logger().info(f"Saved {reason} analysis to {folder}")

    @staticmethod
    def _write_csv(path, rows, header):
        with open(path, "w", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(header)
            writer.writerows(rows)

    @staticmethod
    def _time(rows):
        if not rows:
            return np.array([])
        values = np.asarray(rows, dtype=float)
        return values[:, 0] - values[0, 0]

    @staticmethod
    def _finish_figure(figure, path):
        figure.tight_layout()
        figure.savefig(path, dpi=150)
        plt.close(figure)

    def _plot_imu(self, folder, rows):
        if not rows:
            return
        data = np.asarray(rows, dtype=float)
        time = self._time(rows)
        figure, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        axes[0].plot(time, data[:, 1:4], label=["wx", "wy", "wz"])
        axes[0].set_ylabel("gyro [rad/s]")
        axes[1].plot(time, data[:, 4:7], label=["ax", "ay", "az"])
        axes[1].set_ylabel("accel [m/s^2]")
        axes[2].plot(time, data[:, 7:10], label=["sigma wx", "sigma wy", "sigma wz"])
        axes[2].plot(time, data[:, 10:13], linestyle="--",
                     label=["sigma ax", "sigma ay", "sigma az"])
        axes[2].set_ylabel("reported sigma")
        axes[2].set_xlabel("bag time [s]")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right", ncol=3)
        self._finish_figure(figure, os.path.join(folder, "01_imu_measurements_and_covariance.png"))

    def _plot_imu_bias(self, folder, rows):
        if not rows:
            return
        data = np.asarray(rows, dtype=float)
        time = self._time(rows)
        figure, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
        axes[0].plot(time, data[:, 1:4], label=["x", "y", "z"])
        axes[0].set_ylabel("gyro bias [rad/s]")
        axes[0].set_title("FusionCore IMU bias estimates")
        axes[1].plot(time, data[:, 4:7], label=["x", "y", "z"])
        axes[1].set(xlabel="bag time [s]", ylabel="accel bias [m/s^2]")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right", ncol=3)
        self._finish_figure(figure, os.path.join(folder, "07_imu_bias_estimates.png"))

    def _plot_gnss(self, folder, rows):
        if not rows:
            return
        data = np.asarray(rows, dtype=float)
        time = self._time(rows)
        figure, axes = plt.subplots(2, 2, figsize=(13, 9))
        axes[0, 0].plot(data[:, 1], data[:, 2], ".-", markersize=2, label="GNSS")
        axes[0, 0].set(xlabel="east [m]", ylabel="north [m]", title="GNSS local track")
        axes[0, 0].legend()
        axes[0, 1].plot(time, data[:, 4:7], label=["sigma x", "sigma y", "sigma z"])
        axes[0, 1].set(xlabel="bag time [s]", ylabel="sigma [m]", title="GNSS reported 1-sigma")
        axes[0, 1].legend()
        axes[1, 0].plot(time, data[:, 1:3], label=["east", "north"])
        axes[1, 0].set(xlabel="bag time [s]", ylabel="position [m]", title="GNSS position")
        axes[1, 0].legend()
        axes[1, 1].step(time, data[:, 7], where="post", label="status")
        axes[1, 1].set(xlabel="bag time [s]", ylabel="NavSat status", title="GNSS fix status")
        axes[1, 1].legend()
        for axis in axes.flat:
            axis.grid(True, alpha=0.3)
        self._finish_figure(figure, os.path.join(folder, "02_gnss_track_and_covariance.png"))

    def _plot_pose(self, folder, pose_rows, odom_rows):
        rows = pose_rows or odom_rows
        if not rows:
            return
        data = np.asarray(rows, dtype=float)
        time = self._time(rows)
        figure, axes = plt.subplots(2, 2, figsize=(13, 9))
        axes[0, 0].plot(data[:, 1], data[:, 2], label="fusion pose")
        axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Fusion XY pose")
        axes[0, 0].legend()
        axes[0, 1].plot(time, data[:, 1:3], label=["x", "y"])
        axes[0, 1].set(xlabel="bag time [s]", ylabel="position [m]", title="Fusion position")
        axes[0, 1].legend()
        axes[1, 0].plot(time, np.degrees(unwrap_angles(data[:, 4].tolist())), label="yaw")
        axes[1, 0].set(xlabel="bag time [s]", ylabel="yaw [deg]", title="Fusion yaw")
        axes[1, 0].legend()
        axes[1, 1].plot(time, data[:, [5, 6, 10]], label=["sigma x", "sigma y", "sigma yaw"])
        axes[1, 1].set(xlabel="bag time [s]", ylabel="1-sigma", title="Fusion x/y/yaw covariance")
        axes[1, 1].legend()
        for axis in axes.flat:
            axis.grid(True, alpha=0.3)
        self._finish_figure(figure, os.path.join(folder, "03_fusion_pose_and_covariance.png"))

    def _plot_odom(self, folder, rows):
        if not rows:
            return
        data = np.asarray(rows, dtype=float)
        time = self._time(rows)
        figure, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
        axes[0].plot(time, data[:, 11:17], label=["vx", "vy", "vz", "wx", "wy", "wz"])
        axes[0].set_ylabel("twist")
        axes[1].plot(time, data[:, 17:23], label=["vx", "vy", "vz", "wx", "wy", "wz"])
        axes[1].set(xlabel="bag time [s]", ylabel="sigma", title="/fusion/odom twist covariance")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right", ncol=3)
        self._finish_figure(figure, os.path.join(folder, "04_fusion_odom_twist_and_covariance.png"))

    def _plot_comparison(self, folder, gnss_rows, pose_rows, odom_rows):
        fusion_rows = pose_rows or odom_rows
        if not gnss_rows or not fusion_rows:
            return
        gnss = np.asarray(gnss_rows, dtype=float)
        fusion = np.asarray(fusion_rows, dtype=float)
        figure, axes = plt.subplots(2, 2, figsize=(13, 9))
        axes[0, 0].plot(gnss[:, 1], gnss[:, 2], ".", label="GNSS")
        axes[0, 0].plot(fusion[:, 1], fusion[:, 2], label="Fusion")
        axes[0, 0].set(xlabel="east/x [m]", ylabel="north/y [m]", title="GNSS vs fusion")
        axes[0, 0].legend()
        axes[0, 1].plot(self._time(gnss_rows), gnss[:, 1], label="GNSS east")
        axes[0, 1].plot(self._time(fusion_rows), fusion[:, 1], label="Fusion x")
        axes[0, 1].set(xlabel="bag time [s]", ylabel="east/x [m]")
        axes[0, 1].legend()
        axes[1, 0].plot(self._time(gnss_rows), gnss[:, 2], label="GNSS north")
        axes[1, 0].plot(self._time(fusion_rows), fusion[:, 2], label="Fusion y")
        axes[1, 0].set(xlabel="bag time [s]", ylabel="north/y [m]")
        axes[1, 0].legend()
        axes[1, 1].plot(self._time(fusion_rows), np.degrees(unwrap_angles(fusion[:, 4].tolist())),
                        label="Fusion yaw")
        axes[1, 1].plot(self._time(fusion_rows), np.degrees(fusion[:, 10]),
                        label="yaw sigma")
        axes[1, 1].set(xlabel="bag time [s]", ylabel="degrees", title="Fusion yaw and uncertainty")
        axes[1, 1].legend()
        for axis in axes.flat:
            axis.grid(True, alpha=0.3)
        self._finish_figure(figure, os.path.join(folder, "05_gnss_vs_fusion.png"))

    def _plot_yaw_comparison(self, folder, pose_rows, odom_rows, fake_imu_rows):
        fusion_rows = pose_rows or odom_rows
        if not fusion_rows or not fake_imu_rows:
            return

        fusion = np.asarray(fusion_rows, dtype=float)
        fake_imu = np.asarray(fake_imu_rows, dtype=float)
        time_origin = min(fusion[0, 0], fake_imu[0, 0])
        fusion_time = fusion[:, 0] - time_origin
        fusion_plot_time = self._time(fusion_rows)
        fake_imu_time = fake_imu[:, 0] - time_origin
        fusion_yaw = wrap_angles(fusion[:, 4])
        fusion_yaw_unwrapped = np.asarray(unwrap_angles(fusion[:, 4].tolist()))
        fake_imu_yaw = wrap_angles(fake_imu[:, 1])
        pair_count = min(len(fusion_yaw), len(fake_imu_yaw) + 1)
        fusion_indices = np.arange(1, pair_count)
        fake_imu_indices = fusion_indices - 1
        comparison_time = fusion_time[fusion_indices]
        comparison_fake_time = fake_imu_time[fake_imu_indices]
        comparison_fusion_yaw = fusion_yaw[fusion_indices]
        comparison_fake_yaw = fake_imu_yaw[fake_imu_indices]
        yaw_difference = wrap_angles(comparison_fusion_yaw - comparison_fake_yaw)
        self._write_csv(
            os.path.join(folder, "06_yaw_inputs.csv"),
            [[float(fusion_timestamp), float(fake_timestamp),
              float(fusion_value), float(fake_value), float(difference),
              math.degrees(float(fusion_value)), math.degrees(float(fake_value)),
              math.degrees(float(difference))]
             for fusion_timestamp, fake_timestamp, fusion_value, fake_value, difference
             in zip(comparison_time, comparison_fake_time, comparison_fusion_yaw,
                    comparison_fake_yaw, yaw_difference)],
            ["fusion_t_s", "fake_imu_t_s", "fusioncore_yaw_rad",
             "fake_imu_yaw_rad", "yaw_difference_rad", "fusioncore_yaw_deg",
             "fake_imu_yaw_deg", "yaw_difference_deg"],
        )

        figure, axes = plt.subplots(2, 1, figsize=(13, 8))
        axes[0].plot(fusion_plot_time, np.degrees(fusion_yaw_unwrapped),
                 label="FusionCore yaw")
        axes[0].plot(fake_imu_time, np.degrees(fake_imu_yaw),
                     label="/ona2/fake_imu yaw")
        axes[0].set_ylabel("yaw [deg]")
        axes[0].set_title("FusionCore yaw vs fake IMU yaw")
        axes[0].legend()

        axes[1].plot(comparison_time, np.degrees(yaw_difference),
                     label="FusionCore yaw - previous fake IMU yaw")
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set_xlabel("bag time [s]")
        axes[1].set_ylabel("difference [deg]")
        axes[1].set_title("Wrapped yaw difference")
        axes[1].legend()

        for axis in axes:
            axis.grid(True, alpha=0.3)
        self._finish_figure(
            figure, os.path.join(folder, "06_fusion_yaw_vs_fake_imu.png"))

    def request_shutdown(self, *_args):
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.save_plots("shutdown")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FusioncoreAnalysisNode()
    signal.signal(signal.SIGINT, node.request_shutdown)
    signal.signal(signal.SIGTERM, node.request_shutdown)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if not node.saved:
            node.save_plots("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()