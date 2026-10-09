#!/usr/bin/env python3
"""Publish FusionCore odometry as a path anchored to the heading-estimator path."""

import copy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class CenteredOdomPathNode(Node):
    def __init__(self):
        super().__init__("centered_odom_path")
        self._heading_start = None
        self._first_odom = None
        self._fusion_start_xy = None
        self._pending_odometry = []
        self._frame_id = ""
        self._path = Path()

        self._path_pub = self.create_publisher(Path, "/fusion/odom/centered", 10)
        self.create_subscription(
            Path, "/heading_estimator/path", self._heading_path_callback, 10)
        self.create_subscription(
            Odometry, "/fusion/odom", self._odom_callback, 100)

    def _heading_path_callback(self, message):
        if self._heading_start is not None or not message.poses:
            return

        self._heading_start = copy.deepcopy(message.poses[0].pose.position)
        self._frame_id = message.header.frame_id or message.poses[0].header.frame_id
        self._try_initialize_alignment()

    def _odom_callback(self, message):
        if self._first_odom is None:
            self._first_odom = copy.deepcopy(message)

        if self._fusion_start_xy is None:
            self._pending_odometry.append(copy.deepcopy(message))
            self._try_initialize_alignment()
            return

        self._append_centered_odometry(message)

    def _try_initialize_alignment(self):
        if (self._fusion_start_xy is not None or self._heading_start is None
                or self._first_odom is None):
            return

        odom_frame = self._first_odom.header.frame_id
        fusion_start = self._first_odom.pose.pose.position
        self._fusion_start_xy = (fusion_start.x, fusion_start.y)
        self._frame_id = self._frame_id or odom_frame
        self.get_logger().info(
            "Path alignment: heading start=(%.3f, %.3f), "
            "first fusion odom=(%.3f, %.3f)"
            % (self._heading_start.x, self._heading_start.y,
               self._fusion_start_xy[0], self._fusion_start_xy[1]))
        for odometry in self._pending_odometry:
            self._append_centered_odometry(odometry)
        self._pending_odometry.clear()
        self.get_logger().info(
            "Aligned /fusion/odom path start to /heading_estimator/path start")

    def _append_centered_odometry(self, message):
        pose = PoseStamped()
        pose.header = copy.deepcopy(message.header)
        pose.header.frame_id = self._frame_id
        pose.pose = copy.deepcopy(message.pose.pose)
        if not self._path.poses:
            pose.pose.position.x = self._heading_start.x
            pose.pose.position.y = self._heading_start.y
            self.get_logger().info(
                "First centered pose=(%.3f, %.3f)"
                % (pose.pose.position.x, pose.pose.position.y))
        else:
            pose.pose.position.x = (
                self._heading_start.x
                + message.pose.pose.position.x - self._fusion_start_xy[0])
            pose.pose.position.y = (
                self._heading_start.y
                + message.pose.pose.position.y - self._fusion_start_xy[1])

        self._path.header.stamp = message.header.stamp
        self._path.header.frame_id = self._frame_id
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = CenteredOdomPathNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()