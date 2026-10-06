import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    heading_estimator_share = get_package_share_directory("heading_estimator")
    heading_estimator_config = os.path.join(
        heading_estimator_share,
        "config",
        "heading_estimator_config.yaml",
    )
    bag = LaunchConfiguration("bag")
    bag_delay = LaunchConfiguration("bag_delay")

    start_heading_estimator = Node(
        package="heading_estimator",
        executable="heading_estimator",
        name="heading_estimator",
        output="screen",
        parameters=[
            heading_estimator_config,
            {
                "use_sim_time": True,
                "frame_id": "odom",
                "use_wheel_odom_for_motion_check": True,
                "wheel_odom_topic": "/ona2/local_odom_combined",
            }
        ],
    )

    start_bag = TimerAction(
        period=bag_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play", bag,
                    "--clock",
                    "--remap",
                    "/ona2/sensors/imu_front/imu_uncalib:=/imu/data",
                    "--rate", "1.0",
                    "--start-offset", "22.0",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            default_value=(
                "/home/clement/Downloads/rosbags/ona2_gnss_15_09_2026/"
                "ona2_gnss_15_09_2026_0.db3"
            ),
            description="Path to the rosbag2 directory or database file",
        ),
        DeclareLaunchArgument(
            "bag_delay",
            default_value="0.0",
            description="Seconds to wait before starting rosbag playback",
        ),
        start_heading_estimator,
        start_bag,
    ])