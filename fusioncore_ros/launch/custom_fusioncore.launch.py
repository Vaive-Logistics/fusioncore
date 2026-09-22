import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fusioncore_ros")
    fusioncore_launch = os.path.join(
        package_share,
        "launch",
        "fusioncore.launch.py"
    )

    fusioncore_config = LaunchConfiguration("fusioncore_config")
    bag = LaunchConfiguration("bag")
    bag_delay = LaunchConfiguration("bag_delay")
    analysis_output = LaunchConfiguration("analysis_output")

    # -------------------------------------------------------------------------
    # FusionCore
    # -------------------------------------------------------------------------

    start_fusioncore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fusioncore_launch),
        launch_arguments={
            "fusioncore_config": fusioncore_config,
            "autoconfigure": "true",
            "use_sim_time": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Heading estimator
    # -------------------------------------------------------------------------

    start_heading_estimator = Node(
        package="heading_estimator",
        executable="heading_estimator",
        name="heading_estimator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Analysis
    # -------------------------------------------------------------------------

    #start_analysis = Node(
    #    package="fusioncore_ros",
    #    executable="fusioncore_analysis_node.py",
    #    name="fusioncore_analysis",
    #    output="screen",
    #    parameters=[
    #        {
    #            "output_dir": analysis_output,
    #            "idle_timeout_sec": 5.0,
    #        }
    #    ],
    #)

    # -------------------------------------------------------------------------
    # Rosbag
    # -------------------------------------------------------------------------

    start_bag = TimerAction(
        period=bag_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play", bag,
                    "--clock",
                    "--remap",
                    "/ona2/sensors/imu_front/imu_uncalib:=/imu/data",
                    # "/fix:=/gnss/fix",
                    "--rate", "1.0",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            default_value=(
                "/home/clement/Downloads/rosbags/ona2_gnss_bag/"
                "ona2_gnss_bag_0.db3"
            ),
            description="Path to the rosbag2 directory or database file",
        ),

        DeclareLaunchArgument(
            "fusioncore_config",
            default_value=os.path.join(
                package_share,
                "config",
                "fusioncore.yaml"
            ),
            description="Path to the FusionCore YAML configuration",
        ),

        DeclareLaunchArgument(
            "bag_delay",
            default_value="0.0",
            description="Seconds to wait before starting rosbag playback",
        ),

        DeclareLaunchArgument(
            "analysis_output",
            default_value=os.path.join(
                os.path.expanduser("~"),
                "fusioncore_analysis"
            ),
            description="Root directory for timestamped analysis results",
        ),

        # Start nodes BEFORE the bag
        start_fusioncore,
        start_heading_estimator,
        #start_analysis,

        # Start the clock and all bag topics
        start_bag,
    ])