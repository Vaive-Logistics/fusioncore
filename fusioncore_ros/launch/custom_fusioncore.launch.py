import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    package_share = get_package_share_directory("fusioncore_ros")
    heading_estimator_share = get_package_share_directory("heading_estimator")
    heading_estimator_config = os.path.join(
        heading_estimator_share,
        "config",
        "heading_estimator_config.yaml",
    )
    fusioncore_config = LaunchConfiguration("fusioncore_config")
    bag = LaunchConfiguration("bag")
    analysis_output = LaunchConfiguration("analysis_output")

    # -------------------------------------------------------------------------
    # FusionCore
    # -------------------------------------------------------------------------

    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[
            fusioncore_config,
            {"autostart": False, "use_sim_time": True},
        ],
    )
    activate_fusioncore = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=fusioncore_node,
        start_state="configuring",
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action is fusioncore_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    configure_fusioncore = TimerAction(
        period=10.0,  #decide when to switch to active state
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action is fusioncore_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))],
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
            heading_estimator_config,
            {
                "use_sim_time": True,
                #"frame_id": "ona2/map",
                "frame_id": "odom",                
                "use_wheel_odom_for_motion_check": True,
                "wheel_odom_topic": "/ona2/local_odom_combined",
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Analysis
    # -------------------------------------------------------------------------
    start_analysis = Node(
        package="fusioncore_analysis",
        executable="fusioncore_analysis_node",
        name="fusioncore_analysis",
        output="screen",
        parameters=[
            {
                "output_dir": analysis_output,
                "idle_timeout_sec": 0.0,
            }
        ],
    )

    #node that publish the fusioncore path (poses)
    start_path_publisher = Node(
        package="fusioncore_gazebo",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # -------------------------------------------------------------------------
    # Rosbag
    # -------------------------------------------------------------------------

    start_bag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag,
            "--clock",
            "--remap",
            "/ona2/sensors/imu_front/imu_uncalib:=/imu/data",
            # "/fix:=/gnss/fix",
            "--rate", "1.0",
            "--start-offset", "25.0",
        ],
        output="screen",
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
            "fusioncore_config",
            default_value=os.path.join(
                package_share,
                "config",
                "fusioncore.yaml"
            ),
            description="Path to the FusionCore YAML configuration",
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
        fusioncore_node,
        activate_fusioncore,
        configure_fusioncore,
        start_heading_estimator,
        start_analysis,
        start_path_publisher,

        # Start the clock and all bag topics
        start_bag,
    ])