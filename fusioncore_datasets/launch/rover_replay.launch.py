"""Replay a rover bag through robot_localization, so a field run has a control.

WHY THIS EXISTS

The project's own benchmark discipline is "always run robot_localization as the
control and confirm it moved less than about 1 percent before believing any
FusionCore delta". That is applied rigorously to NCLT. It has never been applied to
a real hardware run: every field claim so far has been FusionCore against RAW GPS,
which is a weaker comparison than the one the project makes publicly.

It costs nothing to fix. `record.sh` on the rover uses `ros2 bag record -a`, so every
input RL needs is already in every bag. This plays the bag back and runs RL over the
same input, offline, with no rover time and no second field trip.

    ros2 launch fusioncore_datasets rover_replay.launch.py bag:=$HOME/fc_field_20260917_1432

It writes a SIDECAR bag next to the original, `<bag>_rl`, containing /rl/odometry.
The original bag is never touched. `hardware/closure_stats.py` picks the sidecar up
automatically and reports three closures per run: raw GPS, robot_localization, and
FusionCore.

ON FAIRNESS

`rl_ekf.yaml` is reused UNCHANGED from the NCLT harness. Its topics already match the
rover (`/odom/wheels`, `/imu/data`), and its `imu0_config` already takes roll and
pitch but not yaw, which is correct here too since the BNO085's RVC yaw is relative to
power on and published invalid. Nothing about RL has been retuned for this: a
benchmark against a handicapped competitor is worthless, and the temptation to tune
the other filter down is exactly the thing to avoid.

The one caveat to state out loud when quoting results: `process_noise_covariance` in
that file is documented as matching "NCLT Microstrain + Segway specs", not this rover.
It is inherited deliberately rather than silently. If someone can defend better
numbers for this chassis, that is a change worth making, and it should be made in the
open and applied to every run in the set.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context).rstrip('/'))
    rate = LaunchConfiguration('rate').perform(context)
    if not os.path.isdir(bag):
        raise RuntimeError(f"bag not found: {bag}")

    out = LaunchConfiguration('out').perform(context) or (bag + '_rl')
    out = os.path.expanduser(out)
    if os.path.exists(out):
        raise RuntimeError(
            f"output bag already exists: {out}\n"
            "Refusing to overwrite. A half-written or appended-to bag is worse than\n"
            "no bag, because it still evaluates to a plausible-looking number.")

    pkg = get_package_share_directory('fusioncore_datasets')
    rl_config = os.path.join(pkg, 'config', 'rl_ekf.yaml')
    nav_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    # --clock so every node runs on bag time. Without it RL runs on wall clock while
    # the data carries bag stamps, and every measurement looks arbitrarily old.
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
        output='screen')

    rl_ekf = Node(
        package='robot_localization', executable='ekf_node', name='rl_ekf',
        output='screen',
        remappings=[('odometry/filtered', '/rl/odometry')],
        parameters=[rl_config, {'use_sim_time': True}])

    # navsat needs the EKF's own output for heading, which is the standard RL
    # feedback loop. wait_for_datum is false in the config, so the local origin
    # latches on the first fix in the bag.
    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        remappings=[
            ('imu/data',          '/imu/data'),
            ('gps/fix',           '/fix'),          # the rover's topic, not /gnss/fix
            ('odometry/filtered', '/rl/odometry'),
            ('gps/filtered',      '/rl/gps/filtered'),
            ('odometry/gps',      '/gps/odometry'),
        ],
        parameters=[nav_config, {'use_sim_time': True}])

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out,
             '/rl/odometry', '/gps/odometry', '/clock'],
        output='screen')

    # Shut everything down when playback ends. Without this the recorder outlives
    # the run and appends to the next one: an NCLT bag came out of exactly that with
    # double the GPS fixes and still evaluated to a plausible number. Shutdown sends
    # SIGINT, which is what the recorder needs to write metadata.yaml; SIGKILL leaves
    # the bag unreadable.
    stop = RegisterEventHandler(OnProcessExit(
        target_action=play,
        on_exit=[LogInfo(msg=f'Playback finished. RL output -> {out}'),
                 EmitEvent(event=Shutdown(reason='playback complete'))]))

    return [LogInfo(msg=f'Replaying {bag} through robot_localization'),
            rl_ekf, navsat, record, play, stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='rover bag directory to replay'),
        DeclareLaunchArgument('out', default_value='',
                              description='output bag, default <bag>_rl'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='playback rate. Leave at 1.0 unless you '
                                          'have checked RL keeps up: a filter that '
                                          'cannot keep up drops measurements rather '
                                          'than slowing down, and does not say so.'),
        OpaqueFunction(function=_setup),
    ])
