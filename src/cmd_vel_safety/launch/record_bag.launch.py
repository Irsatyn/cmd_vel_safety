# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""
Replay the input bag through the pipeline while recording the whole data flow.

    ros2 launch cmd_vel_safety record_bag.launch.py
    ros2 launch cmd_vel_safety record_bag.launch.py output:=/tmp/run_42

Produces a bag containing the raw input, the processed output, the per-cycle
safety report, the odometry and the motion state -- everything needed to audit
each intervention offline, and the input to scripts/check_bag.py.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

TOPICS = [
    '/cmd_vel',
    '/cmd_vel_safe',
    '/velocity_guard/report',
    '/odom',
    '/robot_motion_state',
    '/robot_status',
    '/diagnostics',
    '/e_stop',
    '/clock',
]


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = FindPackageShare('cmd_vel_safety').perform(context)

    out = LaunchConfiguration('output').perform(context)
    if not out:
        out = os.path.join(
            os.path.expanduser('~'),
            'cmd_vel_safety_runs',
            'run_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)

    replay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'replay_bag.launch.py'])),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'bag_path': LaunchConfiguration('bag_path'),
            'rate': LaunchConfiguration('rate'),
            'start_delay': LaunchConfiguration('start_delay'),
        }.items(),
    )

    # Start recording before the replay begins (replay_bag waits start_delay
    # seconds) so that no output message is missed.
    record = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', out] + TOPICS,
            output='screen',
            name='bag_record',
        )],
    )

    return [replay, record]


def generate_launch_description():
    pkg = FindPackageShare('cmd_vel_safety')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output', default_value='',
            description='Output bag directory. Defaults to '
                        '~/cmd_vel_safety_runs/run_<timestamp>.'),
        DeclareLaunchArgument(
            'bag_path',
            default_value=PathJoinSubstitution([pkg, 'bags', 'cmd_vel'])),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_delay', default_value='4.0'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'params.yaml'])),
        OpaqueFunction(function=_launch_setup),
    ])
