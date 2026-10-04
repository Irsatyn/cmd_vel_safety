# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""
Replay the supplied /cmd_vel recording through the safety pipeline.

    ros2 launch cmd_vel_safety replay_bag.launch.py
    ros2 launch cmd_vel_safety replay_bag.launch.py rate:=0.5
    ros2 launch cmd_vel_safety replay_bag.launch.py loop:=true

The bag contains every anomaly the system is designed to handle (NaN, Inf, an
isolated spike, a sustained over-limit command, a non-holonomic request and a
4.1 s dropout), so this single command exercises the whole failure matrix.

`--clock` together with `use_sim_time:=true` is not cosmetic: without it the
control loop would run on the wall clock while the data arrives on the bag's
timeline, so every dt -- and therefore every acceleration limit and the
watchdog itself -- would be computed against the wrong time base as soon as
`rate` is not exactly 1.0.
"""

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

_TRUE = ('true', '1', 'yes', 'on')


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = FindPackageShare('cmd_vel_safety').perform(context)

    bag_path = LaunchConfiguration('bag_path').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    loop = LaunchConfiguration('loop').perform(context).lower() in _TRUE
    delay = float(LaunchConfiguration('start_delay').perform(context))

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'log_level': LaunchConfiguration('log_level'),
            'enable_virtual_robot': LaunchConfiguration('enable_virtual_robot'),
        }.items(),
    )

    # `--clock 100` publishes /clock at 100 Hz, comfortably above the guard's
    # 20 Hz control rate so the ROS timers are not starved of clock updates.
    cmd = ['ros2', 'bag', 'play', bag_path, '--clock', '100', '--rate', rate]
    if loop:
        # Looping restarts the bag clock, which the nodes see as a backwards
        # clock jump and respond to by resetting their safety state. Worth
        # exercising deliberately.
        cmd.append('--loop')

    play = TimerAction(
        # Give the graph time to finish discovery. Publishing into a graph whose
        # subscriptions have not matched yet simply loses the first messages.
        period=delay,
        actions=[ExecuteProcess(cmd=cmd, output='screen', name='bag_play')],
    )

    return [bringup, play]


def generate_launch_description():
    pkg = FindPackageShare('cmd_vel_safety')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=PathJoinSubstitution([pkg, 'bags', 'cmd_vel']),
            description='rosbag2 directory to replay.'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Replay speed multiplier.'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Replay the bag in a loop.'),
        DeclareLaunchArgument(
            'start_delay', default_value='3.0',
            description='Seconds to wait for node discovery before replaying.'),
        DeclareLaunchArgument(
            'enable_virtual_robot', default_value='true',
            description='Run the plant model alongside the guard.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'params.yaml']),
            description='YAML parameter file.'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_setup),
    ])
