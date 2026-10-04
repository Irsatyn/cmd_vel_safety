# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""
One-command visual demo: replay the bag and open rqt_graph plus rqt_plot.

    ros2 launch cmd_vel_safety demo.launch.py

The rqt_plot window shows the three curves that tell the whole story:
  /cmd_vel/linear/x          raw input, with the spike and the over-limit run
  /cmd_vel_safe/linear/x     processed output, smooth and inside the envelope
  /velocity_guard/report/flags  which rule fired, and exactly when

Requires a display. On a headless machine run replay_bag.launch.py instead and
inspect /robot_status with `ros2 topic echo`.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PLOT_TOPICS = [
    '/cmd_vel/linear/x',
    '/cmd_vel_safe/linear/x',
    '/velocity_guard/report/flags',
]


def generate_launch_description():
    pkg = FindPackageShare('cmd_vel_safety')

    replay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'replay_bag.launch.py'])),
        launch_arguments={
            'rate': LaunchConfiguration('rate'),
            'loop': LaunchConfiguration('loop'),
            'start_delay': '6.0',   # rqt takes a few seconds to come up
        }.items(),
    )

    # Started before the replay so the plots capture the run from t=0.
    graph = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=['rqt_graph'], output='log', name='rqt_graph',
            condition=IfCondition(LaunchConfiguration('rqt_graph')))],
    )
    plot = TimerAction(
        period=1.5,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'] + PLOT_TOPICS,
            output='log', name='rqt_plot',
            condition=IfCondition(LaunchConfiguration('rqt_plot')))],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('rqt_graph', default_value='true'),
        DeclareLaunchArgument('rqt_plot', default_value='true'),
        replay, graph, plot,
    ])
