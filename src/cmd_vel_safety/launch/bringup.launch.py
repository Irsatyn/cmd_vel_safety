# Copyright 2026 cmd_vel_safety contributors. Apache-2.0.
"""
Bring up the velocity safety pipeline.

    ros2 launch cmd_vel_safety bringup.launch.py

Every path and topic is a launch argument, so the same file serves the bag
replay, the recording and the live-hardware case.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('cmd_vel_safety')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'params.yaml']),
            description='YAML file with parameters for all three nodes.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Drive the nodes off /clock. Required when replaying a '
                        'bag with --clock so that dt and the watchdog follow '
                        'the replay timeline instead of the wall clock.'),
        DeclareLaunchArgument(
            'enable_virtual_robot', default_value='true',
            description='Run the plant model. Disable when a real base is '
                        'already consuming /cmd_vel_safe.'),
        DeclareLaunchArgument(
            'enable_monitor', default_value='true',
            description='Run the motion state monitor.'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Raw (untrusted) command topic.'),
        DeclareLaunchArgument(
            'cmd_vel_safe_topic', default_value='/cmd_vel_safe',
            description='Processed command topic consumed by the base.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS log severity: debug|info|warn|error|fatal.'),
    ]

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    raw_topic = LaunchConfiguration('cmd_vel_topic')
    safe_topic = LaunchConfiguration('cmd_vel_safe_topic')

    common = {'use_sim_time': use_sim_time}
    cli = ['--ros-args', '--log-level', log_level]

    guard = Node(
        package='cmd_vel_safety',
        executable='velocity_guard',
        name='velocity_guard',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, common],
        # Remapping keeps the node reusable: the topic names in params.yaml are
        # the defaults, these are the deployment-specific overrides.
        remappings=[('/cmd_vel', raw_topic), ('/cmd_vel_safe', safe_topic)],
        arguments=cli,
    )

    monitor = Node(
        package='cmd_vel_safety',
        executable='motion_state_monitor',
        name='motion_state_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, common],
        arguments=cli,
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
    )

    robot = Node(
        package='cmd_vel_safety',
        executable='virtual_robot',
        name='virtual_robot',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, common],
        arguments=cli,
        condition=IfCondition(LaunchConfiguration('enable_virtual_robot')),
    )

    return LaunchDescription(args + [guard, robot, monitor])
