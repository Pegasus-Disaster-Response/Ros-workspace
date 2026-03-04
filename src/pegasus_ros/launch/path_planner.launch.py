#!/usr/bin/env python3
"""
Pegasus Path Planner Launch
Launches the A* global planner node with YAML configuration.

Usage:
    ros2 launch pegasus_ros path_planner.launch.py
    ros2 launch pegasus_ros path_planner.launch.py use_sim_time:=true

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ──────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time from Gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Config file path ──────────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])

    # ── Global Planner Node ───────────────────────────────
    global_planner = Node(
        package='pegasus_ros',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            planner_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        global_planner,
    ])