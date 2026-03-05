#!/usr/bin/env python3
"""
Pegasus Full System Launch
Complete system: Sensors + SLAM + Local Costmap + Path Planning

v2.3 changes:
  - Fix #4: Added local_costmap.launch.py include. Previously the
    full launch did NOT start costmap nodes (lidar_costmap_layer,
    zed_depth_costmap_layer, local_costmap_node), so the entire
    3D costmap pipeline was missing from full system runs.
  - Added enable_costmap argument for disabling costmap in test modes.

Author: Team Pegasus
Date: 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time from Gazebo'
    )

    launch_gazebo_bridge_arg = DeclareLaunchArgument(
        'launch_gazebo_bridge', default_value='false',
        description='Launch Gazebo bridge for simulation'
    )

    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Localization mode (vs mapping)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz visualization'
    )

    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip', default_value='192.168.1.201',
        description='VLP-16 IP address'
    )

    enable_costmap_arg = DeclareLaunchArgument(
        'enable_costmap', default_value='true',
        description='Launch 3D local costmap nodes'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo_bridge = LaunchConfiguration('launch_gazebo_bridge')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    velodyne_ip = LaunchConfiguration('velodyne_ip')
    enable_costmap = LaunchConfiguration('enable_costmap')

    # ── Gazebo Bridge (Optional) ─────────────────────────────
    gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'vtol1_gazebo_bridge_launch.py'
            ])
        ]),
        condition=IfCondition(launch_gazebo_bridge),
    )

    # ── Sensor Drivers ───────────────────────────────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'pegasus_sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'velodyne_ip': velodyne_ip,
        }.items(),
    )

    # ── RTAB-Map SLAM ────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'pegasus_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rviz': rviz,
        }.items(),
    )

    # ── 3D Local Costmap ─────────────────────────────────────
    # Fix #4: This was completely missing from the full launch.
    local_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'local_costmap.launch.py'
            ])
        ]),
        condition=IfCondition(enable_costmap),
    )

    # ── Mission Planning Node ────────────────────────────────
    mission_planner = Node(
        package='pegasus_ros',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── PX4 State Subscriber ─────────────────────────────────
    px4_state_subscriber = Node(
        package='pegasus_ros',
        executable='px4_state_subscriber_node',
        name='px4_state_subscriber_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Global Path Planner (A*) ──────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])

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
        # Arguments
        use_sim_time_arg,
        launch_gazebo_bridge_arg,
        localization_arg,
        rviz_arg,
        velodyne_ip_arg,
        enable_costmap_arg,

        # Launch components
        gazebo_bridge,
        sensors,
        slam,
        local_costmap,              # Fix #4: now included
        mission_planner,
        px4_state_subscriber,
        global_planner,
    ])