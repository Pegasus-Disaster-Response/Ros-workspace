#!/usr/bin/env python3
"""
Pegasus Full System Launch (Real Flight)
Complete: Sensors + SLAM + Costmap + A* + D* Lite 3D + MPC + PX4 Offboard

v2.5: Added PX4 offboard interface node.
      Removed px4_state_subscriber_node (stub — offboard node handles PX4 state).

Author: Team Pegasus
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
        'use_sim_time', default_value='false')
    launch_gazebo_bridge_arg = DeclareLaunchArgument(
        'launch_gazebo_bridge', default_value='false')
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')
    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip', default_value='192.168.1.201')
    fcu_dev_arg = DeclareLaunchArgument(
        'fcu_dev', default_value='/dev/ttyTHS1',
        description='Serial device for Pixhawk XRCE-DDS link (TELEM2 on companion)')
    fcu_baud_arg = DeclareLaunchArgument(
        'fcu_baud', default_value='921600',
        description='Baudrate for Pixhawk XRCE-DDS serial connection')
    enable_costmap_arg = DeclareLaunchArgument(
        'enable_costmap', default_value='true')
    enable_offboard_arg = DeclareLaunchArgument(
        'enable_offboard', default_value='true',
        description='Enable PX4 offboard interface')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo_bridge = LaunchConfiguration('launch_gazebo_bridge')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    velodyne_ip = LaunchConfiguration('velodyne_ip')
    fcu_dev = LaunchConfiguration('fcu_dev')
    fcu_baud = LaunchConfiguration('fcu_baud')
    enable_costmap = LaunchConfiguration('enable_costmap')
    enable_offboard = LaunchConfiguration('enable_offboard')

    # ── Gazebo Bridge ────────────────────────────────────────
    gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'vtol1_gazebo_bridge_launch.py'])]),
        condition=IfCondition(launch_gazebo_bridge))

    # ── Sensors ──────────────────────────────────────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'pegasus_sensors.launch.py'])]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'velodyne_ip': velodyne_ip,
            'fcu_dev': fcu_dev,
            'fcu_baud': fcu_baud}.items())

    # ── SLAM ─────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'pegasus_slam.launch.py'])]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rviz': rviz}.items())

    # ── Local Costmap ────────────────────────────────────────
    local_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pegasus_share, 'launch', 'local_costmap.launch.py'])]),
        condition=IfCondition(enable_costmap))

    # ── Mission Planner ──────────────────────────────────────
    mission_planner = Node(
        package='pegasus_ros',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # ── A* Global Planner ────────────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])
    global_planner = Node(
        package='pegasus_ros',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[planner_config, {'use_sim_time': use_sim_time}])

    # ── D* Lite 3D Replanner ─────────────────────────────────
    dstar_config = PathJoinSubstitution([
        pegasus_share, 'config', 'dstar_lite.yaml'])
    dstar_lite = Node(
        package='pegasus_ros',
        executable='dstar_lite_node',
        name='dstar_lite_node',
        output='screen',
        parameters=[dstar_config, {'use_sim_time': use_sim_time}])

    # ── MPC Trajectory Smoother ──────────────────────────────
    vtol_config = PathJoinSubstitution([
        pegasus_share, 'config', 'vtol_dynamics.yaml'])
    mpc = Node(
        package='pegasus_ros',
        executable='mpc_trajectory_node',
        name='mpc_trajectory_node',
        output='screen',
        parameters=[vtol_config, {'use_sim_time': use_sim_time}])

    # ── PX4 Offboard Interface ───────────────────────────────
    offboard_config = PathJoinSubstitution([
        pegasus_share, 'config', 'px4_offboard.yaml'])
    offboard = Node(
        package='pegasus_ros',
        executable='px4_offboard_node',
        name='px4_offboard_node',
        output='screen',
        parameters=[offboard_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_offboard))

    return LaunchDescription([
        use_sim_time_arg,
        launch_gazebo_bridge_arg,
        localization_arg,
        rviz_arg,
        velodyne_ip_arg,
        fcu_dev_arg,
        fcu_baud_arg,
        enable_costmap_arg,
        enable_offboard_arg,
        gazebo_bridge,
        sensors,
        slam,
        local_costmap,
        mission_planner,
        global_planner,
        dstar_lite,
        mpc,
        offboard,
    ])