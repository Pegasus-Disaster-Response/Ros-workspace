#!/usr/bin/env python3
"""
Pegasus SITL Full System Launch
================================
Launches PX4 SITL + Gazebo and the full autonomy stack.

Usage:
  ros2 launch pegasus_ros sitl_full.launch.py
  ros2 launch pegasus_ros sitl_full.launch.py vehicle:=gz_x500

Prerequisites:
  Start XRCE-DDS agent (UDP for SITL) before or after this launch:
      MicroXRCEAgent udp4 -p 8888

This launches:
  0. PX4 SITL + Gazebo Harmonic (~/PX4-Simulation)
  1. p110_gazebo_bridge  — clock, LiDAR, camera, depth, IMU, odometry, TF
  2. 3D local costmap
  3. A* global planner
  4. D* Lite 3D local replanner
  5. MPC trajectory smoother
  6. PX4 offboard interface
  7. Mission planner
  8. RViz

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ────────────────────────────────────────────
    vehicle_arg = DeclareLaunchArgument(
        'vehicle', default_value='p110_v2',
        description='PX4 SITL Gazebo vehicle model (e.g. p110_v2, x500)')

    gz_world_arg = DeclareLaunchArgument(
        'gz_world', default_value='p110_world',
        description='Gazebo world name (must match PX4_GZ_WORLD)')

    gz_model_arg = DeclareLaunchArgument(
        'gz_model', default_value='p110_v2_0',
        description='Gazebo model instance name (PX4 appends _0)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')



    auto_engage_arg = DeclareLaunchArgument(
        'auto_engage', default_value='false',
        description='Auto-engage offboard mode (SITL only!)')

    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm', default_value='false',
        description='Auto-arm vehicle (SITL only!)')

    gz_world = LaunchConfiguration('gz_world')
    gz_model = LaunchConfiguration('gz_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    auto_engage = LaunchConfiguration('auto_engage')
    auto_arm = LaunchConfiguration('auto_arm')

    # ── 0a. XRCE-DDS Agent ───────────────────────────────────
    xrce_agent = ExecuteProcess(
        cmd=['/home/pegasus/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    # ── 0b. QGroundControl ───────────────────────────────────
    qgc = ExecuteProcess(
        cmd=['flatpak', 'run', 'org.mavlink.qgroundcontrol'],
        output='screen',
    )

    # ── 0c. PX4 SITL + Gazebo ────────────────────────────────
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-lc', ['cd ~/PX4-Simulation && PX4_GZ_MODEL_NAME=p110_v2_0 make px4_sitl gz_p110_v2_p110_world CMAKE_ARGS=-DCMAKE_POLICY_VERSION_MINIMUM=3.5']],
        output='screen',
    )

    # ── 1. Gazebo Bridge (clock, LiDAR, camera, depth, IMU, TF) ─
    gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pegasus_share, 'launch', 'p110_gazebo_bridge_launch.py'])
        ]),
        launch_arguments={
            'gz_world': gz_world,
            'gz_model': gz_model,
            'rviz': 'false',  # RViz launched below
        }.items(),
    )

    # ── 2. Odometry placeholder ───────────────────────────────
    # Publishes a static odom→base_link TF until the full SLAM
    # stack is wired into SITL.
    static_odom = Node(
        package='pegasus_ros',
        executable='static_odom_publisher_node',
        name='static_odom_publisher_node',
        output='screen',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'start_z': 10.0,
            'publish_rate_hz': 10.0,
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'use_sim_time': use_sim_time,
        }])

    # ── 3. Local Costmap ─────────────────────────────────────
    costmap_config = PathJoinSubstitution([
        pegasus_share, 'config', 'local_costmap.yaml'])
    local_costmap = Node(
        package='pegasus_ros',
        executable='local_costmap_node',
        name='local_costmap_node',
        output='screen',
        parameters=[costmap_config, {'use_sim_time': use_sim_time}])

    # ── 4. A* Global Planner ─────────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])
    global_planner = Node(
        package='pegasus_ros',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[planner_config, {'use_sim_time': use_sim_time}])

    # ── 5. D* Lite 3D Replanner ──────────────────────────────
    dstar_config = PathJoinSubstitution([
        pegasus_share, 'config', 'dstar_lite.yaml'])
    dstar_lite = Node(
        package='pegasus_ros',
        executable='dstar_lite_node',
        name='dstar_lite_node',
        output='screen',
        parameters=[dstar_config, {'use_sim_time': use_sim_time}])

    # ── 6. MPC Trajectory Smoother ───────────────────────────
    mpc = Node(
        package='pegasus_ros',
        executable='mpc_trajectory_node',
        name='mpc_trajectory_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_topic':   '/pegasus/path_planner/global_path',
            'odom_topic':   '/odom',
            'cmd_topic':    '/pegasus/trajectory/setpoint',
            'status_topic': '/pegasus/trajectory/status',
        }])

    # ── 7. PX4 Offboard Interface ────────────────────────────
    offboard_config = PathJoinSubstitution([
        pegasus_share, 'config', 'px4_offboard.yaml'])
    offboard = Node(
        package='pegasus_ros',
        executable='px4_offboard_node',
        name='px4_offboard_node',
        output='screen',
        parameters=[
            offboard_config,
            {
                'use_sim_time': use_sim_time,
                'auto_engage': auto_engage,
                'auto_arm': auto_arm,
            },
        ])

    # ── 8. Mission Planner ──────────────────────────────────
    mission_planner = Node(
        package='pegasus_ros',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'auto_start_search': True}])

    # ── 9. RViz ─────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rviz_planner_test.rviz'])
    rviz_node = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
        )]
    )

    return LaunchDescription([
        # Arguments
        vehicle_arg,
        gz_world_arg,
        gz_model_arg,
        use_sim_time_arg,
        auto_engage_arg,
        auto_arm_arg,

        SetParameter(name='use_sim_time', value=use_sim_time),

        # XRCE-DDS + QGroundControl + PX4 SITL + Gazebo + all sensor bridges
        xrce_agent,
        qgc,
        px4_sitl,
        gazebo_bridge,

        # Odometry
        static_odom,

        # Costmap
        local_costmap,

        # Planning pipeline
        global_planner,
        dstar_lite,
        mpc,
        offboard,

        # Mission
        mission_planner,
        rviz_node,
    ])
