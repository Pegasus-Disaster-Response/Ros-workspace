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
  1. p110_gazebo_bridge  — clock, LiDAR, camera, depth, IMU, TF
  2. SLAM (RTAB-Map + ICP + RGB-D odometry, delayed 10 s for Gazebo boot)
  3. 3D local costmap
  4. A* global planner
  5. D* Lite 3D local replanner
  6. MPC trajectory smoother
  7. PX4 offboard interface
  8. Mission planner
  9. RViz

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
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

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/Ros-workspace/maps/pegasus_disaster_map.db'),
        description='Path to RTAB-Map database for SITL')

    gz_world = LaunchConfiguration('gz_world')
    gz_model = LaunchConfiguration('gz_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    database_path = LaunchConfiguration('database_path')

    auto_engage = LaunchConfiguration('auto_engage')
    auto_arm = LaunchConfiguration('auto_arm')

    # ── PX4 Odometry Bridge (NED → ENU /odom_px4) ───────────
    # Converts /fmu/out/vehicle_odometry to nav_msgs/Odometry on
    # /odom_px4. Used as primary odometry in SITL because Gazebo's
    # ground-truth gz topic isn't published by p110_v2 spawn, and
    # ICP drifts heavily on z with a 360-sample lidar.
    px4_odom_bridge = Node(
        package='pegasus_ros',
        executable='px4_odom_bridge_node',
        name='px4_odom_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

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

    # ── 2. SLAM ───────────────────────────────────────────────
    # Delayed 10 s to give PX4 SITL + Gazebo time to start.
    # wait_imu_to_init=true (in icp_odometry.yaml / rtabmap.yaml)
    # means the SLAM nodes will self-block until the IMU arrives,
    # so the delay just needs to be long enough for the bridge to
    # have started bridging topics.
    slam = TimerAction(
        period=4.0,  # was 10.0; the lighter sensors and lower physics load
                      # mean Gazebo's bridge is up well before this fires
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pegasus_share, 'launch', 'pegasus_slam.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': 'false',
                'database_path': database_path,
                # /odom_px4 is published by px4_odom_bridge_node — it
                # converts /fmu/out/vehicle_odometry (NED) to ENU and is
                # the only reliable ground-truth odometry source for SITL
                # (Gazebo OdometryPublisher isn't auto-attached to p110_v2,
                # and ICP drifts heavily in z with sparse lidar).
                'primary_odom_topic': '/odom_px4',
            }.items(),
        )]
    )

    # ── 3a. LiDAR Costmap Layer (raw obstacle preprocessor) ─
    # Reads /velodyne_points, ground-filters, publishes obstacle cloud
    # on /pegasus/lidar_obstacles. Without this, local_costmap_node has
    # nothing to fuse and the costmap stays empty — D*/A* then plan
    # straight through walls.
    lidar_costmap_layer = Node(
        package='pegasus_ros',
        executable='lidar_costmap_layer_node',
        name='lidar_costmap_layer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # ── 3b. Local Costmap (fuses obstacle layers) ────────────
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
                # ParameterValue(..., value_type=bool) forces launch to coerce
                # the LaunchConfiguration string ("true"/"false") to a real
                # bool. Without this the node silently falls back to the YAML
                # default (false) because a string value can't override a
                # bool-typed declared parameter.
                'auto_engage': ParameterValue(auto_engage, value_type=bool),
                'auto_arm':    ParameterValue(auto_arm,    value_type=bool),
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
        database_path_arg,

        SetParameter(name='use_sim_time', value=use_sim_time),

        # XRCE-DDS + QGroundControl + PX4 SITL + Gazebo + all sensor bridges
        xrce_agent,
        qgc,
        px4_sitl,
        gazebo_bridge,

        # PX4 NED → ENU /odom_px4 converter (primary odom in SITL)
        px4_odom_bridge,

        # SLAM
        slam,

        # Costmap pipeline (lidar→obstacles→fused 2D/3D costmap)
        lidar_costmap_layer,
        local_costmap,

        # Planning pipeline
        # A* now publishes the *initial* plan on
        # /pegasus/path_planner/global_path_initial. D* Lite publishes the
        # *live* plan on /pegasus/path_planner/global_path (MPC consumes
        # this) and replans when obstacles appear in the costmap.
        global_planner,
        dstar_lite,
        mpc,
        offboard,

        # Mission
        mission_planner,
        rviz_node,
    ])
