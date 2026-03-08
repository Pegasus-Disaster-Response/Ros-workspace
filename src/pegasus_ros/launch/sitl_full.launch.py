#!/usr/bin/env python3
"""
Pegasus SITL Full System Launch
================================
Launches the full autonomy stack against PX4 SITL in Gazebo Harmonic.

PREREQUISITES (run these BEFORE this launch file):
  Terminal A: Start PX4 SITL + Gazebo
      cd ~/PX4-Autopilot
      make px4_sitl gz_x500

  Terminal B: Start XRCE-DDS agent (UDP for SITL)
      MicroXRCEAgent udp4 -p 8888

  Terminal C: (optional) Start QGroundControl for monitoring

THEN launch this file:
  ros2 launch pegasus_ros sitl_full.launch.py

This launches:
  1. ros_gz_bridge  — clock + simulated sensor bridges
  2. IMU bridge     — PX4 SensorCombined → sensor_msgs/Imu
  3. SLAM (RTAB-Map) with dual odometry
  4. 3D local costmap
  5. A* global planner
  6. D* Lite 3D local replanner
  7. MPC trajectory smoother
  8. PX4 offboard interface
  9. RViz

The PX4 SITL x500 model doesn't include a VLP-16 or ZED X by default.
For initial testing, the planner uses RTAB-Map's grid (which starts
empty and grows) or the local costmap (which starts with the simulated
depth sensor if configured). For full sensor simulation, add LiDAR and
depth camera plugins to the Gazebo model SDF.

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')
    auto_engage_arg = DeclareLaunchArgument(
        'auto_engage', default_value='false',
        description='Auto-engage offboard mode (SITL only!)')
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm', default_value='false',
        description='Auto-arm vehicle (SITL only!)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    auto_engage = LaunchConfiguration('auto_engage')
    auto_arm = LaunchConfiguration('auto_arm')

    # ── 1. ros_gz_bridge: clock ──────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    # ── 2. Static TFs ────────────────────────────────────────
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=['0.52', '0.0', '0.85', '0', '0', '0',
                   'base_link', 'velodyne'])

    tf_base_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_x',
        arguments=['1.78', '0.0', '0.55', '0', '0', '0',
                   'base_link', 'zed_x_camera_center'])

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'imu_link'])

    # ── 3. IMU Bridge ────────────────────────────────────────
    imu_bridge = Node(
        package='pegasus_ros',
        executable='px4_imu_bridge_node',
        name='px4_imu_bridge_node',
        output='screen',
        parameters=[{
            'imu_frame_id': 'imu_link',
            'publish_rate_hz': 50.0,
            'use_sim_time': use_sim_time,
        }])

    # ── 4. Odometry Selector ─────────────────────────────────
    # In SITL without simulated LiDAR/ZED, odometry comes from
    # PX4's internal EKF. We publish a bridge from VehicleOdometry.
    # For now, the static_odom_publisher provides a fixed position
    # until the full sensor sim is configured.
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

    # ── 5. Local Costmap ─────────────────────────────────────
    costmap_config = PathJoinSubstitution([
        pegasus_share, 'config', 'local_costmap.yaml'])
    local_costmap = Node(
        package='pegasus_ros',
        executable='local_costmap_node',
        name='local_costmap_node',
        output='screen',
        parameters=[costmap_config, {'use_sim_time': use_sim_time}])

    # ── 6. A* Global Planner ─────────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])
    global_planner = Node(
        package='pegasus_ros',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[planner_config, {'use_sim_time': use_sim_time}])

    # ── 7. D* Lite 3D Replanner ──────────────────────────────
    dstar_config = PathJoinSubstitution([
        pegasus_share, 'config', 'dstar_lite.yaml'])
    dstar_lite = Node(
        package='pegasus_ros',
        executable='dstar_lite_node',
        name='dstar_lite_node',
        output='screen',
        parameters=[dstar_config, {'use_sim_time': use_sim_time}])

    # ── 8. MPC Trajectory Smoother ───────────────────────────
    vtol_config = PathJoinSubstitution([
        pegasus_share, 'config', 'vtol_dynamics.yaml'])
    mpc = Node(
        package='pegasus_ros',
        executable='mpc_trajectory_node',
        name='mpc_trajectory_node',
        output='screen',
        parameters=[vtol_config, {'use_sim_time': use_sim_time}])

    # ── 9. PX4 Offboard Interface ────────────────────────────
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

    # ── 10. Mission Planner ──────────────────────────────────
    mission_planner = Node(
        package='pegasus_ros',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # ── 11. RViz ─────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rviz_planner_test.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        rviz_arg,
        auto_engage_arg,
        auto_arm_arg,

        SetParameter(name='use_sim_time', value=use_sim_time),

        # Infrastructure
        clock_bridge,
        tf_map_odom,
        tf_base_to_velodyne,
        tf_base_to_zed,
        tf_base_to_imu,

        # Sensor processing
        imu_bridge,
        static_odom,
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