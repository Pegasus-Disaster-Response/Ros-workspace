#!/usr/bin/env python3
"""
Pegasus Sensor Drivers Launch File
Launches: ZED X camera, VLP-16 LiDAR, XRCE-DDS Agent, PX4 IMU Bridge

Changes from v1.0:
  - Velodyne launched as direct nodes (not nested launch includes)
  - ZED X uses ros_params_override_path with zed_x.yaml
  - Added PX4 IMU bridge node (SensorCombined → sensor_msgs/Imu)
  - XRCE-DDS agent uses micro-xrce-dds-agent (run externally, not as ROS node)
  - Parameters loaded from YAML config files

Author: Team Pegasus
Date: 2026
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ══════════════════════════════════════════════════════════
    # Launch Arguments
    # ══════════════════════════════════════════════════════════
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from Gazebo'
    )

    enable_zed_arg = DeclareLaunchArgument(
        'enable_zed',
        default_value='true',
        description='Enable ZED X camera'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable VLP-16 LiDAR'
    )

    enable_xrce_arg = DeclareLaunchArgument(
        'enable_xrce',
        default_value='true',
        description='Enable XRCE-DDS agent (Pixhawk connection)'
    )

    enable_imu_bridge_arg = DeclareLaunchArgument(
        'enable_imu_bridge',
        default_value='true',
        description='Enable PX4 IMU bridge node'
    )

    fcu_dev_arg = DeclareLaunchArgument(
        'fcu_dev',
        default_value='/dev/ttyTHS1',
        description='Serial device for Pixhawk'
    )

    fcu_baud_arg = DeclareLaunchArgument(
        'fcu_baud',
        default_value='921600',
        description='Baudrate for Pixhawk serial connection'
    )

    velodyne_ip_arg = DeclareLaunchArgument(
        'velodyne_ip',
        default_value='192.168.1.201',
        description='VLP-16 IP address'
    )

    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_xrce = LaunchConfiguration('enable_xrce')
    enable_imu_bridge = LaunchConfiguration('enable_imu_bridge')
    fcu_dev = LaunchConfiguration('fcu_dev')
    fcu_baud = LaunchConfiguration('fcu_baud')
    velodyne_ip = LaunchConfiguration('velodyne_ip')

    # ══════════════════════════════════════════════════════════
    # 1. ZED X Stereo Camera
    # ══════════════════════════════════════════════════════════

    zed_launch_dir = PathJoinSubstitution([
        FindPackageShare('zed_wrapper'), 'launch'
    ])

    zed_x = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zed_x',
            'node_name': 'zed_node',

            # CRITICAL: Disable ZED's TF — RTAB-Map handles it
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
            'publish_urdf': 'false',

            # Disable ZED's internal odometry — RTAB-Map does this
            'pos_tracking_enabled': 'false',

            # Override with our config
            'ros_params_override_path': PathJoinSubstitution([
                pegasus_share, 'config', 'zed_x.yaml'
            ]),

            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_zed),
    )

    # ══════════════════════════════════════════════════════════
    # 2. Velodyne VLP-16 LiDAR (direct node declarations)
    # ══════════════════════════════════════════════════════════

    vlp16_config = PathJoinSubstitution([
        pegasus_share, 'config', 'vlp16.yaml'
    ])

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        parameters=[{
            'device_ip': velodyne_ip,
            'port': 2368,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'velodyne',
            'timestamp_first_packet': False,
        }],
        output='screen',
        condition=IfCondition(enable_lidar),
    )

    velodyne_convert = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_convert',
        parameters=[
            vlp16_config,
            {
                'calibration': '/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml',
                'organize_cloud': False,
                'min_range': 0.4,
                'max_range': 100.0,
            },
        ],
        output='screen',
        condition=IfCondition(enable_lidar),
    )

    # ══════════════════════════════════════════════════════════
    # 3. XRCE-DDS Agent — Pixhawk Bridge
    #    Uses micro-xrce-dds-agent (installed via snap)
    #    Runs as a system process, not a ROS node
    # ══════════════════════════════════════════════════════════

    xrce_agent = ExecuteProcess(
        cmd=[
            'micro-xrce-dds-agent', 'serial',
            '--dev', fcu_dev,
            '-b', fcu_baud,
        ],
        output='screen',
        condition=IfCondition(enable_xrce),
    )

    # ══════════════════════════════════════════════════════════
    # 4. PX4 IMU Bridge Node
    #    Converts PX4 SensorCombined → sensor_msgs/Imu
    #    for consumption by RTAB-Map
    # ══════════════════════════════════════════════════════════

    px4_imu_bridge = Node(
        package='pegasus_ros',
        executable='px4_imu_bridge_node',
        name='px4_imu_bridge_node',
        output='screen',
        parameters=[{
            'imu_frame_id': 'imu_link',
            'publish_rate_hz': 50.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(enable_imu_bridge),
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_zed_arg,
        enable_lidar_arg,
        enable_xrce_arg,
        enable_imu_bridge_arg,
        fcu_dev_arg,
        fcu_baud_arg,
        velodyne_ip_arg,

        # Sensors
        zed_x,
        velodyne_driver,
        velodyne_convert,

        # Pixhawk bridge
        xrce_agent,

        # IMU conversion
        px4_imu_bridge,
    ])