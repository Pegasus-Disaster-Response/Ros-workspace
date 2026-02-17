#!/usr/bin/env python3
"""
Pegasus Sensor Drivers Launch File
Launches: ZED X camera, VLP-16 LiDAR, XRCE-DDS Agent (Pixhawk bridge)

Author: Team Pegasus
Date: 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # ============================================
    # Launch Arguments
    # ============================================
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
    
    # Pixhawk connection arguments
    fcu_dev_arg = DeclareLaunchArgument(
        'fcu_dev',
        default_value='/dev/ttyTHS1',  # Jetson UART
        description='Serial device for Pixhawk'
    )
    
    fcu_baud_arg = DeclareLaunchArgument(
        'fcu_baud',
        default_value='921600',
        description='Baudrate for Pixhawk serial connection'
    )
    
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_xrce = LaunchConfiguration('enable_xrce')
    fcu_dev = LaunchConfiguration('fcu_dev')
    fcu_baud = LaunchConfiguration('fcu_baud')
    
    # ============================================
    # ZED X Stereo Camera
    # ============================================
    
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zed_x',
            'node_name': 'zed_node',
            
            # CRITICAL: Disable ZED's TF - RTAB-Map handles it
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'false',
            
            # Disable ZED's odometry - RTAB-Map does this
            'pos_tracking_enabled': 'false',
            
            # Camera settings
            'depth_mode': 'NEURAL',
            'depth_stabilization': '1',
            'grab_resolution': 'HD1080',
            'grab_frame_rate': '30',
            
            # Enable stereo for RTAB-Map
            'publish_stereo': 'true',
            'publish_left_camera': 'true',
            'publish_right_camera': 'true',
            'publish_depth': 'true',
            
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_zed)
    )
    
    # ============================================
    # Velodyne VLP-16 LiDAR
    # ============================================
    
    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne_driver'),
                'launch',
                'velodyne_driver_node-VLP16-launch.py'
            ])
        ]),
        launch_arguments={
            'device_ip': '192.168.1.201',  # Update to your VLP-16 IP
            'gps_time': 'false',
            'frame_id': 'velodyne',
            'model': 'VLP16',
            'rpm': '600.0',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_lidar)
    )
    
    velodyne_convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'),
                'launch',
                'velodyne_convert_node-VLP16-launch.py'
            ])
        ]),
        launch_arguments={
            'calibration': PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'),
                'params',
                'VLP16db.yaml'
            ]),
            'model': 'VLP16',
            'fixed_frame': 'velodyne',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_lidar)
    )
    
    # ============================================
    # XRCE-DDS Agent - Pixhawk Bridge
    # ============================================
    
    xrce_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=[
            'serial',
            '--dev', fcu_dev,
            '--baud', fcu_baud
        ],
        output='screen',
        condition=IfCondition(enable_xrce)
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_zed_arg,
        enable_lidar_arg,
        enable_xrce_arg,
        fcu_dev_arg,
        fcu_baud_arg,
        
        # Sensor launches
        zed_wrapper_launch,
        velodyne_driver_launch,
        velodyne_convert_launch,
        
        # Pixhawk bridge
        xrce_agent,
    ])
