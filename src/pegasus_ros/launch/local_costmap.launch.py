#!/usr/bin/env python3
"""
Pegasus 3D Local Costmap Launch
--------------------------------
Launches all three costmap nodes:
  1. lidar_costmap_layer   — VLP-16 → obstacle points
  2. zed_depth_costmap_layer — ZED X depth → obstacle points
  3. local_costmap_node    — fuses both into 3D voxel grid

Usage:
  ros2 launch pegasus_ros local_costmap.launch.py

  # Disable one sensor layer for testing:
  ros2 launch pegasus_ros local_costmap.launch.py enable_lidar:=false
  ros2 launch pegasus_ros local_costmap.launch.py enable_zed:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('pegasus_ros')
    config_file = os.path.join(pkg_share, 'config', 'local_costmap.yaml')

    # ── Launch arguments ──
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Enable LiDAR costmap layer')
    enable_zed_arg = DeclareLaunchArgument(
        'enable_zed', default_value='true',
        description='Enable ZED X depth costmap layer')

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_zed = LaunchConfiguration('enable_zed')

    # ── LiDAR Costmap Layer ──
    lidar_layer = Node(
        package='pegasus_ros',
        executable='lidar_costmap_layer_node',
        name='lidar_costmap_layer',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(enable_lidar),
        remappings=[
            # Remap if your LiDAR topic differs:
            # ('/velodyne_points', '/your/lidar/topic'),
        ],
    )

    # ── ZED X Depth Costmap Layer ──
    zed_layer = Node(
        package='pegasus_ros',
        executable='zed_depth_costmap_layer_node',
        name='zed_depth_costmap_layer',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(enable_zed),
        remappings=[
            # Remap if your ZED topic differs:
            # ('/zed_x/zed_node/depth/depth_registered', '/your/depth/topic'),
        ],
    )

    # ── 3D Local Costmap Fusion Node ──
    costmap_node = Node(
        package='pegasus_ros',
        executable='local_costmap_node',
        name='local_costmap_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_zed_arg,
        lidar_layer,
        zed_layer,
        costmap_node,
    ])