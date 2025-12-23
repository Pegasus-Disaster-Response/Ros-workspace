#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Front stereo camera node
        Node(
            package='pegasus_autonomy',
            executable='front_stereo_node',
            name='front_stereo_node',
            output='screen',
            parameters=[],
        ),
        
        # Bottom stereo camera node
        Node(
            package='pegasus_autonomy',
            executable='bottom_stereo_node',
            name='bottom_stereo_node',
            output='screen',
            parameters=[],
        ),
        
        # 3D LiDAR node
        Node(
            package='pegasus_autonomy',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[],
        ),
        
        # Side camera logger node (recording only)
        Node(
            package='pegasus_autonomy',
            executable='side_camera_logger_node',
            name='side_camera_logger_node',
            output='screen',
            parameters=[],
        ),
        
        # PX4 state subscriber node
        Node(
            package='pegasus_autonomy',
            executable='px4_state_subscriber_node',
            name='px4_state_subscriber_node',
            output='screen',
            parameters=[],
        ),
        
        # Mission planner node (downstream autonomy)
        Node(
            package='pegasus_autonomy',
            executable='mission_planner_node',
            name='mission_planner_node',
            output='screen',
            parameters=[],
        ),
    ])