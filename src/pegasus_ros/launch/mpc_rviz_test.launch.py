"""
MPC RViz closed-loop test.

Stack:
  mpc_sim_path_publisher  →  /mpc_sim/reference_path
  mpc_sim_vehicle         ←  /mpc_sim/cmd_vel   (integrates to odom + TF)
                          →  /mpc_sim/odom
  mpc_trajectory_node     ←  /mpc_sim/reference_path + /mpc_sim/odom
                          →  /mpc_sim/cmd_vel
  rviz2                   ←  all viz topics

Run:
  ros2 launch pegasus_ros mpc_rviz_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('pegasus_ros')
    rviz_cfg = os.path.join(pkg, 'config', 'rviz_mpc_test.rviz')

    mpc_node = Node(
        package='pegasus_ros',
        executable='mpc_trajectory_node',
        name='mpc_trajectory_node',
        output='screen',
        parameters=[{
            'path_topic': '/mpc_sim/reference_path',
            'odom_topic': '/mpc_sim/odom',
            'cmd_topic': '/mpc_sim/cmd_vel',
            'status_topic': '/mpc_sim/status',
            'control_rate_hz': 20.0,
            'dt': 0.5,
            'horizon_steps': 20,
            'lookahead_points': 10,
            'min_speed_m_s': 12.0,
            'max_speed_m_s': 18.01,
            'max_yaw_rate_rad_s': 0.40,
            'max_vz_m_s': 10.80,
            'arrival_tolerance_m': 15.0,
            'global_reacquire_distance_m': 50.0,
            'loop_path': True,         # loop the circle continuously
            'near_target_slowdown_distance_m': 60.0,
            'min_speed_near_target_m_s': 12.0,
        }],
    )

    path_publisher = Node(
        package='pegasus_ros',
        executable='mpc_sim_path_publisher',
        name='mpc_sim_path_publisher',
        output='screen',
        parameters=[{
            'path_topic': '/mpc_sim/reference_path',
            'publish_rate_hz': 1.0,
            'path_shape': 'figure8',
            'scale_m': 150.0,        # min turn radius = 75 m >> 45 m P110 limit
            'altitude_m': 50.0,
            'waypoint_spacing_m': 10.0,
            'center_x': 0.0,
            'center_y': 0.0,
        }],
    )

    vehicle_sim = Node(
        package='pegasus_ros',
        executable='mpc_sim_vehicle',
        name='mpc_sim_vehicle',
        output='screen',
        parameters=[{
            'cmd_topic': '/mpc_sim/cmd_vel',
            'odom_topic': '/mpc_sim/odom',
            'sim_rate_hz': 50.0,
            'cmd_timeout_s': 0.5,
            # Rightmost tip of lemniscate: x = scale * sqrt(2) ≈ 212 m, y = 0
            # Tangent at that point is +y direction → yaw = 90°
            'initial_x': 212.0,
            'initial_y': 0.0,
            'initial_z': 50.0,
            'initial_yaw_deg': 90.0,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([path_publisher, vehicle_sim, mpc_node, rviz])
