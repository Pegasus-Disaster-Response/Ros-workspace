#!/usr/bin/env python3
"""
Pegasus Gazebo Harmonic Bridge Launch (VTOL1 + VLP-16 + RGBD Camera)
=====================================================================
Bridges Gazebo Harmonic topics into ROS 2 for SITL simulation.

Defaults to vtol1 model in baylands world. Override with launch args.

Usage:
  ros2 launch pegasus_ros p110_gazebo_bridge_launch.py
  ros2 launch pegasus_ros p110_gazebo_bridge_launch.py gz_world:=walls
  ros2 launch pegasus_ros p110_gazebo_bridge_launch.py gz_model:=p110_v1_0 gz_world:=default
  ros2 launch pegasus_ros p110_gazebo_bridge_launch.py rviz:=true

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_camera_bridges(context):
    """Build camera bridge nodes with resolved world/model names."""
    world = context.launch_configurations['gz_world']
    model = context.launch_configurations['gz_model']
    prefix = f'/world/{world}/model/{model}/link/camera_link/sensor/camera'

    image_topic = f'{prefix}/image'
    info_topic = f'{prefix}/camera_info'
    depth_topic = f'{prefix}/depth_image'

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            f'{image_topic}@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            (image_topic, '/zed_x/zed_node/rgb/color/rect/image'),
        ],
        output='screen',
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            f'{info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            (info_topic, '/zed_x/zed_node/rgb/color/rect/camera_info'),
        ],
        output='screen',
    )

    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_bridge',
        arguments=[
            f'{depth_topic}@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            (depth_topic, '/zed_x/zed_node/depth/depth_registered'),
        ],
        output='screen',
    )

    return [camera_bridge, camera_info_bridge, depth_bridge]


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ──────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for visualization')

    gz_world_arg = DeclareLaunchArgument(
        'gz_world', default_value='p110_world',
        description='Gazebo world name (must match PX4_GZ_WORLD)')

    gz_model_arg = DeclareLaunchArgument(
        'gz_model', default_value='p110_v2_0',
        description='Gazebo model instance name (PX4 appends _0)')

    # ── 1. Clock Bridge ───────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── 2. VLP-16 LiDAR Bridge ────────────────────────────
    #    LiDAR uses explicit <topic> in SDF, no world prefix.
    #    Gazebo: /velodyne_points/points → ROS: /velodyne_points
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/velodyne_points/points', '/velodyne_points'),
        ],
        output='screen',
    )

    # ── 3-5. Camera Bridges (image, info, depth) ─────────
    camera_bridges = OpaqueFunction(function=launch_camera_bridges)

    # ── 6. Odometry Bridge ────────────────────────────────
    #    Ground-truth odometry from Gazebo for initial testing.
    #    Once SLAM is running, odometry_selector uses /odom_lidar.
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/model/p110_v2_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance'
        ],
        output='screen',
    )

    # ── 7. IMU Bridge ─────────────────────────────────────
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/imu', '/pegasus/imu/data'),
        ],
        output='screen',
    )

    # ── 8. Static TF Publishers ───────────────────────────
    tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # NOTE: odom→base_link is NOT published here.
    # The odometry_selector_node in the SLAM launch owns that transform.
    # Publishing it here would conflict and cause NaN TF errors.

    # Velodyne: 0.15m above base_link (from vtol1 SDF)
    tf_base_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'velodyne'],
        output='screen',
    )

    # Camera: at (0.12, 0.03, 0.242) from base_link (from vtol1 SDF include pose)
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen',
    )

    # ── 9. RViz (Optional) ────────────────────────────────
    rviz_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rviz_slam.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        gz_world_arg,
        gz_model_arg,

        # Bridges
        clock_bridge,
        lidar_bridge,
        camera_bridges,
        odom_bridge,
        imu_bridge,

        # TF tree
        tf_world_map,
        tf_map_odom,
        tf_base_velodyne,
        tf_base_camera,

        # Visualization
        rviz,
    ])