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


def launch_sensor_bridges(context):
    """Build camera, lidar, and IMU bridge nodes with resolved world/model
    names. Gazebo Harmonic publishes sensor data at
    /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<datatype>
    when the SDF does not specify an explicit <topic>; we subscribe to those
    fully-qualified paths and remap to the canonical Pegasus topics."""
    world = context.launch_configurations['gz_world']
    model = context.launch_configurations['gz_model']

    # Camera (RGB + depth share the same camera_link sensor block)
    camera_prefix = f'/world/{world}/model/{model}/link/camera_link/sensor/camera'
    image_topic = f'{camera_prefix}/image'
    info_topic  = f'{camera_prefix}/camera_info'
    depth_topic = f'{camera_prefix}/depth_image'

    # The p110_v2 SDF sets <topic>velodyne_points/points</topic> on the
    # lidar sensor, so the gz topic is flat (no world/model prefix).
    # Confirmed via `gz topic -i -t /velodyne_points/points` — publisher
    # type is gz.msgs.PointCloudPacked.
    lidar_topic = '/velodyne_points/points'

    # IMU has no <topic> override in the SDF, so it uses the
    # Harmonic-default fully-qualified path. Confirmed via
    # `gz topic -i -t /world/.../base_link/sensor/imu_sensor/imu`.
    imu_topic = (
        f'/world/{world}/model/{model}'
        f'/link/base_link/sensor/imu_sensor/imu'
    )

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

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            f'{lidar_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            (lidar_topic, '/velodyne_points'),
        ],
        output='screen',
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            f'{imu_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            (imu_topic, '/pegasus/imu/data'),
        ],
        output='screen',
    )

    return [camera_bridge, camera_info_bridge, depth_bridge,
            lidar_bridge, imu_bridge]


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

    # ── 2-6. Camera + LiDAR + IMU Bridges ────────────────
    #    All sensor bridges are built inside an OpaqueFunction so the
    #    gz_world / gz_model substitutions can be resolved into the
    #    fully-qualified Gazebo Harmonic topic paths
    #    /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<data>.
    sensor_bridges = OpaqueFunction(function=launch_sensor_bridges)

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

    # ── 8. Static TF Publishers ───────────────────────────
    tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    # NOTE: map→odom is NOT published here. RTAB-Map publishes this
    # dynamically as it localises. A static identity here would fight
    # the SLAM output and pin the vehicle to the map origin.

    # NOTE: base_link→velodyne is NOT published here. pegasus_slam.launch.py
    # owns this TF with the correct pose from the P110 V2 SDF.
    # Publishing it here would conflict and cause incorrect LiDAR projection.

    # NOTE: odom→base_link is NOT published here.
    # The odometry_selector_node in the SLAM launch owns that transform.

    # Camera: at (-0.7, 0, 0) from base_link — CameraJoint pose in p110_v2/model.sdf.
    # Publishes as 'camera_link' to match the Gazebo sensor link name so that the
    # frame_id in bridged Image/CameraInfo messages is resolvable in the TF tree.
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['-0.7', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen',
    )

    # Identity bridge: Gazebo images carry frame_id='camera_link'; RTAB-Map's
    # URDF chain (real ZED wrapper) roots at 'zed_x_camera_link'. This zero-offset
    # TF makes both names resolve to the same physical location so SLAM works
    # in simulation without modifying the SLAM launch file.
    tf_camera_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_zed_x_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'zed_x_camera_link'],
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
        sensor_bridges,
        odom_bridge,

        # TF tree
        tf_world_map,
        tf_base_camera,
        tf_camera_to_zed,

        # Visualization
        rviz,
    ])