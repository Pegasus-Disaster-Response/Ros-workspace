#!/usr/bin/env python3
"""
Pegasus RTAB-Map SLAM Launch File
Multi-sensor SLAM: ZED X (RGB-D) + VLP-16 LiDAR + Pixhawk IMU

Changes from v1.0:
  - Switched to RGB-D mode (uses ZED X neural depth instead of stereo matching)
  - Added ICP LiDAR odometry as primary odometry source
  - Parameters loaded from YAML configs instead of inline dicts
  - Added static TF publishers for all sensors
  - IMU now received from px4_imu_bridge_node (/pegasus/imu/data)
  - Added Mem/UseOdomGravity for gravity-aligned maps

Author: Team Pegasus
Date: 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Launch Arguments ─────────────────────────────────────
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='true=localization only, false=mapping+localization'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=PathJoinSubstitution([
            pegasus_share, 'maps', 'pegasus_disaster_map.db'
        ]),
        description='Path to RTAB-Map database'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Config file path ─────────────────────────────────────
    rtabmap_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rtabmap.yaml'
    ])

    # ── Topic Names ──────────────────────────────────────────
    # RGB-D topics from ZED X (using neural depth)
    rgb_image_topic = '/zed_x/zed_node/rgb/image_rect_color'
    rgb_camera_info_topic = '/zed_x/zed_node/rgb/camera_info'
    depth_image_topic = '/zed_x/zed_node/depth/depth_registered'
    scan_cloud_topic = '/velodyne_points'
    imu_topic = '/pegasus/imu/data'   # From px4_imu_bridge_node

    # ── Frame IDs ────────────────────────────────────────────
    frame_id = 'base_link'
    odom_frame_id = 'odom'
    map_frame_id = 'map'

    # ══════════════════════════════════════════════════════════
    # 1. ICP LiDAR Odometry (primary odometry source)
    # ══════════════════════════════════════════════════════════
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'frame_id': frame_id,
                'odom_frame_id': 'odom_lidar',  # Separate frame for fusion
                'publish_tf': False,  # Fusion node handles TF
                'use_sim_time': use_sim_time,
                'Icp/MaxTranslation': '1.0',
                'Odom/Strategy': '0',
            },
        ],
        remappings=[
            ('scan_cloud', scan_cloud_topic),
            ('odom', '/odom_lidar'),  # Publish to separate topic
            ('imu', imu_topic),
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 2. RGB-D Visual Odometry (backup odometry source)
    # ══════════════════════════════════════════════════════════
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'frame_id': frame_id,
                'odom_frame_id': 'odom_vision',  # Separate frame for fusion
                'publish_tf': False,  # Fusion node handles TF
                'use_sim_time': use_sim_time,
                'Odom/Strategy': '0',  # Frame-to-Map
                'Odom/FeatureType': '6',  # GFTT/BRIEF
                'Odom/ResetCountdown': '1',
                'Odom/ImageDecimation': '1',
                'OdomF2M/MaxSize': '2000',
            },
        ],
        remappings=[
            ('rgb/image', rgb_image_topic),
            ('rgb/camera_info', rgb_camera_info_topic),
            ('depth/image', depth_image_topic),
            ('odom', '/odom_vision'),  # Publish to separate topic
            ('imu', imu_topic),
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 3. Odometry Fusion/Selector (fault-tolerant switching)
    # ══════════════════════════════════════════════════════════
    odometry_selector = Node(
        package='pegasus_ros',
        executable='odometry_selector_node',
        name='odometry_selector',
        output='screen',
        parameters=[{
            'primary_odom_topic': '/odom_lidar',
            'backup_odom_topic': '/odom_vision',
            'timeout_threshold': 0.5,  # Switch if no data for 0.5s
            'use_sim_time': use_sim_time,
        }],
        # Publishes to /odom (what RTAB-Map expects)
    )

    # ══════════════════════════════════════════════════════════
    # 2. RTAB-Map SLAM Node (Mapping Mode)
    # ══════════════════════════════════════════════════════════
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'frame_id': frame_id,
                'odom_frame_id': odom_frame_id,
                'map_frame_id': map_frame_id,
                'publish_tf': True,
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
            },
        ],
        remappings=[
            # RGB-D from ZED X
            ('rgb/image', rgb_image_topic),
            ('rgb/camera_info', rgb_camera_info_topic),
            ('depth/image', depth_image_topic),
            # LiDAR
            ('scan_cloud', scan_cloud_topic),
            # IMU
            ('imu', imu_topic),
        ],
        arguments=['--delete_db_on_start'],
        namespace='rtabmap',
        condition=UnlessCondition(localization),
    )

    # ══════════════════════════════════════════════════════════
    # 3. RTAB-Map SLAM Node (Localization Mode)
    # ══════════════════════════════════════════════════════════
    rtabmap_slam_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {
                'frame_id': frame_id,
                'odom_frame_id': odom_frame_id,
                'map_frame_id': map_frame_id,
                'publish_tf': True,
                'use_sim_time': use_sim_time,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            },
        ],
        remappings=[
            ('rgb/image', rgb_image_topic),
            ('rgb/camera_info', rgb_camera_info_topic),
            ('depth/image', depth_image_topic),
            ('scan_cloud', scan_cloud_topic),
            ('imu', imu_topic),
        ],
        namespace='rtabmap',
        condition=IfCondition(localization),
    )

    # ══════════════════════════════════════════════════════════
    # 4. RTAB-Map Visualization
    # ══════════════════════════════════════════════════════════
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_config],
        remappings=[
            ('rgb/image', rgb_image_topic),
            ('rgb/camera_info', rgb_camera_info_topic),
            ('depth/image', depth_image_topic),
            ('scan_cloud', scan_cloud_topic),
            ('imu', imu_topic),
        ],
        condition=IfCondition(rviz),
    )

    # ══════════════════════════════════════════════════════════
    # 5. Point Cloud Assembler
    # ══════════════════════════════════════════════════════════
    point_cloud_assembler = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        output='screen',
        parameters=[{
            'assembling_time': 1.0,
            'fixed_frame_id': odom_frame_id,
            'voxel_size': 0.05,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud', scan_cloud_topic),
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 6. Static TF Publishers
    #    IMPORTANT: Update x/y/z offsets to match your actual
    #    sensor positions measured from base_link on the UAV.
    # ══════════════════════════════════════════════════════════

    # base_link → velodyne
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=[
            '0.52', '0.0', '0.85',     # x, y, z (meters) — UPDATE THESE
            '0', '0', '0',             # roll, pitch, yaw (radians)
            'base_link', 'velodyne'
        ],
    )

    # base_link → zed_x_camera_center
    tf_base_to_zed_x = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_x',
        arguments=[
            '1.78', '0.0', '0.55',      # x, y, z (meters) — UPDATE THESE
            '0', '0', '0',             # roll, pitch, yaw (radians)
            'base_link', 'zed_x_camera_center'
        ],
    )

    # base_link → imu_link (Pixhawk Cube Orange location)
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0.0', '0.0', '0.0',      # x, y, z (meters) — UPDATE THESE
            '0', '0', '0',             # roll, pitch, yaw (radians)
            'base_link', 'imu_link'
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 7. RViz
    # ══════════════════════════════════════════════════════════
    rviz_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rviz_slam.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Arguments
        localization_arg,
        database_path_arg,
        rviz_arg,
        use_sim_time_arg,

        # Static TFs (must publish before SLAM nodes start)
        tf_base_to_velodyne,
        tf_base_to_zed_x,
        tf_base_to_imu,

        # Odometry (dual sources with fusion)
        icp_odometry,
        rgbd_odometry,
        odometry_selector,

        # SLAM
        rtabmap_slam,
        rtabmap_slam_localization,

        # Utilities
        point_cloud_assembler,

        # Visualization
        rtabmap_viz,
        rviz_node,
    ])