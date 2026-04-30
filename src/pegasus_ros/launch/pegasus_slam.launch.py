#!/usr/bin/env python3
"""
Pegasus RTAB-Map SLAM Launch File
LiDAR-only SLAM: VLP-16 + Pixhawk IMU. ZED X kept running in the
sensors launch for visualization only — not consumed by SLAM.

v3.0 changes (LiDAR-only):
  - rgbd_odometry node removed. Only icp_odometry feeds the selector.
  - rgb/depth remappings stripped from rtabmap mapping/localization
    nodes. rtabmap_viz keeps them so the GUI can still show ZED feed
    alongside the LiDAR map.
  - Loop closure switches to ICP-based geometric proximity
    (RGBD/ProximityBySpace=true, Reg/Strategy=1) — see rtabmap.yaml.
  - IMU still feeds gravity into icp_odometry and into the rtabmap
    pose graph (Optimizer/GravitySigma).

v2.6.1 changes:
  - Fix: Static TF for ZED X now publishes to 'zed_x_camera_center'
    (matching zed_x.yaml camera_frame) instead of 'camera_link' which
    caused silent TF lookup failures for the costmap and RTAB-Map.

v2.6.2 changes:
  - CRITICAL FIX: Removed '--delete_db_on_start' argument from rtabmap_slam
    and rtabmap_slam_localization nodes. This flag was causing RTAB-Map to
    wipe the database file on every single launch — meaning no map was ever
    persisted. The database was being written and immediately deleted on the
    next run. Remove this argument; to intentionally reset use the ROS 2
    service call: ros2 service call /rtabmap/reset std_srvs/srv/Empty
  - CRITICAL FIX: database_path default now resolves to
    ~/Ros-workspace/maps/pegasus_disaster_map.db using os.path.expanduser
    so the file ends up in a predictable, workspace-relative location.
    Previously PathJoinSubstitution([pegasus_share, 'maps', ...]) pointed
    into the install/ tree which (a) may not exist and (b) is wiped on
    colcon builds.
  - FIX: SetParameter('use_sim_time') moved to after all
    DeclareLaunchArgument entries in the LaunchDescription list to match
    the required ordering (args must be declared before SetParameter
    consumes their values).
  - FIX: database_path exposed as a forwarded argument so pegasus_full.launch.py
    can pass it through without having to redeclare it.

Author: Team Pegasus
Date: 2026
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Launch Arguments ─────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='true=localization only, false=mapping+localization'
    )

    # FIX v2.6.2: Default path is now in ~/Ros-workspace/maps/ which is
    # persistent across builds and lives where you can actually find it.
    # Run once: mkdir -p ~/Ros-workspace/maps
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser(
            '~/Ros-workspace/maps/pegasus_disaster_map.db'
        ),
        description='Path to RTAB-Map database (persistent across builds)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Allow SITL to override the primary odometry source. ICP on a
    # sparse 360-sample lidar drifts ~10 m on z; Gazebo's ground-truth
    # odometry (/model/p110_v2_0/odometry_with_covariance) is exact and
    # makes /odom usable for visualisation and the autonomy fallback.
    primary_odom_arg = DeclareLaunchArgument(
        'primary_odom_topic',
        default_value='/odom_lidar',
        description='Primary odometry topic for the selector (override to '
                    'Gazebo ground truth in SITL).'
    )

    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    primary_odom_topic = LaunchConfiguration('primary_odom_topic')

    # ── Config file paths ────────────────────────────────────
    # Fix #3: Each node type gets its own config with params under
    # the matching YAML key (node name).
    rtabmap_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rtabmap.yaml'
    ])
    icp_odom_config = PathJoinSubstitution([
        pegasus_share, 'config', 'icp_odometry.yaml'
    ])

    # ── Topic Names ──────────────────────────────────────────
    # FIX v2.7: Topic names updated to match ZED ROS 2 wrapper v5.1.
    # The wrapper changed from /rgb/image_rect_color to /rgb/color/rect/image
    # and from /rgb/camera_info to /rgb/color/rect/camera_info.
    rgb_image_topic = '/zed_x/zed_node/rgb/color/rect/image'
    rgb_camera_info_topic = '/zed_x/zed_node/rgb/color/rect/camera_info'
    depth_image_topic = '/zed_x/zed_node/depth/depth_registered'
    scan_cloud_topic = '/velodyne_points'
    imu_topic = '/pegasus/imu/data'

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
            icp_odom_config,            # Fix #3: dedicated config
            {
                'frame_id': frame_id,
                'odom_frame_id': 'odom_lidar',
                'publish_tf': False,    # Selector node handles TF
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('scan_cloud', scan_cloud_topic),
            ('odom', '/odom_lidar'),
            ('imu', imu_topic),
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 2. Odometry Fusion/Selector (fault-tolerant switching)
    #    Backup vision odom is disabled in lidar-only mode; the
    #    selector still gates NaN and publishes /odom + TF.
    # ══════════════════════════════════════════════════════════
    odometry_selector = Node(
        package='pegasus_ros',
        executable='odometry_selector_node',
        name='odometry_selector',
        output='screen',
        parameters=[{
            'primary_odom_topic': primary_odom_topic,
            'backup_odom_topic': '/odom_vision',
            'timeout_threshold': 0.5,
            'use_sim_time': use_sim_time,
        }],
    )

    # ══════════════════════════════════════════════════════════
    # 4. RTAB-Map SLAM Node (Mapping Mode)
    #
    #  FIX v2.6.2: '--delete_db_on_start' has been REMOVED.
    #  That argument caused RTAB-Map to delete the database file at
    #  startup, so no map was ever persisted between runs. To
    #  intentionally clear the map (e.g. before a new mapping mission),
    #  either delete the file manually or call the reset service:
    #    ros2 service call /rtabmap/reset std_srvs/srv/Empty
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
            ('odom', '/odom'),              # odometry_selector publishes to /odom;
                                            # namespace='rtabmap' would otherwise
                                            # rewrite this to /rtabmap/odom.
            ('scan_cloud', scan_cloud_topic),
        ],
        # NO arguments=['--delete_db_on_start'] here
        namespace='rtabmap',
        condition=UnlessCondition(localization),
    )

    # ══════════════════════════════════════════════════════════
    # 5. RTAB-Map SLAM Node (Localization Mode)
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
            ('odom', '/odom'),
            ('scan_cloud', scan_cloud_topic),
        ],
        # NO arguments=['--delete_db_on_start'] here either
        namespace='rtabmap',
        condition=IfCondition(localization),
    )

    # ══════════════════════════════════════════════════════════
    # 6. RTAB-Map Visualization
    # ══════════════════════════════════════════════════════════
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        namespace='rtabmap',           # Must match rtabmap node namespace so the
                                       # viz subscribes to /rtabmap/info and
                                       # /rtabmap/mapData, not /info and /mapData.
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
    # 7. Point Cloud Assembler
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
    # 8. Static TF Publishers
    #    IMPORTANT: Update x/y/z offsets to match your actual
    #    sensor positions measured from base_link on the UAV.
    # ══════════════════════════════════════════════════════════
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=[
            '0.0', '0.0', '0.432',
            '0', '0', '0',
            'base_link', 'velodyne'
        ],
    )

    # Target frame is zed_x_camera_link (URDF root published by ZED wrapper's
    # robot_state_publisher). Connects base_link to the full ZED TF chain:
    #   zed_x_camera_link -> zed_x_camera_center -> zed_x_left_camera_frame
    #                     -> zed_x_left_camera_frame_optical (etc.)
    tf_base_to_zed_x = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_x',
        arguments=[
            '0.470', '0.0', '0.051',
            '0', '0', '0',
            'base_link', 'zed_x_camera_link'  # URDF root frame from ZED wrapper
        ],
    )

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0.0', '0.0', '0.0',
            '0', '0', '0',
            'base_link', 'imu_link'
        ],
    )

    # ══════════════════════════════════════════════════════════
    # 9. RViz
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
        # FIX v2.6.2: All DeclareLaunchArgument entries come FIRST,
        # then SetParameter. Previously SetParameter appeared between
        # args in the list, which can cause evaluation-order issues
        # depending on the launch framework version.

        # Arguments (must all be declared before SetParameter)
        use_sim_time_arg,
        localization_arg,
        database_path_arg,
        rviz_arg,
        primary_odom_arg,

        # Global parameter (after all args are declared)
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Static TFs (must publish before SLAM nodes start)
        tf_base_to_velodyne,
        tf_base_to_zed_x,
        tf_base_to_imu,

        # Odometry (LiDAR-only — IMU still feeds gravity into ICP)
        icp_odometry,
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