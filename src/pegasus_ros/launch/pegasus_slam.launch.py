#!/usr/bin/env python3
"""
Pegasus RTAB-Map SLAM Launch File
Multi-sensor SLAM: ZED X stereo + VLP-16 LiDAR + Pixhawk IMU

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
    
    pegasus_ros_dir = get_package_share_directory('pegasus_ros')
    
    # Arguments
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localization mode (true) or mapping mode (false)'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=PathJoinSubstitution([
            pegasus_ros_dir,
            'maps',
            'pegasus_disaster_map.db'
        ]),
        description='Path to RTAB-Map database'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    rtabmapviz_arg = DeclareLaunchArgument(
        'rtabmapviz',
        default_value='false',
        description='Launch RTAB-Map visualizer'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    rtabmapviz = LaunchConfiguration('rtabmapviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Frame IDs
    frame_id = 'base_link'
    odom_frame_id = 'odom'
    map_frame_id = 'map'
    
    # Topic Names
    left_image_topic = '/zed_x/zed_node/left/image_rect_color'
    right_image_topic = '/zed_x/zed_node/right/image_rect_color'
    left_camera_info_topic = '/zed_x/zed_node/left/camera_info'
    right_camera_info_topic = '/zed_x/zed_node/right/camera_info'
    scan_cloud_topic = '/velodyne_points'
    imu_topic = '/fmu/out/sensor_combined'  # From Pixhawk via XRCE-DDS
    
    # RTAB-Map Parameters
    rtabmap_parameters = {
        'frame_id': frame_id,
        'odom_frame_id': odom_frame_id,
        'map_frame_id': map_frame_id,
        'publish_tf': True,
        
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'subscribe_scan_cloud': True,
        
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'queue_size': 30,
        'wait_imu_to_init': True,
        
        'database_path': database_path,
        'Mem/IncrementalMemory': 'true',
        
        'Odom/Strategy': '0',
        'Odom/FeatureType': '6',
        'OdomF2M/MaxSize': '2000',
        'Odom/ImageDecimation': '1',
        'Stereo/MinDisparity': '1.0',
        'Stereo/MaxDisparity': '128.0',
        
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        
        'Optimizer/Strategy': '1',
        'Optimizer/Epsilon': '0.00001',
        'Optimizer/Iterations': '100',
        
        'Grid/Sensor': '1',
        'Grid/RangeMax': '30.0',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'true',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/RayTracing': 'true',
        
        'Kp/MaxFeatures': '500',
        'Vis/MaxFeatures': '1000',
        'Vis/MinInliers': '15',
        
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/PointToPlane': 'true',
    }
    
    # Visual Odometry Node
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': True,
            'approx_sync': True,
            'queue_size': 30,
            'use_sim_time': use_sim_time,
            
            'Odom/Strategy': '0',
            'Odom/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
        }],
        remappings=[
            ('left/image_rect', left_image_topic),
            ('right/image_rect', right_image_topic),
            ('left/camera_info', left_camera_info_topic),
            ('right/camera_info', right_camera_info_topic),
            ('imu', imu_topic),
        ],
        namespace='rtabmap',
    )
    
    # SLAM Node (Mapping)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=[
            ('left/image_rect', left_image_topic),
            ('right/image_rect', right_image_topic),
            ('left/camera_info', left_camera_info_topic),
            ('right/camera_info', right_camera_info_topic),
            ('scan_cloud', scan_cloud_topic),
            ('imu', imu_topic),
        ],
        arguments=['--delete_db_on_start'],
        namespace='rtabmap',
        condition=UnlessCondition(localization)
    )
    
    # SLAM Node (Localization)
    rtabmap_slam_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            **rtabmap_parameters,
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        }],
        remappings=[
            ('left/image_rect', left_image_topic),
            ('right/image_rect', right_image_topic),
            ('left/camera_info', left_camera_info_topic),
            ('right/camera_info', right_camera_info_topic),
            ('scan_cloud', scan_cloud_topic),
            ('imu', imu_topic),
        ],
        namespace='rtabmap',
        condition=IfCondition(localization)
    )
    
    # Point Cloud Assembler
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
    
    # RViz
    rviz_config = PathJoinSubstitution([
        pegasus_ros_dir,
        'config',
        'rviz_slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        
        localization_arg,
        database_path_arg,
        rviz_arg,
        rtabmapviz_arg,
        use_sim_time_arg,
        
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_slam_localization,
        point_cloud_assembler,
        rviz_node,
    ])
