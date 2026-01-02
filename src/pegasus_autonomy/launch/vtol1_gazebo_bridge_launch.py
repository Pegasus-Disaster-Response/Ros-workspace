from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge for clock (needed for RViz time sync)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
        
        # Bridge for VLP-16 LiDAR
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=[
                '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),
        
        # Bridge for Camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),
        
        # Bridge for Camera Info (if needed)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[
                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen'
        ),
        
        # Optional: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '~/ros2_ws/src/vtol1_bringup/config/vtol1.rviz'],
            output='screen'
        ),

        # Bridge for TF (transforms)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            arguments=[
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            output='screen'
        ),
        
        # Static TF publisher for fixed frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # Velodyne is 0.15m above base_link (from your SDF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_velodyne',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'velodyne'],
            output='screen'
        ),
    ])