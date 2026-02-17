#!/usr/bin/env python3
"""
Pegasus Full System Launch
Complete system: Sensors + SLAM + Mission Planning
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from Gazebo'
    )
    
    launch_gazebo_bridge_arg = DeclareLaunchArgument(
        'launch_gazebo_bridge',
        default_value='false',
        description='Launch Gazebo bridge for simulation'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localization mode (vs mapping)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo_bridge = LaunchConfiguration('launch_gazebo_bridge')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    
    # Gazebo Bridge (Optional)
    gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pegasus_ros'),
                'launch',
                'vtol1_gazebo_bridge_launch.py'
            ])
        ]),
        condition=IfCondition(launch_gazebo_bridge)
    )
    
    # Sensor Drivers
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pegasus_ros'),
                'launch',
                'pegasus_sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # RTAB-Map SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pegasus_ros'),
                'launch',
                'pegasus_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rviz': rviz,
        }.items()
    )
    
    # Mission Planning Node
    mission_planner = Node(
        package='pegasus_ros',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # PX4 State Subscriber (Optional)
    px4_state_subscriber = Node(
        package='pegasus_ros',
        executable='px4_state_subscriber_node',
        name='px4_state_subscriber_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_gazebo_bridge_arg,
        localization_arg,
        rviz_arg,
        
        # Launch components
        gazebo_bridge,
        sensors,
        slam,
        mission_planner,
        px4_state_subscriber,
    ])
