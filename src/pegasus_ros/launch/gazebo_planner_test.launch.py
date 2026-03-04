#!/usr/bin/env python3
"""
Pegasus A* Planner — Gazebo SIL Test Launch
=============================================
Minimal launch for testing the global planner against a static map.
No SLAM, no sensors, no PX4 — just A* on a known grid.

What this launches:
  1. ros_gz_bridge     — clock from Gazebo (only with use_gazebo:=true)
  2. static_map_publisher — loads PGM+YAML ground-truth map
  3. global_planner_node  — A* planner
  4. static TF tree       — map → odom → base_link (identity)
  5. RViz2               — visualization

Usage (2D RViz only — no Gazebo needed):
    ros2 launch pegasus_ros gazebo_planner_test.launch.py

Usage (with Gazebo 3D):
    gz sim ~/Ros-workspace/src/pegasus_ros/worlds/pegasus_planning_test.sdf
    ros2 launch pegasus_ros gazebo_planner_test.launch.py use_gazebo:=true use_sim_time:=true

Send a goal:
    ros2 topic pub --once /pegasus/autonomy/target_waypoint \
      geometry_msgs/PoseStamped \
      "{header: {frame_id: 'map'}, pose: {position: {x: 35.0, y: 25.0, z: 10.0}}}"

    Or use RViz "2D Goal Pose" button (remapped to planner topic)

Author: Team Pegasus — Cal Poly Pomona
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pegasus_share = FindPackageShare('pegasus_ros')

    # ── Arguments ──────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use Gazebo simulation clock (set true with Gazebo)')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value=PathJoinSubstitution([
            pegasus_share, 'maps', 'planning_test_map.yaml']),
        description='Path to map YAML file')

    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo', default_value='false',
        description='Launch ros_gz_bridge for Gazebo connection')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # ── 1. ros_gz_bridge: clock (Gazebo only) ────────────
    #    Bridges /clock from Gazebo so use_sim_time works.
    #    Only launched when use_gazebo:=true.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # ── 2. Static Map Publisher ───────────────────────────
    #    Publishes ground-truth occupancy grid from PGM file.
    static_map = Node(
        package='pegasus_ros',
        executable='static_map_publisher_node',
        name='static_map_publisher_node',
        output='screen',
        parameters=[{
            'map_yaml_path': map_yaml,
            'publish_topic': '/rtabmap/grid_map',
            'publish_rate_hz': 1.0,
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 3. Global Planner (A*) ────────────────────────────
    planner_config = PathJoinSubstitution([
        pegasus_share, 'config', 'path_planner.yaml'])

    global_planner = Node(
        package='pegasus_ros',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            planner_config,
            {
                'use_sim_time': use_sim_time,
                # Use SLAM grid topic (where static map publishes)
                'prefer_local_costmap': False,
            },
        ],
    )

    # ── 4. Static Odometry Publisher ──────────────────────
    #    For initial testing, the UAV "starts" at the origin.
    #    Publishes a static Odometry message so the planner
    #    knows where the UAV is. Replace this with the Gazebo
    #    model pose bridge when testing with a moving vehicle.
    static_odom = Node(
        package='pegasus_ros',
        executable='static_odom_publisher_node',
        name='static_odom_publisher_node',
        output='screen',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'start_z': 10.0,
            'publish_rate_hz': 10.0,
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 5. Static TF: map → odom → base_link ─────────────
    #    Identity transforms since there's no SLAM drift
    #    in a static-map test.
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── 6. RViz2 ─────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        pegasus_share, 'config', 'rviz_planner_test.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        use_gazebo_arg,
        clock_bridge,
        static_map,
        global_planner,
        static_odom,
        tf_map_odom,
        rviz,
    ])