import os
from glob import glob
from setuptools import setup

package_name = 'pegasus_ros'

setup(
    name=package_name,
    version='2.6.0',
    # Fix #1: Explicit package + package_dir mapping so setuptools reliably
    # finds the pegasus_autonomy module inside the pegasus_ros source tree.
    # Previously used: packages=[package_name.replace('pegasus_ros', 'pegasus_autonomy')]
    # which is fragile and omits the package_dir hint.
    packages=['pegasus_autonomy'],
    #package_dir={'pegasus_autonomy': 'pegasus_autonomy'},
    data_files=[
        # ── Package index (required by ament) ──
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ── Config files ──
        (os.path.join('share', package_name, 'config'), [
            'config/rtabmap.yaml',
            'config/icp_odometry.yaml',             # NEW (Fix #3)
            'config/rgbd_odometry.yaml',            # NEW (Fix #3)
            'config/vlp16.yaml',
            'config/zed_x.yaml',
            'config/rviz_slam.rviz',
            'config/local_costmap.yaml',
            'config/rviz_local_costmap.rviz',
            'config/path_planner.yaml',
            'config/rviz_planner_test.rviz',
            'config/dstar_lite.yaml',
            'config/vtol_dynamics.yaml',
            'config/px4_offboard.yaml',
        ]),

        # ── Launch files ──
        (os.path.join('share', package_name, 'launch'), [
            'launch/pegasus_full.launch.py',
            'launch/pegasus_sensors.launch.py',
            'launch/pegasus_slam.launch.py',
            'launch/local_costmap.launch.py',
            'launch/path_planner.launch.py',
            'launch/p110_gazebo_bridge_launch.py',
            'launch/sitl_full.launch.py',
        ]),

        # ── World files (Gazebo SDF) ──
        (os.path.join('share', package_name, 'worlds'), [
            'worlds/pegasus_planning_test.sdf',
        ]),

        # ── Map files ──
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Pegasus - Cal Poly Pomona',
    maintainer_email='cbmusonda@cpp.edu',
    description='Pegasus Disaster Response UAV Autonomy Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Core autonomy nodes ──
            'mission_planner_node = pegasus_autonomy.mission_planner_node:main',
            'px4_imu_bridge_node = pegasus_autonomy.px4_imu_bridge_node:main',
            'px4_offboard_node = pegasus_autonomy.px4_offboard_node:main',
            'odometry_selector_node = pegasus_autonomy.odometry_selector_node:main',

            # ── 3D Local costmap nodes ──
            'lidar_costmap_layer_node = pegasus_autonomy.lidar_costmap_layer_node:main',
            'zed_depth_costmap_layer_node = pegasus_autonomy.zed_depth_costmap_layer_node:main',
            'local_costmap_node = pegasus_autonomy.local_costmap_node:main',

            # ── Path planning nodes ──
            'global_planner_node = pegasus_autonomy.global_planner_node:main',
            'dstar_lite_node = pegasus_autonomy.dstar_lite_node:main',
            'mpc_trajectory_node = pegasus_autonomy.mpc_trajectory_node:main',

            # ── SIL test utility nodes ──
            'static_map_publisher_node = pegasus_autonomy.static_map_publisher_node:main',
            'static_odom_publisher_node = pegasus_autonomy.static_odom_publisher_node:main',
        ],
    },
)