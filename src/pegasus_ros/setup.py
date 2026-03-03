import os
from glob import glob
from setuptools import setup

package_name = 'pegasus_ros'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name.replace('pegasus_ros', 'pegasus_autonomy')],
    data_files=[
        # ── Package index (required by ament) ──
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ── Config files ──
        (os.path.join('share', package_name, 'config'), [
            'config/rtabmap.yaml',
            'config/vlp16.yaml',
            'config/zed_x.yaml',
            'config/rviz_slam.rviz',
            'config/local_costmap.yaml',            # 3D local costmap parameters
            'config/rviz_local_costmap.rviz',       # RViz config for costmap visualization
        ]),

        # ── Launch files ──
        (os.path.join('share', package_name, 'launch'), [
            'launch/pegasus_full.launch.py',
            'launch/pegasus_sensors.launch.py',
            'launch/pegasus_slam.launch.py',
            'launch/vtol1_gazebo_bridge_launch.py',
            'launch/local_costmap.launch.py',       # 3D local costmap launch
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
            'front_stereo_node = pegasus_autonomy.front_stereo_node:main',
            'px4_state_subscriber_node = pegasus_autonomy.px4_state_subscriber_node:main',
            'px4_imu_bridge_node = pegasus_autonomy.px4_imu_bridge_node:main',
            'odometry_selector_node = pegasus_autonomy.odometry_selector_node:main',

            # ── 3D Local costmap nodes ──
            'lidar_costmap_layer_node = pegasus_autonomy.lidar_costmap_layer_node:main',
            'zed_depth_costmap_layer_node = pegasus_autonomy.zed_depth_costmap_layer_node:main',
            'local_costmap_node = pegasus_autonomy.local_costmap_node:main',
        ],
    },
)