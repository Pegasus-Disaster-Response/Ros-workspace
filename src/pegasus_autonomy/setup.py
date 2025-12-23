from setuptools import find_packages, setup

package_name = 'pegasus_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pegasus_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changwe',
    maintainer_email='changwe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'front_stereo_node = pegasus_autonomy.front_stereo_node:main',
            'bottom_stereo_node = pegasus_autonomy.bottom_stereo_node:main',
            'lidar_node = pegasus_autonomy.lidar_node:main',
            'side_camera_logger_node = pegasus_autonomy.side_camera_logger_node:main',
            'px4_state_subscriber_node = pegasus_autonomy.px4_state_subscriber_node:main',
            'mission_planner_node = pegasus_autonomy.mission_planner_node:main',
        ],
    },
)
