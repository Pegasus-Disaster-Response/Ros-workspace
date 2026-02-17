from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pegasus_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Maps directory
        (os.path.join('share', package_name, 'maps'), ['maps/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Pegasus - Cal Poly Pomona',
    maintainer_email='changwe@calpoly.edu',
    description='Pegasus Disaster Response UAV Autonomy Package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_planner_node = pegasus_autonomy.mission_planner_node:main',
            'front_stereo_node = pegasus_autonomy.front_stereo_node:main',
            'px4_state_subscriber_node = pegasus_autonomy.px4_state_subscriber_node:main',
        ],
    },
    # Force scripts to install in the correct location
    options={
        'install': {
            'install_scripts': 'lib/' + package_name,
        },
    },
)
import subprocess
import sys

if 'install' in sys.argv:
    install_base = os.path.join(os.path.dirname(__file__), '..', '..', 'install', package_name, 'lib')
    if os.path.exists(install_base):
        src = os.path.join(install_base, 'pegasus_autonomy')
        dst = os.path.join(install_base, package_name)
        if os.path.exists(src) and not os.path.exists(dst):
            try:
                os.symlink('pegasus_autonomy', dst)
                print(f"Created symlink: {dst} -> pegasus_autonomy")
            except:
                pass
