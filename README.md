# Pegasus Disaster Response UAV - Autonomous Navigation System

**Version:** 1.0.0  
**Institution:** California Polytechnic State University, Pomona  
**Team:** Pegasus  
**Sponsor:** Lockheed Martin  
**Project Type:** eVTOL UAV for Disaster Response Applications

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [ROS Topics Reference](#ros-topics-reference)
- [Package Structure](#package-structure)
- [Development Guide](#development-guide)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The Pegasus autonomous navigation system provides complete SLAM-based localization and mission planning capabilities for disaster response operations. The system integrates multi-sensor fusion using stereo vision, LiDAR, and IMU data to enable autonomous navigation in GPS-denied environments.

### Current Capabilities

- Real-time SLAM and 3D mapping using RTAB-Map
- Multi-sensor fusion (ZED X stereo camera, Velodyne VLP-16 LiDAR, Pixhawk IMU)
- Low-latency communication with PX4 flight controller via XRCE-DDS
- Mission planning framework for disaster response scenarios
- 2D occupancy grid generation for path planning
- Modular architecture supporting independent component testing

### Planned Features

- Global path planning using A* and RRT algorithms
- Real-time obstacle avoidance
- Computer vision-based survivor detection
- Fire and smoke detection algorithms
- Autonomous search pattern execution

---

## System Architecture

### Component Overview
```
┌─────────────────────────────────────────────────────────────┐
│                    Pegasus ROS System                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐   │
│  │   Sensors    │  │     SLAM     │  │ Mission Planner │   │
│  │              │  │              │  │                 │   │
│  │ - ZED X      │→→│  RTAB-Map    │→→│  High-level     │   │
│  │ - VLP-16     │  │              │  │  Decision       │   │
│  │ - Pixhawk    │  │  - Visual    │  │  Making         │   │
│  │   (XRCE-DDS) │  │    Odometry  │  │                 │   │
│  │              │  │  - Mapping   │  │                 │   │
│  └──────────────┘  └──────────────┘  └─────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Sensor Layer**: Raw data acquisition from ZED X camera, VLP-16 LiDAR, and Pixhawk IMU
2. **Processing Layer**: RTAB-Map performs visual odometry, loop closure detection, and map building
3. **Decision Layer**: Mission planner processes map data and sensor inputs for autonomous navigation

---

## Hardware Requirements

### Flight Controller
- **Model**: Pixhawk Cube Orange
- **Firmware**: PX4 (latest stable)
- **Interface**: UART (TELEM2) or USB
- **Baudrate**: 921600

### Sensors

#### ZED X Stereo Camera
- **Interface**: GMSL2 (requires capture card)
- **Resolution**: 1920x1080 @ 30 FPS
- **Features**: Neural depth engine, built-in IMU (not used)

#### Velodyne VLP-16 LiDAR
- **Interface**: Ethernet
- **Default IP**: 192.168.1.201
- **Range**: 100 meters
- **Rotation Rate**: 5-20 Hz (configured to 10 Hz)

### Compute Platform
- **Recommended**: NVIDIA Jetson Orin AGX
- **Minimum**: Jetson Xavier NX
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Version**: ROS 2 Humble Hawksbill

---

## Software Dependencies

### Core Dependencies
```bash
# ROS 2 Humble
ros-humble-desktop

# RTAB-Map SLAM
ros-humble-rtabmap-ros

# Sensor Drivers
zed-ros2-wrapper (from Stereolabs)
ros-humble-velodyne

# PX4 Communication
micro-xrce-dds-agent
px4_msgs
```

### Build Tools
```bash
python3-colcon-common-extensions
python3-rosdep
```

---

## Installation

### Prerequisites

Ensure ROS 2 Humble is installed and sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Step 1: Install System Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-rtabmap-ros \
    ros-humble-velodyne \
    python3-colcon-common-extensions \
    git
```

### Step 2: Install micro-XRCE-DDS Agent
```bash
sudo snap install micro-xrce-dds-agent --edge
```

### Step 3: Create and Build px4_msgs Workspace

To optimize build times, px4_msgs is maintained in a separate workspace:
```bash
# Create workspace
mkdir -p ~/px4_ws/src
cd ~/px4_ws/src

# Clone px4_msgs
git clone https://github.com/PX4/px4_msgs.git

# Build
cd ~/px4_ws
colcon build --symlink-install

# Source
echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc
source ~/px4_ws/install/setup.bash
```

### Step 4: Clone and Build Pegasus Workspace
```bash
# Clone repository
cd ~
git clone https://github.com/Pegasus-Disaster-Response/Ros-workspace.git
cd Ros-workspace

# Build
colcon build --symlink-install --packages-select pegasus_ros

# Create symlink for executables
cd install/pegasus_ros/lib
ln -s pegasus_autonomy pegasus_ros

# Source workspace
echo "source ~/Ros-workspace/install/setup.bash" >> ~/.bashrc
source ~/Ros-workspace/install/setup.bash
```

### Step 5: Verify Installation
```bash
# Check package is available
ros2 pkg list | grep pegasus_ros

# Check executables
ros2 pkg executables pegasus_ros

# Expected output:
# pegasus_ros front_stereo_node
# pegasus_ros mission_planner_node
# pegasus_ros px4_state_subscriber_node
```

---

## Configuration

### Pixhawk Serial Connection

Edit `launch/pegasus_sensors.launch.py` (line 37):
```python
fcu_dev_arg = DeclareLaunchArgument(
    'fcu_dev',
    default_value='/dev/ttyTHS1',  # Modify this
    description='Serial device for Pixhawk'
)
```

**Common configurations:**
- `/dev/ttyTHS1` - Jetson UART 1 (TELEM2)
- `/dev/ttyACM0` - USB connection
- `/dev/ttyUSB0` - Telemetry radio

**Set permissions:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login for changes to take effect
```

### VLP-16 Network Configuration

Edit `launch/pegasus_sensors.launch.py` (line 106):
```python
'device_ip': '192.168.1.201',  # LiDAR IP address
```

**Network setup:**
- LiDAR default IP: 192.168.1.201
- Set Jetson static IP: 192.168.1.100
- Subnet mask: 255.255.255.0

### RTAB-Map Parameters

Key parameters in `launch/pegasus_slam.launch.py`:
```python
# Feature detection
'Kp/MaxFeatures': '500',        # Increase for better accuracy
'Vis/MaxFeatures': '1000',

# Map resolution
'Grid/CellSize': '0.05',        # 5cm cells (default)

# Odometry
'Odom/ImageDecimation': '1',    # 1=full resolution, 2=half
```

---

## Usage

### Launch Full System
```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

This starts:
- ZED X camera driver
- VLP-16 LiDAR driver
- XRCE-DDS agent (Pixhawk communication)
- RTAB-Map SLAM
- Mission planner node
- RViz visualization

### Launch Individual Components

#### Sensors Only
```bash
ros2 launch pegasus_ros pegasus_sensors.launch.py
```

Optional arguments:
```bash
ros2 launch pegasus_ros pegasus_sensors.launch.py \
    enable_zed:=false \
    enable_lidar:=false \
    enable_xrce:=false
```

#### SLAM Only
```bash
ros2 launch pegasus_ros pegasus_slam.launch.py
```

Optional arguments:
```bash
# Localization mode (use existing map)
ros2 launch pegasus_ros pegasus_slam.launch.py \
    localization:=true \
    database_path:=/path/to/map.db

# Disable RViz
ros2 launch pegasus_ros pegasus_slam.launch.py \
    rviz:=false
```

#### Individual Nodes
```bash
# Mission planner
ros2 run pegasus_ros mission_planner_node

# PX4 state monitor
ros2 run pegasus_ros px4_state_subscriber_node

# Front camera processing
ros2 run pegasus_ros front_stereo_node
```

### Simulation Mode
```bash
ros2 launch pegasus_ros pegasus_full.launch.py \
    use_sim_time:=true \
    launch_gazebo_bridge:=true
```

---

## ROS Topics Reference

### RTAB-Map SLAM Outputs

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/rtabmap/odom` | `nav_msgs/Odometry` | 10-20 Hz | SLAM-corrected pose estimate |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 1 Hz | 3D point cloud map |
| `/rtabmap/grid_map` | `nav_msgs/OccupancyGrid` | 1 Hz | 2D occupancy grid for navigation |
| `/rtabmap/info` | `rtabmap_msgs/Info` | Variable | SLAM statistics and diagnostics |

### PX4 Topics (via XRCE-DDS)

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | 50 Hz | PX4 EKF position estimate |
| `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | 100 Hz | Raw IMU data |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | 1 Hz | Battery voltage and current |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | 1 Hz | Armed state, flight mode |

### Mission Planner Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/pegasus/autonomy/mission_status` | `std_msgs/String` | 1 Hz | Current mission state |
| `/pegasus/autonomy/target_waypoint` | `geometry_msgs/PoseStamped` | Variable | Goal position for navigation |

### Sensor Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 10 Hz | VLP-16 3D point cloud |
| `/zed_x/zed_node/left/image_rect_color` | `sensor_msgs/Image` | 30 Hz | Left rectified image |
| `/zed_x/zed_node/right/image_rect_color` | `sensor_msgs/Image` | 30 Hz | Right rectified image |
| `/zed_x/zed_node/left/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Camera calibration |

---

## Package Structure
```
pegasus_ros/
├── launch/
│   ├── pegasus_sensors.launch.py      # Sensor drivers + XRCE-DDS
│   ├── pegasus_slam.launch.py         # RTAB-Map SLAM
│   ├── pegasus_full.launch.py         # Complete system
│   └── vtol1_gazebo_bridge_launch.py  # Simulation bridge
│
├── pegasus_autonomy/                   # Python package
│   ├── __init__.py
│   ├── mission_planner_node.py        # High-level mission logic
│   ├── front_stereo_node.py           # Front camera processing
│   └── px4_state_subscriber_node.py   # PX4 state monitoring
│
├── config/
│   └── rviz_slam.rviz                 # RViz configuration
│
├── maps/                               # RTAB-Map database storage
│   └── .gitkeep
│
├── resource/
│   └── pegasus_ros                    # Package marker
│
├── test/                               # Unit tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── package.xml                         # ROS package manifest
├── setup.py                            # Python package configuration
├── setup.cfg                           # Build configuration
└── README.md                           # This file
```

---

## Development Guide

### Building After Changes
```bash
cd ~/Ros-workspace

# Build single package (fast - 2-3 seconds)
colcon build --symlink-install --packages-select pegasus_ros

# Recreate symlink
cd install/pegasus_ros/lib
ln -sf pegasus_autonomy pegasus_ros

# Source workspace
source install/setup.bash
```

### Adding New Nodes

1. Create Python file in `pegasus_autonomy/`:
```bash
   nano pegasus_autonomy/new_node.py
```

2. Add entry point in `setup.py`:
```python
   entry_points={
       'console_scripts': [
           'new_node = pegasus_autonomy.new_node:main',
       ],
   },
```

3. Build and test:
```bash
   colcon build --symlink-install --packages-select pegasus_ros
   ros2 run pegasus_ros new_node
```

### Modifying Launch Files

Launch files support immediate changes with `--symlink-install`:
```bash
# Edit launch file
nano launch/pegasus_full.launch.py

# Run immediately (no rebuild needed)
ros2 launch pegasus_ros pegasus_full.launch.py
```

### Testing
```bash
# Run all tests
cd ~/Ros-workspace
colcon test --packages-select pegasus_ros

# View test results
colcon test-result --verbose
```

### Code Style

- **Python**: Follow PEP 8 guidelines
- **Comments**: Explain intent and rationale, not implementation
- **Docstrings**: Use Google-style docstrings for all public functions
- **Type hints**: Include type annotations for function parameters and returns

### Git Workflow
```bash
# Create feature branch
git checkout -b feature/new-functionality

# Make changes and commit
git add .
git commit -m "Add: Brief description of changes"

# Push and create pull request
git push origin feature/new-functionality
```

---

## Troubleshooting

### XRCE-DDS Agent Connection Issues

**Symptom**: No `/fmu/out/*` topics

**Solution**:
```bash
# Check serial port exists
ls -l /dev/ttyTHS1

# Test agent manually
micro-xrce-dds-agent serial --dev /dev/ttyTHS1 -b 921600

# Expected output: "TermiosAgentLinux.cpp | init | running..."
```

**Common fixes**:
- Verify serial port name in launch file
- Check cable connections
- Ensure PX4 XRCE-DDS client is enabled in firmware

### VLP-16 Not Publishing Data

**Symptom**: No `/velodyne_points` topic

**Solution**:
```bash
# Verify network connectivity
ping 192.168.1.201

# Check network interface
ifconfig

# Verify LiDAR IP in launch file
grep device_ip launch/pegasus_sensors.launch.py
```

**Common fixes**:
- Set Jetson static IP in same subnet (192.168.1.x)
- Check Ethernet cable
- Verify LiDAR power supply

### RTAB-Map Not Starting

**Symptom**: SLAM nodes not appearing in `ros2 node list`

**Solution**:
```bash
# Check RTAB-Map installed
ros2 pkg list | grep rtabmap

# Verify camera topics exist
ros2 topic list | grep zed_x

# Check launch file errors
ros2 launch pegasus_ros pegasus_slam.launch.py --show-args
```

**Common fixes**:
- Ensure ZED X camera is connected and detected
- Verify camera driver is publishing images
- Check RTAB-Map parameters for typos

### Build Failures

**Symptom**: `colcon build` fails

**Solution**:
```bash
# Clean build
cd ~/Ros-workspace
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install --packages-select pegasus_ros

# Recreate symlink
cd install/pegasus_ros/lib
ln -s pegasus_autonomy pegasus_ros
```

### Missing Executables

**Symptom**: `ros2 pkg executables pegasus_ros` returns nothing

**Solution**:
```bash
# Verify symlink exists
ls -la ~/Ros-workspace/install/pegasus_ros/lib/pegasus_ros

# If missing, create it
cd ~/Ros-workspace/install/pegasus_ros/lib
ln -s pegasus_autonomy pegasus_ros

# Source workspace
source ~/Ros-workspace/install/setup.bash
```

---

## Performance Optimization

### RTAB-Map Tuning

For Jetson Orin AGX (high performance):
```python
'Kp/MaxFeatures': '800',
'Vis/MaxFeatures': '1500',
'Odom/ImageDecimation': '1',  # Full resolution
```

For Jetson Xavier NX (balanced):
```python
'Kp/MaxFeatures': '500',
'Vis/MaxFeatures': '1000',
'Odom/ImageDecimation': '1',
```

For CPU-limited platforms:
```python
'Kp/MaxFeatures': '300',
'Vis/MaxFeatures': '600',
'Odom/ImageDecimation': '2',  # Half resolution
```

### Build Time Optimization

Separate `px4_msgs` workspace reduces daily build time from 8+ minutes to 2-3 seconds.

---

## Contributing

### Code Contributions

1. Fork the repository
2. Create a feature branch
3. Make changes following code style guidelines
4. Add tests for new functionality
5. Submit pull request with detailed description

### Bug Reports

Include the following information:
- ROS 2 version and OS
- Hardware configuration
- Steps to reproduce
- Expected vs actual behavior
- Relevant log files

### Feature Requests

Describe:
- Use case and motivation
- Proposed implementation approach
- Potential impact on existing functionality

---

## License

MIT License

Copyright (c) 2025 Cal Poly Pomona - Team Pegasus

---

## Team

**Autonomy Lead**: Changwe  
**Institution**: California Polytechnic State University, Pomona  
**Sponsor**: Lockheed Martin  
**Project**: Disaster Response eVTOL UAV

---

## Acknowledgments

- Lockheed Martin for project sponsorship
- Cal Poly Pomona College of Engineering
- RTAB-Map development team
- PX4 Development Team
- ROS 2 community

---

## References

- [RTAB-Map Documentation](http://wiki.ros.org/rtabmap_ros)
- [PX4 User Guide](https://docs.px4.io/)
- [PX4 XRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Velodyne VLP-16 Manual](https://velodynelidar.com/products/puck/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**Last Updated**: February 2025  
**Version**: 1.0.0