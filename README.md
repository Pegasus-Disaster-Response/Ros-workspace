# Pegasus Disaster Response UAV — Autonomous Navigation System

**Version:** 2.0.0
**Institution:** California Polytechnic State University, Pomona
**Team:** Pegasus
**Sponsor:** Lockheed Martin
**ROS 2 Humble | RTAB-Map (from source) | PX4 via XRCE-DDS**

**Sensors:** ZED X (single, front-facing) · Velodyne VLP-16 · Pixhawk Cube Orange (IMU)
**SLAM:** RTAB-Map (LiDAR ICP odometry + RGB-D loop closure + IMU fusion)

---

## Workspace Structure

```
Ros-workspace/
├── setup_workspace.sh              ← run this first
├── COMPLETE_SUMMARY.md             ← quick reference & roadmap
└── src/
    ├── pegasus_ros/                ← custom package (configs + launches + nodes)
    │   ├── config/
    │   │   ├── rtabmap.yaml        ← RTAB-Map SLAM tuning parameters
    │   │   ├── vlp16.yaml          ← Velodyne point cloud conversion settings
    │   │   ├── zed_x.yaml          ← ZED X camera configuration
    │   │   └── rviz_slam.rviz      ← RViz display configuration
    │   ├── launch/
    │   │   ├── pegasus_full.launch.py      ← complete system
    │   │   ├── pegasus_sensors.launch.py   ← sensor drivers + XRCE-DDS + IMU bridge
    │   │   ├── pegasus_slam.launch.py      ← RTAB-Map SLAM + static TFs
    │   │   └── vtol1_gazebo_bridge_launch.py  ← simulation bridge
    │   └── pegasus_autonomy/
    │       ├── mission_planner_node.py     ← high-level mission logic
    │       ├── px4_imu_bridge_node.py      ← PX4 SensorCombined → sensor_msgs/Imu
    │       ├── front_stereo_node.py        ← front camera processing
    │       └── px4_state_subscriber_node.py ← PX4 state monitoring
    ├── rtabmap/                    ← cloned from source (0.23.x — apt 0.22.1 is too old)
    ├── rtabmap_ros/                ← cloned from source (0.23.x)
    └── zed-ros2-wrapper/           ← cloned from Stereolabs
```

---

## Quick Start

### 1. Run setup script (once)

```bash
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

This script installs all dependencies, removes incompatible apt packages, builds RTAB-Map from source, and configures the workspace.

### 2. Configure your hardware

Set your ZED X serial number:
```bash
nano src/pegasus_ros/config/zed_x.yaml
# Change serial_number: 0 to your actual serial
# Find it with: /usr/local/zed/tools/ZED_Explorer
```

Update sensor mount positions in `launch/pegasus_slam.launch.py`:
```python
# Find the static TF publishers and update x, y, z to match
# your actual sensor positions measured from base_link
tf_base_to_velodyne = ...  # '0.3', '0.0', '0.15'  ← UPDATE THESE
tf_base_to_zed_x    = ...  # '0.2', '0.0', '0.1'   ← UPDATE THESE
tf_base_to_imu      = ...  # '0.0', '0.0', '0.0'   ← UPDATE THESE
```

### 3. Hardware checklist before launch

| Sensor | Check |
|---|---|
| VLP-16 | Set Jetson static IP to `192.168.1.100/24`; verify with `ping 192.168.1.201` |
| ZED X | Serial number set in `zed_x.yaml`; check with `/usr/local/zed/tools/ZED_Explorer` |
| Pixhawk | Connected via UART (`/dev/ttyTHS1`) or USB (`/dev/ttyACM0`); permissions set |
| GMSL daemon | `sudo systemctl status nvargus-daemon` — must be **active (running)** |

### 4. Build

```bash
cd ~/Ros-workspace
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug
source install/setup.bash
```

### 5. Launch SLAM

```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

---

## RTAB-Map Version — CRITICAL

The apt version of RTAB-Map (`0.22.1`) is **incompatible** with the database format written by newer source builds (`0.23.x`). This workspace builds both `rtabmap` core and `rtabmap_ros` from source.

**If you see this error:**
```
Opened database version (0.23.4) is more recent than rtabmap installed version (0.22.1)
```

**Fix:** The apt version is still installed. Remove it and rebuild:
```bash
sudo apt remove -y ros-humble-rtabmap ros-humble-rtabmap-ros
rm ~/.ros/rtabmap.db
cd ~/Ros-workspace && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip zed_debug
source install/setup.bash
```

**Verify correct version:**
```bash
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"
# Expected: RTAB-Map: 0.23.x (NOT 0.22.1)
```

---

## Architecture Changes (v2.0)

### Odometry: LiDAR ICP (was Stereo Visual)

v1.0 used stereo visual odometry from the ZED X camera. v2.0 uses **LiDAR ICP odometry** from the VLP-16 as the primary odometry source. This is more robust in visually degraded disaster environments (smoke, dust, uniform textures).

```
VLP-16 → /velodyne_points → icp_odometry → /odom
                                              ↓
ZED X → rgb + depth ────────→ RTAB-Map SLAM (loop closure + mapping)
                                              ↑
Pixhawk → SensorCombined → px4_imu_bridge → /pegasus/imu/data
```

### RGB-D Mode (was Stereo Mode)

v1.0 subscribed to stereo image pairs and let RTAB-Map compute disparity. v2.0 uses the ZED X's **NEURAL depth engine** directly, which produces significantly higher quality depth maps.

### IMU Bridge

PX4's `SensorCombined` message is not a standard `sensor_msgs/Imu`. The new `px4_imu_bridge_node` converts PX4 IMU data (FRD frame) to standard ROS IMU messages (FLU frame) for RTAB-Map consumption.

### Static TF Tree

All sensor transforms are now explicitly published:
```
map
 └── odom (published by rtabmap)
      └── base_link
           ├── velodyne             [0.3, 0.0, 0.15]   ← UPDATE to your mount
           ├── zed_x_camera_center  [0.2, 0.0, 0.1]    ← UPDATE to your mount
           └── imu_link             [0.0, 0.0, 0.0]     ← UPDATE to your mount
```

---

## Installation (Manual)

If you prefer to set up manually instead of using `setup_workspace.sh`:

### Prerequisites

```bash
# ROS 2 Humble must be installed and sourced
source /opt/ros/humble/setup.bash

# ZED SDK must be installed
# Download from: https://www.stereolabs.com/developers/release
```

### Step 1: Remove apt RTAB-Map

**This is required.** The apt version (0.22.1) conflicts with source builds.

```bash
# Check what's installed
dpkg -l | grep rtabmap

# Remove all apt RTAB-Map packages
sudo apt remove -y \
  ros-humble-rtabmap \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-odom \
  ros-humble-rtabmap-util \
  ros-humble-rtabmap-viz \
  ros-humble-rtabmap-msgs \
  ros-humble-rtabmap-launch

sudo apt autoremove -y

# Verify removal
dpkg -l | grep rtabmap
# Should return nothing
```

### Step 2: Remove other unnecessary packages

If you previously installed packages that are no longer needed:

```bash
# micro_ros_agent — replaced by micro-xrce-dds-agent (snap)
sudo apt remove -y ros-humble-micro-ros-agent 2>/dev/null || true

# rviz extras not needed
sudo apt remove -y ros-humble-rviz-common ros-humble-rviz-default-plugins 2>/dev/null || true

# rclcpp not needed (this is a Python package)
# Note: rclcpp may be pulled in by other packages, only remove if you don't need it
```

### Step 3: Install system dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-velodyne \
  ros-humble-velodyne-driver \
  ros-humble-velodyne-pointcloud \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
```

### Step 4: Install XRCE-DDS Agent

```bash
sudo snap install micro-xrce-dds-agent --edge
```

### Step 5: Build PX4 Messages workspace

```bash
mkdir -p ~/px4_ws/src
cd ~/px4_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/px4_ws
colcon build --symlink-install
echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc
source ~/px4_ws/install/setup.bash
```

### Step 6: Clone source packages

```bash
cd ~/Ros-workspace/src

# RTAB-Map core (from source for 0.23.x)
git clone https://github.com/introlab/rtabmap.git

# RTAB-Map ROS 2 wrapper (from source)
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

# ZED ROS 2 wrapper
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
```

### Step 7: Build

```bash
cd ~/Ros-workspace
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug

source install/setup.bash
echo "source ~/Ros-workspace/install/setup.bash" >> ~/.bashrc
```

---

## Package Verification

Run these checks to confirm everything is installed correctly.

### ROS 2 Environment
```bash
echo $ROS_DISTRO
# Expected: humble
```

### RTAB-Map (from source)
```bash
ros2 pkg list | grep rtabmap
# Expected:
#   rtabmap_msgs
#   rtabmap_odom
#   rtabmap_ros
#   rtabmap_slam
#   rtabmap_util
#   rtabmap_viz

ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"
# Expected: RTAB-Map: 0.23.x (NOT 0.22.1)
```

### Velodyne
```bash
ros2 pkg list | grep velodyne
# Expected:
#   velodyne
#   velodyne_driver
#   velodyne_laserscan
#   velodyne_msgs
#   velodyne_pointcloud
```

### ZED SDK & ROS Wrapper
```bash
# SDK version
cat /usr/local/zed/include/sl/Camera.hpp | grep ZED_SDK_MAJOR_VERSION
# Expected: #define ZED_SDK_MAJOR_VERSION 5

# ROS wrapper
ros2 pkg list | grep zed
# Expected:
#   zed_components
#   zed_ros2_interfaces
#   zed_wrapper
```

### ZED X Camera
```bash
/usr/local/zed/tools/ZED_Explorer -a
# Expected: your ZED X with serial number and State: "AVAILABLE"
```

### GMSL Daemon (Jetson only)
```bash
sudo systemctl status nvargus-daemon
# Expected: active (running)
```

### PX4 Messages
```bash
ros2 pkg list | grep px4_msgs
# Expected: px4_msgs
```

### XRCE-DDS Agent
```bash
which micro-xrce-dds-agent
# Expected: /snap/bin/micro-xrce-dds-agent
```

### Pegasus Package
```bash
ros2 pkg list | grep pegasus_ros
# Expected: pegasus_ros

ros2 pkg executables pegasus_ros
# Expected:
#   pegasus_ros front_stereo_node
#   pegasus_ros mission_planner_node
#   pegasus_ros px4_imu_bridge_node
#   pegasus_ros px4_state_subscriber_node
```

### Velodyne Network
```bash
ping -c 4 192.168.1.201
# Expected: 4 packets transmitted, 4 received, 0% packet loss
```

### Pixhawk Serial Port
```bash
ls -l /dev/ttyTHS1    # Jetson UART
# or
ls -l /dev/ttyACM0    # USB
```

### Full Dependency Check
```bash
cd ~/Ros-workspace
rosdep check --from-paths src --ignore-src -r
# Expected: All system dependencies have been satisfied
```

---

## Usage

### Launch Full System
```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

### Launch Individual Components

```bash
# Sensors only
ros2 launch pegasus_ros pegasus_sensors.launch.py

# SLAM only (sensors must be running)
ros2 launch pegasus_ros pegasus_slam.launch.py

# Localization mode (use existing map)
ros2 launch pegasus_ros pegasus_slam.launch.py \
    localization:=true \
    database_path:=/path/to/map.db
```

### Disable Individual Sensors

```bash
# Without camera
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_zed:=false

# Without LiDAR
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_lidar:=false

# Without Pixhawk
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_xrce:=false

# Without IMU bridge
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_imu_bridge:=false
```

### Custom VLP-16 IP

```bash
ros2 launch pegasus_ros pegasus_full.launch.py velodyne_ip:=192.168.1.201
```

### Simulation Mode

```bash
ros2 launch pegasus_ros pegasus_full.launch.py \
    use_sim_time:=true \
    launch_gazebo_bridge:=true
```

---

## Post-Launch Verification

Run these in a **second terminal** while SLAM is running.

### Check all expected topics
```bash
ros2 topic list
# Expected (among others):
#   /velodyne_points
#   /zed_x/zed_node/rgb/image_rect_color
#   /zed_x/zed_node/rgb/camera_info
#   /zed_x/zed_node/depth/depth_registered
#   /pegasus/imu/data
#   /odom
#   /rtabmap/mapData
#   /rtabmap/grid_map
#   /rtabmap/cloud_map
#   /tf
#   /tf_static
```

### Check topic publish rates
```bash
ros2 topic hz /velodyne_points                             # Expected: ~10 Hz
ros2 topic hz /zed_x/zed_node/rgb/image_rect_color        # Expected: ~30 Hz
ros2 topic hz /pegasus/imu/data                            # Expected: ~50 Hz
ros2 topic hz /odom                                        # Expected: ~10 Hz
```

### Verify TF tree
```bash
ros2 run tf2_tools view_frames
# Open frames.pdf and confirm:
#   map → odom → base_link → velodyne
#                           → zed_x_camera_center
#                           → imu_link
```

### Verify ICP odometry is working
Look for in the terminal output:
```
[icp_odometry] Odom: ratio=0.5xx, std dev=0.00Xm|0.00Xrad
```

If you see `ratio=0.000000`: the VLP-16 is not publishing correctly, or `Icp/MaxTranslation` is too small.

### Check for duplicate Velodyne nodes
```bash
ros2 node list | grep velodyne
# Expected (exactly):
#   /velodyne_convert
#   /velodyne_driver
```

If you see duplicates, kill everything and relaunch:
```bash
killall velodyne_driver_node velodyne_transform_node
pkill -f "ros2 launch"
```

---

## Topic Map

```
/velodyne_points                              ← assembled point cloud (~10 Hz)
/zed_x/zed_node/rgb/image_rect_color         ← ZED X RGB
/zed_x/zed_node/rgb/camera_info              ← ZED X camera calibration
/zed_x/zed_node/depth/depth_registered       ← ZED X neural depth
/pegasus/imu/data                             ← bridged Pixhawk IMU (sensor_msgs/Imu)
/odom                                         ← LiDAR ICP odometry
/fmu/out/vehicle_odometry                     ← PX4 EKF odometry (not used by SLAM)
/fmu/out/sensor_combined                      ← raw PX4 IMU (consumed by bridge)
/fmu/out/vehicle_attitude                     ← PX4 orientation (consumed by bridge)
/fmu/out/battery_status                       ← battery monitoring

/rtabmap/mapData                              ← SLAM graph
/rtabmap/grid_map                             ← 2D occupancy grid
/rtabmap/cloud_map                            ← 3D point cloud map
/rtabmap/odom                                 ← RTAB-Map corrected odometry
/pegasus/autonomy/mission_status              ← mission planner status
/pegasus/autonomy/target_waypoint             ← goal position
/tf, /tf_static                               ← transform tree
```

---

## Configuration Files

All tunable parameters are in YAML files under `config/`:

| File | What it controls |
|---|---|
| `rtabmap.yaml` | SLAM parameters, ICP settings, grid map resolution, visual features |
| `zed_x.yaml` | Camera resolution, depth mode, frame rate, serial number |
| `vlp16.yaml` | Point cloud conversion, min/max range |
| `rviz_slam.rviz` | RViz display layout |

### Tuning for Jetson Orin AGX (high performance)
Edit `config/rtabmap.yaml`:
```yaml
Kp/MaxFeatures: "800"
Vis/MaxFeatures: "1500"
Odom/ImageDecimation: "1"
```

### Tuning for Jetson Xavier NX (balanced)
```yaml
Kp/MaxFeatures: "500"
Vis/MaxFeatures: "1000"
Odom/ImageDecimation: "1"
```

### Tuning for CPU-limited platforms
```yaml
Kp/MaxFeatures: "300"
Vis/MaxFeatures: "600"
Odom/ImageDecimation: "2"
Icp/Iterations: "20"
```

---

## Troubleshooting

### RTAB-Map database version mismatch
```
Opened database version (0.23.4) is more recent than rtabmap installed version (0.22.1)
```
The apt version is still being loaded. Run:
```bash
sudo apt remove -y ros-humble-rtabmap ros-humble-rtabmap-ros
rm ~/.ros/rtabmap.db
# Open fresh terminal and source workspace
```

### Velodyne publishing at 70 Hz instead of 10 Hz
Duplicate nodes from a previous launch:
```bash
killall velodyne_driver_node velodyne_transform_node
pkill -f "ros2 launch"
pkill -f component_container_isolated
```

### ZED X not detected
```bash
sudo systemctl restart nvargus-daemon
# Wait a few seconds, then relaunch
```

### No /pegasus/imu/data topic
Check that the IMU bridge is running and receiving PX4 data:
```bash
ros2 node list | grep imu_bridge
ros2 topic echo /fmu/out/sensor_combined --once
```

### XRCE-DDS agent not connecting
```bash
# Test agent manually
micro-xrce-dds-agent serial --dev /dev/ttyTHS1 -b 921600
# Should show: running...

# Check serial permissions
sudo usermod -a -G dialout $USER
# Logout and login required
```

### VLP-16 not reachable
```bash
ping 192.168.1.201
# If unreachable:
#   1. Check Ethernet cable
#   2. Set Jetson static IP to 192.168.1.100 (same subnet)
#   3. Verify LiDAR power supply
```

### RTAB-Map not creating map nodes
The vehicle must move at least 0.1m or 0.05rad before the first node is created. Move the UAV and check:
```bash
ros2 topic echo /rtabmap/mapData --once
```

### Build failures
```bash
# Clean build
cd ~/Ros-workspace
rm -rf build/ install/ log/
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug
```

---

## Building After Code Changes

```bash
cd ~/Ros-workspace

# Build single package (fast — 2-3 seconds)
colcon build --symlink-install --packages-select pegasus_ros

# Source workspace
source install/setup.bash
```

---

## Recording & Playback

```bash
# Record all topics
ros2 bag record -a -o flight_test_001

# Record specific topics
ros2 bag record \
  /odom /velodyne_points /pegasus/imu/data \
  /zed_x/zed_node/rgb/image_rect_color \
  /zed_x/zed_node/depth/depth_registered \
  -o flight_test_001

# Play back
ros2 bag play flight_test_001 --clock

# Run SLAM on recorded data
ros2 launch pegasus_ros pegasus_slam.launch.py use_sim_time:=true
```

---

## Team

**Autonomy Lead**: Changwe
**Institution**: California Polytechnic State University, Pomona
**Sponsor**: Lockheed Martin
**Project**: Disaster Response eVTOL UAV

---

## References

- [RTAB-Map Documentation](http://wiki.ros.org/rtabmap_ros)
- [RTAB-Map GitHub](https://github.com/introlab/rtabmap)
- [PX4 User Guide](https://docs.px4.io/)
- [PX4 XRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Velodyne VLP-16 Manual](https://velodynelidar.com/products/puck/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**Last Updated**: February 2026
**Version**: 2.0.0