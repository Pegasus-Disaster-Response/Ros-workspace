# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.0.0
**Last Updated:** February 2026

---

## v2.0 Changelog

- **RTAB-Map built from source** (0.23.x) — apt 0.22.1 removed to avoid database version conflicts
- **Switched to RGB-D mode** — uses ZED X NEURAL depth instead of stereo disparity matching
- **LiDAR ICP odometry** — primary odometry from VLP-16 (was stereo visual odometry)
- **PX4 IMU bridge node** — converts PX4 SensorCombined (FRD) → sensor_msgs/Imu (FLU)
- **Static TF publishers** — explicit transforms for all sensors relative to base_link
- **YAML config files** — parameters extracted from launch files into rtabmap.yaml, zed_x.yaml, vlp16.yaml
- **Mem/UseOdomGravity** — map Z-axis aligned with gravity via IMU
- **XRCE-DDS agent fix** — runs as ExecuteProcess (snap binary), not as a ROS node
- **setup_workspace.sh** — reproducible one-script environment setup

---

## How to Use

### Launch Full System
```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

This launches: ZED X camera driver, VLP-16 LiDAR driver, XRCE-DDS agent (Pixhawk), PX4 IMU bridge, ICP odometry, RTAB-Map SLAM, static TFs, mission planner, RViz.

### Launch Individual Components
```bash
# Sensors only
ros2 launch pegasus_ros pegasus_sensors.launch.py

# SLAM only (sensors must be running)
ros2 launch pegasus_ros pegasus_slam.launch.py

# Mission planner only
ros2 run pegasus_ros mission_planner_node

# IMU bridge only
ros2 run pegasus_ros px4_imu_bridge_node

# PX4 state monitor only
ros2 run pegasus_ros px4_state_subscriber_node

# Front camera processing only
ros2 run pegasus_ros front_stereo_node
```

### Launch with Hardware Disabled
```bash
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_zed:=false
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_lidar:=false
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_xrce:=false
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_imu_bridge:=false

# Dry run — no hardware at all
ros2 launch pegasus_ros pegasus_sensors.launch.py \
    enable_zed:=false enable_lidar:=false enable_xrce:=false enable_imu_bridge:=false
```

### SLAM Operations
```bash
# Start mapping (create new map)
ros2 launch pegasus_ros pegasus_slam.launch.py

# Use existing map (localization mode)
ros2 launch pegasus_ros pegasus_slam.launch.py \
    localization:=true database_path:=/path/to/map.db

# Launch without visualizer
ros2 launch pegasus_ros pegasus_slam.launch.py rviz:=false
```

### System Verification
```bash
# Check active nodes
ros2 node list
# Expected:
# /icp_odometry
# /mission_planner_node
# /px4_imu_bridge_node
# /px4_state_subscriber_node
# /rtabmap/rtabmap
# /point_cloud_assembler
# /velodyne_driver
# /velodyne_convert

# SLAM topics
ros2 topic list | grep rtabmap

# PX4 topics
ros2 topic list | grep fmu

# Sensor topics
ros2 topic list | grep -E "(velodyne|zed_x|pegasus/imu)"
```

### Monitor Data Rates
```bash
ros2 topic hz /odom                    # ICP odometry (~10 Hz)
ros2 topic hz /velodyne_points         # LiDAR (~10 Hz)
ros2 topic hz /zed_x/zed_node/rgb/image_rect_color  # Camera (~30 Hz)
ros2 topic hz /pegasus/imu/data        # Bridged IMU (~50 Hz)
ros2 topic hz /fmu/out/sensor_combined # Raw PX4 IMU (~100 Hz)
```

### Verify TF Tree
```bash
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → velodyne / zed_x_camera_center / imu_link
```

### Building After Code Changes
```bash
cd ~/Ros-workspace
colcon build --symlink-install --packages-select pegasus_ros
source install/setup.bash
```

### Recording Flight Data
```bash
ros2 bag record -a -o flight_test_001
ros2 bag play flight_test_001 --clock
ros2 launch pegasus_ros pegasus_slam.launch.py use_sim_time:=true
```

---

## Configuration Files

| File | Purpose | Key Parameters |
|---|---|---|
| `config/rtabmap.yaml` | SLAM tuning | Feature count, ICP settings, grid resolution, gravity alignment |
| `config/zed_x.yaml` | Camera settings | Serial number, depth mode, resolution, frame rate |
| `config/vlp16.yaml` | LiDAR settings | IP address, min/max range, calibration |

### Editing Parameters

Edit YAML files directly — no rebuild needed (with `--symlink-install`):
```bash
nano src/pegasus_ros/config/rtabmap.yaml
# Changes take effect on next launch
```

---

## What Still Needs to Be Added

### Critical Missing Components

**Path Planning Node**
- Status: Not implemented
- Priority: High
- Architecture: A* (global) → D* Lite (local replanning) → MPC (smoothing)
- Required inputs: `/rtabmap/grid_map`, `/odom`, `/pegasus/autonomy/target_waypoint`
- Files to create: `pegasus_autonomy/path_planner_node.py`, `config/path_planner.yaml`

**Obstacle Avoidance Node**
- Status: Not implemented
- Priority: High
- Required inputs: `/velodyne_points`, `/odom`, planned path
- Files to create: `pegasus_autonomy/obstacle_avoidance_node.py`

**Flight Controller Interface (Offboard Commands)**
- Status: Partially implemented (monitoring only)
- Priority: High
- Requires publishing to: `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, `/fmu/in/offboard_control_mode`

### Computer Vision

**Survivor Detection** — YOLOv8 on ZED X camera
**Fire/Smoke Detection** — color + ML on ZED X camera
**Structural Damage Assessment** — edge detection + segmentation

### Configuration Needed

**UAV Physical Specs** — mass, inertia, flight envelope in `config/uav_physical_params.yaml`
**Sensor Calibration** — camera-LiDAR extrinsic calibration procedures
**Static TF Values** — current values are placeholders, need actual measurements

---

## Development Priorities

### Phase 1: Core Navigation (Current → Week 6)
1. ✅ SLAM with multi-sensor fusion
2. ✅ LiDAR ICP odometry
3. ✅ IMU bridge for PX4
4. ✅ Static TF tree
5. Path planning (A* + D* Lite + MPC)
6. Obstacle avoidance
7. PX4 offboard command interface

### Phase 2: Computer Vision (Weeks 7-10)
1. Survivor detection with YOLOv8
2. Fire and smoke detection
3. Integration with mission planner

### Phase 3: Mission Intelligence (Weeks 11-14)
1. Autonomous search patterns
2. Multi-target prioritization
3. Recovery behaviors

### Phase 4: Testing & Validation (Weeks 15-18)
1. SITL verification
2. Hardware-in-the-loop testing
3. Field testing

---

**Last Updated**: February 2026
**Version**: 2.0.0