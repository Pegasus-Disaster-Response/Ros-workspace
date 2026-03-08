# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.6.0
**Last Updated:** March 2026

---

## Changelog

### v2.6 (March 2026)
- **Disaster response simulation world** — `disaster_response.sdf`: 200 m × 200 m world with 5 challenge zones (collapsed downtown, rubble field, residential corridor, dead-end traps, urban canyon), spawn pylons, transition features, and landmark towers. Includes a full VTOL quadplane model with simulated VLP-16 LiDAR (gpu_lidar, 360°, 16 channels, 100 m, 10 Hz) and ZED X-approximating RGBD camera (105° HFOV, 672×376, 0.3–20 m depth, 15 Hz) plus IMU. Required Gazebo Harmonic system plugins (Physics, SceneBroadcaster, UserCommands, Sensors, Imu, Contact) added.
- **setup.py critical fixes:**
  - Removed dead `px4_state_subscriber_node` entry point (file was deleted but entry remained — would cause build failure)
  - Added missing entry points: `dstar_lite_node`, `mpc_trajectory_node`, `px4_offboard_node` (these nodes existed but couldn't be launched via `ros2 run` or launch files)
  - Added missing config files to data_files: `dstar_lite.yaml`, `vtol_dynamics.yaml`, `px4_offboard.yaml`
  - Added missing `sitl_full.launch.py` to data_files
  - Added `disaster_response.sdf` to installed worlds
- **SDF BOM fix** — Removed UTF-8 BOM from uploaded SDF that could cause XML parse errors
- **Documentation refresh** — README.md and COMPLETE_SUMMARY.md fully updated to reflect v2.4–v2.6 additions

### v2.5 (March 2026)
- **PX4 offboard interface** — `px4_offboard_node` bridges MPC trajectory setpoints to PX4 over XRCE-DDS. Handles ENU→NED coordinate conversion, OffboardControlMode heartbeat at 50 Hz, arm/disarm commands, and setpoint timeout safety. Same code works for both real flight and SITL.
- **SITL launch file** — `sitl_full.launch.py` launches the full autonomy stack against PX4 SITL in Gazebo Harmonic. Includes `auto_engage` and `auto_arm` arguments for automated SITL testing.
- **Offboard config** — `config/px4_offboard.yaml` for heartbeat rate, timeout, and auto-engage settings.
- **Updated full launch** — `pegasus_full.launch.py` now includes offboard node with `enable_offboard` argument.

### v2.4 (March 2026)
- D* Lite 3D local replanner (26-connected, full voxel grid, incremental repair)
- MPC trajectory smoother with placeholder VTOL dynamics
- VTOL dynamics config, 3D costmap grid publisher (Int8MultiArray)

### v2.3 (March 2026)
- Fixes: setup.py script_dir, odom wall clock→ROS clock, YAML namespace mismatch, ICP MaxCorrespondenceDistance < VoxelSize, ZED camera_info QoS, RANSAC ground removal, conditional ros_gz_bridge

### v2.2 (March 2026)
- A* global planner, static map publisher, SIL test infrastructure, Gazebo test world

### v2.1 / v2.0
- 3D voxel costmap with dual-sensor fusion, RTAB-Map from source (v0.23.x), dual odometry, IMU bridge

---

## Full Pipeline (v2.6)

```
  Goal ──→ A* ──→ D* Lite 3D ──→ MPC ──→ PX4 Offboard ──→ Pixhawk ──→ Motors
            │         │            │           │
         grid_map  costmap_3d    /odom    XRCE-DDS
```

| Layer | Node | Rate | Input | Output |
|---|---|---|---|---|
| Global plan | global_planner_node | on goal | OccupancyGrid | /global_path |
| Local replan | dstar_lite_node | 5 Hz | 3D voxel grid | /local_path |
| Trajectory | mpc_trajectory_node | 50 Hz | /local_path + /odom | /trajectory/setpoint |
| Flight control | px4_offboard_node | 50 Hz | /trajectory/setpoint | /fmu/in/trajectory_setpoint |

---

## Quick Start

### Real Flight (all sensors + Pixhawk)
```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

### SITL (Gazebo + PX4 software)
```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: XRCE-DDS agent (UDP for SITL)
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Full autonomy stack
cd ~/Ros-workspace && source install/setup.bash
ros2 launch pegasus_ros sitl_full.launch.py

# Terminal 4: Send a goal
ros2 topic pub --once /pegasus/autonomy/target_waypoint \
    geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 10.0}}}"

# Terminal 5: Arm + engage offboard
ros2 topic pub --once /pegasus/offboard/arm std_msgs/Bool "{data: true}"
ros2 topic pub --once /pegasus/offboard/engage std_msgs/Bool "{data: true}"
```

### SITL with auto-arm (for demos)
```bash
ros2 launch pegasus_ros sitl_full.launch.py auto_arm:=true auto_engage:=true
```

### Test without PX4 (planning only)
```bash
ros2 launch pegasus_ros pegasus_full.launch.py enable_offboard:=false
```

### Disaster Response World (standalone Gazebo)
```bash
gz sim ~/Ros-workspace/src/pegasus_ros/worlds/disaster_response.sdf
```

---

## Workspace Structure

```
Ros-workspace/src/pegasus_ros/
├── setup.py                      (v2.6.0)
├── package.xml
├── config/
│   ├── rtabmap.yaml                ← RTAB-Map SLAM
│   ├── icp_odometry.yaml           ← ICP LiDAR odometry
│   ├── rgbd_odometry.yaml          ← RGB-D visual odometry
│   ├── vlp16.yaml                  ← Velodyne VLP-16
│   ├── zed_x.yaml                  ← ZED X camera
│   ├── local_costmap.yaml          ← 3D voxel costmap
│   ├── path_planner.yaml           ← A* global planner
│   ├── dstar_lite.yaml             ← D* Lite 3D replanner
│   ├── vtol_dynamics.yaml          ← VTOL dynamics + MPC tuning
│   ├── px4_offboard.yaml           ← PX4 offboard interface
│   ├── rviz_slam.rviz
│   ├── rviz_local_costmap.rviz
│   └── rviz_planner_test.rviz
├── launch/
│   ├── pegasus_full.launch.py      ← real flight (sensors + SLAM + costmap + planners + PX4)
│   ├── pegasus_sensors.launch.py   ← sensor drivers only
│   ├── pegasus_slam.launch.py      ← SLAM + odometry
│   ├── local_costmap.launch.py     ← 3D costmap nodes
│   ├── path_planner.launch.py      ← A* standalone
│   ├── gazebo_planner_test.launch.py ← A* test (no hardware)
│   ├── sitl_full.launch.py         ← SITL (Gazebo + PX4 SITL)
│   ├── p110_gazebo_bridge_launch.py
│   └── vtol1_gazebo_bridge_launch.py
├── pegasus_autonomy/
│   ├── mission_planner_node.py
│   ├── px4_imu_bridge_node.py
│   ├── px4_offboard_node.py         ← PX4 flight commands (ENU→NED)
│   ├── odometry_selector_node.py    ← fault-tolerant odom switching
│   ├── lidar_costmap_layer_node.py  ← VLP-16 → obstacles (RANSAC)
│   ├── zed_depth_costmap_layer_node.py ← ZED X depth → obstacles
│   ├── local_costmap_node.py        ← 3D voxel fusion + raycasting
│   ├── global_planner_node.py       ← A*
│   ├── dstar_lite_node.py           ← D* Lite 3D (26-connected)
│   ├── mpc_trajectory_node.py       ← MPC smoother
│   ├── front_stereo_node.py         ← placeholder
│   ├── static_map_publisher_node.py
│   └── static_odom_publisher_node.py
├── worlds/
│   ├── disaster_response.sdf        ← 200m, 5 zones + VTOL + sensors
│   └── pegasus_planning_test.sdf    ← 100m, simple planner test
├── maps/
└── test/
```

---

## Topic Map (v2.6)

### Planning Pipeline
| Topic | Type | Rate | Source |
|---|---|---|---|
| /pegasus/path_planner/global_path | Path | on replan | A* |
| /pegasus/path_planner/local_path | Path | ~5 Hz | D* Lite 3D |
| /pegasus/trajectory/setpoint | PoseStamped | 50 Hz | MPC |
| /pegasus/trajectory/velocity_setpoint | TwistStamped | 50 Hz | MPC |
| /pegasus/trajectory/predicted | Path | 50 Hz | MPC |

### PX4 Offboard (to Pixhawk)
| Topic | Type | Rate | Source |
|---|---|---|---|
| /fmu/in/trajectory_setpoint | TrajectorySetpoint | 50 Hz | px4_offboard_node |
| /fmu/in/offboard_control_mode | OffboardControlMode | 50 Hz | px4_offboard_node |
| /fmu/in/vehicle_command | VehicleCommand | on demand | px4_offboard_node |

### Control Topics
| Topic | Type | Purpose |
|---|---|---|
| /pegasus/offboard/arm | Bool | Arm/disarm command |
| /pegasus/offboard/engage | Bool | Enable/disable offboard mode |
| /pegasus/offboard/status | String (JSON) | Offboard interface status |

### Costmap Topics
| Topic | Type | Rate |
|---|---|---|
| /pegasus/local_costmap | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_inflated | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_3d_grid | Int8MultiArray | 5 Hz |
| /pegasus/local_costmap_2d | OccupancyGrid | 10 Hz |
| /pegasus/costmap_metadata | String (JSON) | 1 Hz |
| /pegasus/sensor_status | String | 2 Hz |

### Odometry Topics
| Topic | Source | Priority |
|---|---|---|
| /odom_lidar | ICP LiDAR odometry | Primary |
| /odom_vision | RGB-D visual odometry | Backup |
| /odom | odometry_selector_node | Published (best available) |

---

## SITL Prerequisites

```bash
# PX4
cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh && make px4_sitl gz_x500

# XRCE-DDS
cd ~ && git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/

# Gazebo Harmonic ROS bridge
sudo apt install ros-humble-ros-gzharmonic
```

---

## What Still Needs to Be Added

### Critical
- ⬜ **VTOL dynamics values** — replace placeholders in vtol_dynamics.yaml with real aircraft data
- ⬜ **Obstacle avoidance node** — emergency reactive layer below MPC
- ⬜ **Mission-level safety rules** — speed limiting, battery budget (~1.7x detour), dead-end escape (backtrack-then-climb), progress monitoring

### Simulation
- ⬜ **Custom PX4 VTOL model** — current PX4 SITL uses x500 (MC only); need standard_vtol for transition testing
- ⬜ **Gazebo sensor bridge for disaster_response world** — bridge launch file matching sensor topics from the new SDF

### Computer Vision
- ⬜ Survivor detection (YOLOv8), fire/smoke detection

### Testing
- ⬜ Hypothesis parametric tests for planning + costmap
- ⬜ Bag file replay pipeline
- ⬜ Hardware-in-the-loop testing

### Phase 1 Progress
1. ✅ SLAM (v2.0)
2. ✅ 3D costmap (v2.1)
3. ✅ A* planner (v2.2)
4. ✅ Bug fixes (v2.3)
5. ✅ D* Lite 3D + MPC (v2.4)
6. ✅ PX4 offboard interface + SITL launch (v2.5)
7. ✅ Disaster response simulation world + setup.py fixes (v2.6)
8. ⬜ Obstacle avoidance node
9. ⬜ Full SITL with custom VTOL model + sensor bridges

---

**Last Updated**: March 2026
**Version**: 2.6.0