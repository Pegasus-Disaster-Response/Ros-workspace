# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.1.0
**Last Updated:** March 2026

---

## Changelog

### v2.1 (March 2026)
- **3D local costmap** — rolling voxel grid (40×40×20m) fusing VLP-16 LiDAR + ZED X depth
- **Individual sensor layers** — lidar_costmap_layer_node and zed_depth_costmap_layer_node run independently
- **Sensor health monitoring** — automatic degraded mode detection (nominal/lidar_only/camera_only/all_degraded)
- **3D inflation** — scipy distance_transform_edt for real-time safety buffer computation
- **Multi-format publishing** — PointCloud2, MarkerArray (color-coded by source), 2D OccupancyGrid projection
- **python3-scipy** added as dependency

### v2.0 (February 2026)
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

### Launch Subsystems Individually
```bash
# Sensors → SLAM → Costmap → (future: Planner)
ros2 launch pegasus_ros pegasus_sensors.launch.py
ros2 launch pegasus_ros pegasus_slam.launch.py
ros2 launch pegasus_ros local_costmap.launch.py
```

### Launch Costmap with One Sensor Disabled
```bash
ros2 launch pegasus_ros local_costmap.launch.py enable_lidar:=false
ros2 launch pegasus_ros local_costmap.launch.py enable_zed:=false
```

### Run Individual Nodes
```bash
ros2 run pegasus_ros mission_planner_node
ros2 run pegasus_ros px4_imu_bridge_node
ros2 run pegasus_ros lidar_costmap_layer_node
ros2 run pegasus_ros zed_depth_costmap_layer_node
ros2 run pegasus_ros local_costmap_node
```

---

## Topic Map

### Sensor Topics
| Topic | Type | Rate | Source |
|---|---|---|---|
| `/velodyne_points` | PointCloud2 | ~10 Hz | VLP-16 driver |
| `/zed_x/zed_node/rgb/image_rect_color` | Image | ~30 Hz | ZED X driver |
| `/zed_x/zed_node/depth/depth_registered` | Image (32FC1) | ~30 Hz | ZED X driver |
| `/zed_x/zed_node/rgb/camera_info` | CameraInfo | ~30 Hz | ZED X driver |
| `/fmu/out/sensor_combined` | SensorCombined | ~100 Hz | PX4 via XRCE-DDS |
| `/pegasus/imu/data` | Imu | ~50 Hz | px4_imu_bridge_node |

### SLAM Topics
| Topic | Type | Rate | Source |
|---|---|---|---|
| `/odom` | Odometry | ~10 Hz | ICP odometry |
| `/rtabmap/grid_map` | OccupancyGrid | ~1 Hz | RTAB-Map |
| `/rtabmap/cloud_map` | PointCloud2 | ~1 Hz | RTAB-Map |
| `/tf` (map→odom→base_link) | TF | continuous | RTAB-Map + static publishers |

### 3D Local Costmap Topics
| Topic | Type | Rate | Source |
|---|---|---|---|
| `/pegasus/lidar_obstacles` | PointCloud2 | ~10 Hz | lidar_costmap_layer |
| `/pegasus/lidar_health` | Bool | ~4 Hz | lidar_costmap_layer |
| `/pegasus/zed_obstacles` | PointCloud2 | ~15-30 Hz | zed_depth_costmap_layer |
| `/pegasus/zed_health` | Bool | ~4 Hz | zed_depth_costmap_layer |
| `/pegasus/local_costmap` | PointCloud2 | ~10 Hz | local_costmap_node |
| `/pegasus/local_costmap_markers` | MarkerArray | ~10 Hz | local_costmap_node |
| `/pegasus/local_costmap_2d` | OccupancyGrid | ~10 Hz | local_costmap_node |
| `/pegasus/sensor_status` | String | ~2 Hz | local_costmap_node |

---

## Sensor Health & Degraded Modes

The local costmap node monitors both sensor heartbeats and adjusts behavior:

| Mode | LiDAR | ZED | Behavior |
|---|---|---|---|
| `nominal` | ✅ | ✅ | Full fusion, standard safety margins |
| `lidar_only` | ✅ | ❌ | 360° coverage, safety margins ×1.5 |
| `camera_only` | ❌ | ✅ | Forward cone only, speed limited, RTL after 30s |
| `all_degraded` | ❌ | ❌ | Emergency loiter, RTL after 60s |

Monitor live: `ros2 topic echo /pegasus/sensor_status`

---

## RViz Visualization

### SLAM Visualization
```bash
rviz2 -d $(ros2 pkg prefix pegasus_ros)/share/pegasus_ros/config/rviz_slam.rviz
```

### 3D Costmap Visualization
```bash
rviz2 -d $(ros2 pkg prefix pegasus_ros)/share/pegasus_ros/config/rviz_local_costmap.rviz
```

Costmap marker colors in RViz:
- **Red cubes** — obstacle seen by LiDAR only
- **Blue cubes** — obstacle seen by ZED X only
- **Green cubes** — obstacle confirmed by both sensors
- **Alpha gradient** — higher voxels are more opaque

---

## Configuration Files

| File | Purpose | Key Parameters |
|---|---|---|
| `config/rtabmap.yaml` | SLAM tuning | Feature count, ICP settings, grid resolution, gravity alignment |
| `config/zed_x.yaml` | Camera settings | Serial number, depth mode, resolution, frame rate |
| `config/vlp16.yaml` | LiDAR settings | IP address, min/max range, calibration |
| `config/local_costmap.yaml` | 3D costmap | Voxel resolution, grid size, sensor ranges, inflation radius, decay rate, degraded mode thresholds |

Edit YAML files directly — no rebuild needed (with `--symlink-install`):
```bash
nano src/pegasus_ros/config/local_costmap.yaml
# Changes take effect on next launch
```

---

## What Still Needs to Be Added

### Critical Missing Components

**Path Planning Node**
- Status: Not implemented
- Architecture: A* (global) → D* Lite (local replanning) → MPC (smoothing)
- Required inputs: `/rtabmap/grid_map`, `/odom`, `/pegasus/local_costmap`, target waypoint
- Files to create: `pegasus_autonomy/path_planner_node.py`, `config/path_planner.yaml`

**Obstacle Avoidance Node**
- Status: Not implemented
- Will consume `/pegasus/local_costmap` and planned path to compute safe trajectories
- Files to create: `pegasus_autonomy/obstacle_avoidance_node.py`

**Flight Controller Interface (Offboard Commands)**
- Status: Partially implemented (monitoring only)
- Requires publishing to: `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, `/fmu/in/offboard_control_mode`

### Computer Vision

**Survivor Detection** — YOLOv8 on ZED X camera
**Fire/Smoke Detection** — color + ML on ZED X camera
**Structural Damage Assessment** — edge detection + segmentation

### Configuration Needed

**UAV Physical Specs** — mass, inertia, flight envelope in `config/uav_physical_params.yaml`
**Sensor Calibration** — camera-LiDAR extrinsic calibration procedures
**Static TF Values** — current values are placeholders, need actual measurements from the UAV

---

## Development Priorities

### Phase 1: Core Navigation (Current → Week 6)
1. ✅ SLAM with multi-sensor fusion (v2.0)
2. ✅ LiDAR ICP odometry (v2.0)
3. ✅ IMU bridge for PX4 (v2.0)
4. ✅ Static TF tree (v2.0)
5. ✅ 3D local costmap with sensor fusion (v2.1)
6. ✅ Sensor health monitoring & degraded modes (v2.1)
7. ⬜ Path planning (A* + D* Lite + MPC)
8. ⬜ Obstacle avoidance
9. ⬜ PX4 offboard command interface

### Phase 2: Computer Vision (Weeks 7-10)
1. ⬜ Survivor detection with YOLOv8
2. ⬜ Fire and smoke detection
3. ⬜ Integration with mission planner

### Phase 3: Mission Intelligence (Weeks 11-14)
1. ⬜ Autonomous search patterns
2. ⬜ Multi-target prioritization
3. ⬜ Recovery behaviors

### Phase 4: Testing & Validation (Weeks 15-18)
1. ⬜ SITL verification
2. ⬜ Hardware-in-the-loop testing
3. ⬜ Field testing

---

**Last Updated**: March 2026
**Version**: 2.1.0