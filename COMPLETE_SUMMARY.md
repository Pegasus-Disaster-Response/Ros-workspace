# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.3.0
**Last Updated:** March 2026

---

## Changelog

### v2.3 (March 2026)
- **Fix #1: setup.py** — added explicit `package_dir` mapping for `pegasus_autonomy` module discovery; version bump to 2.3.0
- **Fix #2: Odometry selector clock** — replaced `time.time()` (wall clock) with `self.get_clock()` (ROS clock) so timeout logic works correctly when `use_sim_time=true`
- **Fix #3: Odometry YAML namespacing** — created dedicated `icp_odometry.yaml` and `rgbd_odometry.yaml` config files so subscription parameters (`subscribe_scan_cloud`, `subscribe_rgb`, etc.) are under the correct node name. Previously all params were under `rtabmap:` in `rtabmap.yaml`, which doesn't match the odometry node names — so the nodes never actually subscribed to sensor data.
- **Fix #4: Full launch missing costmap** — `pegasus_full.launch.py` now includes `local_costmap.launch.py`; added `enable_costmap` argument
- **Fix #5: ICP correspondence distance** — increased `Icp/MaxCorrespondenceDistance` from 0.15 to 1.0. With `Icp/VoxelSize=0.3`, voxel centers are at least 0.3m apart, so the old value (0.15) rejected most valid point correspondences.
- **Fix #6: ZED camera_info QoS** — changed `zed_depth_costmap_layer_node` camera_info subscriber from RELIABLE to BEST_EFFORT QoS. The ZED wrapper publishes BEST_EFFORT; a RELIABLE subscriber silently fails to receive, so camera intrinsics never arrived and depth processing never started.
- **Fix #7: RANSAC ground removal** — replaced naive z-threshold ground removal in `lidar_costmap_layer_node` with RANSAC plane fitting. The old method (`points[:, 2] > threshold`) assumed level flight — during pitch/roll the ground plane tilts in base_link frame, causing false obstacles or missed ground removal. New params: `ransac_iterations`, `candidate_z_band_m`.

### v2.2 (March 2026)
- **A* global planner** — weighted A* on 2D occupancy grids with inflation-aware cost penalties, automatic replanning on path deviation, altitude constraints, and JSON status reporting
- **SIL test infrastructure** — static map publisher (PGM+YAML), static odom publisher, dedicated test launch file, RViz config with 2D Goal Pose tool
- **Gazebo test world** — 100×100m disaster environment with buildings, walls, debris, and cylindrical rubble (`pegasus_planning_test.sdf`)
- **Ground-truth map** — auto-generated occupancy grid from the Gazebo world with 1m obstacle inflation (`planning_test_map.pgm`)
- **Conditional Gazebo bridge** — `gazebo_planner_test.launch.py` works without `ros_gz_bridge` installed; enable with `use_gazebo:=true`
- **Updated `pegasus_full.launch.py`** — global planner node included in full system launch
- **Pillow** added as dependency for map loading

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
# Sensors → SLAM → Costmap → Planner
ros2 launch pegasus_ros pegasus_sensors.launch.py
ros2 launch pegasus_ros pegasus_slam.launch.py
ros2 launch pegasus_ros local_costmap.launch.py
ros2 launch pegasus_ros path_planner.launch.py
```

### Test A* Planner (No Hardware)
```bash
# Terminal 1: Launch planner test
cd ~/Ros-workspace
rm -rf build/pegasus_ros install/pegasus_ros log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pegasus_ros
source install/setup.bash
ros2 launch pegasus_ros gazebo_planner_test.launch.py

# Terminal 2: Send goal
cd ~/Ros-workspace && source install/setup.bash
ros2 topic pub --once /pegasus/autonomy/target_waypoint \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 35.0, y: -25.0, z: 10.0}}}"

# Or use RViz "2D Goal Pose" button to click goals on the map
```

### Launch Costmap with One Sensor Disabled
```bash
ros2 launch pegasus_ros local_costmap.launch.py enable_lidar:=false
ros2 launch pegasus_ros local_costmap.launch.py enable_zed:=false
```

### Launch Full System Without Costmap
```bash
ros2 launch pegasus_ros pegasus_full.launch.py enable_costmap:=false
```

### Run Individual Nodes
```bash
ros2 run pegasus_ros mission_planner_node
ros2 run pegasus_ros px4_imu_bridge_node
ros2 run pegasus_ros odometry_selector_node
ros2 run pegasus_ros lidar_costmap_layer_node
ros2 run pegasus_ros zed_depth_costmap_layer_node
ros2 run pegasus_ros local_costmap_node
ros2 run pegasus_ros global_planner_node
ros2 run pegasus_ros static_map_publisher_node --ros-args -p map_yaml_path:=/path/to/map.yaml
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
| `/odom_lidar` | Odometry | ~10 Hz | icp_odometry (primary) |
| `/odom_vision` | Odometry | ~10 Hz | rgbd_odometry (backup) |
| `/odom` | Odometry | ~10 Hz | odometry_selector (fused output) |
| `/odom_status` | String | ~10 Hz | odometry_selector |
| `/rtabmap/grid_map` | OccupancyGrid | ~1 Hz | RTAB-Map |
| `/rtabmap/cloud_map` | PointCloud2 | ~1 Hz | RTAB-Map |
| `/tf` (map→odom→base_link) | TF | continuous | RTAB-Map + odometry_selector + static publishers |

### 3D Local Costmap Topics
| Topic | Type | Rate | Source |
|---|---|---|---|
| `/pegasus/lidar_obstacles` | PointCloud2 | ~10 Hz | lidar_costmap_layer |
| `/pegasus/lidar_origin` | PointStamped | ~10 Hz | lidar_costmap_layer |
| `/pegasus/lidar_health` | Bool | ~4 Hz | lidar_costmap_layer |
| `/pegasus/zed_obstacles` | PointCloud2 | ~15-30 Hz | zed_depth_costmap_layer |
| `/pegasus/zed_origin` | PointStamped | ~15-30 Hz | zed_depth_costmap_layer |
| `/pegasus/zed_health` | Bool | ~4 Hz | zed_depth_costmap_layer |
| `/pegasus/local_costmap` | PointCloud2 | ~10 Hz | local_costmap_node |
| `/pegasus/local_costmap_inflated` | PointCloud2 | ~10 Hz | local_costmap_node |
| `/pegasus/local_costmap_markers` | MarkerArray | ~10 Hz | local_costmap_node |
| `/pegasus/local_costmap_2d` | OccupancyGrid | ~10 Hz | local_costmap_node |
| `/pegasus/sensor_status` | String | ~2 Hz | local_costmap_node |
| `/pegasus/costmap_metadata` | String (JSON) | ~1 Hz | local_costmap_node |

### Path Planning Topics
| Topic | Type | Rate | Source |
|---|---|---|---|
| `/pegasus/autonomy/target_waypoint` | PoseStamped | on demand | mission planner / RViz / CLI |
| `/pegasus/path_planner/global_path` | Path | on replan | global_planner_node |
| `/pegasus/path_planner/status` | String (JSON) | ~1 Hz | global_planner_node |
| `/pegasus/path_planner/replan_request` | Bool | on demand | external trigger |

---

## Configuration Files

| File | Purpose | Key Parameters |
|---|---|---|
| `config/rtabmap.yaml` | SLAM tuning (RTAB-Map node only) | ICP settings, grid resolution, loop closure, gravity alignment |
| `config/icp_odometry.yaml` | ICP LiDAR odometry (primary) | subscribe_scan_cloud, ICP voxel size, correspondence distance |
| `config/rgbd_odometry.yaml` | RGB-D visual odometry (backup) | subscribe_rgb/depth, feature type, F2M size |
| `config/zed_x.yaml` | Camera settings | Serial number, depth mode, resolution, frame rate |
| `config/vlp16.yaml` | LiDAR settings | IP address, min/max range, calibration |
| `config/local_costmap.yaml` | 3D costmap | Voxel resolution, grid size, sensor ranges, inflation radius, decay rate, RANSAC ground removal, degraded mode thresholds |
| `config/path_planner.yaml` | A* planner | Heuristic weight, diagonal movement, cost penalty factor, lethal threshold, altitude limits, replan frequency, goal tolerance |

Edit YAML files directly — no rebuild needed (with `--symlink-install`):
```bash
nano src/pegasus_ros/config/rtabmap.yaml
# Changes take effect on next launch
```

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

### A* Planner Test Visualization
```bash
rviz2 -d $(ros2 pkg prefix pegasus_ros)/share/pegasus_ros/config/rviz_planner_test.rviz
```

Costmap marker colors in RViz:
- **Red cubes** — obstacle seen by LiDAR only
- **Blue cubes** — obstacle seen by ZED X only
- **Green cubes** — obstacle confirmed by both sensors
- **Alpha gradient** — higher voxels are more opaque

Planner test displays:
- **Grey/white grid** — free space on the static map
- **Black shapes** — obstacles (buildings, walls, debris)
- **Green line** — A* planned path
- **Axes marker** — UAV position (base_link frame)

---

## What Still Needs to Be Added

### Critical Missing Components

**D* Lite Local Replanner**
- Status: Not implemented
- Architecture: Takes A* global path as seed, incrementally replans when costmap changes
- Inputs: `/pegasus/path_planner/global_path`, `/pegasus/local_costmap_2d` (live updates)
- Outputs: locally adjusted path for obstacle avoidance node

**MPC Trajectory Smoother**
- Status: Not implemented
- Architecture: Converts D* Lite grid path into kinodynamically feasible trajectory
- Constraints: minimum turn radius, max climb angle, speed envelope, UAV wingspan (15 ft)
- Outputs: smooth trajectory for PX4 offboard interface

**Obstacle Avoidance Node**
- Status: Not implemented
- Will monitor D* Lite path + MPC predicted trajectory against live 3D costmap
- Feeds costmap change notifications back to D* Lite
- Emergency bypass to PX4 for immediate collision avoidance

**Flight Controller Interface (Offboard Commands)**
- Status: Partially implemented (monitoring only)
- Requires publishing to: `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, `/fmu/in/offboard_control_mode`
- Needs offboard mode manager (arm, switch to offboard, heartbeat at >2 Hz)

**PX4 Mission Waypoint Bridge**
- Status: Not implemented
- Reads QGroundControl mission waypoints from PX4 XRCE-DDS topics
- Converts PX4 NED coordinates to ROS ENU/SLAM map frame
- Republishes as PoseStamped on `/pegasus/autonomy/target_waypoint`

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
7. ✅ A* global path planning (v2.2)
8. ✅ SIL test infrastructure — static map + RViz (v2.2)
9. ✅ Bug fixes: odom YAML namespacing, ICP params, QoS, ground removal, full launch (v2.3)
10. ⬜ D* Lite local replanner
11. ⬜ MPC trajectory smoothing
12. ⬜ Obstacle avoidance node
13. ⬜ PX4 offboard command interface

### Phase 2: Computer Vision (Weeks 7-10)
1. ⬜ Survivor detection with YOLOv8
2. ⬜ Fire and smoke detection
3. ⬜ Integration with mission planner

### Phase 3: Mission Intelligence (Weeks 11-14)
1. ⬜ Autonomous search patterns
2. ⬜ Multi-target prioritization
3. ⬜ Recovery behaviors

### Phase 4: Testing & Validation (Weeks 15-18)
1. ⬜ Full SITL verification with Gazebo + PX4
2. ⬜ Hardware-in-the-loop testing
3. ⬜ Field testing

---

**Last Updated**: March 2026
**Version**: 2.3.0