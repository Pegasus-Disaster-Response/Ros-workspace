# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.7.0
**Last Updated:** March 2026

---

## Changelog

### v2.7 (March 2026)
- **ZED X wrapper rebuild** — Rebuilt the ZED ROS 2 wrapper in `~/camera_ws` against ZED SDK 5.2.0. The previous wrapper was built for SDK 5.1.0, which caused an `enable_right_side_measure` crash and a gravity alignment stall that prevented the camera from publishing any frames. Also cloned the new `zed-ros2-description` package that the v5.2 wrapper requires as a separate dependency.
- **ZED X config rewrite** — `zed_x.yaml` completely rewritten to use the correct nested YAML parameter structure (`general.*`, `depth.*`, `pos_tracking.*`, `sensors.*`, `mapping.*`) matching the wrapper's `common_stereo.yaml` format. The previous config used flat parameter names at the top level of `ros__parameters` which the wrapper silently ignored, resulting in no overrides taking effect.
- **ZED X topic name update** — The v5.2 wrapper publishes RGB on `/zed_x/zed_node/rgb/color/rect/image` (was `/rgb/image_rect_color`) and camera info on `/zed_x/zed_node/rgb/color/rect/camera_info` (was `/rgb/camera_info`). Updated topic references in `pegasus_slam.launch.py`, `p110_gazebo_bridge_launch.py`, `zed_depth_costmap_layer_node.py`, and `local_costmap.yaml`.
- **TF tree fix** — Static TF from `base_link` now targets `zed_x_camera_link` (the ZED URDF root frame) instead of `zed_x_camera_center`. Z offset adjusted from 0.10 m to 0.084 m to account for the 16 mm offset between `zed_x_camera_link` and `zed_x_camera_center` in the URDF. Previously the TF tree was split into two disconnected trees, causing RTAB-Map to fail with "Could not find a connection between base_link and zed_x_left_camera_frame_optical".
- **Full multi-sensor SLAM** — Re-enabled `subscribe_rgb: true`, `subscribe_depth: true`, `subscribe_imu: true` in `rtabmap.yaml`. Changed `Grid/Sensor` from `0` (LiDAR only) back to `2` (both LiDAR and depth camera). Re-enabled `Mem/UseOdomGravity: true`. Set `wait_imu_to_init: true` in all three config files (`rtabmap.yaml`, `icp_odometry.yaml`, `rgbd_odometry.yaml`). The `wait_imu_to_init: true` setting is the critical fix: in v2.6.3, `subscribe_imu` was set to `true` but `wait_imu_to_init` was `false`, causing a SIGABRT when ICP tried to use an uninitialized gravity vector. This crash led to all IMU and camera subscriptions being disabled as a workaround.
- **Database persistence** — Confirmed that the RTAB-Map database at `~/Ros-workspace/maps/pegasus_disaster_map.db` saves and grows during multi-sensor SLAM mapping sessions. The `--delete_db_on_start` fix from v2.6.2 remains in place.

### v2.6 (March 2026)
- Disaster response simulation world (`disaster_response.sdf`): 200 m × 200 m, 5 challenge zones, VTOL model with simulated VLP-16 + ZED X + IMU
- setup.py critical fixes (dead entry points, missing configs, missing launch files)
- SDF BOM fix

### v2.5 (March 2026)
- PX4 offboard interface (`px4_offboard_node`), SITL launch, offboard config

### v2.4 (March 2026)
- D* Lite 3D local replanner (26-connected, incremental), MPC trajectory smoother, VTOL dynamics config

### v2.3 (March 2026)
- Fixes: setup.py, odom clock, YAML namespace mismatch, ICP params, ZED QoS, RANSAC ground removal

### v2.2 (March 2026)
- A* global planner, static map publisher, SIL test infrastructure, Gazebo test world

### v2.1 / v2.0
- 3D voxel costmap with dual-sensor fusion, RTAB-Map from source (v0.23.x), dual odometry, IMU bridge

---

## Full Pipeline (v2.7)

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

### Mapping (sensors + SLAM)

```bash
# Terminal 1 — Sensors
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_sensors.launch.py fcu_dev:=/dev/ttyUSB0

# Terminal 2 — SLAM
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_slam.launch.py

# Terminal 3 — Monitor
ros2 topic hz /zed_x/zed_node/rgb/color/rect/image   # ~15 Hz
ros2 topic hz /velodyne_points                         # ~10 Hz
ros2 topic hz /pegasus/imu/data                        # ~50 Hz
ros2 topic hz /odom                                    # should match LiDAR rate
watch -n 5 ls -la ~/Ros-workspace/maps/pegasus_disaster_map.db
```

### Localization (reuse existing map)

```bash
# Terminal 1 — Sensors (same as above)
# Terminal 2 — SLAM in localization mode
ros2 launch pegasus_ros pegasus_slam.launch.py localization:=true
```

### Real Flight (full pipeline)

```bash
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_full.launch.py fcu_dev:=/dev/ttyUSB0
```

### View Existing Map (offline)

```bash
rtabmap-databaseViewer ~/Ros-workspace/maps/pegasus_disaster_map.db
```

### Start Fresh

```bash
rm ~/Ros-workspace/maps/pegasus_disaster_map.db
```

### SITL (Gazebo + PX4)

```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: XRCE-DDS agent (UDP for SITL)
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Full autonomy stack
cd ~/Ros-workspace && source install/setup.bash
ros2 launch pegasus_ros sitl_full.launch.py

# Terminal 4: Send a goal, arm, and engage
ros2 topic pub --once /pegasus/autonomy/target_waypoint \
    geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 10.0}}}"
ros2 topic pub --once /pegasus/offboard/arm std_msgs/Bool "{data: true}"
ros2 topic pub --once /pegasus/offboard/engage std_msgs/Bool "{data: true}"
```

### A* Planner Test (no hardware)

```bash
ros2 launch pegasus_ros gazebo_planner_test.launch.py
```

---

## Viewing in RViz2

RViz launches automatically with the SLAM launch. To add displays, click "Add" at the bottom of the Displays panel and choose "By topic" to browse available data streams.

### SLAM Visualization

Set **Fixed Frame** to `map` (top of Displays panel, under Global Options).

| Display Type | Topic | What It Shows |
|---|---|---|
| Map | /rtabmap/grid_map | 2D occupancy grid from SLAM |
| PointCloud2 | /rtabmap/cloud_map | 3D accumulated point cloud |
| PointCloud2 | /velodyne_points | Live LiDAR scan |
| Image | /zed_x/zed_node/rgb/color/rect/image | Live RGB camera feed |
| Image | /zed_x/zed_node/depth/depth_registered | Live depth image |
| TF | (enable all) | Full coordinate frame tree |

### Local Costmap Visualization

Set **Fixed Frame** to `base_link`.

| Display Type | Topic | What It Shows |
|---|---|---|
| PointCloud2 | /pegasus/local_costmap | Raw 3D obstacle voxels |
| PointCloud2 | /pegasus/local_costmap_inflated | Inflated safety-margin voxels |
| OccupancyGrid | /pegasus/local_costmap_2d | 2D projection of the costmap |

### Planning Visualization

Set **Fixed Frame** to `map`.

| Display Type | Topic | What It Shows |
|---|---|---|
| Path | /pegasus/path_planner/global_path | A* global path |
| Path | /pegasus/path_planner/local_path | D* Lite local path |
| Path | /pegasus/trajectory/predicted | MPC predicted trajectory |

### Adding an Image Display

Click "Add", then choose "By display type" (not "By topic"). Select "Image", click OK. In the new Image display properties, set the **Image Topic** to the desired topic. This creates a small image window inside RViz.

---

## TF Frame Tree

```
map
 └── odom                              (RTAB-Map)
      └── base_link                    (odometry_selector)
           ├── velodyne                (static: 0, 0, 0.25)
           ├── imu_link                (static: 0, 0, 0)
           └── zed_x_camera_link       (static: 0.46, 0, 0.084)
                └── zed_x_camera_center
                     ├── zed_x_left_camera_frame
                     │    └── zed_x_left_camera_frame_optical
                     └── zed_x_right_camera_frame
                          └── zed_x_right_camera_frame_optical
```

---

## Workspace Structure

```
Ros-workspace/src/pegasus_ros/
├── setup.py                      (v2.7.0)
├── package.xml
├── config/
│   ├── rtabmap.yaml                ← multi-sensor SLAM (LiDAR + ZED + IMU)
│   ├── icp_odometry.yaml           ← ICP LiDAR odometry + IMU gravity
│   ├── rgbd_odometry.yaml          ← RGB-D visual odometry + IMU gravity
│   ├── vlp16.yaml                  ← Velodyne VLP-16
│   ├── zed_x.yaml                  ← ZED X camera (v5.2 nested param format)
│   ├── local_costmap.yaml          ← 3D voxel costmap
│   ├── path_planner.yaml           ← A* global planner
│   ├── dstar_lite.yaml             ← D* Lite 3D replanner
│   ├── vtol_dynamics.yaml          ← VTOL dynamics + MPC tuning
│   ├── px4_offboard.yaml           ← PX4 offboard interface
│   ├── rviz_slam.rviz
│   ├── rviz_local_costmap.rviz
│   └── rviz_planner_test.rviz
├── launch/
│   ├── pegasus_full.launch.py
│   ├── pegasus_sensors.launch.py
│   ├── pegasus_slam.launch.py
│   ├── local_costmap.launch.py
│   ├── path_planner.launch.py
│   ├── gazebo_planner_test.launch.py
│   ├── sitl_full.launch.py
│   ├── p110_gazebo_bridge_launch.py
│   └── vtol1_gazebo_bridge_launch.py
├── pegasus_autonomy/
│   ├── mission_planner_node.py
│   ├── px4_imu_bridge_node.py
│   ├── px4_offboard_node.py
│   ├── odometry_selector_node.py
│   ├── lidar_costmap_layer_node.py
│   ├── zed_depth_costmap_layer_node.py
│   ├── local_costmap_node.py
│   ├── global_planner_node.py
│   ├── dstar_lite_node.py
│   ├── mpc_trajectory_node.py
│   ├── front_stereo_node.py
│   ├── static_map_publisher_node.py
│   └── static_odom_publisher_node.py
├── worlds/
├── maps/
└── test/
```

---

## Topic Map (v2.7)

### Sensors
| Topic | Type | Rate | Source |
|---|---|---|---|
| /velodyne_points | PointCloud2 | ~10 Hz | VLP-16 |
| /zed_x/zed_node/rgb/color/rect/image | Image | ~15 Hz | ZED X |
| /zed_x/zed_node/rgb/color/rect/camera_info | CameraInfo | ~30 Hz | ZED X |
| /zed_x/zed_node/depth/depth_registered | Image | ~15 Hz | ZED X |
| /pegasus/imu/data | Imu | 50 Hz | px4_imu_bridge |

### Odometry
| Topic | Source | Priority |
|---|---|---|
| /odom_lidar | ICP LiDAR odometry | Primary |
| /odom_vision | RGB-D visual odometry | Backup |
| /odom | odometry_selector_node | Best available |

### Planning
| Topic | Type | Rate | Source |
|---|---|---|---|
| /pegasus/path_planner/global_path | Path | on replan | A* |
| /pegasus/path_planner/local_path | Path | ~5 Hz | D* Lite 3D |
| /pegasus/trajectory/setpoint | PoseStamped | 50 Hz | MPC |
| /pegasus/trajectory/predicted | Path | 50 Hz | MPC |

### PX4 Offboard
| Topic | Type | Rate |
|---|---|---|
| /fmu/in/trajectory_setpoint | TrajectorySetpoint | 50 Hz |
| /fmu/in/offboard_control_mode | OffboardControlMode | 50 Hz |
| /fmu/in/vehicle_command | VehicleCommand | on demand |

### Control
| Topic | Type | Purpose |
|---|---|---|
| /pegasus/offboard/arm | Bool | Arm/disarm |
| /pegasus/offboard/engage | Bool | Enable/disable offboard |
| /pegasus/offboard/status | String (JSON) | Status |

### Costmap
| Topic | Type | Rate |
|---|---|---|
| /pegasus/local_costmap | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_inflated | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_3d_grid | Int8MultiArray | 5 Hz |
| /pegasus/local_costmap_2d | OccupancyGrid | 10 Hz |
| /pegasus/sensor_status | String | 2 Hz |

---

## Hardware Setup

### Pixhawk Cube Orange
- **Serial:** TELEM2 → YP-05 USB-to-serial → `/dev/ttyUSB0` at 921600 baud
- **PX4 params:** `MAV_1_CONFIG=0`, `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600`

### ZED X Camera
- **GMSL2** via Stereolabs ZED Link Duo capture card
- **Serial:** 40709032
- **SDK:** 5.2.0
- **Workspace:** `~/camera_ws` (separate, contains `zed-ros2-wrapper` + `zed_description`)

### Velodyne VLP-16
- **Ethernet:** IP 192.168.1.201, host must be on 192.168.1.x subnet

---

## What Still Needs to Be Added

### Critical
- ⬜ **VTOL dynamics values** — replace placeholders in vtol_dynamics.yaml with real aircraft data
- ⬜ **Obstacle avoidance node** — emergency reactive layer below MPC
- ⬜ **Mission-level safety rules** — speed limiting, battery budget, dead-end escape, progress monitoring

### Simulation
- ⬜ **Custom PX4 VTOL model** — current SITL uses x500 (MC only)
- ⬜ **Gazebo sensor bridge for disaster_response world**

### Computer Vision
- ⬜ Survivor detection (YOLOv8), fire/smoke detection

### Testing
- ⬜ Parametric tests, bag replay, hardware-in-the-loop

### Phase 1 Progress
1. ✅ SLAM (v2.0)
2. ✅ 3D costmap (v2.1)
3. ✅ A* planner (v2.2)
4. ✅ Bug fixes (v2.3)
5. ✅ D* Lite 3D + MPC (v2.4)
6. ✅ PX4 offboard + SITL (v2.5)
7. ✅ Disaster response world (v2.6)
8. ✅ Multi-sensor SLAM + ZED X + database persistence (v2.7)
9. ⬜ Obstacle avoidance node
10. ⬜ Full SITL with custom VTOL + sensor bridges

---

**Last Updated:** March 2026
**Version:** 2.7.0