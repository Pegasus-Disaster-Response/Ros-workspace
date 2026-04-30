# Pegasus Disaster Response UAV — Autonomous Navigation System

**Version:** 2.8.0
**Institution:** California Polytechnic State University, Pomona
**Team:** Pegasus
**Sponsor:** Lockheed Martin
**Competition:** GoAERO
**ROS 2 Humble | Gazebo Harmonic | RTAB-Map (from source) | PX4 via XRCE-DDS**

**Vehicle:** Quadplane eVTOL, ~50 kg / 110 lb, ~15 ft / 4.6 m wingspan
**Compute:** Jetson Orin AGX (Ubuntu 22.04, ARM64)
**Sensors:** ZED X (single, front-facing, neural depth) · Velodyne VLP-16 · Pixhawk Cube Orange (IMU)
**SLAM:** RTAB-Map (LiDAR ICP primary odometry + RGB-D visual backup + Pixhawk IMU fusion)
**Local Mapping:** 3D voxel costmap with dual-sensor fusion, RANSAC ground removal, speed-adaptive decay
**Planning:** A* global → D* Lite 3D local replanner → MPC trajectory smoother → PX4 offboard interface

---

## Full Autonomy Pipeline (v2.8)

```
  Goal ──→ A* (initial) ──→ D* Lite 3D (live) ──→ MPC ──→ PX4 Offboard ──→ Pixhawk ──→ Motors
              │                  │                  │          │
        global_path_initial   global_path        target_pose  XRCE-DDS
                                  ↑                  +
                                grid_map         twist (vz, yaw_rate)
```

A* publishes the *initial* plan; D* Lite publishes the *live* plan that re-routes around obstacles
seen in the local costmap. MPC consumes D*'s live path and emits a position lookahead pose plus a
yaw-rate / vz twist. The PX4 offboard node carrot-follows the path waypoints (continuous projection,
not chord-cutting) and sends NED `TrajectorySetpoint`s to PX4 at 50 Hz.

| Layer | Node | Rate | Input | Output |
|---|---|---|---|---|
| Global plan | global_planner_node | on goal/replan | OccupancyGrid | /pegasus/path_planner/global_path_initial |
| Local replan | dstar_lite_node | 5 Hz | 3D voxel grid (Int8MultiArray) | /pegasus/path_planner/global_path |
| Trajectory | mpc_trajectory_node | 20 Hz | global_path + /odom | /pegasus/trajectory/setpoint (Twist) + /pegasus/trajectory/target_pose (Pose) |
| Flight control | px4_offboard_node | 50 Hz | target_pose + global_path + vehicle_odometry | /fmu/in/trajectory_setpoint |

---

## Workspace Structure

```
Ros-workspace/
├── setup_workspace.sh                 ← run this first
├── COMPLETE_SUMMARY.md                ← quick reference & roadmap
├── maps/                              ← RTAB-Map database files (persistent)
│   └── pegasus_disaster_map.db        ← default map database
└── src/
    ├── pegasus_ros/                   ← custom package
    │   ├── config/
    │   │   ├── rtabmap.yaml           ← RTAB-Map SLAM (multi-sensor mode)
    │   │   ├── icp_odometry.yaml      ← ICP LiDAR odometry + IMU
    │   │   ├── rgbd_odometry.yaml     ← RGB-D visual odometry + IMU
    │   │   ├── vlp16.yaml             ← Velodyne VLP-16
    │   │   ├── zed_x.yaml            ← ZED X camera (v5.2 wrapper format)
    │   │   ├── local_costmap.yaml     ← 3D voxel costmap
    │   │   ├── path_planner.yaml      ← A* global planner
    │   │   ├── dstar_lite.yaml        ← D* Lite 3D replanner
    │   │   ├── vtol_dynamics.yaml     ← VTOL dynamics + MPC tuning
    │   │   ├── px4_offboard.yaml      ← PX4 offboard interface
    │   │   ├── rviz_slam.rviz
    │   │   ├── rviz_local_costmap.rviz
    │   │   └── rviz_planner_test.rviz
    │   ├── launch/
    │   │   ├── pegasus_full.launch.py          ← real flight (all sensors + full pipeline + PX4)
    │   │   ├── pegasus_sensors.launch.py       ← sensor drivers + XRCE-DDS + IMU bridge
    │   │   ├── pegasus_slam.launch.py          ← RTAB-Map SLAM + dual odometry
    │   │   ├── local_costmap.launch.py         ← 3D costmap (LiDAR + ZED layers + fusion)
    │   │   ├── path_planner.launch.py          ← A* standalone
    │   │   ├── gazebo_planner_test.launch.py   ← SIL test: A* on static map in RViz
    │   │   ├── sitl_full.launch.py             ← SITL (Gazebo + PX4 SITL + full pipeline)
    │   │   ├── p110_gazebo_bridge_launch.py    ← Gazebo Harmonic sensor bridge
    │   │   └── vtol1_gazebo_bridge_launch.py   ← legacy bridge
    │   ├── worlds/
    │   │   └── pegasus_planning_test.sdf       ← simple 100m planner test world
    │   ├── maps/
    │   │   ├── planning_test_map.pgm + .yaml
    │   │   └── dense_disaster_map.pgm + .yaml
    │   ├── pegasus_autonomy/
    │   │   ├── mission_planner_node.py            ← high-level mission logic + lawnmower auto-start
    │   │   ├── px4_imu_bridge_node.py             ← PX4 SensorCombined → sensor_msgs/Imu
    │   │   ├── px4_offboard_node.py               ← MPC + path → PX4 TrajectorySetpoint (carrot-on-path, ENU→NED)
    │   │   ├── px4_odom_bridge_node.py            ← PX4 vehicle_odometry → /odom_px4 (NED→ENU) for SITL
    │   │   ├── odometry_selector_node.py          ← fault-tolerant odom switching (NaN-safe)
    │   │   ├── lidar_costmap_layer_node.py        ← VLP-16 → obstacle points (RANSAC ground removal)
    │   │   ├── zed_depth_costmap_layer_node.py    ← ZED X depth → obstacle points
    │   │   ├── local_costmap_node.py              ← 3D voxel fusion + raycasting + inflation
    │   │   ├── global_planner_node.py             ← A* path planning on occupancy grids
    │   │   ├── dstar_lite_node.py                 ← D* Lite 3D (26-connected) local replanner
    │   │   ├── mpc_trajectory_node.py             ← MPC trajectory smoother (VTOL dynamics)
    │   │   ├── front_stereo_node.py               ← front camera processing (placeholder)
    │   │   ├── static_map_publisher_node.py       ← loads PGM map for SIL testing
    │   │   └── static_odom_publisher_node.py      ← fixed-position odom for SIL testing
    │   └── test/
    └── ros_gz/                        ← Gazebo ROS bridge (cloned)
```

---

## Quick Start

### 1. First-Time Setup

```bash
cd ~/Ros-workspace
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

### 2. ZED X Camera Workspace (one-time)

The ZED X requires a separate camera workspace built against the installed ZED SDK version. The wrapper must match the SDK exactly (e.g., SDK 5.2 requires the wrapper release_5.2 branch).

```bash
cd ~/camera_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-description.git zed_description
cd ~/camera_ws
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

Verify with `/usr/local/zed/tools/ZED_Explorer` that the SDK version matches the wrapper before building.

### 3. Mapping (sensors + SLAM, no planning)

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
ros2 topic hz /zed_x/zed_node/rgb/color/rect/image
ros2 topic hz /velodyne_points
ros2 topic hz /pegasus/imu/data
ros2 topic hz /odom
watch -n 5 ls -la ~/Ros-workspace/maps/pegasus_disaster_map.db
```

Move the vehicle around. RTAB-Map creates new nodes after 0.1 m linear or 0.05 rad angular movement. The database at `~/Ros-workspace/maps/pegasus_disaster_map.db` grows as you map.

### 4. Localization (reuse existing map)

```bash
# Terminal 1 — Sensors (same as mapping)
# Terminal 2 — SLAM in localization mode
ros2 launch pegasus_ros pegasus_slam.launch.py localization:=true
```

### 5. Real Flight (all sensors + full pipeline + PX4)

```bash
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_full.launch.py fcu_dev:=/dev/ttyUSB0
```

### 6. View an Existing Map (offline, no sensors needed)

```bash
rtabmap-databaseViewer ~/Ros-workspace/maps/pegasus_disaster_map.db
```

```bash
rtabmap-databaseViewer ~/Ros-workspace/maps/sitl_map.db
```

### 7. Start Fresh (delete existing map)

```bash
rm ~/Ros-workspace/maps/pegasus_disaster_map.dbitl

```bash
rm ~/Ros-workspace/maps/sitl_map.db
```
### 8. SITL (Gazebo + PX4) — full autonomous lawnmower test

`sitl_full.launch.py` brings up XRCE-DDS, PX4 SITL with Gazebo Harmonic, the sensor bridges, SLAM,
the costmap pipeline (with the LiDAR obstacle layer wired in), A*+D*, MPC, the PX4 offboard node,
and the mission planner. With `auto_arm:=true auto_engage:=true` the drone takes off and starts
flying a lawnmower search pattern by itself — no manual goal, arm, or mode switch needed.

```bash
cd ~/Ros-workspace && source install/setup.bash
ros2 launch pegasus_ros sitl_full.launch.py \
    gz_world:=p110_world \
    auto_arm:=true \
    auto_engage:=true \
    database_path:=$HOME/Ros-workspace/maps/sitl_map.db
```

**Boot sequence (~10–20 s of sim time on a Jetson):**

1. PX4 SITL + Gazebo come up (Gazebo runs headless `-s`; open the GUI client with `gz sim -g`).
2. XRCE-DDS bridge handshakes, sensor bridges (clock, lidar, IMU, camera, depth, odom) come online.
3. SLAM starts after a 4 s timer. The mission planner reaches `READY` once the px4_odom_bridge produces `/odom_px4`.
4. `px4_offboard_node` streams `OffboardControlMode` heartbeats at 50 Hz, then sends `ARM` and `DO_SET_MODE → OFFBOARD` and **retries every 0.3 s** until `/fmu/out/vehicle_status_v1` confirms.
5. Mission planner sees `arming_state == 2 && nav_state == NAVIGATION_STATE_OFFBOARD` → transitions `READY → SEARCH_PATTERN` and begins publishing waypoints.
6. A* publishes the initial path, D* refines and publishes the live path. MPC tracks. The drone climbs to `default_alt = 10 m` (altitude-priority before any lateral motion), then carrot-follows the lawnmower at ~3 m/s.

**Monitoring during a run (run each in its own terminal):**

```bash
# State machine + altitude + nav state
ros2 topic echo /pegasus/autonomy/mission_status

# Offboard handshake (armed, offboard_engaged, has_setpoint, setpoint_age_s)
ros2 topic echo /pegasus/offboard/status

# Did D* find a path?
ros2 topic hz /pegasus/path_planner/global_path

# Final NED setpoint going to PX4
ros2 topic echo /fmu/in/trajectory_setpoint --qos-reliability best_effort --once
```

**SITL launch arguments:**

| Arg | Default | Purpose |
|---|---|---|
| `gz_world` | `p110_world` | Gazebo world (must match `PX4_GZ_WORLD`) |
| `gz_model` | `p110_v2_0` | Gazebo model instance name |
| `auto_arm` | `false` | Auto-send arm command (SITL only — NEVER true on hardware) |
| `auto_engage` | `false` | Auto-switch PX4 to OFFBOARD mode (SITL only) |
| `database_path` | `~/Ros-workspace/maps/pegasus_disaster_map.db` | RTAB-Map DB file. Use a separate file for SITL to avoid mixing with real-flight maps. |

**SITL-specific things to know:**

- **Odometry source**: SITL overrides `primary_odom_topic` to `/odom_px4` (published by `px4_odom_bridge_node`, which converts `/fmu/out/vehicle_odometry` from NED → ENU). LiDAR ICP drifts ~10 m on z with the reduced 360-sample lidar on Jetson, and the gz `OdometryPublisher` plugin isn't auto-attached to the `p110_v2` model, so PX4's own EKF is the only ground-truth-quality source.
- **Reduced sensors**: the `p110_v2` LiDAR is 360 × 16 @ 5 Hz (was 1800 × 16 @ 10 Hz) and the camera is 640×480 @ 10 Hz (was 1920×1200 @ 30 Hz). These are SDF edits in `~/PX4-Simulation/Tools/simulation/gz/models/{p110_v2,better_cam}/model.sdf` — they raise Gazebo's real-time factor from ~0.05 to ~0.4 on a Jetson.
- **Stopping cleanly**: `Ctrl+C` in the launch terminal. The `make`-wrapped subprocess sometimes leaves zombies; if `pgrep -af 'gz sim|px4|MicroXRCE'` shows leftovers, run `pkill -9 -f 'px4_sitl_default|gz sim|MicroXRCEAgent|make px4_sitl'`.

**Database options (same as before):**

- Default DB: `~/Ros-workspace/maps/pegasus_disaster_map.db`. SITL will read/write the same file as real-sensor runs unless you override.
- Separate SITL DB: pass `database_path:=~/Ros-workspace/maps/sitl_map.db` (recommended).
- Fresh map: `rm ~/Ros-workspace/maps/sitl_map.db`, or `ros2 service call /rtabmap/reset std_srvs/srv/Empty` mid-run.


### 9. A* Planner Test (no hardware)

```bash
ros2 launch pegasus_ros gazebo_planner_test.launch.py
```

---

## Viewing in RViz2

RViz launches automatically with the SLAM launch file (`rviz:=true` by default). To configure displays manually, click "Add" in the Displays panel.

### SLAM Visualization

Set **Fixed Frame** to `map` in the Global Options panel.

| Display Type | Topic | What It Shows |
|---|---|---|
| Map | /rtabmap/grid_map | 2D occupancy grid from SLAM |
| PointCloud2 | /rtabmap/cloud_map | 3D accumulated point cloud |
| PointCloud2 | /velodyne_points | Live LiDAR scan |
| Image | /zed_x/zed_node/rgb/color/rect/image | Live RGB camera feed |
| Image | /zed_x/zed_node/depth/depth_registered | Live depth image |
| TF | (enable all) | Coordinate frame tree |

To add each display: click "Add" at the bottom of the Displays panel, select "By topic", find the topic in the tree, and click OK. For Image displays, expand the topic node and select "Image" (not "CompressedImage").

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

---

## System Architecture

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                        SENSOR LAYER                                 │
  │                                                                     │
  │  VLP-16 ──→ /velodyne_points     ZED X ──→ /zed_x/.../rect/image  │
  │                                          ──→ /zed_x/.../depth       │
  │  Pixhawk ──→ SensorCombined ──→ px4_imu_bridge ──→ /pegasus/imu   │
  └──────┬──────────────────────────────────┬───────────────────────────┘
         │                                  │
         ▼                                  ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    SLAM / ESTIMATION LAYER                          │
  │                                                                     │
  │  VLP-16 + IMU ──→ icp_odometry ──→ /odom_lidar (primary)          │
  │  ZED X + IMU  ──→ rgbd_odometry ──→ /odom_vision (backup)         │
  │  odometry_selector ──→ /odom (best available)                       │
  │  RTAB-Map ──→ /tf (map→odom), /rtabmap/grid_map, /rtabmap/cloud   │
  └──────┬──────────────────────────────────┬───────────────────────────┘
         │                                  │
         ▼                                  ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    LOCAL MAPPING LAYER                               │
  │                                                                     │
  │  lidar_costmap_layer ──┐                                            │
  │                        ├──→ local_costmap_node ──→ 3D voxel grid   │
  │  zed_depth_costmap  ───┘                                            │
  └──────┬──────────────────────────────────────────────────────────────┘
         │
         ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    PLANNING LAYER                                    │
  │                                                                     │
  │  A* (grid_map) ──→ D* Lite 3D (voxel grid) ──→ MPC ──→ PX4       │
  └─────────────────────────────────────────────────────────────────────┘
```

---

## TF Frame Tree

```
map
 └── odom                              (RTAB-Map)
      └── base_link                    (odometry_selector)
           ├── velodyne                (static TF: 0, 0, 0.25)
           ├── imu_link                (static TF: 0, 0, 0)
           └── zed_x_camera_link       (static TF: 0.46, 0, 0.084)
                └── zed_x_camera_center
                     ├── zed_x_left_camera_frame
                     │    └── zed_x_left_camera_frame_optical
                     └── zed_x_right_camera_frame
                          └── zed_x_right_camera_frame_optical
```

The static TF from `base_link` targets `zed_x_camera_link` (the ZED URDF root frame). The wrapper's robot_state_publisher then publishes the internal camera chain. RTAB-Map reads RGB-D data from `zed_x_left_camera_frame_optical`.

---

## Topic Map

### Sensors
| Topic | Type | Rate | Source |
|---|---|---|---|
| /velodyne_points | PointCloud2 | ~10 Hz | VLP-16 |
| /zed_x/zed_node/rgb/color/rect/image | Image | ~15 Hz | ZED X |
| /zed_x/zed_node/rgb/color/rect/camera_info | CameraInfo | ~30 Hz | ZED X |
| /zed_x/zed_node/depth/depth_registered | Image | ~15 Hz | ZED X |
| /pegasus/imu/data | Imu | 50 Hz | px4_imu_bridge |

### Odometry
| Topic | Source | Used as |
|---|---|---|
| /odom_lidar | ICP LiDAR odometry (`pegasus_slam.launch.py`) | Primary on hardware |
| /odom_vision | RGB-D visual odometry | Backup (currently disabled) |
| /odom_px4 | `px4_odom_bridge_node` (PX4 NED → ENU) | Primary in SITL |
| /odom | `odometry_selector_node` | Final published `/odom` (selected source) |

The `primary_odom_topic` is a launch arg on `pegasus_slam.launch.py` (default `/odom_lidar`). SITL
overrides it to `/odom_px4`.

### Planning Pipeline
| Topic | Type | Rate | Source |
|---|---|---|---|
| /pegasus/autonomy/target_waypoint | PoseStamped | 2 Hz | mission_planner_node |
| /pegasus/path_planner/global_path_initial | Path | on replan | A* (`global_planner_node`) |
| /pegasus/path_planner/global_path | Path | ~5 Hz | D* Lite (live, replans on costmap change) |
| /pegasus/trajectory/setpoint | TwistStamped | 20 Hz | MPC (linear.x = body fwd, linear.z = vz, angular.z = yaw_rate) |
| /pegasus/trajectory/target_pose | PoseStamped | 20 Hz | MPC (path lookahead point, ENU) |
| /pegasus/autonomy/mission_status | String | 1 Hz | mission_planner_node (state + arm + nav + pos) |

### PX4 Offboard
| Topic | Type | Rate | Notes |
|---|---|---|---|
| /fmu/in/trajectory_setpoint | TrajectorySetpoint | 50 Hz | NED, position-mode (carrot-on-path) + velocity FF + yaw |
| /fmu/in/offboard_control_mode | OffboardControlMode | 50 Hz | Heartbeat (PX4 drops OFFBOARD without it) |
| /fmu/in/vehicle_command | VehicleCommand | retry 0.3 Hz | ARM (400) + DO_SET_MODE (176) — sent until vehicle_status_v1 confirms |
| /fmu/out/vehicle_status_v1 | VehicleStatus | best-effort | Real arm/nav state (PX4 v1.15+ versioned topic) |
| /fmu/out/vehicle_odometry | VehicleOdometry | best-effort | NED pose used by px4_odom_bridge + offboard yaw extraction |
| /fmu/out/vehicle_command_ack | VehicleCommandAck | per command | PX4 response for arm/mode commands |

### Control
| Topic | Type | Purpose |
|---|---|---|
| /pegasus/offboard/arm | Bool | Arm/disarm |
| /pegasus/offboard/engage | Bool | Enable/disable offboard mode |
| /pegasus/offboard/status | String (JSON) | Status |

### Costmap
| Topic | Type | Rate |
|---|---|---|
| /pegasus/local_costmap | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_inflated | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_3d_grid | Int8MultiArray | 5 Hz |
| /pegasus/local_costmap_2d | OccupancyGrid | 10 Hz |
| /pegasus/costmap_metadata | String (JSON) | 1 Hz |
| /pegasus/sensor_status | String | 2 Hz |

---

## Hardware Configuration

### Pixhawk Connection (Jetson Orin AGX)
- **Port:** TELEM2 via YP-05 USB-to-serial adapter at `/dev/ttyUSB0`, 921600 baud
- **PX4 params:** `MAV_1_CONFIG=0`, `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600`
- **Hardware launch:** `ros2 launch pegasus_ros pegasus_sensors.launch.py fcu_dev:=/dev/ttyUSB0`
- **SITL:** `MicroXRCEAgent udp4 -p 8888`

### ZED X Camera
- **Connection:** GMSL2 via Stereolabs capture card
- **Serial:** 40709032
- **SDK:** 5.2.0 (wrapper must be built from matching branch)
- **Workspace:** `~/camera_ws` (separate from ROS workspace)
- **Required packages:** `zed-ros2-wrapper` + `zed-ros2-description`
- **Verify:** `/usr/local/zed/tools/ZED_Explorer`

### Velodyne VLP-16
- **Connection:** Ethernet, IP 192.168.1.201
- **Host must be on:** 192.168.1.x subnet (e.g., 192.168.1.100)

### First-Flight Strategy
1. QGroundControl GPS waypoint flight with passive SLAM mapping (builds `~/Ros-workspace/maps/pegasus_disaster_map.db`)
2. Subsequent flights use `localization:=true` to reuse the saved map
3. Launch argument swap only, no code changes needed

---

## Fault Tolerance

- **Odometry:** LiDAR ICP primary → Visual RGB-D backup → Hold last good TF (never publishes NaN)
- **IMU dependency:** Both ICP and RGB-D odometry use `wait_imu_to_init: true` to prevent SIGABRT on uninitialized gravity vector. If the Pixhawk is not connected or XRCE-DDS is not running, odometry will not start.
- **Planning:** D* Lite on live costmap from sensor startup; A* on SLAM map after ~40% coverage
- **Costmap:** base_link frame, no TF/SLAM dependency, speed-adaptive decay (~3 m ghost cap)
- **Sensors:** Dual-layer costmap degrades gracefully; sensor_status reports mode

---

## Build

```bash
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
colcon build --packages-select pegasus_ros
source install/setup.bash
```

Always source the camera workspace first so the ZED wrapper packages are found during build and runtime.

---

## Changelog

### v2.7 (March 2026)
- **ZED X wrapper compatibility** — Rebuilt ZED ROS 2 wrapper against SDK 5.2.0 (was built for 5.1.0, causing `enable_right_side_measure` crash and gravity alignment stall). Cloned separate `zed_description` package required by the new wrapper version.
- **ZED X config rewrite** — `zed_x.yaml` rewritten with correct nested parameter structure (`general.*`, `depth.*`, `pos_tracking.*`) matching the v5.2 wrapper. Previously used flat parameter names that were silently ignored by the wrapper.
- **ZED X topic names** — Updated all topic references from `/zed_x/zed_node/rgb/image_rect_color` to `/zed_x/zed_node/rgb/color/rect/image` and from `/zed_x/zed_node/rgb/camera_info` to `/zed_x/zed_node/rgb/color/rect/camera_info` across `pegasus_slam.launch.py`, `p110_gazebo_bridge_launch.py`, and `zed_depth_costmap_layer_node.py`.
- **TF tree fix** — Static TF target changed from `zed_x_camera_center` to `zed_x_camera_link` (the ZED URDF root frame) with Z offset adjusted from 0.10 to 0.084 m. Previously created two disconnected TF trees, preventing RTAB-Map from resolving `base_link → zed_x_left_camera_frame_optical`.
- **Multi-sensor SLAM** — Re-enabled `subscribe_rgb`, `subscribe_depth`, `subscribe_imu` in `rtabmap.yaml`; `Grid/Sensor` restored to `2` (both LiDAR and depth camera). `wait_imu_to_init` set to `true` in all three config files (`rtabmap.yaml`, `icp_odometry.yaml`, `rgbd_odometry.yaml`) to prevent the SIGABRT crash on uninitialized gravity vector that caused v2.6.3 to disable all IMU and camera subscriptions.
- **Database persistence confirmed** — Map database at `~/Ros-workspace/maps/pegasus_disaster_map.db` verified saving and growing during multi-sensor SLAM sessions.

### v2.6 (March 2026)
- Disaster response simulation world (`disaster_response.sdf`): 200 m world with 5 zones
- setup.py critical fixes, SDF BOM fix

### v2.5
- PX4 offboard interface node, SITL launch, offboard config

### v2.4
- D* Lite 3D (26-connected), MPC trajectory smoother, VTOL dynamics config

### v2.3
- Fixes: setup.py, odom clock, YAML namespaces, ICP params, QoS, RANSAC ground removal

### v2.2
- A* global planner, SIL test infrastructure, Gazebo test world

### v2.0–v2.1
- 3D costmap, RTAB-Map from source, dual odometry, IMU bridge

---

**Last Updated:** March 2026