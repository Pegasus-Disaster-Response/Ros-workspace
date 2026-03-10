# Pegasus Disaster Response UAV — Autonomous Navigation System

**Version:** 2.6.0
**Institution:** California Polytechnic State University, Pomona
**Team:** Pegasus
**Sponsor:** Lockheed Martin
**Competition:** GoAERO
**ROS 2 Humble | Gazebo Harmonic | RTAB-Map (from source) | PX4 via XRCE-DDS**

**Vehicle:** Quadplane eVTOL, ~50 kg / 110 lb, ~15 ft / 4.6 m wingspan
**Compute:** Jetson Orin AGX
**Sensors:** ZED X (single, front-facing, neural depth) · Velodyne VLP-16 · Pixhawk Cube Orange + ADS-B
**SLAM:** RTAB-Map (LiDAR ICP primary odometry + RGB-D visual backup + IMU fusion)
**Local Mapping:** 3D voxel costmap with dual-sensor fusion, RANSAC ground removal, speed-adaptive decay
**Planning:** A* global → D* Lite 3D local replanner → MPC trajectory smoother → PX4 offboard interface

---

## Full Autonomy Pipeline (v2.6)

```
  Goal ──→ A* ──→ D* Lite 3D ──→ MPC ──→ PX4 Offboard ──→ Pixhawk ──→ Motors
            │         │            │           │
         grid_map  costmap_3d    /odom    XRCE-DDS
```

| Layer | Node | Rate | Input | Output |
|---|---|---|---|---|
| Global plan | global_planner_node | on goal/replan | OccupancyGrid | /pegasus/path_planner/global_path |
| Local replan | dstar_lite_node | 5 Hz | 3D voxel grid (Int8MultiArray) | /pegasus/path_planner/local_path |
| Trajectory | mpc_trajectory_node | 50 Hz | /local_path + /odom | /pegasus/trajectory/setpoint |
| Flight control | px4_offboard_node | 50 Hz | /trajectory/setpoint | /fmu/in/trajectory_setpoint |

---

## Workspace Structure

```
Ros-workspace/
├── setup_workspace.sh                 ← run this first
├── COMPLETE_SUMMARY.md                ← quick reference & roadmap
└── src/
    ├── pegasus_ros/                   ← custom package
    │   ├── config/
    │   │   ├── rtabmap.yaml           ← RTAB-Map SLAM tuning
    │   │   ├── icp_odometry.yaml      ← ICP LiDAR odometry
    │   │   ├── rgbd_odometry.yaml     ← RGB-D visual odometry
    │   │   ├── vlp16.yaml             ← Velodyne VLP-16
    │   │   ├── zed_x.yaml            ← ZED X camera
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
    │   │   ├── mission_planner_node.py            ← high-level mission logic
    │   │   ├── px4_imu_bridge_node.py             ← PX4 SensorCombined → sensor_msgs/Imu
    │   │   ├── px4_offboard_node.py               ← MPC → PX4 TrajectorySetpoint (ENU→NED)
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

### 2. Real Flight (all sensors + Pixhawk)

```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

### 3. SITL (Gazebo + PX4)

```bash
# Terminal 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Full stack
cd ~/Ros-workspace && source install/setup.bash
ros2 launch pegasus_ros sitl_full.launch.py

# Terminal 4: Send goal + arm
ros2 topic pub --once /pegasus/autonomy/target_waypoint \
    geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 10.0}}}"
ros2 topic pub --once /pegasus/offboard/arm std_msgs/Bool "{data: true}"
ros2 topic pub --once /pegasus/offboard/engage std_msgs/Bool "{data: true}"
```

### 4. Disaster World (standalone Gazebo)

```bash
gz sim ~/Ros-workspace/src/pegasus_ros/worlds/disaster_response.sdf
```

### 5. A* Planner Test (no hardware)

```bash
ros2 launch pegasus_ros gazebo_planner_test.launch.py
```

---

## Simulation Worlds

### disaster_response.sdf (200 m × 200 m)

Primary simulation world with five challenge zones:

- **Zone 1 — Collapsed Downtown** (NE): Tilted buildings, fallen slabs, narrow gaps
- **Zone 2 — Rubble Field** (NW): Dense scattered debris, leaning poles
- **Zone 3 — Residential Corridor** (SW): Narrow alleyways, chicanes, overhead beams, dead ends
- **Zone 4 — Dead-End Traps** (SE): U-shaped cul-de-sacs, L-shaped turns for backtracking
- **Zone 5 — Urban Canyon** (center-south): Tall tower rows, skybridge, fallen section

Includes VTOL model at origin with simulated VLP-16 LiDAR (360°, 16-channel, 100 m, 10 Hz) and ZED X RGBD camera (105° HFOV, 672×376, 0.3–20 m depth, 15 Hz).

### pegasus_planning_test.sdf (100 m × 100 m)

Simple world for A* planner testing. No vehicle model — used with static_map_publisher_node.

---

## Topic Map

### Planning Pipeline
| Topic | Type | Rate | Source |
|---|---|---|---|
| /pegasus/path_planner/global_path | Path | on replan | A* |
| /pegasus/path_planner/local_path | Path | ~5 Hz | D* Lite 3D |
| /pegasus/trajectory/setpoint | PoseStamped | 50 Hz | MPC |
| /pegasus/trajectory/velocity_setpoint | TwistStamped | 50 Hz | MPC |

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
| /pegasus/offboard/engage | Bool | Enable/disable offboard mode |
| /pegasus/offboard/status | String (JSON) | Status |

### Costmap
| Topic | Type | Rate |
|---|---|---|
| /pegasus/local_costmap | PointCloud2 | 10 Hz |
| /pegasus/local_costmap_3d_grid | Int8MultiArray | 5 Hz |
| /pegasus/local_costmap_2d | OccupancyGrid | 10 Hz |
| /pegasus/costmap_metadata | String (JSON) | 1 Hz |
| /pegasus/sensor_status | String | 2 Hz |

### Odometry
| Topic | Type | Source |
|---|---|---|
| /odom_lidar | Odometry | ICP (primary) |
| /odom_vision | Odometry | RGB-D (backup) |
| /odom | Odometry | Best available (selector) |

---

## Hardware Configuration

### Pixhawk Connection (Jetson Orin AGX)
- **Port:** TELEM2 UART at 921600 baud on `/dev/ttyTHS1`
- **Connector:** JST-GH 6-pin, 3.3V TTL, no level shifter
- **PX4 params:** `MAV_1_CONFIG=0`, `UXRCE_DDS_CFG=102`
- **SITL:** `MicroXRCEAgent udp4 -p 8888`
- **Hardware:** `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600`

### First-Flight Strategy
1. QGroundControl GPS waypoint flight with passive SLAM mapping → builds `~/.ros/rtabmap.db`
2. Subsequent flights use `localization:=true` to reuse the map
3. Launch argument swap only, no code changes needed

---

## Fault Tolerance

- **Odometry:** LiDAR ICP primary → Visual RGB-D backup → Hold last good TF (never NaN)
- **Planning:** D* Lite on live costmap from sensor startup; A* on SLAM map after ~40% coverage
- **Costmap:** base_link frame, no TF/SLAM dependency, speed-adaptive decay (~3 m ghost cap)
- **Sensors:** Dual-layer costmap degrades gracefully; sensor_status reports mode

---

## Build

```bash
cd ~/Ros-workspace
colcon build --packages-select pegasus_ros --symlink-install
source install/setup.bash
```

---

## Changelog

### v2.6 (March 2026)
- **Disaster response world** — `disaster_response.sdf`: 200 m world with 5 zones, VTOL + VLP-16 + ZED X sensors, full Gazebo Harmonic plugins
- **setup.py fixes** — Removed dead `px4_state_subscriber_node`, added missing entry points (`dstar_lite_node`, `mpc_trajectory_node`, `px4_offboard_node`), added missing configs and launch files to data_files
- **SDF BOM fix** — Removed UTF-8 BOM from SDF

### v2.5
- PX4 offboard interface node, SITL launch, offboard config

### v2.4
- D* Lite 3D (26-connected), MPC trajectory smoother, 3D grid publisher, VTOL dynamics config

### v2.3
- Fixes: setup.py, odom clock, YAML namespaces, ICP params, QoS, RANSAC ground removal

### v2.2
- A* global planner, SIL test infrastructure, Gazebo test world

### v2.0–v2.1
- 3D costmap, RTAB-Map from source, dual odometry, IMU bridge

---

**Last Updated:** March 2026