# Pegasus Disaster Response UAV — Autonomous Navigation System

**Version:** 2.3.0
**Institution:** California Polytechnic State University, Pomona
**Team:** Pegasus
**Sponsor:** Lockheed Martin
**ROS 2 Humble | RTAB-Map (from source) | PX4 via XRCE-DDS**

**Sensors:** ZED X (single, front-facing) · Velodyne VLP-16 · Pixhawk Cube Orange (IMU)
**SLAM:** RTAB-Map (LiDAR ICP odometry + RGB-D loop closure + IMU fusion)
**Local Mapping:** 3D voxel costmap with dual-sensor fusion + RANSAC ground removal + degraded mode handling
**Path Planning:** A* global planner with static map SIL testing

---

## Workspace Structure

```
Ros-workspace/
├── setup_workspace.sh              ← run this first
├── COMPLETE_SUMMARY.md             ← quick reference & roadmap
├── LOCAL_COSTMAP_GUIDE.md          ← 3D costmap testing & dependencies
└── src/
    ├── pegasus_ros/                ← custom package (configs + launches + nodes)
    │   ├── config/
    │   │   ├── rtabmap.yaml        ← RTAB-Map SLAM tuning parameters
    │   │   ├── icp_odometry.yaml   ← ICP LiDAR odometry config (v2.3)
    │   │   ├── rgbd_odometry.yaml  ← RGB-D visual odometry config (v2.3)
    │   │   ├── vlp16.yaml          ← Velodyne point cloud conversion settings
    │   │   ├── zed_x.yaml          ← ZED X camera configuration
    │   │   ├── rviz_slam.rviz      ← RViz display configuration (SLAM)
    │   │   ├── local_costmap.yaml  ← 3D costmap parameters (voxel grid, fusion, inflation)
    │   │   ├── rviz_local_costmap.rviz  ← RViz display configuration (costmap)
    │   │   ├── path_planner.yaml   ← A* global planner parameters
    │   │   └── rviz_planner_test.rviz   ← RViz display configuration (planner SIL test)
    │   ├── launch/
    │   │   ├── pegasus_full.launch.py        ← complete system (sensors + SLAM + costmap + planner)
    │   │   ├── pegasus_sensors.launch.py     ← sensor drivers + XRCE-DDS + IMU bridge
    │   │   ├── pegasus_slam.launch.py        ← RTAB-Map SLAM + dual odometry + static TFs
    │   │   ├── local_costmap.launch.py       ← 3D costmap (LiDAR layer + ZED layer + fusion)
    │   │   ├── path_planner.launch.py        ← A* global planner (standalone)
    │   │   ├── gazebo_planner_test.launch.py ← SIL test: A* on static map in RViz
    │   │   └── vtol1_gazebo_bridge_launch.py ← simulation bridge
    │   ├── worlds/
    │   │   └── pegasus_planning_test.sdf     ← Gazebo world with disaster obstacles
    │   ├── maps/
    │   │   ├── planning_test_map.pgm         ← ground-truth occupancy grid (from SDF)
    │   │   └── planning_test_map.yaml        ← map metadata (resolution, origin)
    │   └── pegasus_autonomy/
    │       ├── mission_planner_node.py           ← high-level mission logic
    │       ├── px4_imu_bridge_node.py            ← PX4 SensorCombined → sensor_msgs/Imu
    │       ├── front_stereo_node.py              ← front camera processing
    │       ├── px4_state_subscriber_node.py      ← PX4 state monitoring
    │       ├── odometry_selector_node.py         ← selects best odometry source
    │       ├── lidar_costmap_layer_node.py       ← VLP-16 → 3D obstacle points
    │       ├── zed_depth_costmap_layer_node.py   ← ZED X depth → 3D obstacle points
    │       ├── local_costmap_node.py             ← 3D voxel fusion + publishing
    │       ├── global_planner_node.py            ← A* path planning on occupancy grids
    │       ├── static_map_publisher_node.py      ← loads PGM map for SIL testing
    │       └── static_odom_publisher_node.py     ← fixed-position odom for SIL testing
    ├── rtabmap/                    ← cloned from source (0.23.x — apt 0.22.1 is too old)
    ├── rtabmap_ros/                ← cloned from source (0.23.x)
    └── zed-ros2-wrapper/           ← cloned from Stereolabs
```

---

## Quick Start

### 1. First-Time Setup

```bash
# Run the setup script (installs deps, clones packages, builds)
cd ~/Ros-workspace
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

### 2. Launch Full System

```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

This launches: ZED X camera driver, VLP-16 LiDAR driver, XRCE-DDS agent (Pixhawk), PX4 IMU bridge, ICP odometry, RTAB-Map SLAM, static TFs, 3D local costmap, A* global planner, mission planner, RViz.

### 3. Launch Individual Subsystems

```bash
# Sensors only
ros2 launch pegasus_ros pegasus_sensors.launch.py

# SLAM only (sensors must be running)
ros2 launch pegasus_ros pegasus_slam.launch.py

# 3D local costmap only (sensors + SLAM must be running)
ros2 launch pegasus_ros local_costmap.launch.py

# A* global planner only (costmap or static map must be running)
ros2 launch pegasus_ros path_planner.launch.py

# 3D costmap with only one sensor layer enabled
ros2 launch pegasus_ros local_costmap.launch.py enable_lidar:=false
ros2 launch pegasus_ros local_costmap.launch.py enable_zed:=false
```

### 4. Test A* Planner in RViz (No Hardware Needed)

```bash
# Terminal 1: Clean build and launch
cd ~/Ros-workspace
rm -rf build/pegasus_ros install/pegasus_ros log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pegasus_ros
source install/setup.bash
ros2 launch pegasus_ros gazebo_planner_test.launch.py

# Terminal 2: Send a goal waypoint
cd ~/Ros-workspace && source install/setup.bash
ros2 topic pub --once /pegasus/autonomy/target_waypoint \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 35.0, y: -25.0, z: 10.0}}}"
```

Or use the "2D Goal Pose" button in RViz to click goals directly on the map.

With Gazebo Harmonic installed:
```bash
# Terminal 1: Start Gazebo
gz sim ~/Ros-workspace/src/pegasus_ros/worlds/pegasus_planning_test.sdf

# Terminal 2: Launch with Gazebo bridge
ros2 launch pegasus_ros gazebo_planner_test.launch.py use_gazebo:=true use_sim_time:=true
```

### 5. Launch with Hardware Disabled

```bash
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_zed:=false
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_lidar:=false
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_xrce:=false

# Dry run — no hardware at all
ros2 launch pegasus_ros pegasus_sensors.launch.py \
    enable_zed:=false enable_lidar:=false enable_xrce:=false enable_imu_bridge:=false
```

### 6. SLAM Operations

```bash
# Start mapping (create new map)
ros2 launch pegasus_ros pegasus_slam.launch.py

# Use existing map (localization mode)
ros2 launch pegasus_ros pegasus_slam.launch.py \
    localization:=true database_path:=/path/to/map.db

# Launch without RViz
ros2 launch pegasus_ros pegasus_slam.launch.py rviz:=false
```

---

## System Architecture

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                        SENSOR LAYER                                 │
  │                                                                     │
  │  VLP-16 ──→ /velodyne_points     ZED X ──→ /zed_x/.../depth       │
  │  Pixhawk ──→ SensorCombined ──→ px4_imu_bridge ──→ /pegasus/imu   │
  └──────┬──────────────────────────────────┬───────────────────────────┘
         │                                  │
         ▼                                  ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    SLAM / ESTIMATION LAYER                          │
  │                                                                     │
  │  VLP-16 ──→ icp_odometry ──→ /odom_lidar ──┐                       │
  │  ZED X RGB-D ──→ rgbd_odometry ──→ /odom_vision ──┤                │
  │                                   odometry_selector ──→ /odom      │
  │  RTAB-Map ← /odom + RGB-D + LiDAR + IMU ──→ /tf (map→odom)       │
  └──────┬──────────────────────────────────┬───────────────────────────┘
         │                                  │
         ▼                                  ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    LOCAL MAPPING LAYER                               │
  │                                                                     │
  │  /velodyne_points ──→ [lidar_costmap_layer] ──→ /pegasus/lidar_obstacles
  │  /zed_x/.../depth ──→ [zed_depth_costmap_layer] ──→ /pegasus/zed_obstacles
  │                                  │                                  │
  │                                  ▼                                  │
  │                  [local_costmap_node] (3D voxel grid fusion)        │
  │                          │         │          │          │          │
  │                          ▼         ▼          ▼          ▼          │
  │                  /local_costmap  /markers  /costmap_2d  /sensor_status
  └──────┬──────────────────────────────────────────────────────────────┘
         │
         ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                    PATH PLANNING LAYER                              │
  │                                                                     │
  │  [global_planner_node] ← /rtabmap/grid_map OR /costmap_2d         │
  │         │                ← /odom (current position)                │
  │         │                ← /pegasus/autonomy/target_waypoint       │
  │         ▼                                                          │
  │  /pegasus/path_planner/global_path (nav_msgs/Path)                 │
  │         │                                                          │
  │         ▼                                                          │
  │  D* Lite local replanner (planned) ──→ MPC smoothing (planned)     │
  │         │                                                          │
  │         ▼                                                          │
  │  PX4 Offboard Interface (planned) ──→ /fmu/in/trajectory_setpoint  │
  └─────────────────────────────────────────────────────────────────────┘
```

---

## v2.3 Architecture Details

### A* Global Planner (v2.2 — NEW)

The global planner runs weighted A* search on a 2D occupancy grid to compute an initial obstacle-free path from the UAV's current position to a goal waypoint.

The planner subscribes to two grid sources: the RTAB-Map SLAM grid (`/rtabmap/grid_map`) for missions with a built map, and the local costmap 2D projection (`/pegasus/local_costmap_2d`) for real-time obstacle awareness. The `prefer_local_costmap` parameter controls which source is preferred when both are available.

Key features:
- **Weighted A* with 8-connected movement** — octile distance heuristic, configurable epsilon for speed vs optimality tradeoff
- **Inflation-aware cost penalties** — cells in the costmap inflation zone are penalized proportionally, pushing paths away from obstacles (tunable via `cost_penalty_factor`)
- **Unknown cell handling** — unknown cells are traversable with a mild penalty, allowing planning through unexplored areas
- **Start cell recovery** — if the UAV position lands on an inflated cell, the planner spirals outward to find the nearest free cell
- **Automatic replanning** — triggers when the UAV drifts beyond `path_deviation_threshold_m` from the planned path
- **Goal monitoring** — detects when the UAV reaches the goal within `goal_tolerance_m`
- **Altitude constraints** — path waypoints are clamped between `min_altitude_m` and `max_altitude_m`
- **Status reporting** — publishes JSON status on `/pegasus/path_planner/status`

Configuration: `config/path_planner.yaml`

### SIL Test Infrastructure (v2.2 — NEW)

For testing the planner without any hardware, SLAM, or sensor pipeline:

- **static_map_publisher_node** — loads a PGM+YAML map (standard ROS map format) and publishes it as `OccupancyGrid` with transient-local QoS. The included `planning_test_map` is auto-generated from the Gazebo world file with 1m inflation around obstacles.
- **static_odom_publisher_node** — publishes a fixed-position `Odometry` message representing the UAV at a configurable start location. For isolated planner testing only — does not interfere with `odometry_selector_node` since they are launched in separate launch configurations.
- **gazebo_planner_test.launch.py** — minimal launch that starts only the static map publisher, static odom, global planner, TF, and RViz. The `ros_gz_bridge` clock bridge is conditionally included via `use_gazebo:=true` for when Gazebo Harmonic is installed.
- **pegasus_planning_test.sdf** — Gazebo world with buildings, walls, debris, and cylindrical rubble in a 100×100m area for realistic disaster-scenario testing.

### 3D Local Costmap (v2.1)

Three standalone nodes process sensor data into a rolling 3D voxel grid centered on the UAV:

- **lidar_costmap_layer_node** — VLP-16 point cloud → RANSAC ground plane removal → height band filter → voxel downsample → obstacle points
- **zed_depth_costmap_layer_node** — ZED X depth image → deproject to 3D → range filter → height band filter → obstacle points
- **local_costmap_node** — Fuses both obstacle streams into a 3D voxel grid (40×40×20m at 0.3m resolution). Handles obstacle decay, 3D inflation via distance transform, sensor health monitoring, and publishes both 3D markers (RViz) and a 2D occupancy grid projection.

Sensor health monitoring detects degraded modes automatically:
- **Nominal** — both sensors active, full fusion
- **LiDAR only** — 360° coverage, increased safety margins (1.5×)
- **Camera only** — forward cone only, speed restricted, RTL requested after 30s
- **All degraded** — emergency loiter, RTL after 60s

### SLAM (v2.0)

v2.0 uses **LiDAR ICP odometry** from the VLP-16 as the primary odometry source, with **RGB-D visual odometry** as a backup. The `odometry_selector_node` monitors both sources and publishes the best available one to `/odom`. Each odometry node loads its own dedicated YAML config (`icp_odometry.yaml`, `rgbd_odometry.yaml`) so that subscription parameters are correctly namespaced.

```
VLP-16 → /velodyne_points → icp_odometry → /odom_lidar ──┐
                                                           ├→ odometry_selector → /odom
ZED X → rgb + depth → rgbd_odometry → /odom_vision ──────┘        ↓
                                                            RTAB-Map SLAM
ZED X → rgb + depth ────────────────────────────────────→ (loop closure + mapping)
                                                                   ↑
Pixhawk → SensorCombined → px4_imu_bridge → /pegasus/imu/data ────┘
```

### IMU Bridge

PX4's `SensorCombined` message is not a standard `sensor_msgs/Imu`. The `px4_imu_bridge_node` converts PX4 IMU data (FRD frame) to standard ROS IMU messages (FLU frame) for RTAB-Map consumption.

### Static TF Tree

All sensor transforms are explicitly published:
```
map
 └── odom (published by rtabmap — map→odom)
      └── base_link (published by odometry_selector — odom→base_link)
           ├── velodyne             [0.52, 0.0, 0.85]  ← UPDATE to your mount
           ├── zed_x_camera_center  [1.78, 0.0, 0.55]  ← UPDATE to your mount
           └── imu_link             [0.0, 0.0, 0.0]    ← UPDATE to your mount
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
    ros-humble-rtabmap ros-humble-rtabmap-ros \
    ros-humble-rtabmap-slam ros-humble-rtabmap-odom \
    ros-humble-rtabmap-util ros-humble-rtabmap-viz \
    ros-humble-rtabmap-msgs ros-humble-rtabmap-rviz-plugins \
    ros-humble-rtabmap-conversions ros-humble-rtabmap-sync \
    ros-humble-rtabmap-launch ros-humble-rtabmap-examples \
    ros-humble-rtabmap-demos ros-humble-rtabmap-python
```

### Step 2: Install Dependencies

```bash
sudo apt install -y \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher ros-humble-velodyne \
    ros-humble-rviz2 python3-pip

pip install numpy scipy Pillow pyyaml --break-system-packages
```

### Step 3: Install XRCE-DDS Agent

```bash
sudo snap install micro-xrce-dds-agent --edge
```

### Step 4: Build PX4 Messages workspace

```bash
mkdir -p ~/px4_ws/src
cd ~/px4_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/px4_ws
colcon build --symlink-install
echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc
source ~/px4_ws/install/setup.bash
```

### Step 5: Clone source packages

```bash
cd ~/Ros-workspace/src

# RTAB-Map core (from source for 0.23.x)
git clone https://github.com/introlab/rtabmap.git

# RTAB-Map ROS 2 wrapper (from source)
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

# ZED ROS 2 wrapper
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
```

### Step 6: Build

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

### RTAB-Map Version

```bash
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"
# Expected: RTAB-Map: 0.23.x (NOT 0.22.1)

# Verify no apt version remains
dpkg -l | grep rtabmap
# Should return nothing
```

### Workspace Packages

```bash
ros2 pkg list | grep -E "(pegasus|rtabmap|velodyne|zed)"
# Expected: pegasus_ros, rtabmap_slam, rtabmap_odom, rtabmap_util,
#           rtabmap_viz, rtabmap_msgs, velodyne_driver,
#           velodyne_pointcloud, zed_wrapper
```

### Node Executables

```bash
ros2 pkg executables pegasus_ros
# Expected:
#   pegasus_ros mission_planner_node
#   pegasus_ros front_stereo_node
#   pegasus_ros px4_state_subscriber_node
#   pegasus_ros px4_imu_bridge_node
#   pegasus_ros odometry_selector_node
#   pegasus_ros lidar_costmap_layer_node
#   pegasus_ros zed_depth_costmap_layer_node
#   pegasus_ros local_costmap_node
#   pegasus_ros global_planner_node
#   pegasus_ros static_map_publisher_node
#   pegasus_ros static_odom_publisher_node
```

### Python Dependencies

```bash
python3 -c "import rclpy; import numpy; from scipy.ndimage import distance_transform_edt; from PIL import Image; print('All OK')"
```

### TF Tree

```bash
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → velodyne / zed_x_camera_center / imu_link
```

### XRCE-DDS Agent

```bash
micro-xrce-dds-agent --help
# Should show usage info (installed via snap)
```

---

## System Verification (with sensors running)

### Check Active Nodes

```bash
ros2 node list
# Expected:
#   /icp_odometry
#   /mission_planner_node
#   /px4_imu_bridge_node
#   /px4_state_subscriber_node
#   /odometry_selector_node
#   /rtabmap/rtabmap
#   /velodyne_driver
#   /velodyne_convert
#   /lidar_costmap_layer
#   /zed_depth_costmap_layer
#   /local_costmap_node
#   /global_planner_node
```

### Monitor Data Rates

```bash
ros2 topic hz /odom                                      # ICP odometry (~10 Hz)
ros2 topic hz /velodyne_points                            # LiDAR (~10 Hz)
ros2 topic hz /zed_x/zed_node/rgb/image_rect_color       # Camera (~30 Hz)
ros2 topic hz /pegasus/imu/data                           # Bridged IMU (~50 Hz)
ros2 topic hz /pegasus/local_costmap                      # 3D costmap (~10 Hz)
ros2 topic hz /pegasus/lidar_obstacles                    # LiDAR layer (~10 Hz)
ros2 topic hz /pegasus/zed_obstacles                      # ZED layer (~15-30 Hz)
ros2 topic hz /pegasus/path_planner/global_path           # Path updates (on demand)
```

### Check Sensor Health

```bash
ros2 topic echo /pegasus/sensor_status
# Expected: "nominal" when both sensors are active

ros2 topic echo /pegasus/lidar_health
ros2 topic echo /pegasus/zed_health
```

### Check Planner Status

```bash
ros2 topic echo /pegasus/path_planner/status
# Expected: JSON with state, grid availability, path length
```

### Check Costmap Topics

```bash
ros2 topic list | grep pegasus
# Expected:
#   /pegasus/imu/data
#   /pegasus/lidar_obstacles
#   /pegasus/lidar_health
#   /pegasus/zed_obstacles
#   /pegasus/zed_health
#   /pegasus/local_costmap
#   /pegasus/local_costmap_inflated
#   /pegasus/local_costmap_markers
#   /pegasus/local_costmap_2d
#   /pegasus/sensor_status
#   /pegasus/costmap_metadata
#   /pegasus/path_planner/global_path
#   /pegasus/path_planner/status
#   /pegasus/autonomy/target_waypoint
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
| `config/rtabmap.yaml` | SLAM tuning (RTAB-Map node only) | ICP settings, grid resolution, loop closure, gravity alignment |
| `config/icp_odometry.yaml` | ICP LiDAR odometry (primary) | subscribe_scan_cloud, ICP voxel/correspondence, odom strategy |
| `config/rgbd_odometry.yaml` | RGB-D visual odometry (backup) | subscribe_rgb/depth, feature type, F2M size |
| `config/zed_x.yaml` | Camera settings | Serial number, depth mode, resolution, frame rate |
| `config/vlp16.yaml` | LiDAR settings | IP address, min/max range, calibration |
| `config/local_costmap.yaml` | 3D costmap | Voxel grid size/resolution, sensor ranges, inflation, decay, RANSAC ground removal, degraded modes |
| `config/path_planner.yaml` | A* global planner | Heuristic weight, diagonal movement, cost penalty, lethal threshold, altitude limits, replan frequency |

### Editing Parameters

Edit YAML files directly — no rebuild needed (with `--symlink-install`):
```bash
nano src/pegasus_ros/config/path_planner.yaml
# Changes take effect on next launch
```

### A* Tuning Guide

| Parameter | Effect | Suggested Range |
|---|---|---|
| `heuristic_weight` | 1.0 = optimal, higher = faster but suboptimal | 1.0–2.0 |
| `cost_penalty_factor` | How aggressively paths avoid inflation zones | 2.0–10.0 |
| `lethal_cost_threshold` | Cells at or above this value are impassable | 80–100 |
| `max_iterations` | Hard cap on node expansions | 10000–100000 |
| `goal_tolerance_m` | 3D distance to consider goal reached | 1.0–5.0 |
| `path_deviation_threshold_m` | Drift distance that triggers replanning | 3.0–10.0 |

To push paths further from obstacles, increase `cost_penalty_factor` (e.g., from 2.0 to 5.0 or 8.0).

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| RTAB-Map: "Database version mismatch" | apt 0.22.1 still installed | `dpkg -l \| grep rtabmap` and remove all apt packages |
| RTAB-Map returns nothing on `--version` | Not built from source yet | Clone + build rtabmap and rtabmap_ros from source |
| No `/velodyne_points` topic | LiDAR not connected or wrong IP | Check `config/vlp16.yaml`, verify with `ping 192.168.1.201` |
| No depth topic from ZED | Wrong serial number or SDK issue | Run `/usr/local/zed/tools/ZED_Explorer` to verify camera |
| TF: "Could not find transform" | Static TFs not running | Verify SLAM launch is running, check `ros2 run tf2_tools view_frames` |
| Costmap empty in RViz | TF or no sensor data | Check `ros2 topic hz /pegasus/lidar_obstacles` and `/pegasus/zed_obstacles` |
| `/pegasus/sensor_status` = "all_degraded" | No sensor data arriving | Check `ros2 topic hz /velodyne_points` and ZED depth topic |
| High costmap latency (>50ms) | Resolution too fine | Increase `costmap.resolution_m` in local_costmap.yaml |
| scipy import error | Not installed | `pip install scipy --break-system-packages` |
| Planner: "No occupancy grid available" | No map publishing | Check `ros2 topic hz /rtabmap/grid_map` or `/pegasus/local_costmap_2d` |
| Planner: "Start or goal outside grid" | Goal coordinates beyond map bounds | Map covers -50m to +50m; use coordinates within that range |
| A* path hugs obstacles | `cost_penalty_factor` too low | Increase from 2.0 to 5.0+ in `path_planner.yaml` |
| Map not visible in RViz | QoS mismatch | Set Map display Durability to "Transient Local" |
| `ros_gz_bridge` not found | Gazebo Harmonic not installed | Launch with default `use_gazebo:=false` (uses wall clock) |
| Planner test: no path after goal | Planner not subscribed yet | Wait for "Global Planner: A*..." log before sending goal |

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

**Last Updated**: March 2026
**Version**: 2.3.0