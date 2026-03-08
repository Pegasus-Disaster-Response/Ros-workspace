# Pegasus ROS — Quick Reference & Development Roadmap

**Version:** 2.5.0
**Last Updated:** March 2026

---

## Changelog

### v2.5 (March 2026)
- **PX4 offboard interface** — `px4_offboard_node` bridges MPC trajectory setpoints to PX4 over XRCE-DDS. Handles ENU→NED coordinate conversion, OffboardControlMode heartbeat at 50 Hz, arm/disarm commands, and setpoint timeout safety. Same code works for both real flight and SITL.
- **SITL launch file** — `sitl_full.launch.py` launches the full autonomy stack against PX4 SITL in Gazebo Harmonic. Includes `auto_engage` and `auto_arm` arguments for automated SITL testing.
- **Offboard config** — `config/px4_offboard.yaml` for heartbeat rate, timeout, and auto-engage settings.
- **Updated full launch** — `pegasus_full.launch.py` now includes offboard node with `enable_offboard` argument.

### v2.4 (March 2026)
- D* Lite 3D local replanner (26-connected, full voxel grid)
- MPC trajectory smoother with placeholder VTOL dynamics
- VTOL dynamics config, 3D costmap grid publisher

### v2.3 (March 2026)
- Fixes: setup.py, odom clock, YAML namespacing, ICP params, QoS, RANSAC ground removal

### v2.2 (March 2026)
- A* global planner, SIL test infrastructure, Gazebo test world

### v2.1 / v2.0
- 3D costmap, RTAB-Map from source, dual odometry, IMU bridge

---

## Full Pipeline (v2.5)

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
cd ~/PX4-Autopilot
make px4_sitl gz_x500

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

---

## Workspace Structure

```
Ros-workspace/src/pegasus_ros/
├── setup.py
├── package.xml
├── COMPLETE_SUMMARY.md
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
│   ├── pegasus_full.launch.py      ← real flight (all sensors + PX4)
│   ├── pegasus_sensors.launch.py   ← sensor drivers only
│   ├── pegasus_slam.launch.py      ← SLAM + odometry
│   ├── local_costmap.launch.py     ← 3D costmap nodes
│   ├── path_planner.launch.py      ← A* standalone
│   ├── gazebo_planner_test.launch.py ← A* test (no hardware)
│   ├── sitl_full.launch.py         ← SITL (Gazebo + PX4 SITL)
│   └── vtol1_gazebo_bridge_launch.py
├── pegasus_autonomy/
│   ├── mission_planner_node.py
│   ├── px4_imu_bridge_node.py
│   ├── px4_state_subscriber_node.py
│   ├── px4_offboard_node.py        ← NEW: PX4 flight commands
│   ├── odometry_selector_node.py
│   ├── lidar_costmap_layer_node.py
│   ├── zed_depth_costmap_layer_node.py
│   ├── local_costmap_node.py
│   ├── global_planner_node.py      ← A*
│   ├── dstar_lite_node.py          ← D* Lite 3D
│   ├── mpc_trajectory_node.py      ← MPC smoother
│   ├── static_map_publisher_node.py
│   ├── static_odom_publisher_node.py
│   └── front_stereo_node.py
├── maps/
├── worlds/
└── test/
```

---

## Topic Map (v2.5)

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

---

## SITL Prerequisites

Before running SITL, install PX4 and Gazebo:

```bash
# Clone PX4
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# Build for SITL (first build takes ~10 min)
make px4_sitl gz_x500

# Install XRCE-DDS agent (if not already via snap)
cd ~
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/

# Install Gazebo Harmonic ROS bridge
sudo apt install ros-humble-ros-gzharmonic
```

---

## What Still Needs to Be Added

### Critical
- ⬜ **Simulated sensor plugins** — LiDAR + depth camera in Gazebo x500 model SDF
- ⬜ **Custom VTOL Gazebo model** — x500 is MC only; need standard_vtol for FW mode testing
- ⬜ **VTOL dynamics values** — replace placeholders in vtol_dynamics.yaml
- ⬜ **Obstacle avoidance node** — emergency reactive layer below MPC

### Computer Vision
- ⬜ Survivor detection (YOLOv8), fire/smoke detection

### Phase 1 Progress
1. ✅ SLAM (v2.0)
2. ✅ 3D costmap (v2.1)
3. ✅ A* planner (v2.2)
4. ✅ Bug fixes (v2.3)
5. ✅ D* Lite 3D + MPC (v2.4)
6. ✅ PX4 offboard interface + SITL launch (v2.5)
7. ⬜ Obstacle avoidance node
8. ⬜ Full SITL with simulated sensors

---

**Last Updated**: March 2026
**Version**: 2.5.0