# Gazebo SITL Quickstart

End-to-end runbook for the Pegasus SITL stack: PX4 + Gazebo Harmonic + the full autonomy pipeline.
The drone arms itself, takes off, flies a lawnmower search pattern with obstacle avoidance, and
returns to base — no manual goal, arm, or mode switch needed.

For deeper details on the autonomy chain, see the v2.8 changelog in [README.md](README.md).

---

## 1. Launch

```bash
cd ~/Ros-workspace && source install/setup.bash

ros2 launch pegasus_ros sitl_full.launch.py \
    gz_world:=p110_world \
    auto_arm:=true \
    auto_engage:=true \
    database_path:=$HOME/Ros-workspace/maps/sitl_map.db
```

That single command starts:

| Order | Component |
|---|---|
| immediately | XRCE-DDS agent (UDP 8888) |
| immediately | QGroundControl (Flatpak) — optional, monitor only |
| immediately | PX4 SITL + Gazebo Harmonic (`gz sim` runs **headless** with `-s`) |
| immediately | Sensor bridges (clock, lidar, IMU, camera, depth, odometry, `px4_odom_bridge`) |
| +4 s | RTAB-Map SLAM (ICP odometry + selector + rtabmap) |
| +0 s | Costmap pipeline (`lidar_costmap_layer_node` → `local_costmap_node`) |
| +0 s | A* (initial) + D* Lite (live) + MPC + offboard + mission planner |
| +5 s | RViz (config: `rviz_planner_test.rviz`) |

To open the Gazebo GUI client (the server runs headless), in another terminal:

```bash
gz sim -g
```

---

## 2. Database Path Conventions

The default RTAB-Map database is shared with real-hardware runs:
`~/Ros-workspace/maps/pegasus_disaster_map.db`. **Don't use that for SITL** — it'll mix simulated
and real-flight maps. Always pass an explicit `database_path`:

```bash
database_path:=$HOME/Ros-workspace/maps/sitl_map.db          # everyday SITL test
database_path:=$HOME/Ros-workspace/maps/sitl_p110world.db    # per-world DB if you switch worlds
database_path:=$HOME/Ros-workspace/maps/sitl_$(date +%F).db  # per-day DB if you want history
```

- Same launch arg both creates the file (first run) and continues mapping into it (subsequent runs).
- The map only flushes to disk on **clean shutdown** (`Ctrl+C`, not `kill -9`). The launch terminal must terminate cleanly.
- To force a flush mid-run without stopping: `ros2 service call /rtabmap/backup std_srvs/srv/Empty`

---

## 3. Viewing the Map

### 3a. RTAB-Map's built-in viewer (recommended for quick inspection)

```bash
rtabmap-databaseViewer ~/Ros-workspace/maps/sitl_map.db
```

Loads the entire map graph including loop closures, scans, IMU traces, and per-node statistics.
Lighter than CloudCompare and runs fine on Jetson.

### 3b. Export to CloudCompare (for cleanup, mesh generation, or screenshots)

RTAB-Map's `.db` is a SQLite container, not a point cloud. Export to PLY first:

```bash
# RTAB-Map locks the live DB during a run, so snapshot a copy
cp ~/Ros-workspace/maps/sitl_map.db /tmp/sitl_map_snapshot.db

# Export — use --scan because this is a LiDAR-only SLAM (no depth/stereo data)
rtabmap-export \
    --output_dir ~/Ros-workspace/maps \
    --output sitl_map_cloud \
    --scan \
    --voxel 0.05 \
    /tmp/sitl_map_snapshot.db
# Produces ~/Ros-workspace/maps/sitl_map_cloud.ply

# Open in CloudCompare
CloudCompare ~/Ros-workspace/maps/sitl_map_cloud.ply &
```

Useful `rtabmap-export` flags for LiDAR-only:

| Flag | Effect |
|---|---|
| `--scan` | Required — assemble the cloud from saved laser scans rather than from depth/stereo |
| `--voxel 0.05` | Voxel-downsample to 5 cm cells (smaller file, faster load) |
| `--max_range 30` | Drop points beyond 30 m (cuts the noise tail) |
| `--mesh` | Build a triangulated mesh in addition to the cloud |

### 3c. Live RViz visualization during a flight

RViz launches automatically. Useful displays (set Fixed Frame to `map`):

| Display | Topic | Shows |
|---|---|---|
| PointCloud2 | `/rtabmap/cloud_map` | Assembled SLAM cloud |
| Map | `/rtabmap/grid_map` | 2D occupancy grid |
| Path | `/pegasus/path_planner/global_path_initial` | A* initial route |
| Path | `/pegasus/path_planner/global_path` | D* live route (the path the drone follows) |
| PointCloud2 | `/pegasus/local_costmap_inflated` | Costmap voxels with obstacle inflation |
| MarkerArray | `/pegasus/local_costmap_markers` | Color-coded 3D obstacle blocks |

---

## 4. Deleting / Resetting the Map

### 4a. Wipe the DB before a launch (start completely fresh)

```bash
rm -f ~/Ros-workspace/maps/sitl_map.db
```

The next launch creates a new file at that path.

### 4b. Reset mid-run without restarting the launch

```bash
ros2 service call /rtabmap/reset std_srvs/srv/Empty
```

Clears RTAB-Map's working memory in place. The DB on disk is overwritten on next clean shutdown.

### 4c. Discard a partial map at run end (don't save)

There is no "discard" command — RTAB-Map writes to the DB during the run. To throw away a run,
delete the file *after* the launch terminates cleanly:

```bash
rm -f ~/Ros-workspace/maps/sitl_map.db
```

If the launch crashed (no clean shutdown), the DB file may be incomplete or empty anyway.

---

## 5. Stopping the Sim Cleanly

`Ctrl+C` in the launch terminal. The `make`-wrapped PX4 subprocess sometimes leaves zombies; the
following command cleans up everything:

```bash
pkill -9 -f 'px4_sitl_default|gz sim|MicroXRCEAgent|make px4_sitl|cmake.*gz_p110|parameter_bridge|px4_offboard|mission_planner|rtabmap|icp_odometry|odometry_selector|local_costmap|global_planner|dstar_lite|mpc_trajectory|lidar_costmap|px4_odom'
pkill -INT -f 'ros2 launch'
sleep 4

# Verify
pgrep -af 'gz sim|px4|MicroXRCE|parameter_bridge' || echo clean
```

`pgrep` returning `clean` confirms everything is down. Always run this between launches if you
notice the sim port is busy or QGC fails to connect on a fresh start.

---

## 6. Monitoring a Live Run

Run each in its own terminal:

```bash
# Mission state machine + altitude + arm/nav + position
ros2 topic echo /pegasus/autonomy/mission_status

# Offboard handshake (armed, offboard_engaged, has_setpoint, setpoints_sent)
ros2 topic echo /pegasus/offboard/status

# Did D* find a path?
ros2 topic hz /pegasus/path_planner/global_path

# Final NED setpoint going to PX4
ros2 topic echo /fmu/in/trajectory_setpoint --qos-reliability best_effort --once

# Real-time factor (lower than 1.0 means sim is running slower than wall clock)
gz stats
```

**Healthy state after ~10–20 s of sim:**
- `mission_status`: `State: SEARCH_PATTERN | SLAM: OK | PX4: OK | arm: Y | nav: 14 | …`
- `offboard/status`: `armed: true, offboard_engaged: true, has_setpoint: true, setpoint_age_s < 0.1`
- `/pegasus/path_planner/global_path`: 5–10 Hz
- `/fmu/in/trajectory_setpoint`: ~50 Hz with finite NED position values
- `gz stats`: real-time factor ≥ 0.4 on a Jetson Orin AGX (with the reduced sensor rates)

---

## 7. Common Issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Stuck at `INITIALIZING`, `SLAM: INIT` | `/odom_px4` not flowing — `px4_odom_bridge_node` crashed | Check launch terminal for `px4_odom_bridge` errors; rebuild package |
| Stuck at `READY`, `arm: N` | PX4 isn't accepting arm — preflight failure | Look in launch terminal for `commander: REJECTED` / `Preflight Fail`. Test with QGC manual arm |
| `arm: Y, nav: 2` (not 14) | OFFBOARD command rejected by PX4 — usually no setpoint stream yet | Already fixed in v2.8; if it recurs, verify `OffboardControlMode` heartbeat at 50 Hz on `/fmu/in/offboard_control_mode` |
| Drone hits walls then routes around | Costmap inflation too soft — planner cuts close | Lower `lethal_cost_threshold` further in `dstar_lite.yaml` / `path_planner.yaml`, or drop `MAX_STEP_M` in `px4_offboard_node.py` |
| Drone hovers but doesn't move | MPC's path is empty (D* not publishing) | `ros2 topic hz /pegasus/path_planner/global_path` — if 0 Hz, `lidar_costmap_layer_node` likely missing or crashed |
| RViz shows path floating above drone | `/odom` source has z drift (running on ICP instead of `/odom_px4`) | Confirm SITL launch sets `primary_odom_topic:=/odom_px4` (default since v2.8) |
| Gazebo GUI not appearing | `gz sim` is running headless with `-s` | Open client manually: `gz sim -g` in another terminal |
| RTF < 0.1 on Jetson | Sensor load too heavy | Confirm SDF reductions in `~/PX4-Simulation/Tools/simulation/gz/models/{p110_v2,better_cam}/model.sdf` are present |

---

## 8. SDF Reductions for Jetson Performance

These edits live in PX4's source tree (not this workspace):

```
~/PX4-Simulation/Tools/simulation/gz/models/p110_v2/model.sdf       # VLP-16: 360x16 @ 5 Hz, range 50 m
~/PX4-Simulation/Tools/simulation/gz/models/better_cam/model.sdf    # RGBD: 640x480 @ 10 Hz
```

Without these reductions, Gazebo runs at ~5 % real-time factor on a Jetson Orin AGX, making the
test loop unworkable. Restore the originals only if running on x86 hardware where the GPU can
keep up at 1800 × 16 lidar @ 10 Hz and 1920 × 1200 camera @ 30 Hz.
