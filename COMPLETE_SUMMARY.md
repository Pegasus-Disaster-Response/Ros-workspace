# Pegasus ROS - Quick Reference & Development Roadmap

**Version:** 1.0.0  
**Last Updated:** February 2025

---

## How to Use

### Launch Full System
```bash
ros2 launch pegasus_ros pegasus_full.launch.py
```

This launches: ZED X camera driver, VLP-16 LiDAR driver, XRCE-DDS agent (Pixhawk communication), RTAB-Map SLAM, Mission planner node, and RViz visualization.

### Launch Individual Components
```bash
# Sensors only
ros2 launch pegasus_ros pegasus_sensors.launch.py

# SLAM only
ros2 launch pegasus_ros pegasus_slam.launch.py

# Mission planner only
ros2 run pegasus_ros mission_planner_node

# PX4 state monitor only
ros2 run pegasus_ros px4_state_subscriber_node

# Front camera processing only
ros2 run pegasus_ros front_stereo_node
```

### Launch with Hardware Disabled
```bash
# Without camera
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_zed:=false

# Without LiDAR
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_lidar:=false

# Without Pixhawk
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_xrce:=false

# Dry run - no hardware at all
ros2 launch pegasus_ros pegasus_sensors.launch.py enable_zed:=false enable_lidar:=false enable_xrce:=false
```

### SLAM Operations
```bash
# Start mapping (create new map)
ros2 launch pegasus_ros pegasus_slam.launch.py

# Use existing map (localization mode)
ros2 launch pegasus_ros pegasus_slam.launch.py localization:=true database_path:=/path/to/your/map.db

# Launch without RViz
ros2 launch pegasus_ros pegasus_slam.launch.py rviz:=false
```

Map saves automatically to: `~/Ros-workspace/src/pegasus_ros/maps/pegasus_disaster_map.db`

### Simulation Mode
```bash
ros2 launch pegasus_ros pegasus_full.launch.py use_sim_time:=true launch_gazebo_bridge:=true
```

### System Verification
```bash
# Check active nodes
ros2 node list

# Expected output:
# /mission_planner_node
# /px4_state_subscriber_node
# /rtabmap/rtabmap
# /rtabmap/rtabmap_odom
# /point_cloud_assembler

# List all topics
ros2 topic list

# SLAM topics only
ros2 topic list | grep rtabmap

# PX4 topics only
ros2 topic list | grep fmu

# Sensor topics
ros2 topic list | grep -E "(velodyne|zed_x)"
```

### Monitor Data Rates
```bash
# SLAM pose (should be ~10-20 Hz)
ros2 topic hz /rtabmap/odom

# LiDAR points (should be ~10 Hz)
ros2 topic hz /velodyne_points

# Camera images (should be ~30 Hz)
ros2 topic hz /zed_x/zed_node/left/image_rect_color

# PX4 IMU (should be ~100 Hz)
ros2 topic hz /fmu/out/sensor_combined

# PX4 odometry (should be ~50 Hz)
ros2 topic hz /fmu/out/vehicle_odometry
```

### View Data
```bash
# View SLAM pose
ros2 topic echo /rtabmap/odom

# View mission status
ros2 topic echo /pegasus/autonomy/mission_status

# View PX4 battery
ros2 topic echo /fmu/out/battery_status

# View PX4 vehicle status
ros2 topic echo /fmu/out/vehicle_status
```

### Visualization
```bash
# View ROS graph
rqt_graph

# View TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing transform tree

# View in RViz
rviz2 -d ~/Ros-workspace/src/pegasus_ros/config/rviz_slam.rviz
```

### Building After Code Changes
```bash
cd ~/Ros-workspace
colcon build --symlink-install --packages-select pegasus_ros
cd install/pegasus_ros/lib
ln -sf pegasus_autonomy pegasus_ros
source install/setup.bash
```

### Testing XRCE-DDS Connection
```bash
# Test agent manually (without launch file)
micro-xrce-dds-agent serial --dev /dev/ttyTHS1 -b 921600

# Should show: [timestamp] info | TermiosAgentLinux.cpp | init | running...

# In another terminal, check topics
ros2 topic list | grep fmu

# Should show /fmu/out/* topics if connected
```

### Recording Flight Data
```bash
# Record all topics
ros2 bag record -a -o flight_test_001

# Record specific topics only
ros2 bag record /rtabmap/odom /velodyne_points /zed_x/zed_node/left/image_rect_color -o flight_test_001

# Play back recorded data
ros2 bag play flight_test_001 --clock

# Use recorded data with SLAM (in another terminal)
ros2 launch pegasus_ros pegasus_slam.launch.py use_sim_time:=true
```

### Viewing Logs
```bash
# ROS logs location
ls ~/.ros/log/

# View latest log
cd ~/.ros/log/latest/

# View specific node log
tail -f ~/.ros/log/latest/mission_planner_node/stdout.log

# View launch output
ros2 launch pegasus_ros pegasus_full.launch.py 2>&1 | tee ~/pegasus_launch.log
```

### Git Operations
```bash
cd ~/Ros-workspace

# Check status
git status

# Stage all changes
git add -A

# Commit
git commit -m "Description of changes"

# Push to GitHub
git push origin main
```

---

## What Still Needs to Be Added

### Critical Missing Components

**Path Planning Node**
- Status: Not implemented
- Priority: High
- Description: Global path planning from current position to goal using A* or RRT algorithms
- Required inputs: `/rtabmap/grid_map` (2D occupancy grid), `/rtabmap/odom` (current pose), `/pegasus/autonomy/target_waypoint` (goal)
- Expected outputs: `/pegasus/path_planner/global_path` (planned waypoint sequence)
- Estimated effort: 2-3 weeks
- Files to create: `pegasus_autonomy/path_planner_node.py`, `config/path_planner_params.yaml`

**Obstacle Avoidance Node**
- Status: Not implemented
- Priority: High
- Description: Real-time local obstacle avoidance using potential field or VFH methods
- Required inputs: `/velodyne_points` (LiDAR), `/pegasus/path_planner/global_path` (planned route), `/rtabmap/odom` (current pose)
- Expected outputs: `/pegasus/obstacle_avoidance/cmd_vel` (velocity commands to PX4), `/pegasus/obstacle_avoidance/obstacle_detected` (boolean flag)
- Estimated effort: 2-3 weeks
- Files to create: `pegasus_autonomy/obstacle_avoidance_node.py`, `config/obstacle_avoidance_params.yaml`

**Flight Controller Interface**
- Status: Partially implemented (only monitoring)
- Priority: High
- Description: Send trajectory commands to PX4 via XRCE-DDS
- Required: Publish to `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, `/fmu/in/offboard_control_mode`
- Estimated effort: 1 week
- Files to modify: `pegasus_autonomy/mission_planner_node.py`

### Computer Vision Components

**Survivor Detection System**
- Status: Not implemented
- Priority: Medium-High
- Description: YOLOv8-based person detection using front ZED X camera
- Required inputs: `/zed_x/zed_node/left/image_rect_color`, `/zed_x/zed_node/depth/depth_registered`
- Expected outputs: `/pegasus/detection/survivors` (detected person positions in 3D), `/pegasus/detection/survivor_image` (annotated image)
- Estimated effort: 2 weeks
- Files to create: `pegasus_autonomy/survivor_detection_node.py`
- Dependencies: `ultralytics` (YOLOv8), trained model weights

**Fire and Smoke Detection**
- Status: Not implemented
- Priority: Medium
- Description: Color-based and ML-based fire/smoke detection
- Required inputs: `/zed_x/zed_node/left/image_rect_color`
- Expected outputs: `/pegasus/detection/fire` (fire locations), `/pegasus/detection/smoke` (smoke regions)
- Estimated effort: 2 weeks
- Files to create: `pegasus_autonomy/fire_smoke_detection_node.py`

**Structural Damage Assessment**
- Status: Not implemented
- Priority: Low-Medium
- Description: Edge detection and segmentation for identifying cracks, collapsed structures
- Required inputs: Camera images, LiDAR point clouds
- Expected outputs: Damage assessment reports, annotated images
- Estimated effort: 3-4 weeks
- Files to create: `pegasus_autonomy/damage_assessment_node.py`

### Mission Planning Enhancements

**Autonomous Search Patterns**
- Status: Not implemented
- Priority: Medium-High
- Description: Systematic search pattern generation (grid, spiral, lawnmower patterns)
- Implementation: Add search pattern generators to mission planner
- Estimated effort: 1 week
- Files to modify: `pegasus_autonomy/mission_planner_node.py`

**Multi-Target Coordination**
- Status: Not implemented
- Priority: Medium
- Description: Prioritize and sequence multiple points of interest
- Implementation: Task scheduling algorithm in mission planner
- Estimated effort: 1-2 weeks
- Files to modify: `pegasus_autonomy/mission_planner_node.py`

**Emergency Procedures**
- Status: Partially implemented (basic battery monitoring)
- Priority: High
- Description: Return-to-home on low battery, safe landing on critical failures
- Implementation: State machine for emergency handling
- Estimated effort: 1 week
- Files to modify: `pegasus_autonomy/mission_planner_node.py`

### Navigation Stack Integration

**Global-Local Planner Integration**
- Status: Not implemented
- Priority: High
- Description: Connect global path planner output to local obstacle avoidance input
- Implementation: Topic remapping and velocity command fusion
- Estimated effort: 3-4 days
- Files affected: Launch files, both planner nodes

**Recovery Behaviors**
- Status: Not implemented
- Priority: Medium
- Description: Behaviors when stuck or path blocked (rotate in place, back up, replan)
- Implementation: Recovery behavior state machine
- Estimated effort: 1 week
- Files to create: `pegasus_autonomy/recovery_behaviors_node.py`

### Configuration and Parameters

**UAV Physical Specifications File**
- Status: Not created
- Priority: High
- Description: YAML file with mass, inertia, max speeds, flight envelope limits
- Location: `config/uav_physical_params.yaml`
- Required from: Structures and aerodynamics teams
- Usage: Path planner, obstacle avoidance, mission planner for constraint checking

**PX4 Parameter Configuration**
- Status: Not documented
- Priority: Medium
- Description: Document required PX4 parameters for offboard control
- Implementation: Create parameter checklist and QGroundControl settings guide
- Estimated effort: 2-3 days
- Files to create: `docs/PX4_CONFIGURATION.md`

**Sensor Calibration Procedures**
- Status: Not documented
- Priority: Medium
- Description: Camera-LiDAR extrinsic calibration, IMU calibration procedures
- Implementation: Calibration launch files and documentation
- Estimated effort: 3-4 days
- Files to create: `launch/calibration.launch.py`, `docs/CALIBRATION.md`

### Testing Infrastructure

**Unit Tests**
- Status: Skeleton only
- Priority: Medium
- Description: Unit tests for individual nodes and functions
- Implementation: pytest-based test suite
- Estimated effort: Ongoing
- Files to modify: `test/` directory

**Integration Tests**
- Status: Not implemented
- Priority: Medium-High
- Description: End-to-end tests with simulated sensor data
- Implementation: Automated test launch files with bag playback
- Estimated effort: 2 weeks
- Files to create: `test/integration/`, test bag files

**Simulation Environment**
- Status: Partially implemented (Gazebo bridge exists)
- Priority: Medium
- Description: Full Gazebo simulation with physics, sensors, and environment
- Implementation: Gazebo world files, sensor plugins, UAV model
- Estimated effort: 3-4 weeks
- Files to create: `simulation/` directory

### Documentation

**Hardware Setup Guide**
- Status: Partially documented
- Priority: Medium
- Description: Step-by-step hardware assembly, wiring diagrams, connector pinouts
- Files to create: `docs/HARDWARE_SETUP.md`

**Flight Test Procedures**
- Status: Not created
- Priority: High
- Description: Pre-flight checklist, test protocols, safety procedures
- Files to create: `docs/FLIGHT_TEST_PROCEDURES.md`

**Troubleshooting Flowcharts**
- Status: Text-only troubleshooting
- Priority: Low
- Description: Visual flowcharts for common issues
- Files to create: `docs/troubleshooting/` with diagrams

### Performance Optimization

**RTAB-Map Parameter Tuning**
- Status: Default parameters only
- Priority: Medium
- Description: Tune for Pegasus hardware and disaster environment
- Implementation: Systematic parameter sweep and performance benchmarking
- Estimated effort: 1-2 weeks
- Files to modify: `launch/pegasus_slam.launch.py`

**Computational Profiling**
- Status: Not performed
- Priority: Low-Medium
- Description: Identify computational bottlenecks, optimize critical paths
- Implementation: Use `ros2 topic bw`, `top`, profiling tools
- Estimated effort: 3-4 days

**Network Optimization**
- Status: Not performed
- Priority: Low
- Description: Optimize topic rates, compression for bandwidth-limited scenarios
- Implementation: Topic decimation, image compression
- Estimated effort: 2-3 days

### Safety Features

**Geofencing**
- Status: Not implemented
- Priority: High
- Description: Virtual boundaries to prevent flight into restricted areas
- Implementation: Position checking in mission planner, failsafe to PX4
- Estimated effort: 3-4 days
- Files to modify: `pegasus_autonomy/mission_planner_node.py`

**Collision Prediction**
- Status: Not implemented
- Priority: High
- Description: Predict potential collisions based on current trajectory
- Implementation: Forward simulation in obstacle avoidance node
- Estimated effort: 1 week
- Files to modify: `pegasus_autonomy/obstacle_avoidance_node.py`

**Health Monitoring**
- Status: Partially implemented (battery only)
- Priority: Medium
- Description: Monitor all critical systems (sensors, communication, compute resources)
- Implementation: Diagnostic aggregator node
- Estimated effort: 1 week
- Files to create: `pegasus_autonomy/system_health_node.py`

---

## Development Priorities

### Phase 1: Core Navigation (Weeks 1-6)
1. Path planning node with A* algorithm
2. Obstacle avoidance node with potential field method
3. Integration of global and local planners
4. PX4 trajectory command interface
5. Basic emergency procedures (RTH, low battery)

### Phase 2: Computer Vision (Weeks 7-10)
1. Survivor detection with YOLOv8
2. Fire and smoke detection
3. Integration with mission planner for target investigation

### Phase 3: Mission Intelligence (Weeks 11-14)
1. Autonomous search pattern generation
2. Multi-target prioritization
3. Recovery behaviors
4. Advanced emergency handling

### Phase 4: Testing & Validation (Weeks 15-18)
1. Unit test suite completion
2. Integration testing with simulation
3. Hardware-in-the-loop testing
4. Field testing and parameter tuning

### Phase 5: Safety & Reliability (Weeks 19-20)
1. Geofencing implementation
2. Collision prediction
3. System health monitoring
4. Comprehensive documentation

---

## Quick Reference - File Locations
```
Configuration Files:
- UAV parameters: config/uav_physical_params.yaml (TO BE CREATED)
- SLAM parameters: launch/pegasus_slam.launch.py (line 80-130)
- Sensor settings: launch/pegasus_sensors.launch.py (line 37, 106)

Node Files:
- Mission planner: pegasus_autonomy/mission_planner_node.py
- Path planner: pegasus_autonomy/path_planner_node.py (TO BE CREATED)
- Obstacle avoidance: pegasus_autonomy/obstacle_avoidance_node.py (TO BE CREATED)
- Survivor detection: pegasus_autonomy/survivor_detection_node.py (TO BE CREATED)

Launch Files:
- Full system: launch/pegasus_full.launch.py
- Sensors only: launch/pegasus_sensors.launch.py
- SLAM only: launch/pegasus_slam.launch.py

Documentation:
- Main README: README.md
- This file: USAGE_AND_ROADMAP.md
- Hardware setup: docs/HARDWARE_SETUP.md (TO BE CREATED)
- Flight procedures: docs/FLIGHT_TEST_PROCEDURES.md (TO BE CREATED)
```

---

**Note:** This roadmap is subject to change based on project requirements, hardware availability, and testing results. Estimated efforts are approximate and may vary based on team size and experience level.