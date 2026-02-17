# 🚁 Pegasus Disaster Response UAV - Autonomy System

**Team Pegasus | Cal Poly Pomona | Sponsored by Lockheed Martin**

Complete autonomy stack for disaster response eVTOL UAV with RTAB-Map SLAM integration.

---

## 📋 **System Overview**

### **Hardware**
- **Flight Controller:** Pixhawk Cube Orange (PX4 firmware)
- **Onboard Computer:** NVIDIA Jetson Orin AGX
- **Primary Camera:** ZED X stereo camera (GMSL2 interface)
- **LiDAR:** Velodyne VLP-16 Puck
- **IMU:** Integrated in Pixhawk Cube Orange

### **Software Stack**
- **ROS 2** (Humble/Jazzy)
- **RTAB-Map SLAM:** Multi-sensor 3D mapping and localization
- **MAVROS:** PX4-ROS2 bridge
- **Custom Mission Planning:** Disaster response autonomy

---

## 🗂️ **Package Structure**

```
pegasus_autonomy/
├── launch/
│   ├── pegasus_sensors.launch.py      # Sensor drivers (ZED X, VLP-16, MAVROS)
│   ├── pegasus_slam.launch.py         # RTAB-Map SLAM
│   ├── pegasus_full.launch.py         # Complete system
│   └── vtol1_gazebo_bridge_launch.py  # Gazebo simulation
│
├── pegasus_autonomy/
│   ├── mission_planner_node.py        # Main autonomy node
│   ├── front_stereo_node.py           # Front camera processing
│   └── px4_state_subscriber_node.py   # PX4 monitoring
│
├── config/
│   └── rviz_slam.rviz                 # RViz configuration
│
├── maps/
│   └── .gitkeep                       # RTAB-Map databases
│
├── package.xml
└── setup.py
```

---

## 🔧 **Installation**

### **Prerequisites**

```bash
# ROS 2 (Humble or newer)
sudo apt install ros-humble-desktop-full

# RTAB-Map
sudo apt install ros-humble-rtabmap-ros

# MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# ZED SDK (from Stereolabs website)
# Follow: https://www.stereolabs.com/docs/installation/jetson/

# Velodyne drivers
sudo apt install ros-humble-velodyne
```

### **Build Workspace**

```bash
cd ~/Ros-workspace
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 **Quick Start**

### **1. Hardware Setup**

**Connect Pixhawk to Jetson:**
```bash
# Check serial connection
ls /dev/tty*

# Common options:
# /dev/ttyTHS1 - Jetson UART (fastest)
# /dev/ttyACM0 - USB connection
# /dev/ttyUSB0 - Telemetry radio

# Set permissions (if needed)
sudo usermod -a -G dialout $USER
# Logout and login for changes to take effect
```

**Configure Pixhawk Serial Port:**
Update `fcu_url` in `pegasus_sensors.launch.py` if needed:
```python
'fcu_url': '/dev/ttyTHS1:921600',  # Adjust to your connection
```

**Network Setup for VLP-16:**
```bash
# Configure static IP for Velodyne LiDAR
# LiDAR default IP: 192.168.1.201
# Set your PC/Jetson: 192.168.1.100

# Update in pegasus_sensors.launch.py:
'device_ip': '192.168.1.201',  # Your VLP-16 IP
```

### **2. Test Individual Components**

**Test Sensors:**
```bash
ros2 launch pegasus_autonomy pegasus_sensors.launch.py

# In another terminal, check topics:
ros2 topic list
ros2 topic hz /mavros/imu/data
ros2 topic hz /zed_x/zed_node/left/image_rect_color
ros2 topic hz /velodyne_points
```

**Test SLAM (with rosbag):**
```bash
# Record test data first
ros2 bag record -a -o test_flight

# Play back and test SLAM
ros2 bag play test_flight --clock
ros2 launch pegasus_autonomy pegasus_slam.launch.py use_sim_time:=true
```

### **3. Launch Full System**

**Hardware Flight:**
```bash
ros2 launch pegasus_autonomy pegasus_full.launch.py
```

**Simulation:**
```bash
ros2 launch pegasus_autonomy pegasus_full.launch.py \
    use_sim_time:=true \
    launch_gazebo_bridge:=true
```

**Localization Mode (using existing map):**
```bash
ros2 launch pegasus_autonomy pegasus_full.launch.py \
    localization:=true
```

---

## 📊 **ROS Topics**

### **RTAB-Map Outputs**
```
/rtabmap/odom                        # Corrected odometry (nav_msgs/Odometry)
/rtabmap/cloud_map                   # 3D point cloud map (sensor_msgs/PointCloud2)
/rtabmap/grid_map                    # 2D occupancy grid (nav_msgs/OccupancyGrid)
/rtabmap/mapGraph                    # SLAM graph (rtabmap_msgs/MapGraph)
/rtabmap/info                        # SLAM statistics (rtabmap_msgs/Info)
```

### **Sensor Topics**
```
/mavros/imu/data                     # IMU from Pixhawk (sensor_msgs/Imu)
/mavros/local_position/pose          # PX4 position (geometry_msgs/PoseStamped)
/zed_x/zed_node/left/image_rect_color   # Left camera (sensor_msgs/Image)
/zed_x/zed_node/right/image_rect_color  # Right camera (sensor_msgs/Image)
/velodyne_points                     # LiDAR point cloud (sensor_msgs/PointCloud2)
```

### **Mission Planning**
```
/pegasus/autonomy/mission_status     # Mission state (std_msgs/String)
/pegasus/autonomy/target_waypoint    # Target position (geometry_msgs/PoseStamped)
/pegasus/autonomy/hazard_markers     # Detected hazards (visualization_msgs/MarkerArray)
/pegasus/autonomy/emergency_stop     # Emergency flag (std_msgs/Bool)
```

---

## 🎛️ **Configuration**

### **Key RTAB-Map Parameters**

Located in `pegasus_slam.launch.py`:

**Performance (Jetson Orin):**
```python
'Odom/ImageDecimation': '1',      # 1=full res, 2=half (use 2 if slow)
'Kp/MaxFeatures': '500',          # Features per frame
'Vis/MaxFeatures': '1000',        # Features for matching
```

**Map Quality:**
```python
'RGBD/AngularUpdate': '0.05',     # Update threshold (radians)
'RGBD/LinearUpdate': '0.05',      # Update threshold (meters)
'Grid/CellSize': '0.05',          # Grid resolution (5cm)
'Grid/RangeMax': '30.0',          # Max LiDAR range
```

**UAV-Specific:**
```python
'Grid/MaxObstacleHeight': '2.0',  # Max height for obstacles
'Grid/MaxGroundHeight': '0.0',    # Ground plane
```

### **Troubleshooting Parameters**

**If SLAM is slow:**
- Increase `Odom/ImageDecimation` to 2 (half resolution)
- Decrease `Kp/MaxFeatures` to 400
- Decrease `Grid/CellSize` to 0.10 (10cm)

**If losing tracking:**
- Increase `Kp/MaxFeatures` to 600
- Decrease `Vis/MinInliers` to 12
- Check IMU calibration

**If map is noisy:**
- Increase `Vis/MinInliers` to 20
- Decrease `Grid/RangeMax` to 20.0
- Enable `Grid/RayTracing`

---

## 🔍 **Monitoring & Debugging**

### **Check SLAM Health**
```bash
# Watch SLAM info
ros2 topic echo /rtabmap/info

# Key metrics:
# - features: Should be > 200
# - loop_closure_id: > 0 means loop closures detected
# - time_total: Should be < 100ms for real-time
```

### **Visualize TF Tree**
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf showing complete TF tree
```

### **Monitor CPU/GPU Usage**
```bash
# Jetson stats
sudo jtop

# Watch for:
# - GPU usage (RTAB-Map uses CUDA)
# - CPU temperature
# - Memory usage
```

### **Check Sensor Status**
```bash
# List all topics
ros2 topic list

# Check frequency
ros2 topic hz /zed_x/zed_node/left/image_rect_color  # Should be ~30 Hz
ros2 topic hz /velodyne_points                        # Should be ~10 Hz
ros2 topic hz /mavros/imu/data                        # Should be ~100 Hz
```

---

## 🗺️ **Map Management**

### **Save Map**
Maps are automatically saved to:
```
~/Ros-workspace/install/pegasus_autonomy/share/pegasus_autonomy/maps/pegasus_disaster_map.db
```

### **Use Saved Map (Localization)**
```bash
ros2 launch pegasus_autonomy pegasus_full.launch.py \
    localization:=true \
    database_path:=/path/to/your/map.db
```

### **Export Map for Analysis**
```bash
# Use RTAB-Map standalone tool
rtabmap-export database.db

# Or view in RTAB-Map GUI
rtabmap-database database.db
```

---

## 📝 **Migration from Old System**

### **What Changed**

**✅ KEPT:**
- `front_stereo_node.py` - Can still be used for custom vision
- `px4_state_subscriber_node.py` - PX4 monitoring
- `vtol1_gazebo_bridge_launch.py` - Simulation bridge

**❌ REMOVED (replaced by RTAB-Map):**
- `bottom_stereo_node.py` - Depth processing now in RTAB-Map
- `lidar_node.py` - LiDAR processing now in RTAB-Map
- `side_camera_logger_node.py` - Not needed for core SLAM

**✨ UPDATED:**
- `mission_planner_node.py` - Now uses RTAB-Map outputs
- All launch files - New sensor and SLAM integration

### **Topic Mapping**

Old → New:
```
/pegasus/lidar/points          → Use /rtabmap/cloud_map
/pegasus/bottom_stereo/depth   → Use /rtabmap/grid_map
/fmu/out/vehicle_local_position → Use /rtabmap/odom (more accurate)
```

---

## 🤝 **Contributing**

Team Pegasus members:
1. Always test in simulation before hardware
2. Record rosbags for debugging
3. Document parameter changes
4. Keep this README updated

---

## 📚 **Resources**

- [RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki)
- [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros)
- [PX4 User Guide](https://docs.px4.io/)
- [MAVROS Documentation](https://docs.px4.io/main/en/ros/mavros_installation.html)
- [ZED X Documentation](https://www.stereolabs.com/docs/)

---

## 📧 **Contact**

Team Pegasus - Cal Poly Pomona  
Lockheed Martin Sponsored Project  
Disaster Response UAV Development

---

**Last Updated:** 2025  
**Version:** 1.0.0 (RTAB-Map Integration)
