# Complete Pegasus Workspace Update Summary

## **What Was Done**

A comprehensive update of the Pegasus Disaster Response UAV workspace to integrate **RTAB-Map SLAM** for professional-grade 3D mapping and localization.

---

## **How to Use**

### **Quick Start**
```bash
# 1. Build workspace
cd ~/Ros-workspace
colcon build --symlink-install
source install/setup.bash

# 2. Launch complete system
ros2 launch pegasus_autonomy pegasus_full.launch.py
```

### **Testing Individual Components**
```bash
# Test sensors only
ros2 launch pegasus_autonomy pegasus_sensors.launch.py

# Test SLAM only (with rosbag)
ros2 bag play test_flight
ros2 launch pegasus_autonomy pegasus_slam.launch.py
```

### **Hardware Configuration**

**1. Update Pixhawk Connection:**
Edit `launch/pegasus_sensors.launch.py`:
```python
'fcu_url': '/dev/ttyTHS1:921600',  # Change to your serial port
```

**2. Update VLP-16 IP:**
Edit `launch/pegasus_sensors.launch.py`:
```python
'device_ip': '192.168.1.201',  # Change to your LiDAR IP
```

---

## **ROS Topic Architecture**

### **Data Flow Diagram**

```
┌─────────────────────────────────────────────────┐
│              HARDWARE SENSORS                   │
│  - ZED X Camera (stereo)                        │
│  - VLP-16 LiDAR                                 │
│  - Pixhawk Cube Orange (IMU)                    │
└───────────────┬─────────────────────────────────┘
                │
                │ Raw sensor data
                ▼
┌─────────────────────────────────────────────────┐
│           SENSOR DRIVERS                        │
│  - zed_wrapper                                  │
│  - velodyne_driver                              │
│  - MAVROS                                       │
└───────────────┬─────────────────────────────────┘
                │
                │ ROS topics:
                │ /zed_x/left/image_rect_color
                │ /zed_x/right/image_rect_color
                │ /velodyne_points
                │ /mavros/imu/data
                ▼
┌─────────────────────────────────────────────────┐
│            RTAB-MAP SLAM                        │
│  - rtabmap_odom (visual odometry)               │
│  - rtabmap_slam (mapping & loop closure)        │
└───────────────┬─────────────────────────────────┘
                │
                │ SLAM outputs:
                │ /rtabmap/odom (corrected pose)
                │ /rtabmap/cloud_map (3D map)
                │ /rtabmap/grid_map (2D occupancy)
                │ TF: map → odom → base_link
                ▼
┌─────────────────────────────────────────────────┐
│         MISSION PLANNER NODE                    │
│  - Uses SLAM outputs for navigation             │
│  - Disaster response logic                      │
│  - Publishes waypoints and commands             │
└─────────────────────────────────────────────────┘
```

---

##  **Configuration Parameters**

### **Critical RTAB-Map Parameters** (in `pegasus_slam.launch.py`)

**Performance Tuning:**
```python
'Odom/ImageDecimation': '1'     # 1=full, 2=half resolution
'Kp/MaxFeatures': '500'         # Features per frame
'Vis/MaxFeatures': '1000'       # Matching features
```

**Map Quality:**
```python
'Grid/CellSize': '0.05'         # 5cm grid resolution
'Grid/RangeMax': '30.0'         # VLP-16 max range
'Grid/MaxObstacleHeight': '2.0' # For UAV navigation
```

**Synchronization:**
```python
'approx_sync': True             # CRITICAL for multi-sensor
'approx_sync_max_interval': 0.1 # 100ms tolerance
'wait_imu_to_init': True        # Wait for IMU
```

---

##  **Verification Checklist**

Before flying, verify:

###  **1. Sensors Connected**
```bash
ros2 topic hz /mavros/imu/data           # ~100 Hz
ros2 topic hz /velodyne_points           # ~10 Hz
ros2 topic hz /zed_x/zed_node/left/image_rect_color  # ~30 Hz
```

###  **2. SLAM Running**
```bash
ros2 topic echo /rtabmap/info
# Check: features > 200, loop_closure_id > 0
```

###  **3. TF Tree Complete**
```bash
ros2 run tf2_tools view_frames
# Should see: map → odom → base_link → sensors
```

###  **4. Mission Planner Active**
```bash
ros2 topic echo /pegasus/autonomy/mission_status
# Should show: "Status: READY | SLAM: OK"
```

---

##  **Performance Expectations**

### **Jetson Orin AGX (MAXN Mode)**
- **SLAM Processing:** 15-20 Hz
- **Visual Odometry:** 20-30 Hz
- **Total CPU Usage:** 60-80%
- **GPU Usage:** 40-60%
- **Memory:** ~4-6 GB

### **If Performance Issues:**
1. Reduce `Odom/ImageDecimation` to 2
2. Decrease `Kp/MaxFeatures` to 400
3. Increase `Grid/CellSize` to 0.10
4. Ensure Jetson in MAXN mode: `sudo nvpmodel -m 0`

---

##  **Troubleshooting Quick Reference**

| Issue | Solution |
|-------|----------|
| No MAVROS topics | Check `/dev/ttyTHS1`, verify permissions |
| No LiDAR data | Ping `192.168.1.201`, check network |
| SLAM too slow | Reduce image resolution, decrease features |
| Lost tracking | Increase features, check IMU calibration |
| TF errors | Verify robot_state_publisher running |
| No loop closures | Move slower, revisit same areas |

---

## **Documentation Files**

1. **README.md** - Main documentation with quick start
2. **MIGRATION_GUIDE.md** - Detailed migration steps
3. **THIS FILE** - Complete summary and overview

---

##**Learning Resources**

- **RTAB-Map:** https://github.com/introlab/rtabmap/wiki
- **MAVROS:** https://docs.px4.io/main/en/ros/mavros_installation.html
- **ROS 2 Nav2:** https://navigation.ros.org/
- **ZED X Docs:** https://www.stereolabs.com/docs/

---

## **Team Pegasus**

**Project:** Disaster Response eVTOL UAV  
**Sponsor:** Lockheed Martin  
**Institution:** Cal Poly Pomona  
**Year:** 2025-2026

**Key Features:**
-  Multi-sensor SLAM (stereo + LiDAR + IMU)
-  Real-time 3D mapping
-  Autonomous navigation ready
-  Disaster response mission planning
-  Professional-grade localization

---

##  **What's Next?**

1. **Integration Testing** - Test full system in lab
2. **Parameter Tuning** - Optimize for your specific hardware
3. **Mission Logic** - Implement disaster response behaviors
4. **Path Planning** - Integrate Nav2 for autonomous waypoint navigation
5. **Object Detection** - Add AI inference for survivor detection
6. **Field Testing** - Outdoor flights with real disaster scenarios

---

**Status:**  **READY FOR TESTING**  
**Version:** 1.0.0  
**Last Updated:** 2026  
**Compatibility:** ROS 2 Humble/Jazzy
