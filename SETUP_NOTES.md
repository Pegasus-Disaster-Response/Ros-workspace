# Pegasus ROS — Setup Notes & Run Commands
**Last updated: 2026-04-26**

---

## What Was Changed and Why

### 1. PX4 Parameter Fix — `UXRCE_DDS_CFG`

**Problem:** The parameter `UXRCE_DDS_CFG` was accidentally corrupted (set to `1120665600`
instead of `102`) by a pymavlink script that wrote `float(102)` using the wrong encoding for
an INT32 parameter. This caused the uXRCE-DDS client to not start at boot, meaning no PX4
topics appeared in ROS2.

**Fix applied via MAVLink:** `UXRCE_DDS_CFG` was restored to `102` (= TELEM2) using the
correct INT32 byte encoding, then PX4 was rebooted.

**Confirmed working config:**
| Parameter | Value | Meaning |
|---|---|---|
| `UXRCE_DDS_CFG` | `102` | uXRCE-DDS runs on TELEM2 |
| `SER_TEL2_BAUD` | `921600` | TELEM2 baud rate |
| `MAV_0_CONFIG` | `101` | MAVLink on TELEM1 (no conflict with TELEM2) |

There is **no conflict** between MAVLink and uXRCE-DDS — they are on different ports.
MAVLink uses TELEM1 + USB (ACM0). uXRCE-DDS uses TELEM2 (USB0/FTDI adapter).

---

### 2. `px4_msgs` Built in Workspace

The `px4_msgs` package (PX4 v1.14) was cloned and built so ROS2 can decode PX4 DDS topics:

```bash
cd ~/Ros-workspace/src
git clone --depth 1 --branch release/1.14 https://github.com/PX4/px4_msgs.git
cd ~/Ros-workspace
colcon build --packages-select px4_msgs
```

---

### 3. Launch File Fix — `pegasus_sensors.launch.py`

**Problem:** The launch file called `micro-xrce-dds-agent` (the snap wrapper binary).
On this Jetson, snap confinement fails with:
```
required permitted capability cap_dac_override not found
```

**Fix:** The launch file now calls the agent binary directly with its bundled libraries:

```python
# BEFORE (broken on this Jetson):
cmd=['micro-xrce-dds-agent', 'serial', '--dev', fcu_dev, '-b', fcu_baud]

# AFTER (working):
cmd=['/snap/micro-xrce-dds-agent/current/usr/bin/MicroXRCEAgent', 'serial',
     '-D', fcu_dev, '-b', fcu_baud]
additional_env={'LD_LIBRARY_PATH': '/snap/micro-xrce-dds-agent/current/usr/lib/aarch64-linux-gnu'
                                   ':/snap/micro-xrce-dds-agent/current/usr/lib'}
```

---

### 4. IMU Data Path (End-to-End)

```
PX4 CubeOrange
  └─ TELEM2 (ttyS4 @ 921600)
       └─ FTDI USB adapter (/dev/ttyUSB0)
            └─ MicroXRCEAgent (serial, 921600)
                 └─ /fmu/out/sensor_combined   (px4_msgs/SensorCombined, ~100 Hz)
                 └─ /fmu/out/vehicle_attitude  (px4_msgs/VehicleAttitude, ~50 Hz)
                      └─ px4_imu_bridge_node
                           └─ /pegasus/imu/data  (sensor_msgs/Imu, 50 Hz, FRD→FLU)
```

The bridge node (`px4_imu_bridge_node.py`) converts PX4's FRD body frame to ROS FLU:
- `FLU.x =  FRD.x`
- `FLU.y = -FRD.y`
- `FLU.z = -FRD.z`

---

## CRITICAL: Boot Order Rule

**The MicroXRCEAgent must be the first process to open `/dev/ttyUSB0` after every boot.**

If QGroundControl, pymavlink, or any other tool opens `/dev/ttyUSB0` first, the FTDI chip's
flow-control state is corrupted and the agent cannot connect until the Pixhawk is power-cycled.

- **`/dev/ttyUSB0`** — TELEM2, exclusively for uXRCE-DDS. Do not open with QGC.
- **`/dev/ttyACM0`** — Native USB, for MAVLink / QGroundControl (not simultaneously with the above).

---

## Run Commands — Mapping Mode (Sensors + SLAM)

Open terminals **in this order**. Wait for each to finish initialising before opening the next.

### Terminal 1 — Sensors (XRCE Agent + IMU Bridge + Camera + LiDAR)

```bash
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_sensors.launch.py fcu_dev:=/dev/ttyUSB0
```

Wait until you see:
```
[px4_imu_bridge_node] Publishing IMU data to /pegasus/imu/data
```

### Terminal 2 — SLAM (RTAB-Map)

```bash
cd ~/Ros-workspace
source ~/camera_ws/install/setup.bash
source install/setup.bash
ros2 launch pegasus_ros pegasus_slam.launch.py
```

### Terminal 3 — Monitor Topic Health

```bash
# Check all sensor streams are live
ros2 topic hz /zed_x/zed_node/rgb/color/rect/image
ros2 topic hz /velodyne_points
ros2 topic hz /pegasus/imu/data          # should be ~50 Hz
ros2 topic hz /odom

# Watch map file growing
watch -n 5 ls -la ~/Ros-workspace/maps/pegasus_disaster_map.db
```

---

## IMU-Only Quick Test (no camera/lidar)

If you just want to verify IMU data is flowing without starting the full sensor stack:

```bash
# Terminal 1 — start agent and bridge only
cd ~/Ros-workspace
source install/setup.bash
ros2 launch pegasus_ros pegasus_sensors.launch.py \
    fcu_dev:=/dev/ttyUSB0 \
    enable_zed:=false \
    enable_lidar:=false

# Terminal 2 — verify
source install/setup.bash
ros2 topic hz /pegasus/imu/data
ros2 topic echo /pegasus/imu/data
```

---

## If the Agent Fails to Connect

Symptom: no `/fmu/out/*` topics appear, `px4_imu_bridge_node` never logs "Publishing IMU data".

1. **Power cycle the Pixhawk** (unplug power, not just USB).
2. Make sure nothing else has opened `/dev/ttyUSB0`:
   ```bash
   fuser /dev/ttyUSB0   # should return nothing
   ```
3. Re-launch Terminal 1.

Do **not** run `micro-xrce-dds-agent udp4` — PX4 is configured for serial transport.
