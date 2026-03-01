#!/bin/bash
# ============================================================
#  Pegasus Disaster Response UAV — Workspace Setup
#  Sensors: ZED X (single) + Velodyne VLP-16 + Pixhawk Cube Orange
#  ROS 2 Humble | RTAB-Map (from source) | PX4 via XRCE-DDS
#
#  Run this script ONCE on a fresh Jetson Orin AGX to set up
#  the full development environment.
# ============================================================

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
WS_DIR="$HOME/Ros-workspace"
SRC_DIR="$WS_DIR/src"
PX4_WS="$HOME/px4_ws"

echo "════════════════════════════════════════════════════"
echo "  Pegasus Workspace Setup"
echo "  ROS Distro:  $ROS_DISTRO"
echo "  Workspace:   $WS_DIR"
echo "  PX4 Msgs WS: $PX4_WS"
echo "════════════════════════════════════════════════════"

# ── 1. System Dependencies ──────────────────────────────────
echo ""
echo "[1/7] Installing system dependencies..."

sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-velodyne \
  ros-$ROS_DISTRO-velodyne-driver \
  ros-$ROS_DISTRO-velodyne-laserscan \
  ros-$ROS_DISTRO-velodyne-pointcloud \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  git

# ── 2. Remove apt RTAB-Map (we build from source) ───────────
echo ""
echo "[2/7] Removing apt RTAB-Map (will build from source)..."

# The apt version (0.22.1) is incompatible with source-built
# databases (0.23.x). Remove to avoid version conflicts.
sudo apt remove -y \
  ros-$ROS_DISTRO-rtabmap \
  ros-$ROS_DISTRO-rtabmap-ros \
  ros-$ROS_DISTRO-rtabmap-slam \
  ros-$ROS_DISTRO-rtabmap-odom \
  ros-$ROS_DISTRO-rtabmap-util \
  ros-$ROS_DISTRO-rtabmap-viz \
  ros-$ROS_DISTRO-rtabmap-msgs \
  ros-$ROS_DISTRO-rtabmap-launch \
  2>/dev/null || true

sudo apt autoremove -y

# Also remove any stale database that was written by 0.22.1
if [ -f "$HOME/.ros/rtabmap.db" ]; then
  echo "  Removing stale rtabmap.db (may be incompatible version)..."
  rm "$HOME/.ros/rtabmap.db"
fi

# ── 3. Check ZED SDK ────────────────────────────────────────
echo ""
echo "[3/7] Checking ZED SDK..."

if [ ! -f "/usr/local/zed/tools/ZED_Diagnostic" ]; then
  echo "  ⚠️  ZED SDK not found."
  echo "  Please download and install from:"
  echo "  https://www.stereolabs.com/developers/release"
  echo "  Then re-run this script."
  exit 1
else
  echo "  ✅ ZED SDK found."
fi

# ── 4. Install XRCE-DDS Agent ───────────────────────────────
echo ""
echo "[4/7] Installing micro-xrce-dds-agent..."

if ! command -v micro-xrce-dds-agent &> /dev/null; then
  sudo snap install micro-xrce-dds-agent --edge
  echo "  ✅ micro-xrce-dds-agent installed."
else
  echo "  ✅ micro-xrce-dds-agent already installed."
fi

# ── 5. Clone Source Packages ────────────────────────────────
echo ""
echo "[5/7] Setting up source packages..."

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

# RTAB-Map core library (build from source for 0.23.x)
if [ ! -d "rtabmap" ]; then
  echo "  Cloning rtabmap (core library)..."
  git clone https://github.com/introlab/rtabmap.git
else
  echo "  rtabmap already cloned, skipping."
fi

# RTAB-Map ROS 2 wrapper (build from source)
if [ ! -d "rtabmap_ros" ]; then
  echo "  Cloning rtabmap_ros (ROS 2 wrapper)..."
  git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
else
  echo "  rtabmap_ros already cloned, skipping."
fi

# ZED ROS 2 wrapper
if [ ! -d "zed-ros2-wrapper" ]; then
  echo "  Cloning zed-ros2-wrapper..."
  git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
else
  echo "  zed-ros2-wrapper already cloned, skipping."
fi

# ── 5b. PX4 Messages (separate workspace for fast builds) ──
echo ""
echo "  Setting up px4_msgs workspace..."

mkdir -p "$PX4_WS/src"
if [ ! -d "$PX4_WS/src/px4_msgs" ]; then
  cd "$PX4_WS/src"
  git clone https://github.com/PX4/px4_msgs.git
fi

cd "$PX4_WS"
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source "$PX4_WS/install/setup.bash"

# ── 6. rosdep + Build ──────────────────────────────────────
echo ""
echo "[6/7] Running rosdep and building workspace..."

cd "$WS_DIR"
source /opt/ros/$ROS_DISTRO/setup.bash
source "$PX4_WS/install/setup.bash"

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "  Building workspace (this may take 10-20 minutes on first run)..."
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug 2>&1 | tail -5

source "$WS_DIR/install/setup.bash"

# ── 7. Configure ~/.bashrc ──────────────────────────────────
echo ""
echo "[7/7] Updating ~/.bashrc..."

add_to_bashrc() {
  local line="$1"
  if ! grep -qF "$line" ~/.bashrc; then
    echo "$line" >> ~/.bashrc
  fi
}

add_to_bashrc "# ROS 2 Humble"
add_to_bashrc "source /opt/ros/$ROS_DISTRO/setup.bash"
add_to_bashrc ""
add_to_bashrc "# PX4 Messages Workspace"
add_to_bashrc "source $PX4_WS/install/setup.bash"
add_to_bashrc ""
add_to_bashrc "# Pegasus Workspace"
add_to_bashrc "source $WS_DIR/install/setup.bash"

echo ""
echo "════════════════════════════════════════════════════"
echo "  ✅ Workspace setup complete!"
echo "════════════════════════════════════════════════════"
echo ""
echo "  Next steps:"
echo "  1. source ~/.bashrc"
echo "  2. Set your ZED X serial number in:"
echo "       src/pegasus_ros/config/zed_x.yaml"
echo "  3. Verify camera: /usr/local/zed/tools/ZED_Explorer"
echo "  4. Update sensor TF offsets in:"
echo "       launch/pegasus_slam.launch.py"
echo "  5. ros2 launch pegasus_ros pegasus_full.launch.py"
echo ""
echo "  Verify RTAB-Map version (should be 0.23.x):"
echo "    ros2 run rtabmap_slam rtabmap --version 2>&1 | grep RTAB-Map"
echo ""