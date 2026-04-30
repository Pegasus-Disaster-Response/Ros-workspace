#!/usr/bin/env python3
"""
Pegasus Mission Planner Node - Disaster Response Autonomy
Uses RTAB-Map SLAM outputs and PX4 via XRCE-DDS

Author: Team Pegasus
Date: 2026
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from px4_msgs.msg import VehicleOdometry, BatteryStatus, VehicleStatus


BATTERY_RETURN_THRESHOLD   = 0.20   # 20% → return to base
BATTERY_EMERGENCY_THRESHOLD = 0.10  # 10% → emergency land

WAYPOINT_ACCEPT_RADIUS = 3.0   # metres — advance to next waypoint within this distance
SEARCH_ALTITUDE        = 10.0  # metres AGL for lawnmower pattern (was 25.0; lowered for SITL test loop)
SEARCH_LANE_SPACING    = 5.0   # metres between lawnmower lanes (was 10.0; tightened for smaller test box)
SEARCH_FALLBACK_SIZE   = 30.0  # metres — fallback box edge length when no SLAM grid yet (was 100.0)


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('mission_planner_node')

        self.declare_parameter('auto_start_search', False)
        self.auto_start_search = self.get_parameter('auto_start_search').value

        self.mission_status  = 'INITIALIZING'
        self.slam_initialized = False
        self.odom_initialized = False  # fallback: /odom when SLAM absent
        self.px4_connected   = False
        self.current_pose    = None
        self.px4_odom        = None
        self.latest_grid     = None
        self.latest_cloud    = None
        self.vehicle_armed   = False
        self.nav_state       = None
        self.battery_level   = 1.0
        self.home_position   = None
        self.poi_target      = None
        self.search_waypoints = []
        self.search_wp_index  = 0
        self._auto_started    = False

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Odometry, '/rtabmap/odom',
            self.rtabmap_odom_callback, 10)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map',
            self.rtabmap_cloud_callback, 10)
        self.create_subscription(OccupancyGrid, '/rtabmap/grid_map',
            self.rtabmap_grid_callback, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.px4_odom_callback, px4_qos)
        self.create_subscription(Imu, '/pegasus/imu/data',
            self.sensor_callback, 10)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status',
            self.battery_callback, px4_qos)
        # PX4 v1.15+ publishes the versioned _v1 topic. The unversioned
        # /fmu/out/vehicle_status has 0 publishers in this PX4 build.
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, px4_qos)
        # External commands: START_SEARCH | RETURN_HOME | ABORT | RESET
        self.create_subscription(String, '/pegasus/autonomy/mission_command',
            self.mission_command_callback, 10)
        # Point-of-interest from detection pipeline or operator
        self.create_subscription(PoseStamped, '/pegasus/autonomy/poi',
            self.poi_callback, 10)
        # Fallback odometry when SLAM is not running (e.g. SITL)
        self.create_subscription(Odometry, '/odom',
            self.odom_fallback_callback, 10)

        # ── Publishers ───────────────────────────────────────────────
        self.status_pub   = self.create_publisher(String,
            '/pegasus/autonomy/mission_status', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped,
            '/pegasus/autonomy/target_waypoint', 10)

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(0.5, self.mission_planning_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Mission Planner started — waiting for SLAM and PX4')

    # ── SLAM Callbacks ───────────────────────────────────────────────

    def rtabmap_odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if not self.slam_initialized:
            self.slam_initialized = True
            self.home_position = msg.pose.pose.position
            p = self.home_position
            self.get_logger().info(
                f'SLAM initialized — home [{p.x:.1f}, {p.y:.1f}, {p.z:.1f}]')

    def rtabmap_cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    def rtabmap_grid_callback(self, msg: OccupancyGrid):
        self.latest_grid = msg

    # ── PX4 Callbacks ────────────────────────────────────────────────

    def px4_odom_callback(self, msg: VehicleOdometry):
        self.px4_odom = msg
        if not self.px4_connected:
            self.px4_connected = True
            self.get_logger().info('PX4 connected via XRCE-DDS')

    def odom_fallback_callback(self, msg: Odometry):
        if self.slam_initialized:
            return  # SLAM is running — don't override with fallback
        self.current_pose = msg.pose.pose
        if not self.odom_initialized:
            self.odom_initialized = True
            self.home_position = msg.pose.pose.position
            p = self.home_position
            self.get_logger().info(
                f'Odom fallback initialized — home [{p.x:.1f}, {p.y:.1f}, {p.z:.1f}]')

    def sensor_callback(self, msg: Imu):
        pass  # Available for vibration monitoring or tilt detection

    def battery_callback(self, msg: BatteryStatus):
        self.battery_level = msg.remaining
        if msg.remaining < BATTERY_EMERGENCY_THRESHOLD:
            if self.mission_status not in ('EMERGENCY', 'INITIALIZING'):
                self.get_logger().error(
                    f'CRITICAL battery {msg.remaining*100:.0f}% — emergency land')
                self._transition('EMERGENCY')
        elif msg.remaining < BATTERY_RETURN_THRESHOLD:
            if self.mission_status not in ('RETURN_TO_BASE', 'EMERGENCY',
                                           'INITIALIZING', 'READY'):
                self.get_logger().warn(
                    f'Low battery {msg.remaining*100:.0f}% — returning to base')
                self._transition('RETURN_TO_BASE')

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_armed = (msg.arming_state == 2)  # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    # ── Command Callbacks ─────────────────────────────────────────────

    def mission_command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Command: {cmd}')
        if cmd == 'START_SEARCH':
            if self.mission_status == 'READY':
                self._transition('SEARCH_PATTERN')
            else:
                self.get_logger().warn(
                    f'START_SEARCH ignored — current state: {self.mission_status}')
        elif cmd == 'RETURN_HOME':
            self._transition('RETURN_TO_BASE')
        elif cmd == 'ABORT':
            self._transition('EMERGENCY')
        elif cmd == 'RESET':
            self._transition('READY')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def poi_callback(self, msg: PoseStamped):
        self.poi_target = msg.pose
        p = msg.pose.position
        self.get_logger().info(f'POI received at [{p.x:.1f}, {p.y:.1f}, {p.z:.1f}]')
        if self.mission_status in ('SEARCH_PATTERN', 'READY'):
            self._transition('INVESTIGATE_POI')

    # ── State Machine ─────────────────────────────────────────────────

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.mission_status} → {new_state}')
        self.mission_status = new_state

        if new_state == 'SEARCH_PATTERN':
            self._build_search_pattern()
            self.search_wp_index = 0

        elif new_state == 'RETURN_TO_BASE':
            if self.home_position:
                self._publish_waypoint(
                    self.home_position.x, self.home_position.y, SEARCH_ALTITUDE)

        elif new_state == 'EMERGENCY':
            if self.current_pose:
                # Command ground level at current XY — offboard node handles landing
                self._publish_waypoint(
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    0.0)

    def mission_planning_loop(self):
        pose_ready = self.slam_initialized or self.odom_initialized
        if not pose_ready or self.current_pose is None:
            return

        if self.mission_status == 'INITIALIZING':
            self._transition('READY')

        # Auto-start: re-evaluates every loop tick (no _auto_started latch
        # so we retry if the first window misses). Uses the px4_msgs
        # constant rather than the hardcoded 14, since PX4 v1.16+ may
        # report OFFBOARD under a different value.
        if (self.auto_start_search
                and self.mission_status == 'READY'
                and self.vehicle_armed
                and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().info(
                f'Auto-starting search pattern '
                f'(armed={self.vehicle_armed}, nav_state={self.nav_state})')
            self._transition('SEARCH_PATTERN')

        elif self.mission_status == 'SEARCH_PATTERN':
            self._run_search_pattern()

        elif self.mission_status == 'INVESTIGATE_POI':
            self._run_investigate_poi()

        elif self.mission_status == 'RETURN_TO_BASE':
            self._run_return_to_base()

        elif self.mission_status == 'EMERGENCY':
            self._publish_waypoint(
                self.current_pose.position.x,
                self.current_pose.position.y,
                0.0)

    # ── State Handlers ────────────────────────────────────────────────

    def _run_search_pattern(self):
        if not self.search_waypoints:
            self.get_logger().warn('No search waypoints — rebuilding')
            self._build_search_pattern()
            return

        if self.search_wp_index >= len(self.search_waypoints):
            self.get_logger().info('Search pattern complete — returning to base')
            self._transition('RETURN_TO_BASE')
            return

        target = self.search_waypoints[self.search_wp_index]
        self._publish_waypoint(target[0], target[1], SEARCH_ALTITUDE)

        if self._distance_to(target[0], target[1]) < WAYPOINT_ACCEPT_RADIUS:
            self.search_wp_index += 1
            if self.search_wp_index < len(self.search_waypoints):
                nxt = self.search_waypoints[self.search_wp_index]
                self.get_logger().info(
                    f'Search WP {self.search_wp_index}/{len(self.search_waypoints)}'
                    f' → [{nxt[0]:.1f}, {nxt[1]:.1f}]')

    def _run_investigate_poi(self):
        if self.poi_target is None:
            self._transition('SEARCH_PATTERN')
            return

        self._publish_waypoint(
            self.poi_target.position.x,
            self.poi_target.position.y,
            max(self.poi_target.position.z, 5.0))  # minimum 5m above POI

        if self._distance_to(
                self.poi_target.position.x,
                self.poi_target.position.y) < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info('POI reached — resuming search')
            self.poi_target = None
            self._transition('SEARCH_PATTERN')

    def _run_return_to_base(self):
        if self.home_position is None:
            return
        self._publish_waypoint(
            self.home_position.x, self.home_position.y, SEARCH_ALTITUDE)
        if self._distance_to(self.home_position.x, self.home_position.y) \
                < WAYPOINT_ACCEPT_RADIUS:
            self.get_logger().info('Home reached — mission complete')
            self._transition('READY')

    # ── Helpers ───────────────────────────────────────────────────────

    def _build_search_pattern(self):
        """Lawnmower waypoints derived from occupancy grid bounds or a 100×100m fallback."""
        self.search_waypoints = []

        if self.latest_grid is not None:
            info = self.latest_grid.info
            origin_x = info.origin.position.x
            origin_y = info.origin.position.y
            width_m  = info.width  * info.resolution
            height_m = info.height * info.resolution
        elif self.current_pose is not None:
            half = SEARCH_FALLBACK_SIZE / 2.0
            origin_x = self.current_pose.position.x - half
            origin_y = self.current_pose.position.y - half
            width_m  = SEARCH_FALLBACK_SIZE
            height_m = SEARCH_FALLBACK_SIZE
        else:
            self.get_logger().warn('Cannot build search pattern — no pose or map')
            return

        y = origin_y
        lane = 0
        while y <= origin_y + height_m:
            if lane % 2 == 0:
                self.search_waypoints.append((origin_x, y))
                self.search_waypoints.append((origin_x + width_m, y))
            else:
                self.search_waypoints.append((origin_x + width_m, y))
                self.search_waypoints.append((origin_x, y))
            y    += SEARCH_LANE_SPACING
            lane += 1

        self.get_logger().info(
            f'Search pattern: {len(self.search_waypoints)} waypoints '
            f'over {width_m:.0f}×{height_m:.0f}m')

    def _publish_waypoint(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)

    def _distance_to(self, x: float, y: float) -> float:
        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - x
        dy = self.current_pose.position.y - y
        return math.sqrt(dx * dx + dy * dy)

    def publish_status(self):
        pos_str = ''
        if self.current_pose:
            p = self.current_pose.position
            pos_str = f' | Pos: [{p.x:.1f}, {p.y:.1f}, {p.z:.1f}]'
        nav = self.nav_state if self.nav_state is not None else '?'
        msg = String()
        msg.data = (
            f'State: {self.mission_status} | '
            f'SLAM: {"OK" if self.slam_initialized else "INIT"} | '
            f'PX4: {"OK" if self.px4_connected else "WAIT"} | '
            f'arm: {"Y" if self.vehicle_armed else "N"} | '
            f'nav: {nav} | '
            f'Batt: {self.battery_level*100:.0f}%'
            f'{pos_str}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Mission Planner shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
