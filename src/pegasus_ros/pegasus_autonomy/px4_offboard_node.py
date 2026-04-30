#!/usr/bin/env python3
"""
Pegasus PX4 Offboard Interface Node
-------------------------------------
Bridges MPC trajectory setpoints to PX4 offboard control commands
over XRCE-DDS. This is the final link in the autonomy chain:

    A* → D* Lite 3D → MPC → THIS NODE → PX4 → motors

The node handles:
  1. Publishing OffboardControlMode at >2 Hz (PX4 requires this
     to stay in offboard mode — drops out after 0.5s without it)
  2. Converting MPC PoseStamped + TwistStamped setpoints to
     PX4 TrajectorySetpoint messages (ROS ENU → PX4 NED conversion)
  3. Arming and switching to offboard mode on command
  4. Safety: returns to HOLD mode if MPC stops publishing

Frame conventions:
  - ROS uses ENU (East-North-Up) with FLU body frame
  - PX4 uses NED (North-East-Down) with FRD body frame
  - Conversion: NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z

Publishes (to PX4 via XRCE-DDS):
    /fmu/in/trajectory_setpoint      (px4_msgs/TrajectorySetpoint)
    /fmu/in/offboard_control_mode    (px4_msgs/OffboardControlMode)
    /fmu/in/vehicle_command          (px4_msgs/VehicleCommand)

Subscribes:
    /pegasus/trajectory/setpoint           (PoseStamped — from MPC)
    /pegasus/trajectory/velocity_setpoint  (TwistStamped — from MPC)
    /pegasus/offboard/arm                  (Bool — arm command)
    /pegasus/offboard/engage               (Bool — switch to offboard)

Publishes (status):
    /pegasus/offboard/status         (String — JSON status)

Author: Team Pegasus — Cal Poly Pomona
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)


class PX4OffboardNode(Node):

    # PX4 VehicleCommand constants
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_DO_VTOL_TRANSITION = 3000   # param1: 3=MC, 4=FW

    def __init__(self):
        super().__init__('px4_offboard_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('offboard_heartbeat_hz', 50.0)
        self.declare_parameter('setpoint_timeout_s', 0.5)
        self.declare_parameter('auto_engage', False)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('default_altitude_ned_m', -10.0)

        self.heartbeat_hz = self.get_parameter('offboard_heartbeat_hz').value
        self.setpoint_timeout = self.get_parameter('setpoint_timeout_s').value
        self.auto_engage = self.get_parameter('auto_engage').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.default_alt = self.get_parameter('default_altitude_ned_m').value

        # ── PX4 QoS — must be BEST_EFFORT for XRCE-DDS ─────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── State ───────────────────────────────────────────
        self.latest_mpc_twist = None        # TwistStamped from MPC (body-frame)
        self.last_setpoint_time = 0.0
        self.latest_target_pose_enu = None  # PoseStamped from MPC, ENU map frame
        self.last_target_pose_time = 0.0
        self.current_yaw_ned = None         # latest yaw from /fmu/out/vehicle_odometry
        self.current_ned_pos = None         # (x, y, z) NED, latest from PX4
        self.offboard_engaged = False
        self.armed = False
        self.setpoint_count = 0
        self.heartbeat_count = 0
        # Lookahead horizon for integrating MPC velocity into a position
        # setpoint. 1.0 s gives PX4 a meaningful target ~6 m ahead at
        # max_speed_m_s=6, so its position controller has something to
        # actually pull toward instead of fighting NaN.
        self.position_lookahead_s = 1.0

        # ── Subscribers (from MPC) ──────────────────────────
        # MPC publishes a TwistStamped on this topic with:
        #   linear.x  = body-frame forward speed (m/s)
        #   linear.z  = vertical speed (ENU, up positive)
        #   angular.z = yaw rate (ENU CCW positive)
        # We use linear.z and angular.z directly. linear.x (body-fwd) is
        # ignored for multicopter offboard — MPC's unicycle model leaves
        # the drone rotating in place. Instead we take the position
        # target from /pegasus/trajectory/target_pose, which is the
        # path lookahead point in ENU/map frame.
        self.create_subscription(
            TwistStamped, '/pegasus/trajectory/setpoint',
            self._mpc_twist_cb, 10)
        self.create_subscription(
            PoseStamped, '/pegasus/trajectory/target_pose',
            self._target_pose_cb, 10)
        # Subscribe to the actual path so we can carrot-follow waypoint
        # segments rather than chord-cut from current straight to a far
        # lookahead. Without this, the drone shortcuts curves around
        # obstacles and clips walls before the planner has a chance to
        # route the next setpoint clear.
        self.create_subscription(
            Path, '/pegasus/path_planner/global_path',
            self._path_cb, 10)
        self.latest_path_ned = []   # list of (x, y, z) NED tuples
        self.create_subscription(
            Bool, '/pegasus/offboard/arm',
            self._arm_cb, 10)
        self.create_subscription(
            Bool, '/pegasus/offboard/engage',
            self._engage_cb, 10)

        # ── Subscribers (from PX4 — for status monitoring) ──
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._vehicle_odom_cb, px4_qos)
        # PX4 v1.15+ publishes the versioned _v1 topic. We rely on this
        # for real arm / offboard feedback rather than optimistic flags.
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._vehicle_status_cb, px4_qos)
        self._last_engage_attempt = 0.0
        self._engage_retry_interval = 0.3  # seconds between arm/mode-switch retries
        self._engage_warmup_ticks = 15      # heartbeats before first try (~0.3s @ 50Hz)

        # ── Publishers (to PX4 via XRCE-DDS) ────────────────
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint', px4_qos)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode', px4_qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command', px4_qos)

        # ── Status publisher ────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/pegasus/offboard/status', 10)

        # ── Timers ──────────────────────────────────────────
        self.create_timer(
            1.0 / self.heartbeat_hz, self._heartbeat_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'PX4 Offboard Interface: heartbeat={self.heartbeat_hz}Hz, '
            f'timeout={self.setpoint_timeout}s, '
            f'auto_engage={self.auto_engage}, auto_arm={self.auto_arm}')

    # ── Callbacks ────────────────────────────────────────────

    def _mpc_twist_cb(self, msg: TwistStamped):
        self.latest_mpc_twist = msg
        self.last_setpoint_time = self.get_clock().now().nanoseconds / 1e9

    def _target_pose_cb(self, msg: PoseStamped):
        """Path lookahead point from MPC, in ENU/map frame."""
        self.latest_target_pose_enu = msg
        self.last_target_pose_time = self.get_clock().now().nanoseconds / 1e9

    def _path_cb(self, msg: Path):
        """Cache the live path (D* output) in NED for carrot-on-path."""
        self.latest_path_ned = [
            (float(p.pose.position.y),       # NED N = ENU Y
             float(p.pose.position.x),       # NED E = ENU X
             float(-p.pose.position.z))      # NED D = -ENU Up
            for p in msg.poses
        ]

    def _carrot_on_path(self, cx, cy, max_step):
        """Continuous carrot along the path: project current pose onto
        the nearest path segment, then advance exactly max_step metres
        along the path from that projection. Returns (nx, ny) in NED,
        or None if the path is unavailable.

        Continuous projection (not discrete waypoint jumps) removes the
        jitter when the drone is between waypoints — the carrot moves
        smoothly forward as the drone advances rather than snapping
        from one waypoint to the next."""
        path = self.latest_path_ned
        if not path or len(path) < 2:
            return None

        # 1. Find the path segment whose perpendicular projection is
        # closest to the drone, and where on that segment (parameter t).
        best_seg = 0
        best_t = 0.0
        best_d2 = float('inf')
        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
            sx = p1[0] - p0[0]
            sy = p1[1] - p0[1]
            seg_len2 = sx * sx + sy * sy
            if seg_len2 < 1e-6:
                continue
            t = ((cx - p0[0]) * sx + (cy - p0[1]) * sy) / seg_len2
            t = max(0.0, min(1.0, t))
            px = p0[0] + t * sx
            py = p0[1] + t * sy
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_seg = i
                best_t = t

        # 2. Walk forward along path from (best_seg, best_t) by max_step.
        seg_idx = best_seg
        seg_t = best_t
        remaining = max_step
        while seg_idx < len(path) - 1:
            p0 = path[seg_idx]
            p1 = path[seg_idx + 1]
            seg_len = math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2)
            seg_remaining = seg_len * (1.0 - seg_t)
            if seg_remaining >= remaining and seg_len > 1e-6:
                t_advance = remaining / seg_len
                new_t = seg_t + t_advance
                return (p0[0] + new_t * (p1[0] - p0[0]),
                        p0[1] + new_t * (p1[1] - p0[1]))
            remaining -= seg_remaining
            seg_idx += 1
            seg_t = 0.0
        # Fell off the end — head for the goal
        last = path[-1]
        return (last[0], last[1])

    def _vehicle_odom_cb(self, msg: VehicleOdometry):
        """PX4 publishes orientation as quaternion [w, x, y, z] and
        position[3] in NED. We need both: the yaw for body→world
        rotation, and the position to integrate velocity into a
        position setpoint (PX4 won't act on velocity feedforward
        alone when position is NaN in v1.15+ offboard mode)."""
        q = msg.q  # [w, x, y, z]
        siny_cosp = 2.0 * (q[0]*q[3] + q[1]*q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3])
        self.current_yaw_ned = math.atan2(siny_cosp, cosy_cosp)
        self.current_ned_pos = (
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        )

    def _vehicle_status_cb(self, msg: VehicleStatus):
        """Update real arm + nav-state from PX4 feedback. Replaces the
        previous optimistic flag-setting in arm() / engage_offboard()."""
        self.armed = (msg.arming_state == 2)  # ARMING_STATE_ARMED
        self.offboard_engaged = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _arm_cb(self, msg: Bool):
        if msg.data:
            self.arm()
        else:
            self.disarm()

    def _engage_cb(self, msg: Bool):
        if msg.data:
            self.engage_offboard()
        else:
            self.offboard_engaged = False
            self.get_logger().info('Offboard mode disengaged')

    # ── Main heartbeat loop ──────────────────────────────────

    def _heartbeat_tick(self):
        """
        Runs at offboard_heartbeat_hz (~50 Hz).

        PX4 requires OffboardControlMode at >2 Hz to stay in offboard
        mode. We publish it every tick alongside the trajectory setpoint.
        If MPC stops publishing setpoints for longer than setpoint_timeout,
        we stop sending trajectory commands (PX4 will fall back to HOLD).
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # Always publish OffboardControlMode (PX4 heartbeat)
        self._publish_offboard_control_mode()
        self.heartbeat_count += 1

        # Always publish a TrajectorySetpoint. PX4 refuses to enter
        # OFFBOARD without a fresh setpoint stream. When MPC's twist is
        # stale or unavailable, _publish_trajectory_setpoint emits a
        # zero-velocity hover at default_altitude_ned_m.
        self._publish_trajectory_setpoint()
        self.setpoint_count += 1

        # Auto-engage sequence: send arm + mode-switch repeatedly until
        # /fmu/out/vehicle_status_v1 confirms armed + OFFBOARD. The
        # original code fired both commands once optimistically, which
        # missed the window when PX4's EKF/GPS had not converged yet.
        if self.auto_engage and self.heartbeat_count > self._engage_warmup_ticks:
            if (now - self._last_engage_attempt) >= self._engage_retry_interval:
                if not self.armed and self.auto_arm:
                    self.arm()
                if not self.offboard_engaged:
                    self.engage_offboard()
                self._last_engage_attempt = now

    # ── ENU → NED conversion + publishing ────────────────────

    def _publish_trajectory_setpoint(self):
        """Build a PX4 TrajectorySetpoint from the latest MPC TwistStamped.

        MPC publishes body-frame forward speed + vz + yaw_rate in ENU.
        PX4 expects NED, world-frame velocity. Conversion:

            world_vx_enu = speed * cos(enu_yaw)
            world_vy_enu = speed * sin(enu_yaw)
            ned_vx       = world_vy_enu      (North = ENU Y)
            ned_vy       = world_vx_enu      (East  = ENU X)
            ned_vz       = -vz_enu           (Down  = -ENU Z)
            ned_yaw_rate = -yaw_rate_enu     (CW    = -CCW)

        Z-position is held at default_alt (NED, negative = up). XY position
        is left NaN so PX4 controls it purely via the velocity setpoint.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        now = self.get_clock().now().nanoseconds / 1e9
        target_age = now - self.last_target_pose_time if self.last_target_pose_time > 0 else float('inf')
        target_fresh = (self.latest_target_pose_enu is not None
                        and target_age <= self.setpoint_timeout * 4)

        twist_age = now - self.last_setpoint_time if self.last_setpoint_time > 0 else float('inf')
        twist_fresh = (self.latest_mpc_twist is not None
                       and twist_age <= self.setpoint_timeout)

        if target_fresh and self.current_ned_pos is not None:
            # Path lookahead point in ENU map frame → NED. We ignore the
            # path's z entirely and always cruise at default_alt — the
            # planner's z values can drift from the drone's cruise
            # altitude, causing it to chase the path vertically and never
            # settle. Lawnmower coverage is a 2D problem at fixed alt.
            p = self.latest_target_pose_enu.pose.position
            tx_ned = float(p.y)              # NED N = ENU Y
            ty_ned = float(p.x)              # NED E = ENU X

            cx, cy, cz = self.current_ned_pos

            # Altitude priority: if more than 2 m off cruise altitude,
            # climb/descend in place before moving laterally. Stops the
            # drone tilting hard on takeoff while trying to fly 30 m
            # away.
            altitude_error = abs(cz - self.default_alt)
            if altitude_error > 2.0:
                msg.position[0] = cx
                msg.position[1] = cy
                msg.position[2] = self.default_alt
            else:
                # Carrot-follow the actual path. Continuous projection
                # along path segments avoids chord-cutting and produces
                # smooth motion. 3 m is a reasonable speed/safety
                # tradeoff: PX4 tracks ~3 m/s, fast enough for a real
                # test loop but slow enough that D* can replan before
                # the drone reaches the next obstacle.
                MAX_STEP_M = 3.0
                carrot_xy = self._carrot_on_path(cx, cy, MAX_STEP_M)
                if carrot_xy is not None:
                    msg.position[0] = carrot_xy[0]
                    msg.position[1] = carrot_xy[1]
                    dy_full = carrot_xy[1] - cy
                    dx_full = carrot_xy[0] - cx
                    d_horiz = math.sqrt(dx_full * dx_full + dy_full * dy_full)
                else:
                    dx_full = tx_ned - cx
                    dy_full = ty_ned - cy
                    d_horiz = math.sqrt(dx_full * dx_full + dy_full * dy_full)
                    dx, dy = dx_full, dy_full
                    if d_horiz > MAX_STEP_M:
                        scale = MAX_STEP_M / d_horiz
                        dx = dx_full * scale
                        dy = dy_full * scale
                    msg.position[0] = cx + dx
                    msg.position[1] = cy + dy
                msg.position[2] = self.default_alt   # always cruise alt

                # Face direction of travel
                if d_horiz > 0.5:
                    msg.yaw = math.atan2(dy_full, dx_full)
        elif self.current_ned_pos is not None:
            # No fresh target → hover in place at default alt
            msg.position[0] = self.current_ned_pos[0]
            msg.position[1] = self.current_ned_pos[1]
            msg.position[2] = self.default_alt
        else:
            msg.position[0] = float('nan')
            msg.position[1] = float('nan')
            msg.position[2] = self.default_alt

        # Velocity feedforward + yawspeed from MPC (still useful even
        # when position comes from the path).
        if twist_fresh:
            vz_enu       = float(self.latest_mpc_twist.twist.linear.z)
            yaw_rate_enu = float(self.latest_mpc_twist.twist.angular.z)
            msg.velocity[0] = float('nan')   # let PX4 derive xy velocity from position
            msg.velocity[1] = float('nan')
            msg.velocity[2] = -vz_enu
            msg.yawspeed    = -yaw_rate_enu
        else:
            msg.velocity[0] = float('nan')
            msg.velocity[1] = float('nan')
            msg.velocity[2] = float('nan')
            msg.yawspeed    = 0.0

        msg.yaw = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')

        self.traj_pub.publish(msg)

    def _publish_offboard_control_mode(self):
        """
        Tell PX4 which axes we're commanding.
        Position + velocity = PX4 uses position controller with
        velocity feedforward.
        """
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    # ── Arm / Disarm / Engage ────────────────────────────────

    def arm(self):
        """Send arm command to PX4. State (self.armed) is updated from
        /fmu/out/vehicle_status_v1 in _vehicle_status_cb, not here."""
        self._send_vehicle_command(
            self.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)
        self.get_logger().info('ARM command sent', throttle_duration_sec=2.0)

    def disarm(self):
        """Send disarm command to PX4."""
        self._send_vehicle_command(
            self.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0)
        self.get_logger().info('DISARM command sent', throttle_duration_sec=2.0)

    def engage_offboard(self):
        """Switch PX4 to offboard mode. State updated from feedback."""
        # PX4 custom mode for OFFBOARD: main_mode=6, sub_mode=0
        # Encoded as: base_mode=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        # For DO_SET_MODE: param1=base_mode, param2=custom_main_mode
        self._send_vehicle_command(
            self.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0)    # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.get_logger().info(
            'OFFBOARD mode command sent', throttle_duration_sec=2.0)

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0,
                               param3=0.0, param4=0.0, param5=0.0,
                               param6=0.0, param7=0.0):
        """Send a VehicleCommand to PX4."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    # ── Status ───────────────────────────────────────────────

    def _publish_status(self):
        now = self.get_clock().now().nanoseconds / 1e9
        age = now - self.last_setpoint_time if self.last_setpoint_time > 0 else -1.0

        status = {
            "armed": self.armed,
            "offboard_engaged": self.offboard_engaged,
            "has_setpoint": self.latest_mpc_twist is not None,
            "setpoint_age_s": round(age, 2),
            "setpoints_sent": self.setpoint_count,
            "heartbeats_sent": self.heartbeat_count,
            "auto_engage": self.auto_engage,
            "auto_arm": self.auto_arm,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Offboard interface shutting down')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()