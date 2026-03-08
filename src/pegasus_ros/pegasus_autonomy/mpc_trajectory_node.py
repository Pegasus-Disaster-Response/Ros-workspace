#!/usr/bin/env python3
"""
Pegasus MPC Trajectory Smoother Node
--------------------------------------
Converts the D* Lite local path (jagged grid waypoints) into a smooth,
kinodynamically feasible trajectory that respects the VTOL's physical
limits. Outputs position+velocity setpoints for PX4 offboard control.

Pipeline position:
    A* → D* Lite → MPC (THIS NODE) → PX4 Offboard Interface

The MPC uses a simple receding-horizon approach:
  1. Sample a lookahead window from the local path
  2. Fit a smooth trajectory (cubic spline) through the waypoints
  3. Enforce velocity, acceleration, and turn-rate limits
  4. Output setpoints at the control rate (50 Hz)

Placeholder dynamics are based on typical eVTOL specs (~50 kg class).
Update config/vtol_dynamics.yaml with your actual aircraft parameters.

Publishes:
    /pegasus/trajectory/setpoint     (geometry_msgs/PoseStamped — current target)
    /pegasus/trajectory/predicted     (nav_msgs/Path — MPC predicted trajectory)
    /pegasus/trajectory/status        (std_msgs/String — JSON status)

Subscribes:
    /pegasus/path_planner/local_path (nav_msgs/Path — from D* Lite)
    /odom                            (nav_msgs/Odometry — current state)

Author: Team Pegasus — Cal Poly Pomona
"""

import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header, String


class MPCTrajectoryNode(Node):

    def __init__(self):
        super().__init__('mpc_trajectory_node')

        # ── VTOL Dynamics Parameters ────────────────────────
        # These are loaded from vtol_dynamics.yaml.
        # Placeholder values based on ~50 kg eVTOL, 15ft wingspan.
        self.declare_parameter('vtol.mass_kg', 50.0)
        self.declare_parameter('vtol.wingspan_m', 4.572)
        self.declare_parameter('vtol.max_speed_ms', 20.0)
        self.declare_parameter('vtol.min_speed_ms', 0.0)
        self.declare_parameter('vtol.max_accel_ms2', 3.0)
        self.declare_parameter('vtol.max_decel_ms2', 4.0)
        self.declare_parameter('vtol.max_climb_rate_ms', 3.0)
        self.declare_parameter('vtol.max_descent_rate_ms', 2.0)
        self.declare_parameter('vtol.max_bank_angle_deg', 30.0)
        self.declare_parameter('vtol.max_yaw_rate_degs', 60.0)
        self.declare_parameter('vtol.flight_mode', 'MC')

        # ── MPC Parameters ──────────────────────────────────
        self.declare_parameter('mpc.control_rate_hz', 50.0)
        self.declare_parameter('mpc.horizon_s', 2.0)
        self.declare_parameter('mpc.horizon_steps', 20)
        self.declare_parameter('mpc.lookahead_m', 15.0)
        self.declare_parameter('mpc.path_smoothing_factor', 0.3)
        self.declare_parameter('mpc.waypoint_reach_tolerance_m', 1.5)

        # ── Altitude constraints ────────────────────────────
        self.declare_parameter('altitude.min_m', 5.0)
        self.declare_parameter('altitude.max_m', 120.0)

        # Read parameters
        self.mass = self.get_parameter('vtol.mass_kg').value
        self.wingspan = self.get_parameter('vtol.wingspan_m').value
        self.max_speed = self.get_parameter('vtol.max_speed_ms').value
        self.min_speed = self.get_parameter('vtol.min_speed_ms').value
        self.max_accel = self.get_parameter('vtol.max_accel_ms2').value
        self.max_decel = self.get_parameter('vtol.max_decel_ms2').value
        self.max_climb = self.get_parameter('vtol.max_climb_rate_ms').value
        self.max_descent = self.get_parameter('vtol.max_descent_rate_ms').value
        self.max_bank_rad = math.radians(
            self.get_parameter('vtol.max_bank_angle_deg').value)
        self.max_yaw_rate = math.radians(
            self.get_parameter('vtol.max_yaw_rate_degs').value)
        self.flight_mode = self.get_parameter('vtol.flight_mode').value

        self.control_hz = self.get_parameter('mpc.control_rate_hz').value
        self.horizon_s = self.get_parameter('mpc.horizon_s').value
        self.horizon_steps = self.get_parameter('mpc.horizon_steps').value
        self.lookahead = self.get_parameter('mpc.lookahead_m').value
        self.smooth_factor = self.get_parameter('mpc.path_smoothing_factor').value
        self.wp_tolerance = self.get_parameter('mpc.waypoint_reach_tolerance_m').value
        self.min_alt = self.get_parameter('altitude.min_m').value
        self.max_alt = self.get_parameter('altitude.max_m').value

        # Derived: minimum turn radius in FW mode
        # R = v² / (g * tan(bank_angle))
        if self.flight_mode == 'FW' and self.max_bank_rad > 0.01:
            self.min_turn_radius = (
                self.max_speed ** 2 /
                (9.81 * math.tan(self.max_bank_rad)))
        else:
            self.min_turn_radius = 0.0  # MC mode can turn in place

        # ── State ───────────────────────────────────────────
        self.local_path = None
        self.current_odom = None
        self.current_wp_idx = 0
        self.controller_state = "IDLE"
        self.setpoint_count = 0

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            Path, '/pegasus/path_planner/local_path',
            self._local_path_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── Publishers ──────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/pegasus/trajectory/setpoint', 10)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/pegasus/trajectory/velocity_setpoint', 10)
        self.predicted_pub = self.create_publisher(
            Path, '/pegasus/trajectory/predicted', 10)
        self.status_pub = self.create_publisher(
            String, '/pegasus/trajectory/status', 10)

        # ── Timers ──────────────────────────────────────────
        self.create_timer(1.0 / self.control_hz, self._control_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'MPC Trajectory: {self.control_hz}Hz, horizon={self.horizon_s}s, '
            f'mode={self.flight_mode}, mass={self.mass}kg, '
            f'max_speed={self.max_speed}m/s, '
            f'min_turn_r={self.min_turn_radius:.1f}m')

    # ── Callbacks ────────────────────────────────────────────

    def _local_path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.local_path = None
            self.controller_state = "WAITING_FOR_PATH"
            return
        self.local_path = msg
        self.current_wp_idx = 0
        self.controller_state = "TRACKING"

    def _odom_cb(self, msg: Odometry):
        self.current_odom = msg

    # ── Control Loop ─────────────────────────────────────────

    def _control_tick(self):
        """Main MPC control loop at control_rate_hz."""
        if self.local_path is None or self.current_odom is None:
            return
        if self.controller_state != "TRACKING":
            return

        poses = self.local_path.poses
        if self.current_wp_idx >= len(poses):
            self.controller_state = "GOAL_REACHED"
            self.get_logger().info('MPC: trajectory complete')
            return

        # Current state
        px = self.current_odom.pose.pose.position.x
        py = self.current_odom.pose.pose.position.y
        pz = self.current_odom.pose.pose.position.z
        vx = self.current_odom.twist.twist.linear.x
        vy = self.current_odom.twist.twist.linear.y
        vz = self.current_odom.twist.twist.linear.z
        speed = math.sqrt(vx*vx + vy*vy)

        # Advance waypoint index if close enough
        while self.current_wp_idx < len(poses) - 1:
            wp = poses[self.current_wp_idx].pose.position
            dx = wp.x - px
            dy = wp.y - py
            if math.sqrt(dx*dx + dy*dy) < self.wp_tolerance:
                self.current_wp_idx += 1
            else:
                break

        # Collect lookahead window from current waypoint
        lookahead_pts = []
        dist_accum = 0.0
        for i in range(self.current_wp_idx, len(poses)):
            p = poses[i].pose.position
            lookahead_pts.append((p.x, p.y, p.z))
            if i > self.current_wp_idx:
                prev = poses[i-1].pose.position
                seg = math.sqrt(
                    (p.x - prev.x)**2 + (p.y - prev.y)**2)
                dist_accum += seg
                if dist_accum >= self.lookahead:
                    break

        if not lookahead_pts:
            return

        # ── Smooth trajectory through lookahead points ──
        # Simple approach: exponential moving average smoothing
        # then enforce velocity/acceleration limits per step.
        smooth_pts = self._smooth_path(lookahead_pts, px, py, pz)

        # ── Enforce kinodynamic constraints ──
        constrained = self._apply_constraints(
            smooth_pts, speed, px, py, pz)

        # ── Publish the immediate setpoint ──
        if constrained:
            target = constrained[0]
            self._publish_setpoint(target, constrained)

            # Publish predicted trajectory
            self._publish_predicted(constrained)

            self.setpoint_count += 1

    def _smooth_path(self, pts, px, py, pz):
        """
        Smooth waypoints using exponential filter.
        Prepend current position for continuity.
        """
        if len(pts) < 2:
            return pts

        # Prepend current position
        all_pts = [(px, py, pz)] + list(pts)

        # Exponential moving average
        alpha = self.smooth_factor
        smoothed = [all_pts[0]]
        for i in range(1, len(all_pts)):
            sx = alpha * all_pts[i][0] + (1 - alpha) * smoothed[-1][0]
            sy = alpha * all_pts[i][1] + (1 - alpha) * smoothed[-1][1]
            sz = alpha * all_pts[i][2] + (1 - alpha) * smoothed[-1][2]
            smoothed.append((sx, sy, sz))

        return smoothed[1:]  # remove prepended current pos

    def _apply_constraints(self, pts, current_speed, px, py, pz):
        """
        Enforce velocity, acceleration, and altitude constraints.
        Returns list of (x, y, z, vx, vy, vz) tuples.
        """
        if not pts:
            return []

        dt = self.horizon_s / max(self.horizon_steps, 1)
        result = []

        prev_x, prev_y, prev_z = px, py, pz
        prev_speed = current_speed

        for i, (tx, ty, tz) in enumerate(pts[:self.horizon_steps]):
            # Direction to target
            dx = tx - prev_x
            dy = ty - prev_y
            dz = tz - prev_z
            dist_2d = math.sqrt(dx*dx + dy*dy)

            if dist_2d < 0.01:
                result.append((tx, ty, tz, 0.0, 0.0, 0.0))
                continue

            # Desired speed: proportional to distance, clamped
            desired_speed = min(self.max_speed, dist_2d / dt)

            # Enforce acceleration limits
            speed_diff = desired_speed - prev_speed
            if speed_diff > 0:
                desired_speed = min(
                    desired_speed,
                    prev_speed + self.max_accel * dt)
            else:
                desired_speed = max(
                    desired_speed,
                    prev_speed - self.max_decel * dt)

            desired_speed = max(self.min_speed, desired_speed)

            # Velocity components
            heading = math.atan2(dy, dx)

            # Check yaw rate limit
            if i > 0 and len(result) > 0:
                prev_heading = math.atan2(
                    result[-1][4], result[-1][3])
                if abs(result[-1][3]) + abs(result[-1][4]) > 0.01:
                    yaw_diff = heading - prev_heading
                    # Normalize to [-pi, pi]
                    yaw_diff = (yaw_diff + math.pi) % (2*math.pi) - math.pi
                    max_yaw_step = self.max_yaw_rate * dt
                    if abs(yaw_diff) > max_yaw_step:
                        heading = prev_heading + math.copysign(
                            max_yaw_step, yaw_diff)

            vx_cmd = desired_speed * math.cos(heading)
            vy_cmd = desired_speed * math.sin(heading)

            # Vertical rate limits
            vz_desired = dz / dt if dt > 0 else 0.0
            vz_cmd = max(-self.max_descent,
                         min(self.max_climb, vz_desired))

            # Altitude clamp
            new_z = prev_z + vz_cmd * dt
            new_z = max(self.min_alt, min(self.max_alt, new_z))

            # Position prediction
            new_x = prev_x + vx_cmd * dt
            new_y = prev_y + vy_cmd * dt

            result.append((new_x, new_y, new_z, vx_cmd, vy_cmd, vz_cmd))

            prev_x, prev_y, prev_z = new_x, new_y, new_z
            prev_speed = math.sqrt(vx_cmd*vx_cmd + vy_cmd*vy_cmd)

        return result

    def _publish_setpoint(self, target, trajectory):
        """Publish immediate position setpoint."""
        msg = PoseStamped()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='map')
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]

        # Yaw from velocity
        if abs(target[3]) + abs(target[4]) > 0.01:
            yaw = math.atan2(target[4], target[3])
        else:
            yaw = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.setpoint_pub.publish(msg)

        # Also publish velocity setpoint
        vel_msg = TwistStamped()
        vel_msg.header = msg.header
        vel_msg.twist.linear.x = target[3]
        vel_msg.twist.linear.y = target[4]
        vel_msg.twist.linear.z = target[5]
        self.velocity_pub.publish(vel_msg)

    def _publish_predicted(self, trajectory):
        """Publish the MPC predicted trajectory for visualization."""
        path_msg = Path()
        path_msg.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='map')
        for x, y, z, vx, vy, vz in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            if abs(vx) + abs(vy) > 0.01:
                yaw = math.atan2(vy, vx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.predicted_pub.publish(path_msg)

    def _publish_status(self):
        status = {
            "state": self.controller_state,
            "flight_mode": self.flight_mode,
            "has_path": self.local_path is not None,
            "has_odom": self.current_odom is not None,
            "current_wp_idx": self.current_wp_idx,
            "total_waypoints": len(self.local_path.poses) if self.local_path else 0,
            "setpoints_published": self.setpoint_count,
            "min_turn_radius_m": round(self.min_turn_radius, 2),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()