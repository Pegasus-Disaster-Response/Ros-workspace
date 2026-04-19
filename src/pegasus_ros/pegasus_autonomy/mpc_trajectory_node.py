#!/usr/bin/env python3
import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


class MPCTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('mpc_tracker_node')

        self.declare_parameter('path_topic', '/mpc_sim/reference_path')
        self.declare_parameter('odom_topic', '/mpc_sim/odom')
        self.declare_parameter('cmd_topic', '/mpc_sim/cmd_vel')
        self.declare_parameter('status_topic', '/mpc_sim/status')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('lookahead_points', 6)
        self.declare_parameter('min_speed_m_s', 0.8)
        self.declare_parameter('max_speed_m_s', 4.0)
        self.declare_parameter('max_yaw_rate_rad_s', 1.2)
        self.declare_parameter('max_vz_m_s', 1.5)
        self.declare_parameter('arrival_tolerance_m', 1.0)
        self.declare_parameter('global_reacquire_distance_m', 8.0)
        self.declare_parameter('loop_path', True)
        self.declare_parameter('near_target_slowdown_distance_m', 6.0)
        self.declare_parameter('min_speed_near_target_m_s', 0.15)

        self.path_topic = self.get_parameter('path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = float(self.get_parameter('dt').value)
        self.horizon_steps = int(self.get_parameter('horizon_steps').value)
        self.lookahead_points = int(self.get_parameter('lookahead_points').value)
        self.min_speed = float(self.get_parameter('min_speed_m_s').value)
        self.max_speed = float(self.get_parameter('max_speed_m_s').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self.max_vz = float(self.get_parameter('max_vz_m_s').value)
        self.arrival_tol = float(self.get_parameter('arrival_tolerance_m').value)
        self.global_reacquire_distance = float(self.get_parameter('global_reacquire_distance_m').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.near_target_slowdown_distance = float(self.get_parameter('near_target_slowdown_distance_m').value)
        self.min_speed_near_target = float(self.get_parameter('min_speed_near_target_m_s').value)

        self.path_points: List[Tuple[float, float, float]] = []
        self.current_state: Optional[Tuple[float, float, float, float]] = None
        self.last_nearest_index = 0

        self.create_subscription(Path, self.path_topic, self._path_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_timer(1.0 / max(1.0, self.control_rate_hz), self._control_tick)

    def _path_cb(self, msg: Path) -> None:
        self.path_points = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in msg.poses
        ]
        if self.last_nearest_index >= len(self.path_points):
            self.last_nearest_index = 0

    def _quat_to_yaw(self, z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        yaw = self._quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.current_state = (x, y, z, yaw)

    def _find_nearest_index(self, x: float, y: float, z: float) -> int:
        if not self.path_points:
            return 0

        def dist_sq(idx: int) -> float:
            px, py, pz = self.path_points[idx]
            return (px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2

        if self.loop_path:
            nearest = min(range(len(self.path_points)), key=dist_sq)
            self.last_nearest_index = nearest
            return nearest

        start = max(0, self.last_nearest_index - 10)
        end = min(len(self.path_points), self.last_nearest_index + 30)
        if start >= end:
            start, end = 0, len(self.path_points)

        nearest = start
        best = float('inf')
        for idx in range(start, end):
            d = dist_sq(idx)
            if d < best:
                best = d
                nearest = idx

        if math.sqrt(best) > self.global_reacquire_distance:
            nearest = min(range(len(self.path_points)), key=dist_sq)

        self.last_nearest_index = nearest
        return nearest

    def _simulate_cost(
        self,
        state: Tuple[float, float, float, float],
        target: Tuple[float, float, float],
        speed: float,
        yaw_rate: float,
        vz: float,
    ) -> float:
        x, y, z, yaw = state
        total_cost = 0.0

        for _ in range(self.horizon_steps):
            yaw += yaw_rate * self.dt
            x += speed * math.cos(yaw) * self.dt
            y += speed * math.sin(yaw) * self.dt
            z += vz * self.dt

            tx, ty, tz = target
            dx = tx - x
            dy = ty - y
            dz = tz - z
            pos_cost = dx * dx + dy * dy + 1.2 * dz * dz

            desired_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
            heading_cost = 0.3 * yaw_err * yaw_err
            control_cost = 0.05 * (speed * speed + yaw_rate * yaw_rate + vz * vz)

            total_cost += pos_cost + heading_cost + control_cost

        terminal_dx = target[0] - x
        terminal_dy = target[1] - y
        terminal_dz = target[2] - z
        total_cost += 2.0 * (terminal_dx * terminal_dx + terminal_dy * terminal_dy + terminal_dz * terminal_dz)

        return total_cost

    def _publish_command(self, speed: float, yaw_rate: float, vz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(speed)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(msg)

    def _publish_status(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _control_tick(self) -> None:
        if self.current_state is None or len(self.path_points) < 2:
            self._publish_command(0.0, 0.0, 0.0)
            self._publish_status({'mpc_active': False, 'reason': 'waiting_for_path_or_odom'})
            return

        x, y, z, yaw = self.current_state
        nearest_idx = self._find_nearest_index(x, y, z)
        if self.loop_path:
            target_idx = (nearest_idx + self.lookahead_points) % len(self.path_points)
        else:
            target_idx = min(len(self.path_points) - 1, nearest_idx + self.lookahead_points)
        target = self.path_points[target_idx]
        final_goal = self.path_points[-1]

        dist_to_goal = math.sqrt(
            (final_goal[0] - x) ** 2 + (final_goal[1] - y) ** 2 + (final_goal[2] - z) ** 2
        )
        if (not self.loop_path) and dist_to_goal < self.arrival_tol:
            self._publish_command(0.0, 0.0, 0.0)
            self._publish_status({
                'mpc_active': False,
                'reason': 'goal_reached',
                'distance_to_goal_m': round(dist_to_goal, 3),
            })
            return

        dist_to_target = math.sqrt(
            (target[0] - x) ** 2 + (target[1] - y) ** 2 + (target[2] - z) ** 2
        )

        if dist_to_target < self.near_target_slowdown_distance:
            scale = max(0.0, min(1.0, dist_to_target / max(0.01, self.near_target_slowdown_distance)))
            effective_min_speed = self.min_speed_near_target + (self.min_speed - self.min_speed_near_target) * scale
        else:
            effective_min_speed = self.min_speed

        speed_candidates = [0.0, effective_min_speed, 0.5 * (effective_min_speed + self.max_speed), self.max_speed]
        speed_candidates = sorted({round(max(0.0, min(self.max_speed, s)), 4) for s in speed_candidates})
        yaw_candidates = [-self.max_yaw_rate, -0.5 * self.max_yaw_rate, 0.0, 0.5 * self.max_yaw_rate, self.max_yaw_rate]
        vz_candidates = [-self.max_vz, -0.5 * self.max_vz, 0.0, 0.5 * self.max_vz, self.max_vz]

        best = None
        best_cost = float('inf')
        for speed in speed_candidates:
            for yaw_rate in yaw_candidates:
                for vz in vz_candidates:
                    cost = self._simulate_cost((x, y, z, yaw), target, speed, yaw_rate, vz)
                    if cost < best_cost:
                        best_cost = cost
                        best = (speed, yaw_rate, vz)

        if best is None:
            self._publish_command(0.0, 0.0, 0.0)
            return

        speed_cmd, yaw_rate_cmd, vz_cmd = best
        self._publish_command(speed_cmd, yaw_rate_cmd, vz_cmd)
        self._publish_status({
            'mpc_active': True,
            'nearest_index': int(nearest_idx),
            'target_index': int(target_idx),
            'distance_to_target_m': round(dist_to_target, 3),
            'distance_to_goal_m': round(dist_to_goal, 3),
            'speed_cmd_m_s': round(speed_cmd, 3),
            'yaw_rate_cmd_rad_s': round(yaw_rate_cmd, 3),
            'vz_cmd_m_s': round(vz_cmd, 3),
            'objective': round(best_cost, 3),
        })


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
