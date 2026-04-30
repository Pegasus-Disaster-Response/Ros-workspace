#!/usr/bin/env python3
import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker


class MPCTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('mpc_tracker_node')

        self.declare_parameter('path_topic', '/mpc_sim/reference_path')
        self.declare_parameter('odom_topic', '/mpc_sim/odom')
        self.declare_parameter('cmd_topic', '/mpc_sim/cmd_vel')
        self.declare_parameter('status_topic', '/mpc_sim/status')

        # [EST] 20 Hz is sufficient; P110 actuator bandwidth not yet measured.
        # Raise to 50 Hz after actuator time constants are confirmed from bench test.
        self.declare_parameter('control_rate_hz', 20.0)

        # [EST] MPC prediction timestep. At cruise (18 m/s), 0.5 s/step → ~9 m per step.
        # Shorter dt increases compute load; longer dt loses resolution in tight turns.
        self.declare_parameter('dt', 0.5)

        # [EST] 20 steps × 0.5 s = 10 s / ~180 m horizon at cruise.
        # Covers the deceleration window and a full approach segment.
        self.declare_parameter('horizon_steps', 20)

        # [EST] Waypoints ahead to use as the MPC target. At ~5 m path resolution,
        # 10 points ≈ 50 m lookahead, giving ~2.8 s foresight at cruise.
        self.declare_parameter('lookahead_points', 10)

        # [PDR] FW stall speed = 12.0 m/s (must keep for fixed-wing flight).
        # SITL test uses the VTOL in multicopter mode where hover (0 m/s) is fine,
        # so we drop the floor here. Restore to 12.0 for fixed-wing missions.
        self.declare_parameter('min_speed_m_s', 0.0)

        # [PDR] Nominal cruise 18.01 m/s. Capped at 6.0 for SITL test on the
        # 30x30 m search box — gives readable on-screen motion at full RTF
        # and avoids overshooting waypoints by a full lane width per tick.
        self.declare_parameter('max_speed_m_s', 6.0)

        # [EST] FW yaw rate is bank-angle limited, not motor limited.
        # At cruise (18 m/s) with 35° bank: R = v²/(g·tan35°) ≈ 47 m → ω = v/R ≈ 0.38 rad/s.
        # 0.40 rad/s adds a small margin. Tighten after flight-test turn data.
        self.declare_parameter('max_yaw_rate_rad_s', 0.40)

        # [PDR] Max rate of climb: 24.15 mph = 10.80 m/s.
        # Descent rate not in PDR — [EST] 8.0 m/s (≈ 0.74× climb rate). Confirm from flight test.
        self.declare_parameter('max_vz_m_s', 10.80)

        # [EST] At 12 m/s near-target (stall-limited), 15 m gives ~1.25 s to confirm arrival.
        # Tighten after testing GPS/odom accuracy at low altitude.
        # SITL test value: 3 m. Original 15 m makes the drone stop almost immediately
        # in the small 30x30 m lawnmower box (half the path is within 15 m of every
        # waypoint). Keep at 3 m here for the test loop; raise back to 15 m for
        # real fixed-wing flight where stopping precisely at a corner doesn't matter.
        self.declare_parameter('arrival_tolerance_m', 3.0)

        # [EST] At 18 m/s cruise, path divergence of 50 m is reachable in ~2.8 s.
        # If the vehicle strays farther than this, do a full nearest-point search.
        self.declare_parameter('global_reacquire_distance_m', 50.0)

        # P110 is a point-to-point mission vehicle, not a looping platform.
        self.declare_parameter('loop_path', False)

        # [EST] Deceleration from cruise (18 m/s) to stall (12 m/s) at ~3 m/s²
        # requires ~12 m. 60 m gives comfortable margin and smooth speed ramp.
        self.declare_parameter('near_target_slowdown_distance_m', 60.0)

        # [PDR] FW stall floor = 12.0 m/s (don't reduce in fixed-wing flight).
        # SITL test value 0.0 lets the multicopter slow to a stop near a waypoint.
        self.declare_parameter('min_speed_near_target_m_s', 0.0)

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
        self.target_marker_pub = self.create_publisher(Marker, '/mpc_sim/viz/target', 10)
        self.lookahead_path_pub = self.create_publisher(Path, '/mpc_sim/viz/lookahead_path', 10)
        # Publish the lookahead target as a PoseStamped (ENU/map frame).
        # The downstream PX4 offboard node uses this as a position setpoint
        # — MPC's unicycle Twist alone leaves a multicopter rotating in
        # place when it needs to move sideways to a target.
        from geometry_msgs.msg import PoseStamped as _PS
        self.target_pose_pub = self.create_publisher(
            _PS, '/pegasus/trajectory/target_pose', 10)

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
        self._publish_target_pose(target)
        self._publish_viz(target, nearest_idx, target_idx)
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


    def _publish_target_pose(self, target: Tuple[float, float, float]) -> None:
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

    def _publish_viz(
        self,
        target: Tuple[float, float, float],
        nearest_idx: int,
        target_idx: int,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id='map')
        marker.ns = 'mpc_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = target[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 5.0
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9)
        marker.lifetime.sec = 1
        self.target_marker_pub.publish(marker)

        path_msg = Path()
        path_msg.header = Header(stamp=stamp, frame_id='map')
        end = min(len(self.path_points), target_idx + 1)
        for pt in self.path_points[nearest_idx:end]:
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            ps.pose.position.z = pt[2]
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.lookahead_path_pub.publish(path_msg)


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
