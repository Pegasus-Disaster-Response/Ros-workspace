#!/usr/bin/env python3
"""
Publishes a reference path on /mpc_sim/reference_path for MPC RViz testing.

Supported shapes (path_shape parameter):
  circle   - simple circle, radius = scale_m
  figure8  - lemniscate of Bernoulli (infinity symbol), half-width = scale_m * sqrt(2)
             Min radius of curvature = scale_m / 2  (must stay > P110 min turn radius ~45 m)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Header


class MpcSimPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('mpc_sim_path_publisher')

        self.declare_parameter('path_topic', '/mpc_sim/reference_path')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('path_shape', 'figure8')   # 'circle' | 'figure8'
        self.declare_parameter('scale_m', 150.0)          # radius for circle; 'a' for figure8
        self.declare_parameter('altitude_m', 50.0)
        self.declare_parameter('waypoint_spacing_m', 10.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)

        topic = self.get_parameter('path_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        shape = self.get_parameter('path_shape').value
        scale = float(self.get_parameter('scale_m').value)
        alt = float(self.get_parameter('altitude_m').value)
        spacing = float(self.get_parameter('waypoint_spacing_m').value)
        cx = float(self.get_parameter('center_x').value)
        cy = float(self.get_parameter('center_y').value)

        if shape == 'figure8':
            self.path_msg = self._build_figure8(cx, cy, alt, scale, spacing)
        else:
            self.path_msg = self._build_circle(cx, cy, alt, scale, spacing)

        self.pub = self.create_publisher(Path, topic, 10)
        self.create_timer(1.0 / max(0.1, rate), self._publish)

        self.get_logger().info(
            f'MPC sim path: shape={shape} scale={scale}m alt={alt}m '
            f'{len(self.path_msg.poses)} waypoints')

    # ── Path builders ────────────────────────────────────────────

    def _build_circle(
        self, cx: float, cy: float, alt: float, radius: float, spacing: float
    ) -> Path:
        circumference = 2.0 * math.pi * radius
        n = max(8, int(circumference / spacing))
        raw = []
        for i in range(n):
            a = 2.0 * math.pi * i / n
            raw.append((cx + radius * math.cos(a), cy + radius * math.sin(a), alt))
        return self._points_to_path(raw)

    def _build_figure8(
        self, cx: float, cy: float, alt: float, a: float, spacing: float
    ) -> Path:
        # Lemniscate of Bernoulli parametric form:
        #   x = a√2·cos(t) / (sin²(t) + 1)
        #   y = a√2·sin(t)·cos(t) / (sin²(t) + 1)
        # Min radius of curvature = a/2 (at the crossing, t=π/2 & 3π/2).
        # With a=150 m: min R = 75 m >> P110 min turn radius of ~45 m.
        dense = 2000
        raw = []
        for i in range(dense):
            t = 2.0 * math.pi * i / dense
            denom = math.sin(t) ** 2 + 1.0
            x = cx + a * math.sqrt(2.0) * math.cos(t) / denom
            y = cy + a * math.sqrt(2.0) * math.sin(t) * math.cos(t) / denom
            raw.append((x, y, alt))
        return self._points_to_path(self._resample(raw, spacing))

    # ── Helpers ──────────────────────────────────────────────────

    @staticmethod
    def _resample(
        pts: list, spacing: float
    ) -> list:
        """Resample a dense point list to approximately even arc-length spacing."""
        if len(pts) < 2:
            return pts
        resampled = [pts[0]]
        acc = 0.0
        for i in range(1, len(pts)):
            px, py, pz = pts[i - 1]
            cx, cy, cz = pts[i]
            seg = math.sqrt((cx - px) ** 2 + (cy - py) ** 2 + (cz - pz) ** 2)
            acc += seg
            if acc >= spacing:
                resampled.append(pts[i])
                acc = 0.0
        return resampled

    @staticmethod
    def _points_to_path(pts: list) -> Path:
        path = Path()
        for x, y, z in pts:
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header = Header(stamp=stamp, frame_id='map')
        for ps in self.path_msg.poses:
            ps.header = self.path_msg.header
        self.pub.publish(self.path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcSimPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
