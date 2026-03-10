#!/usr/bin/env python3
"""
Odometry Selector Node - Fault-Tolerant Odometry for RTAB-Map SLAM

Intelligently switches between LiDAR ICP odometry (primary) and
Visual RGB-D odometry (backup) based on sensor availability.

v2.4 changes:
  - Fix: NEVER publish NaN quaternions to TF. When both odometry
    sources fail, hold the last known good transform instead of
    publishing NaN, which was poisoning the entire TF tree and
    blocking SLAM from processing any data.
  - Fix: Validate quaternion before broadcasting to TF.
  - Added initial identity transform on startup so TF tree is
    valid from the first frame (prevents "frame does not exist"
    errors during odometry initialization).

Author: Team Pegasus
Date: 2026
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdometrySelector(Node):
    """
    Monitors multiple odometry sources and publishes the best available one.

    Priority:
    1. LiDAR ICP odometry (primary) - 360° coverage, weather robust
    2. Visual RGB-D odometry (backup) - fallback when LiDAR fails

    Publishes:
    - /odom : Selected odometry (consumed by RTAB-Map)
    - /odom_status : Current odometry source status
    - TF: odom → base_link transform
    """

    def __init__(self):
        super().__init__('odometry_selector')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('primary_odom_topic', '/odom_lidar')
        self.declare_parameter('backup_odom_topic', '/odom_vision')
        self.declare_parameter('timeout_threshold', 0.5)  # seconds

        primary_topic = self.get_parameter('primary_odom_topic').value
        backup_topic = self.get_parameter('backup_odom_topic').value
        self.timeout = self.get_parameter('timeout_threshold').value

        # ── Subscribers ──────────────────────────────────────────
        self.primary_sub = self.create_subscription(
            Odometry,
            primary_topic,
            self.primary_callback,
            10
        )
        self.backup_sub = self.create_subscription(
            Odometry,
            backup_topic,
            self.backup_callback,
            10
        )

        # ── Publishers ───────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/odom_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── State ────────────────────────────────────────────────
        self.last_primary_time = 0.0
        self.last_backup_time = 0.0
        self.current_mode = 'NONE'
        self.last_primary_msg = None
        self.last_backup_msg = None

        # Last known good transform — initialized to identity
        # so TF tree is valid from startup
        self.last_good_transform = TransformStamped()
        self.last_good_transform.header.frame_id = 'odom'
        self.last_good_transform.child_frame_id = 'base_link'
        self.last_good_transform.transform.rotation.w = 1.0  # identity quaternion
        self.has_good_transform = False

        # Statistics
        self.primary_count = 0
        self.backup_count = 0
        self.failure_count = 0
        self.hold_count = 0

        # ── Health Monitor Timer ─────────────────────────────────
        self.create_timer(0.1, self.health_check)

        # Publish identity TF immediately so the tree is valid
        self._publish_held_transform()

        self.get_logger().info('═══════════════════════════════════════════════')
        self.get_logger().info('  Odometry Selector Node v2.4 Initialized')
        self.get_logger().info(f'  Primary: {primary_topic}')
        self.get_logger().info(f'  Backup:  {backup_topic}')
        self.get_logger().info(f'  Timeout: {self.timeout}s')
        self.get_logger().info('  NaN protection: ENABLED')
        self.get_logger().info('═══════════════════════════════════════════════')

    def _now_sec(self) -> float:
        """Return current ROS time in seconds (respects use_sim_time)."""
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _is_valid_quaternion(q) -> bool:
        """Check quaternion is not NaN and is approximately normalized."""
        vals = [q.x, q.y, q.z, q.w]
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            return False
        norm = math.sqrt(sum(v * v for v in vals))
        return 0.9 < norm < 1.1  # allow small numerical drift

    @staticmethod
    def _is_valid_position(p) -> bool:
        """Check position is not NaN or Inf."""
        vals = [p.x, p.y, p.z]
        return not any(math.isnan(v) or math.isinf(v) for v in vals)

    def _is_valid_odometry(self, odom_msg) -> bool:
        """Validate an odometry message has non-NaN pose."""
        if odom_msg is None:
            return False
        return (self._is_valid_position(odom_msg.pose.pose.position) and
                self._is_valid_quaternion(odom_msg.pose.pose.orientation))

    def primary_callback(self, msg):
        """Receive LiDAR ICP odometry"""
        if self._is_valid_odometry(msg):
            self.last_primary_msg = msg
            self.last_primary_time = self._now_sec()
        else:
            self.get_logger().warning(
                'Rejected PRIMARY odometry: invalid pose (NaN/Inf)',
                throttle_duration_sec=5.0
            )

    def backup_callback(self, msg):
        """Receive Visual RGB-D odometry"""
        if self._is_valid_odometry(msg):
            self.last_backup_msg = msg
            self.last_backup_time = self._now_sec()
        else:
            self.get_logger().warning(
                'Rejected BACKUP odometry: invalid pose (NaN/Inf)',
                throttle_duration_sec=5.0
            )

    def health_check(self):
        """
        Main decision logic - runs at 10 Hz
        Selects best available odometry source and publishes.
        NEVER publishes NaN — holds last good transform on failure.
        """
        now = self._now_sec()

        primary_alive = (now - self.last_primary_time) < self.timeout
        backup_alive = (now - self.last_backup_time) < self.timeout

        if primary_alive and self._is_valid_odometry(self.last_primary_msg):
            if self.current_mode != 'PRIMARY':
                self.get_logger().info(
                    'USING PRIMARY ODOMETRY (LiDAR ICP) - '
                    'Range: 100m, Coverage: 360°'
                )
                self.current_mode = 'PRIMARY'
            self.publish_odometry(self.last_primary_msg, 'PRIMARY_LIDAR')
            self.primary_count += 1

        elif backup_alive and self._is_valid_odometry(self.last_backup_msg):
            if self.current_mode != 'BACKUP':
                self.get_logger().warning(
                    '    PRIMARY ODOMETRY FAILED - SWITCHING TO BACKUP (Visual RGB-D)\n'
                    '    Reduced range: 20m, Limited FOV: 110°'
                )
                self.current_mode = 'BACKUP'
            self.publish_odometry(self.last_backup_msg, 'BACKUP_VISION')
            self.backup_count += 1

        else:
            # CRITICAL FIX: Never publish NaN. Hold last good transform.
            if self.current_mode != 'FAILED':
                self.get_logger().error(
                    '   ALL ODOMETRY SOURCES UNAVAILABLE\n'
                    '   Holding last known good transform\n'
                    '   SLAM may stall until odometry recovers'
                )
                self.current_mode = 'FAILED'
            self._publish_held_transform()
            self.publish_status('HOLDING')
            self.failure_count += 1
            self.hold_count += 1
            if self.failure_count % 50 == 0:
                self.log_statistics()

    def publish_odometry(self, odom_msg, status):
        if odom_msg is None:
            return
        # Rewrite frame_id to 'odom' so RTABMAP's odom_frame_id matches.
        # ICP odometry publishes with frame_id='odom_lidar' but RTABMAP
        # expects 'odom' — mismatch causes it to silently drop all nodes.
        out = Odometry()
        out.header = odom_msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = odom_msg.child_frame_id
        out.pose = odom_msg.pose
        out.twist = odom_msg.twist
        self.odom_pub.publish(out)
        self.publish_status(status)
        self.broadcast_tf(out)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def broadcast_tf(self, odom_msg):
        """Broadcast odom→base_link TF with NaN validation."""
        # Double-check: never broadcast invalid data
        if not self._is_valid_odometry(odom_msg):
            self.get_logger().warning(
                'Blocked NaN TF broadcast',
                throttle_duration_sec=2.0
            )
            self._publish_held_transform()
            return

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Save as last known good
        self.last_good_transform = t
        self.has_good_transform = True

    def _publish_held_transform(self):
        """Publish the last known good transform with current timestamp."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform = self.last_good_transform.transform
        self.tf_broadcaster.sendTransform(t)

    def log_statistics(self):
        total = self.primary_count + self.backup_count + self.failure_count
        if total == 0:
            return
        primary_pct = (self.primary_count / total) * 100
        backup_pct = (self.backup_count / total) * 100
        failure_pct = (self.failure_count / total) * 100
        self.get_logger().info(
            f'\n'
            f'═══════════════════════════════════════════════\n'
            f'  Odometry Statistics (last {total} updates):\n'
            f'  ├─ Primary (LiDAR):  {primary_pct:.1f}%\n'
            f'  ├─ Backup (Vision):  {backup_pct:.1f}%\n'
            f'  ├─ Holding (last good): {self.hold_count}\n'
            f'  └─ Failures:         {failure_pct:.1f}%\n'
            f'═══════════════════════════════════════════════'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometrySelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Odometry Selector shutting down...')
        node.log_statistics()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()