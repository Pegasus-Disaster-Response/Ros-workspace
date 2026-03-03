#!/usr/bin/env python3
"""
Odometry Selector Node - Fault-Tolerant Odometry for RTAB-Map SLAM

Intelligently switches between LiDAR ICP odometry (primary) and 
Visual RGB-D odometry (backup) based on sensor availability.

Author: Team Pegasus
Date: 2026
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time


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
        self.declare_parameter('use_sim_time', False)
        
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
        # Main odometry output (consumed by RTAB-Map)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Status monitoring
        self.status_pub = self.create_publisher(String, '/odom_status', 10)
        
        # TF broadcaster for odom → base_link
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ── State ────────────────────────────────────────────────
        self.last_primary_time = 0.0
        self.last_backup_time = 0.0
        self.current_mode = 'NONE'
        self.last_primary_msg = None
        self.last_backup_msg = None
        
        # Statistics
        self.primary_count = 0
        self.backup_count = 0
        self.failure_count = 0
        
        # ── Health Monitor Timer ─────────────────────────────────
        # Check odometry health at 10 Hz
        self.create_timer(0.1, self.health_check)
        
        self.get_logger().info('═══════════════════════════════════════════════')
        self.get_logger().info('  Odometry Selector Node Initialized')
        self.get_logger().info(f'  Primary: {primary_topic}')
        self.get_logger().info(f'  Backup:  {backup_topic}')
        self.get_logger().info(f'  Timeout: {self.timeout}s')
        self.get_logger().info('═══════════════════════════════════════════════')
    
    def primary_callback(self, msg):
        """Receive LiDAR ICP odometry"""
        self.last_primary_msg = msg
        self.last_primary_time = time.time()
    
    def backup_callback(self, msg):
        """Receive Visual RGB-D odometry"""
        self.last_backup_msg = msg
        self.last_backup_time = time.time()
    
    def health_check(self):
        """
        Main decision logic - runs at 10 Hz
        Selects best available odometry source and publishes
        """
        now = time.time()
        
        # Check if odometry sources are alive
        primary_alive = (now - self.last_primary_time) < self.timeout
        backup_alive = (now - self.last_backup_time) < self.timeout
        
        # ── Decision Logic ───────────────────────────────────────
        
        if primary_alive:
            # PRIMARY MODE: Use LiDAR ICP odometry
            if self.current_mode != 'PRIMARY':
                self.get_logger().info(
                    'USING PRIMARY ODOMETRY (LiDAR ICP) - '
                    f'Range: 100m, Coverage: 360°'
                )
                self.current_mode = 'PRIMARY'
            
            self.publish_odometry(self.last_primary_msg, 'PRIMARY_LIDAR')
            self.primary_count += 1
            
        elif backup_alive:
            # BACKUP MODE: Fallback to Visual odometry
            if self.current_mode != 'BACKUP':
                self.get_logger().warning(
                    '    PRIMARY ODOMETRY FAILED - SWITCHING TO BACKUP (Visual RGB-D)\n'
                    '    Reduced range: 20m, Limited FOV: 110°'
                )
                self.current_mode = 'BACKUP'
            
            self.publish_odometry(self.last_backup_msg, 'BACKUP_VISION')
            self.backup_count += 1
            
        else:
            #  FAILURE MODE: Both odometry sources failed!
            if self.current_mode != 'FAILED':
                self.get_logger().error(
                    '   CRITICAL: ALL ODOMETRY SOURCES FAILED!\n'
                    '   - LiDAR ICP: No data\n'
                    '   - Visual RGB-D: No data\n'
                    '   RTAB-Map SLAM will degrade or fail!'
                )
                self.current_mode = 'FAILED'
            
            self.publish_status('FAILED')
            self.failure_count += 1
            
            # Every 5 seconds, log statistics
            if self.failure_count % 50 == 0:  # 10Hz * 5s = 50 ticks
                self.log_statistics()
    
    def publish_odometry(self, odom_msg, status):
        """
        Publish selected odometry to /odom topic and broadcast TF
        """
        if odom_msg is None:
            return
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish status
        self.publish_status(status)
        
        # Broadcast TF: odom → base_link
        self.broadcast_tf(odom_msg)
    
    def publish_status(self, status):
        """Publish current odometry source status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def broadcast_tf(self, odom_msg):
        """
        Broadcast odom → base_link transform
        """
        t = TransformStamped()
        
        # Header
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        # Broadcast
        self.tf_broadcaster.sendTransform(t)
    
    def log_statistics(self):
        """Log odometry usage statistics"""
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
        node.log_statistics()  # Final stats on shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()