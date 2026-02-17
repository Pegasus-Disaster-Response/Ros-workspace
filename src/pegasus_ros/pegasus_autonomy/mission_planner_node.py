#!/usr/bin/env python3
"""
Pegasus Mission Planner Node - Disaster Response Autonomy
Uses RTAB-Map SLAM outputs and PX4 via XRCE-DDS

Author: Team Pegasus
Date: 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# RTAB-Map SLAM messages
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray

# PX4 messages (via XRCE-DDS)
from px4_msgs.msg import (
    VehicleOdometry,
    SensorCombined,
    BatteryStatus,
    VehicleStatus
)


class MissionPlannerNode(Node):
    """
    Main mission planning node for disaster response operations.
    Integrates RTAB-Map SLAM outputs with PX4 via XRCE-DDS.
    """
    
    def __init__(self):
        super().__init__('mission_planner_node')
        
        # Mission state
        self.current_pose = None
        self.px4_odom = None
        self.mission_status = "INITIALIZING"
        self.slam_initialized = False
        self.px4_connected = False
        
        # QoS for PX4 topics (CRITICAL!)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ===========================================
        # RTAB-Map SLAM Subscribers
        # ===========================================
        
        self.rtabmap_odom_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.rtabmap_odom_callback,
            10
        )
        
        self.rtabmap_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.rtabmap_cloud_callback,
            10
        )
        
        self.rtabmap_grid_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/grid_map',
            self.rtabmap_grid_callback,
            10
        )
        
        # ===========================================
        # PX4 Subscribers (via XRCE-DDS)
        # ===========================================
        
        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.px4_odom_callback,
            px4_qos
        )
        
        self.sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_callback,
            px4_qos
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            px4_qos
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            px4_qos
        )
        
        # ===========================================
        # Publishers
        # ===========================================
        
        self.status_pub = self.create_publisher(
            String,
            '/pegasus/autonomy/mission_status',
            10
        )
        
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/pegasus/autonomy/target_waypoint',
            10
        )
        
        # ===========================================
        # Timers
        # ===========================================
        
        self.mission_timer = self.create_timer(0.5, self.mission_planning_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('═════════════════════════════════════════════════')
        self.get_logger().info('Mission Planner Node Started (XRCE-DDS)')
        self.get_logger().info('Waiting for RTAB-Map SLAM and PX4...')
        self.get_logger().info('═════════════════════════════════════════════════')
    
    # ===============================================
    # RTAB-Map Callbacks
    # ===============================================
    
    def rtabmap_odom_callback(self, msg: Odometry):
        """SLAM-corrected odometry"""
        self.current_pose = msg.pose.pose
        if not self.slam_initialized:
            self.slam_initialized = True
            self.get_logger().info('✓ RTAB-Map SLAM initialized')
    
    def rtabmap_cloud_callback(self, msg: PointCloud2):
        """3D point cloud map - use for hazard detection"""
        pass
    
    def rtabmap_grid_callback(self, msg: OccupancyGrid):
        """2D occupancy grid - use for navigation"""
        pass
    
    # ===============================================
    # PX4 Callbacks (XRCE-DDS)
    # ===============================================
    
    def px4_odom_callback(self, msg: VehicleOdometry):
        """PX4 odometry from EKF"""
        self.px4_odom = msg
        if not self.px4_connected:
            self.px4_connected = True
            self.get_logger().info('✓ PX4 connected via XRCE-DDS')
    
    def sensor_callback(self, msg: SensorCombined):
        """Raw IMU data"""
        pass
    
    def battery_callback(self, msg: BatteryStatus):
        """Battery monitoring"""
        if msg.remaining < 0.2:
            self.get_logger().warn(
                f'⚠️  Low battery: {msg.remaining*100:.0f}%',
                throttle_duration_sec=10.0
            )
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """Vehicle armed state, flight mode"""
        pass
    
    # ===============================================
    # Mission Planning Logic
    # ===============================================
    
    def mission_planning_loop(self):
        """Main mission logic - runs at 2 Hz"""
        
        if not self.slam_initialized:
            return
        
        if self.current_pose is None:
            return
        
        # State machine
        if self.mission_status == "INITIALIZING":
            self.mission_status = "READY"
            self.get_logger().info('✓ Mission planner ready for tasking')
        
        elif self.mission_status == "READY":
            # Waiting for mission commands
            pass
        
        # TODO: Add mission states:
        # - SEARCH_PATTERN
        # - INVESTIGATE_POI
        # - RETURN_TO_BASE
        # - EMERGENCY
    
    def publish_status(self):
        """Publish mission status"""
        status_msg = String()
        
        if self.current_pose:
            status_msg.data = (
                f"Status: {self.mission_status} | "
                f"SLAM: {'OK' if self.slam_initialized else 'INIT'} | "
                f"PX4: {'OK' if self.px4_connected else 'WAIT'} | "
                f"Pos: [{self.current_pose.position.x:.1f}, "
                f"{self.current_pose.position.y:.1f}, "
                f"{self.current_pose.position.z:.1f}]"
            )
        else:
            status_msg.data = (
                f"Status: {self.mission_status} | "
                f"SLAM: {'OK' if self.slam_initialized else 'INIT'} | "
                f"PX4: {'OK' if self.px4_connected else 'WAIT'}"
            )
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Mission Planner shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
