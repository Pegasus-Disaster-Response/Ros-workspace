#!/usr/bin/env python3
"""
Pegasus PX4 Odometry Bridge
---------------------------
Converts PX4's /fmu/out/vehicle_odometry (NED) into a nav_msgs/Odometry
message in ENU on /odom_px4 — used as the primary odometry source for
SITL because Gazebo's /model/<name>/odometry_with_covariance topic is
not actually published by the p110_v2 spawn (no OdometryPublisher
plugin), and ICP drifts heavily in z with reduced lidar samples.

Frame conversion (NED → ENU):
    ENU.x = NED.y    (East = NED Y)
    ENU.y = NED.x    (North = NED X)
    ENU.z = -NED.z   (Up   = -Down)

Quaternion conversion (NED → ENU):
    Apply axis swap: rotate by -π/2 about z, then flip about x.
    Equivalent quaternion composition handled below.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


class PX4OdomBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_bridge_node')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._cb, px4_qos)
        self.pub = self.create_publisher(Odometry, '/odom_px4', 10)

        self.get_logger().info(
            'PX4 odom bridge: /fmu/out/vehicle_odometry (NED) → /odom_px4 (ENU)')

    def _cb(self, msg: VehicleOdometry):
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'

        # NED → ENU position
        out.pose.pose.position.x = float(msg.position[1])    # ENU X = NED Y
        out.pose.pose.position.y = float(msg.position[0])    # ENU Y = NED X
        out.pose.pose.position.z = float(-msg.position[2])   # ENU Z = -NED D

        # NED → ENU velocity
        out.twist.twist.linear.x = float(msg.velocity[1])
        out.twist.twist.linear.y = float(msg.velocity[0])
        out.twist.twist.linear.z = float(-msg.velocity[2])

        # Orientation: PX4 reports quaternion as [w, x, y, z] in NED.
        # ENU quaternion = q_enu_ned * q_ned_body where q_enu_ned is the
        # fixed rotation between frames. Closed form for [w,x,y,z]:
        #   q_enu = [ (q_w + q_z) / sqrt(2),
        #             (q_x + q_y) / sqrt(2),
        #             (q_x - q_y) / sqrt(2),  (sign flip vs the obvious)
        #             (q_w - q_z) / sqrt(2) ]
        # Reference: PX4 docs / ROS REP-103.
        qw, qx, qy, qz = (
            float(msg.q[0]), float(msg.q[1]),
            float(msg.q[2]), float(msg.q[3]),
        )
        s = 1.0 / math.sqrt(2.0)
        out.pose.pose.orientation.w = (qw + qz) * s
        out.pose.pose.orientation.x = (qx + qy) * s
        out.pose.pose.orientation.y = (qx - qy) * s
        out.pose.pose.orientation.z = (qw - qz) * s

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
