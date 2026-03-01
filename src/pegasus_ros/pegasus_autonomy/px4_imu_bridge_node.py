#!/usr/bin/env python3
"""
PX4 IMU Bridge Node — Pegasus Disaster Response UAV

Converts PX4 SensorCombined messages (XRCE-DDS) to standard
sensor_msgs/Imu messages that RTAB-Map can consume.

PX4 SensorCombined contains:
  - gyro_rad[3]        (rad/s, FRD body frame)
  - accelerometer_m_s2[3] (m/s², FRD body frame)

This node:
  1. Reads SensorCombined from /fmu/out/sensor_combined
  2. Converts PX4 FRD (Front-Right-Down) to ROS FLU (Front-Left-Up)
  3. Publishes sensor_msgs/Imu on /pegasus/imu/data

The FRD→FLU conversion:
  FLU.x =  FRD.x   (forward stays forward)
  FLU.y = -FRD.y   (right becomes left)
  FLU.z = -FRD.z   (down becomes up)

Author: Team Pegasus
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined, VehicleAttitude

import math


class PX4ImuBridgeNode(Node):
    """
    Bridges PX4 IMU data to standard ROS Imu messages.
    Subscribes to SensorCombined (accel + gyro) and VehicleAttitude (orientation).
    Publishes a fused sensor_msgs/Imu message for RTAB-Map.
    """

    def __init__(self):
        super().__init__('px4_imu_bridge_node')

        # Parameters
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        # PX4 QoS — must be BEST_EFFORT for XRCE-DDS topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State
        self._latest_accel = None
        self._latest_gyro = None
        self._latest_orientation = None
        self._msg_count = 0

        # Subscribers
        self.sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self._sensor_combined_callback,
            px4_qos
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            px4_qos
        )

        # Publisher — standard ROS Imu
        self.imu_pub = self.create_publisher(
            Imu,
            '/pegasus/imu/data',
            10
        )

        # Timer to publish at fixed rate
        timer_period = 1.0 / publish_rate
        self.publish_timer = self.create_timer(timer_period, self._publish_imu)

        self.get_logger().info(
            f'PX4 IMU Bridge started — publishing on /pegasus/imu/data '
            f'at {publish_rate} Hz (frame: {self.imu_frame_id})'
        )

    def _sensor_combined_callback(self, msg: SensorCombined):
        """Cache latest accelerometer and gyroscope data."""
        # PX4 FRD → ROS FLU conversion
        self._latest_accel = (
             msg.accelerometer_m_s2[0],   # x: forward
            -msg.accelerometer_m_s2[1],   # y: right→left
            -msg.accelerometer_m_s2[2],   # z: down→up
        )
        self._latest_gyro = (
             msg.gyro_rad[0],
            -msg.gyro_rad[1],
            -msg.gyro_rad[2],
        )

    def _attitude_callback(self, msg: VehicleAttitude):
        """Cache latest orientation quaternion."""
        # PX4 quaternion is [w, x, y, z] in FRD → convert to FLU
        # FRD→FLU quaternion: q_flu = [w, x, -y, -z]
        self._latest_orientation = (
            msg.q[0],     # w
            msg.q[1],     # x (forward stays)
            -msg.q[2],    # y (negate: right→left)
            -msg.q[3],    # z (negate: down→up)
        )

    def _publish_imu(self):
        """Publish fused Imu message at fixed rate."""
        if self._latest_accel is None or self._latest_gyro is None:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        # Orientation (if available from VehicleAttitude)
        if self._latest_orientation is not None:
            imu_msg.orientation.w = self._latest_orientation[0]
            imu_msg.orientation.x = self._latest_orientation[1]
            imu_msg.orientation.y = self._latest_orientation[2]
            imu_msg.orientation.z = self._latest_orientation[3]
            # Low covariance — PX4 EKF provides good orientation
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
        else:
            # Unknown orientation — set first element to -1 per REP 145
            imu_msg.orientation_covariance[0] = -1.0

        # Angular velocity (gyroscope)
        imu_msg.angular_velocity.x = self._latest_gyro[0]
        imu_msg.angular_velocity.y = self._latest_gyro[1]
        imu_msg.angular_velocity.z = self._latest_gyro[2]
        imu_msg.angular_velocity_covariance[0] = 0.001
        imu_msg.angular_velocity_covariance[4] = 0.001
        imu_msg.angular_velocity_covariance[8] = 0.001

        # Linear acceleration (accelerometer)
        imu_msg.linear_acceleration.x = self._latest_accel[0]
        imu_msg.linear_acceleration.y = self._latest_accel[1]
        imu_msg.linear_acceleration.z = self._latest_accel[2]
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(imu_msg)

        # Log first message
        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info('Publishing IMU data to /pegasus/imu/data')


def main(args=None):
    rclpy.init(args=args)
    node = PX4ImuBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('PX4 IMU Bridge shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()