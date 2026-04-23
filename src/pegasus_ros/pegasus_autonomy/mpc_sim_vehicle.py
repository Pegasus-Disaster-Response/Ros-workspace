#!/usr/bin/env python3
"""
Kinematic vehicle simulator for MPC RViz testing.
Integrates TwistStamped commands (speed, yaw_rate, vz) → Odometry + TF.
Matches the exact kinematic model used in MPCTrackerNode._simulate_cost so
the sim behaviour is consistent with MPC predictions.
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class MpcSimVehicle(Node):
    def __init__(self) -> None:
        super().__init__('mpc_sim_vehicle')

        self.declare_parameter('cmd_topic', '/mpc_sim/cmd_vel')
        self.declare_parameter('odom_topic', '/mpc_sim/odom')
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        # Start on the circle edge facing tangent (CCW)
        self.declare_parameter('initial_x', 150.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 50.0)
        self.declare_parameter('initial_yaw_deg', 90.0)

        cmd_topic = self.get_parameter('cmd_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.sim_rate = float(self.get_parameter('sim_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.yaw = math.radians(float(self.get_parameter('initial_yaw_deg').value))

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.vz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(TwistStamped, cmd_topic, self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.dt = 1.0 / max(1.0, self.sim_rate)
        self.create_timer(self.dt, self._step)

        self.get_logger().info(
            f'MPC sim vehicle: start=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}) '
            f'yaw={math.degrees(self.yaw):.1f}°')

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self.speed = msg.twist.linear.x
        self.vz = msg.twist.linear.z
        self.yaw_rate = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _step(self) -> None:
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.speed = 0.0
            self.yaw_rate = 0.0
            self.vz = 0.0

        self.yaw += self.yaw_rate * self.dt
        self.x += self.speed * math.cos(self.yaw) * self.dt
        self.y += self.speed * math.sin(self.yaw) * self.dt
        self.z += self.vz * self.dt

        stamp = self.get_clock().now().to_msg()
        q = self._yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='map')
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = Header(stamp=stamp, frame_id='map')
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpcSimVehicle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
