#!/usr/bin/env python3
"""
Pegasus Static Odometry Publisher
----------------------------------
Publishes a fixed-position Odometry message for SIL testing.
Simulates the UAV sitting at a known start position.

Replace this with the Gazebo model pose bridge when testing
with a moving vehicle in simulation.

Author: Team Pegasus — Cal Poly Pomona
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header


class StaticOdomPublisherNode(Node):

    def __init__(self):
        super().__init__('static_odom_publisher_node')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 10.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.z = self.get_parameter('start_z').value
        rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame = self.get_parameter('child_frame_id').value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Static odom: ({self.x}, {self.y}, {self.z}) '
            f'at {rate} Hz on /odom')

    def _publish(self):
        msg = Odometry()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id)
        msg.child_frame_id = self.child_frame

        msg.pose.pose.position = Point(
            x=float(self.x), y=float(self.y), z=float(self.z))
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticOdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()