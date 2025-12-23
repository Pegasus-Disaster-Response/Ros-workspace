#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.pc_pub = self.create_publisher(PointCloud2, '/pegasus/lidar/points', 10)
        self.get_logger().info('LiDAR Node started')


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()