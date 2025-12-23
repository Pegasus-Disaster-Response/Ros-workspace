#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SideCameraLoggerNode(Node):
    def __init__(self):
        super().__init__('side_camera_logger_node')
        self.left_pub = self.create_publisher(Image, '/pegasus/side_left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/pegasus/side_right/image_raw', 10)
        self.get_logger().info('Side Camera Logger Node started - recording only')


def main(args=None):
    rclpy.init(args=args)
    node = SideCameraLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()