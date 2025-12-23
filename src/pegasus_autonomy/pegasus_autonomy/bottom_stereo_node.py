#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class BottomStereoNode(Node):
    def __init__(self):
        super().__init__('bottom_stereo_node')
        self.left_pub = self.create_publisher(Image, '/pegasus/bottom_stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/pegasus/bottom_stereo/right/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/pegasus/bottom_stereo/depth/image_raw', 10)
        self.get_logger().info('Bottom Stereo Node started')


def main(args=None):
    rclpy.init(args=args)
    node = BottomStereoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()