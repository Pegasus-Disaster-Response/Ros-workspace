#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PX4StateSubscriberNode(Node):
    def __init__(self):
        super().__init__('px4_state_subscriber_node')
        
        # Subscriber to PX4 position (via XRCE-DDS)
        # Note: Real topic would be /fmu/out/vehicle_local_position
        # Using simplified topic for demo
        self.pos_sub = self.create_subscription(
            PoseStamped, 
            '/fmu/out/vehicle_local_position', 
            self.position_callback, 
            10)
        
        self.get_logger().info('PX4 State Subscriber Node started - waiting for PX4 data')

    def position_callback(self, msg):
        self.get_logger().info(f'Received PX4 position data')


def main(args=None):
    rclpy.init(args=args)
    node = PX4StateSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()