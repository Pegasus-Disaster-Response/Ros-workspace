#!/usr/bin/env python3
"""
Pegasus Static Map Publisher Node
----------------------------------
Loads a pre-generated occupancy grid map (PGM image + YAML metadata)
and publishes it as nav_msgs/OccupancyGrid on a configurable topic.

This replaces the entire SLAM pipeline for SIL testing of the path
planner in Gazebo. The map represents ground-truth obstacles from
the Gazebo world — no sensor noise, no drift, no SLAM lag.

Usage:
    1. Create a map from your Gazebo world (see maps/README.md)
    2. Launch this node with the map file path
    3. Publish a goal waypoint
    4. Watch A* plan a path in RViz

Publishes:
    /pegasus/static_map   (nav_msgs/OccupancyGrid — latched)

Author: Team Pegasus — Cal Poly Pomona
"""

import os
import yaml

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class StaticMapPublisherNode(Node):

    def __init__(self):
        super().__init__('static_map_publisher_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('publish_topic', '/pegasus/static_map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('frame_id', 'map')

        yaml_path = self.get_parameter('map_yaml_path').value
        topic = self.get_parameter('publish_topic').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        if not yaml_path:
            self.get_logger().fatal(
                'No map_yaml_path provided! '
                'Set it in the launch file or via --ros-args -p map_yaml_path:=/path/to/map.yaml')
            raise SystemExit(1)

        # ── Load map ────────────────────────────────────────
        self.map_msg = self._load_map(yaml_path)

        if self.map_msg is None:
            self.get_logger().fatal(f'Failed to load map from {yaml_path}')
            raise SystemExit(1)

        self.get_logger().info(
            f'Map loaded: {self.map_msg.info.width}x{self.map_msg.info.height} '
            f'@ {self.map_msg.info.resolution}m/cell '
            f'({self.map_msg.info.width * self.map_msg.info.resolution:.0f}m x '
            f'{self.map_msg.info.height * self.map_msg.info.resolution:.0f}m)')

        # ── Publisher (latched via transient local durability) ──
        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1)

        self.map_pub = self.create_publisher(
            OccupancyGrid, topic, latched_qos)

        # Publish immediately, then at a slow rate for late-joining subscribers
        self.map_pub.publish(self.map_msg)
        self.create_timer(1.0 / rate_hz, self._publish_map)

        self.get_logger().info(
            f'Publishing on {topic} at {rate_hz} Hz (frame: {self.frame_id})')

    def _load_map(self, yaml_path: str) -> OccupancyGrid:
        """
        Load a standard ROS map (YAML + PGM).

        The YAML file format is the same as nav2_map_server:
            image: map.pgm
            resolution: 0.3
            origin: [-15.0, -15.0, 0.0]
            negate: 0
            occupied_thresh: 0.65
            free_thresh: 0.196

        The PGM image uses:
            white (254) = free space
            black (0)   = occupied
            grey  (205) = unknown
        """
        yaml_path = os.path.expanduser(yaml_path)

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f'Map YAML not found: {yaml_path}')
            return None

        with open(yaml_path, 'r') as f:
            map_meta = yaml.safe_load(f)

        # Resolve image path relative to YAML location
        image_file = map_meta.get('image', '')
        if not os.path.isabs(image_file):
            image_file = os.path.join(os.path.dirname(yaml_path), image_file)

        if not os.path.isfile(image_file):
            self.get_logger().error(f'Map image not found: {image_file}')
            return None

        resolution = float(map_meta.get('resolution', 0.3))
        origin = map_meta.get('origin', [0.0, 0.0, 0.0])
        negate = int(map_meta.get('negate', 0))
        occ_thresh = float(map_meta.get('occupied_thresh', 0.65))
        free_thresh = float(map_meta.get('free_thresh', 0.196))

        # Load image
        img = Image.open(image_file).convert('L')
        img_array = np.array(img, dtype=np.float64)

        # PGM convention: image row 0 is top of image = max Y in world
        # OccupancyGrid convention: data[0] is bottom-left (min Y)
        # So flip vertically
        img_array = np.flipud(img_array)

        if negate:
            img_array = 255.0 - img_array

        # Normalize to [0, 1]
        img_norm = img_array / 255.0

        # Convert to occupancy values
        height, width = img_norm.shape
        occ_data = np.full(height * width, -1, dtype=np.int8)

        flat = img_norm.ravel()

        # Free: pixel value > (1 - free_thresh) → low raw value = dark
        # Actually in standard convention after normalization:
        # high pixel (white) = free, low pixel (black) = occupied
        free_mask = flat > (1.0 - free_thresh)
        occupied_mask = flat < (1.0 - occ_thresh)

        occ_data[free_mask] = 0
        occ_data[occupied_mask] = 100

        # Build message
        msg = OccupancyGrid()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id)

        msg.info = MapMetaData()
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = occ_data.tolist()
        return msg

    def _publish_map(self):
        """Re-publish with updated timestamp."""
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    try:
        node = StaticMapPublisherNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()