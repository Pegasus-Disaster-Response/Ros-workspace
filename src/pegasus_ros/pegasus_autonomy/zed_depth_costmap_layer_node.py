#!/usr/bin/env python3
"""
Pegasus ZED X Depth Costmap Layer Node (v2)
--------------------------------------------
Subscribes to ZED X depth + camera_info, deprojects depth pixels into
3D obstacle points in base_link frame, publishes obstacle cloud + sensor
origin so the costmap node can raycast for free-space marking.

v2.3 changes:
  - Fix #6: camera_info subscriber QoS changed from RELIABLE to
    BEST_EFFORT. The ZED wrapper publishes camera_info with BEST_EFFORT
    by default. A RELIABLE subscriber cannot receive from a BEST_EFFORT
    publisher, which caused intrinsics to never arrive and depth
    processing to silently never start.

Publishes:
  /pegasus/zed_obstacles  (PointCloud2 in base_link)
  /pegasus/zed_origin     (PointStamped in base_link — camera position)
  /pegasus/zed_health     (Bool)
"""

import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header, Bool

import tf2_ros


# ── Helpers ──────────────────────────────────────────────────────────

def xyz_to_pointcloud2(points: np.ndarray, frame_id: str,
                        stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    n = points.shape[0]
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.shape[0] == 0:
        return points
    indices = np.floor(points / voxel_size).astype(np.int32)
    keys = (indices[:, 0].astype(np.int64) * 73856093 ^
            indices[:, 1].astype(np.int64) * 19349663 ^
            indices[:, 2].astype(np.int64) * 83492791)
    _, unique_idx = np.unique(keys, return_index=True)
    return points[unique_idx]


# ── Node ─────────────────────────────────────────────────────────────

class ZedDepthCostmapLayerNode(Node):

    def __init__(self):
        super().__init__('zed_depth_costmap_layer')

        # ── Parameters ──
        self.declare_parameter('zed_depth.topic',
                               '/zed_x/zed_node/depth/depth_registered')
        self.declare_parameter('zed_depth.camera_info_topic',
                               '/zed_x/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('zed_depth.frame_id', 'zed_x_camera_center')
        self.declare_parameter('zed_depth.timeout_ms', 500)
        self.declare_parameter('zed_depth.range_min_m', 0.5)
        self.declare_parameter('zed_depth.range_max_m', 15.0)
        self.declare_parameter('zed_depth.downsample_factor', 4)
        self.declare_parameter('zed_depth.height_band.min_m', -2.0)
        self.declare_parameter('zed_depth.height_band.max_m', 15.0)
        self.declare_parameter('zed_depth.min_points_for_obstacle', 8)
        self.declare_parameter('costmap.resolution_m', 0.3)

        self.depth_topic = self.get_parameter('zed_depth.topic').value
        self.cam_info_topic = self.get_parameter('zed_depth.camera_info_topic').value
        self.camera_frame = self.get_parameter('zed_depth.frame_id').value
        self.timeout_ms = self.get_parameter('zed_depth.timeout_ms').value
        self.range_min = self.get_parameter('zed_depth.range_min_m').value
        self.range_max = self.get_parameter('zed_depth.range_max_m').value
        self.downsample = self.get_parameter('zed_depth.downsample_factor').value
        self.height_min = self.get_parameter('zed_depth.height_band.min_m').value
        self.height_max = self.get_parameter('zed_depth.height_band.max_m').value
        self.min_pts = self.get_parameter('zed_depth.min_points_for_obstacle').value
        self.voxel_size = self.get_parameter('costmap.resolution_m').value

        # Camera intrinsics (set from camera_info)
        self.fx = self.fy = self.cx = self.cy = None
        self.img_height = self.img_width = 0
        self.intrinsics_ready = False
        self._u_grid = self._v_grid = None

        # ── TF2 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # Fix #6: camera_info QoS changed to BEST_EFFORT to match
        # the ZED wrapper's default. RELIABLE cannot receive from
        # BEST_EFFORT, causing silent failure.
        cam_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.cam_info_topic,
            self.camera_info_callback, cam_info_qos)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos)

        # ── Publishers ──
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/pegasus/zed_obstacles', 10)
        self.origin_pub = self.create_publisher(
            PointStamped, '/pegasus/zed_origin', 10)
        self.health_pub = self.create_publisher(
            Bool, '/pegasus/zed_health', 10)

        # ── State ──
        self.last_msg_time = None
        self.lock = threading.Lock()
        self.health_timer = self.create_timer(0.25, self.publish_health)

        self.get_logger().info(
            f'ZED X costmap layer v2.3 — subscribing to {self.depth_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics_ready:
            return

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_height = msg.height
        self.img_width = msg.width
        self.intrinsics_ready = True

        # Pre-compute downsampled pixel coordinate grids
        h_ds = self.img_height // self.downsample
        w_ds = self.img_width // self.downsample
        v_indices = np.arange(0, self.img_height, self.downsample)[:h_ds]
        u_indices = np.arange(0, self.img_width, self.downsample)[:w_ds]
        self._u_grid, self._v_grid = np.meshgrid(u_indices, v_indices)
        self._u_grid = self._u_grid.astype(np.float32).ravel()
        self._v_grid = self._v_grid.astype(np.float32).ravel()

        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f} ({self.img_width}x{self.img_height})')

    def get_transform_to_base(self, source_frame, stamp):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', source_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {source_frame}→base_link unavailable: {e}',
                throttle_duration_sec=2.0)
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)  ],
            [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)  ],
            [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[0, 3] = t.x; T[1, 3] = t.y; T[2, 3] = t.z
        return T

    def transform_points(self, points, T):
        ones = np.ones((points.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack([points, ones])
        pts_t = (T @ pts_h.T).T
        return pts_t[:, :3].astype(np.float32)

    def depth_callback(self, msg: Image):
        with self.lock:
            self.last_msg_time = self.get_clock().now()

        if not self.intrinsics_ready:
            return

        # Decode depth image
        if msg.encoding == '32FC1':
            depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth_array = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            self.get_logger().warn(
                f'Unexpected depth encoding: {msg.encoding}',
                throttle_duration_sec=5.0)
            return

        # Downsample
        h_ds = self.img_height // self.downsample
        w_ds = self.img_width // self.downsample
        depth_flat = depth_array[::self.downsample, ::self.downsample][:h_ds, :w_ds].ravel()

        # Valid depth mask
        valid = (np.isfinite(depth_flat) &
                 (depth_flat > self.range_min) &
                 (depth_flat < self.range_max))
        if valid.sum() == 0:
            return

        # Deproject to 3D camera-frame
        z = depth_flat[valid]
        x = (self._u_grid[valid] - self.cx) * z / self.fx
        y = (self._v_grid[valid] - self.cy) * z / self.fy
        points_cam = np.column_stack([x, y, z])

        # Transform camera → base_link
        T = self.get_transform_to_base(self.camera_frame, msg.header.stamp)
        if T is None:
            return
        points_base = self.transform_points(points_cam, T)

        # Publish sensor origin (camera position in base_link)
        origin_msg = PointStamped()
        origin_msg.header = Header(stamp=msg.header.stamp, frame_id='base_link')
        origin_msg.point.x = float(T[0, 3])
        origin_msg.point.y = float(T[1, 3])
        origin_msg.point.z = float(T[2, 3])
        self.origin_pub.publish(origin_msg)

        # Height band
        mask = (points_base[:, 2] >= self.height_min) & \
               (points_base[:, 2] <= self.height_max)
        obstacles = points_base[mask]
        if obstacles.shape[0] < self.min_pts:
            return

        # Voxel downsample
        obstacles = voxel_downsample(obstacles, self.voxel_size)

        # Publish
        self.obstacle_pub.publish(
            xyz_to_pointcloud2(obstacles, 'base_link', msg.header.stamp))

    def publish_health(self):
        healthy = Bool()
        if self.last_msg_time is None:
            healthy.data = False
        else:
            age_ms = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e6
            healthy.data = age_ms < self.timeout_ms
        self.health_pub.publish(healthy)


def main(args=None):
    rclpy.init(args=args)
    node = ZedDepthCostmapLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()