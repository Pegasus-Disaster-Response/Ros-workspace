#!/usr/bin/env python3
"""
Pegasus LiDAR Costmap Layer Node (v2)
--------------------------------------
Subscribes to /velodyne_points, performs ground removal and height-band
filtering, publishes obstacle voxels + sensor origin so the costmap node
can raycast for free-space marking.

v2.3 changes:
  - Fix #7: Ground removal upgraded from naive z-threshold
    (points_base[:, 2] > ground_height) to RANSAC plane fitting.
    The old method assumed level flight — during pitch/roll maneuvers
    the ground plane tilts relative to base_link, causing false
    positives (ground classified as obstacle) or missed removal.
    RANSAC fits the dominant plane in the lower portion of the cloud
    and removes inliers regardless of UAV attitude.

Publishes:
  /pegasus/lidar_obstacles  (PointCloud2 in base_link)
  /pegasus/lidar_origin     (PointStamped in base_link — sensor position)
  /pegasus/lidar_health     (Bool)
"""

import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header, Bool

import tf2_ros


# ── PointCloud2 helpers ──────────────────────────────────────────────

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 to Nx3 float32 numpy array."""
    field_names = [f.name for f in msg.fields]
    field_offsets = [f.offset for f in msg.fields]
    field_dtypes = []
    for f in msg.fields:
        dtype_map = {
            PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
            PointField.UINT8: 'u1', PointField.UINT16: 'u2',
            PointField.UINT32: 'u4', PointField.INT8: 'i1',
            PointField.INT16: 'i2', PointField.INT32: 'i4',
        }
        field_dtypes.append(dtype_map.get(f.datatype, 'f4'))

    dt = np.dtype({
        'names': field_names, 'formats': field_dtypes,
        'offsets': field_offsets, 'itemsize': msg.point_step
    })
    cloud = np.frombuffer(msg.data, dtype=dt)

    if 'x' in field_names and 'y' in field_names and 'z' in field_names:
        xyz = np.column_stack([
            cloud['x'].astype(np.float32),
            cloud['y'].astype(np.float32),
            cloud['z'].astype(np.float32)
        ])
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]
    raise ValueError("PointCloud2 missing x/y/z fields")


def xyz_to_pointcloud2(points: np.ndarray, frame_id: str,
                        stamp) -> PointCloud2:
    """Convert Nx3 float32 array to PointCloud2."""
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
    """Fast voxel-grid downsampling via integer quantization."""
    if points.shape[0] == 0:
        return points
    indices = np.floor(points / voxel_size).astype(np.int32)
    keys = (indices[:, 0].astype(np.int64) * 73856093 ^
            indices[:, 1].astype(np.int64) * 19349663 ^
            indices[:, 2].astype(np.int64) * 83492791)
    _, unique_idx = np.unique(keys, return_index=True)
    return points[unique_idx]


def ransac_ground_removal(points: np.ndarray,
                           distance_threshold: float = 0.3,
                           n_iterations: int = 50,
                           candidate_z_max: float = 0.5) -> np.ndarray:
    """
    Fix #7: RANSAC plane fitting for ground removal.

    Instead of a naive z-threshold (which fails during pitch/roll),
    this fits the dominant plane in the lower portion of the cloud.
    Points within distance_threshold of the best-fit plane are
    classified as ground and removed.

    Steps:
      1. Select candidate ground points (lowest z-values in base_link)
      2. Run RANSAC on candidates to find the dominant plane
      3. Remove all points within distance_threshold of that plane

    Args:
        points:              Nx3 points in base_link frame
        distance_threshold:  max distance from plane to be considered ground
        n_iterations:        RANSAC iterations
        candidate_z_max:     only consider points below this z for plane fitting
                             (relative to the cloud's min z)

    Returns:
        Nx3 obstacle points (ground removed)
    """
    if points.shape[0] < 10:
        return points

    # Step 1: Select candidate ground points (lowest portion of cloud)
    z_min = np.percentile(points[:, 2], 5)
    candidate_mask = points[:, 2] < (z_min + candidate_z_max)
    candidates = points[candidate_mask]

    if candidates.shape[0] < 3:
        return points

    # Step 2: RANSAC plane fitting on candidates
    best_inlier_count = 0
    best_normal = None
    best_d = None
    rng = np.random.default_rng(42)  # Deterministic for reproducibility

    for _ in range(n_iterations):
        # Pick 3 random points
        idx = rng.choice(candidates.shape[0], 3, replace=False)
        p1, p2, p3 = candidates[idx[0]], candidates[idx[1]], candidates[idx[2]]

        # Compute plane normal via cross product
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-8:
            continue
        normal = normal / norm_len

        # Plane equation: normal · (p - p1) = 0  →  normal · p = d
        d = np.dot(normal, p1)

        # Count inliers among ALL points (not just candidates)
        distances = np.abs(np.dot(points, normal) - d)
        inlier_count = np.sum(distances < distance_threshold)

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_normal = normal
            best_d = d

    if best_normal is None:
        # RANSAC failed — fall back to returning all points
        return points

    # Step 3: Remove ground inliers
    distances = np.abs(np.dot(points, best_normal) - best_d)
    obstacle_mask = distances >= distance_threshold

    return points[obstacle_mask]


# ── Node ─────────────────────────────────────────────────────────────

class LidarCostmapLayerNode(Node):

    def __init__(self):
        super().__init__('lidar_costmap_layer')

        # ── Parameters ──
        self.declare_parameter('lidar.topic', '/velodyne_points')
        self.declare_parameter('lidar.frame_id', 'velodyne')
        self.declare_parameter('lidar.timeout_ms', 500)
        self.declare_parameter('lidar.range_min_m', 0.5)
        self.declare_parameter('lidar.range_max_m', 50.0)
        self.declare_parameter('lidar.voxel_size_m', 0.2)
        self.declare_parameter('lidar.ground_removal.enabled', True)
        self.declare_parameter('lidar.ground_removal.height_m', 0.3)
        self.declare_parameter('lidar.ground_removal.ransac_iterations', 50)
        self.declare_parameter('lidar.ground_removal.candidate_z_band_m', 0.5)
        self.declare_parameter('lidar.height_band.min_m', -2.0)
        self.declare_parameter('lidar.height_band.max_m', 15.0)
        self.declare_parameter('lidar.min_points_for_obstacle', 3)

        self.lidar_topic = self.get_parameter('lidar.topic').value
        self.lidar_frame = self.get_parameter('lidar.frame_id').value
        self.timeout_ms = self.get_parameter('lidar.timeout_ms').value
        self.range_min = self.get_parameter('lidar.range_min_m').value
        self.range_max = self.get_parameter('lidar.range_max_m').value
        self.voxel_size = self.get_parameter('lidar.voxel_size_m').value
        self.ground_enabled = self.get_parameter('lidar.ground_removal.enabled').value
        self.ground_dist_thresh = self.get_parameter('lidar.ground_removal.height_m').value
        self.ransac_iters = self.get_parameter('lidar.ground_removal.ransac_iterations').value
        self.candidate_z_band = self.get_parameter('lidar.ground_removal.candidate_z_band_m').value
        self.height_min = self.get_parameter('lidar.height_band.min_m').value
        self.height_max = self.get_parameter('lidar.height_band.max_m').value
        self.min_pts = self.get_parameter('lidar.min_points_for_obstacle').value

        # ── TF2 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, sensor_qos)

        # ── Publishers ──
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/pegasus/lidar_obstacles', 10)
        self.origin_pub = self.create_publisher(
            PointStamped, '/pegasus/lidar_origin', 10)
        self.health_pub = self.create_publisher(
            Bool, '/pegasus/lidar_health', 10)

        # ── State ──
        self.last_msg_time = None
        self.lock = threading.Lock()
        self.health_timer = self.create_timer(0.25, self.publish_health)

        ground_method = 'RANSAC' if self.ground_enabled else 'disabled'
        self.get_logger().info(
            f'LiDAR costmap layer v2.3 — subscribing to {self.lidar_topic} '
            f'(ground removal: {ground_method})')

    def get_transform_to_base(self, source_frame, stamp):
        """Look up 4x4 homogeneous transform source_frame → base_link."""
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

    def lidar_callback(self, msg: PointCloud2):
        with self.lock:
            self.last_msg_time = self.get_clock().now()

        try:
            points = pointcloud2_to_xyz(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to parse PointCloud2: {e}')
            return

        if points.shape[0] == 0:
            return

        # Range filter in sensor frame
        distances = np.linalg.norm(points, axis=1)
        points = points[(distances >= self.range_min) & (distances <= self.range_max)]
        if points.shape[0] == 0:
            return

        # Transform to base_link
        T = self.get_transform_to_base(self.lidar_frame, msg.header.stamp)
        if T is None:
            return
        points_base = self.transform_points(points, T)

        # Publish sensor origin in base_link (the translation of the TF)
        origin_msg = PointStamped()
        origin_msg.header = Header(stamp=msg.header.stamp, frame_id='base_link')
        origin_msg.point.x = float(T[0, 3])
        origin_msg.point.y = float(T[1, 3])
        origin_msg.point.z = float(T[2, 3])
        self.origin_pub.publish(origin_msg)

        # Voxel downsample
        points_base = voxel_downsample(points_base, self.voxel_size)

        # Ground removal — Fix #7: RANSAC instead of naive z-threshold
        if self.ground_enabled:
            points_base = ransac_ground_removal(
                points_base,
                distance_threshold=self.ground_dist_thresh,
                n_iterations=self.ransac_iters,
                candidate_z_max=self.candidate_z_band,
            )
        if points_base.shape[0] == 0:
            return

        # Height band filter
        mask = (points_base[:, 2] >= self.height_min) & \
               (points_base[:, 2] <= self.height_max)
        obstacles = points_base[mask]
        if obstacles.shape[0] < self.min_pts:
            return

        # Publish obstacles
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
    node = LidarCostmapLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()