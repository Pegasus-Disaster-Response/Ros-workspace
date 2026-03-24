#!/usr/bin/env python3
"""
Pegasus 3D Local Costmap Node (v2.4)
--------------------------------------
Fuses obstacle point clouds from the LiDAR and ZED X costmap layers
into a 3D voxel grid centered on the UAV (base_link frame).

v2.4 changes:
  - NEW: Publishes the full 3D inflated voxel grid as a flat Int8MultiArray
    on /pegasus/local_costmap_3d_grid for consumption by the 3D D* Lite
    planner. The array is (nx * ny * nz) in row-major order with grid
    metadata in the JSON metadata topic.
  - The 2D projection is still published for backward compatibility.

Publishes:
  /pegasus/local_costmap           (PointCloud2 — raw occupied voxels)
  /pegasus/local_costmap_inflated  (PointCloud2 — occupied + inflated with cost)
  /pegasus/local_costmap_3d_grid   (std_msgs/Int8MultiArray — full 3D inflated grid)
  /pegasus/local_costmap_markers   (MarkerArray — colored 3D cubes for RViz)
  /pegasus/local_costmap_2d        (OccupancyGrid — 2D inflated slice for A*)
  /pegasus/sensor_status           (String — "nominal" / "lidar_only" / etc.)
  /pegasus/costmap_metadata        (String — JSON with grid info for planners)

Subscribes:
  /pegasus/lidar_obstacles   (PointCloud2 in base_link)
  /pegasus/lidar_origin      (PointStamped — sensor position for raycasting)
  /pegasus/zed_obstacles     (PointCloud2 in base_link)
  /pegasus/zed_origin        (PointStamped — sensor position for raycasting)
  /pegasus/lidar_health      (Bool)
  /pegasus/zed_health        (Bool)
  /odom                      (Odometry — velocity for adaptive decay)

No SLAM/TF dependency — operates entirely in base_link frame.
"""

import json
import math
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, Odometry
from builtin_interfaces.msg import Time
from std_msgs.msg import Header, Bool, String, ColorRGBA, Int8MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from scipy.ndimage import distance_transform_edt


# ── PointCloud2 helpers ──────────────────────────────────────────────

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
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
        return xyz[np.isfinite(xyz).all(axis=1)]
    return np.empty((0, 3), dtype=np.float32)


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


def xyz_cost_to_pointcloud2(points: np.ndarray, costs: np.ndarray,
                             frame_id: str, stamp) -> PointCloud2:
    """Pack [x, y, z, intensity] into a PointCloud2 for cost-gradient display."""
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    n = points.shape[0]
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    xyzc = np.column_stack([points.astype(np.float32),
                            costs.astype(np.float32)])
    msg.data = xyzc.tobytes()
    return msg


# ── 3D Voxel Grid ───────────────────────────────────────────────────

class VoxelGrid3D:
    """
    3D voxel grid centered on the robot (base_link frame).

    Each voxel stores:
      cost      (float32): -1=unknown, 0=free, 100=occupied
      timestamp (float64): last time this voxel was updated
      source    (uint8):   0=none, 1=lidar, 2=zed, 3=both
    """

    def __init__(self, size_x, size_y, size_z, resolution):
        self.resolution = resolution
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.nx = int(size_x / resolution)
        self.ny = int(size_y / resolution)
        self.nz = int(size_z / resolution)

        self.cost = np.full((self.nx, self.ny, self.nz), -1, dtype=np.float32)
        self.timestamps = np.zeros((self.nx, self.ny, self.nz), dtype=np.float64)
        self.source = np.zeros((self.nx, self.ny, self.nz), dtype=np.uint8)

    def world_to_voxel(self, points: np.ndarray) -> np.ndarray:
        shifted = points.copy()
        shifted[:, 0] += self.size_x / 2.0
        shifted[:, 1] += self.size_y / 2.0
        shifted[:, 2] += self.size_z / 2.0
        return np.floor(shifted / self.resolution).astype(np.int32)

    def world_to_voxel_single(self, x, y, z):
        ix = int((x + self.size_x / 2.0) / self.resolution)
        iy = int((y + self.size_y / 2.0) / self.resolution)
        iz = int((z + self.size_z / 2.0) / self.resolution)
        return ix, iy, iz

    def in_bounds(self, idx: np.ndarray) -> np.ndarray:
        return ((idx[:, 0] >= 0) & (idx[:, 0] < self.nx) &
                (idx[:, 1] >= 0) & (idx[:, 1] < self.ny) &
                (idx[:, 2] >= 0) & (idx[:, 2] < self.nz))

    def in_bounds_single(self, ix, iy, iz) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny and 0 <= iz < self.nz

    def insert_obstacles(self, points_base, timestamp, source_id):
        if points_base.shape[0] == 0:
            return
        idx = self.world_to_voxel(points_base)
        valid = self.in_bounds(idx)
        idx = idx[valid]
        if idx.shape[0] == 0:
            return
        ix, iy, iz = idx[:, 0], idx[:, 1], idx[:, 2]
        self.cost[ix, iy, iz] = 100.0
        self.timestamps[ix, iy, iz] = timestamp
        self.source[ix, iy, iz] |= source_id

    def mark_free_along_rays(self, origin, obstacle_points, timestamp):
        if obstacle_points.shape[0] == 0:
            return
        ox, oy, oz = self.world_to_voxel_single(
            origin[0], origin[1], origin[2])
        max_rays = 200
        if obstacle_points.shape[0] > max_rays:
            indices = np.random.choice(
                obstacle_points.shape[0], max_rays, replace=False)
            obstacle_points = obstacle_points[indices]
        for i in range(obstacle_points.shape[0]):
            ex, ey, ez = self.world_to_voxel_single(
                obstacle_points[i, 0], obstacle_points[i, 1],
                obstacle_points[i, 2])
            self._raycast_bresenham(ox, oy, oz, ex, ey, ez, timestamp)

    def _raycast_bresenham(self, x0, y0, z0, x1, y1, z1, timestamp):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        dz = abs(z1 - z0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        sz = 1 if z1 > z0 else -1
        dm = max(dx, dy, dz)
        if dm == 0:
            return
        x, y, z = x0, y0, z0
        if dx >= dy and dx >= dz:
            err_y = 2 * dy - dx
            err_z = 2 * dz - dx
            for _ in range(dx):
                if self.in_bounds_single(x, y, z):
                    if self.cost[x, y, z] < 100.0:
                        self.cost[x, y, z] = 0.0
                        self.timestamps[x, y, z] = timestamp
                x += sx
                if err_y > 0:
                    y += sy
                    err_y -= 2 * dx
                if err_z > 0:
                    z += sz
                    err_z -= 2 * dx
                err_y += 2 * dy
                err_z += 2 * dz
        elif dy >= dx and dy >= dz:
            err_x = 2 * dx - dy
            err_z = 2 * dz - dy
            for _ in range(dy):
                if self.in_bounds_single(x, y, z):
                    if self.cost[x, y, z] < 100.0:
                        self.cost[x, y, z] = 0.0
                        self.timestamps[x, y, z] = timestamp
                y += sy
                if err_x > 0:
                    x += sx
                    err_x -= 2 * dy
                if err_z > 0:
                    z += sz
                    err_z -= 2 * dy
                err_x += 2 * dx
                err_z += 2 * dz
        else:
            err_x = 2 * dx - dz
            err_y = 2 * dy - dz
            for _ in range(dz):
                if self.in_bounds_single(x, y, z):
                    if self.cost[x, y, z] < 100.0:
                        self.cost[x, y, z] = 0.0
                        self.timestamps[x, y, z] = timestamp
                z += sz
                if err_x > 0:
                    x += sx
                    err_x -= 2 * dz
                if err_y > 0:
                    y += sy
                    err_y -= 2 * dz
                err_x += 2 * dx
                err_y += 2 * dy

    def decay_stale(self, current_time, persistence_s):
        occupied = self.cost >= 100.0
        if not occupied.any():
            return
        age = current_time - self.timestamps
        stale = occupied & (age > persistence_s)
        self.cost[stale] = 0.0
        self.source[stale] = 0

    def get_occupied_with_source(self):
        occupied = np.argwhere(self.cost >= 100.0)
        if occupied.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint8)
        centers = np.column_stack([
            (occupied[:, 0] + 0.5) * self.resolution - self.size_x / 2.0,
            (occupied[:, 1] + 0.5) * self.resolution - self.size_y / 2.0,
            (occupied[:, 2] + 0.5) * self.resolution - self.size_z / 2.0,
        ]).astype(np.float32)
        sources = self.source[occupied[:, 0], occupied[:, 1], occupied[:, 2]]
        return centers, sources

    def compute_inflation(self, radius_m, scaling_factor):
        occupied = (self.cost >= 100.0).astype(np.float32)
        if occupied.sum() == 0:
            return np.zeros_like(self.cost)
        free_mask = 1.0 - occupied
        dist = distance_transform_edt(free_mask, sampling=self.resolution)
        inflated = np.zeros_like(self.cost)
        within_radius = dist <= radius_m
        inflated[within_radius] = 100.0 * np.exp(
            -scaling_factor * dist[within_radius] / radius_m)
        inflated[occupied > 0] = 100.0
        return inflated

    def get_inflated_as_int8(self, inflated_grid):
        """
        Convert the 3D inflated grid to int8 for publishing.

        Values:
          -1  = unknown (never observed)
           0  = free (observed clear)
          1-100 = cost (inflation gradient or occupied)

        The raw cost grid is used to distinguish unknown from free:
        if a voxel has cost < 0 (unknown) AND no inflation, it stays -1.
        """
        result = np.full(
            (self.nx, self.ny, self.nz), -1, dtype=np.int8)
        # Cells that have been observed (cost >= 0) but no inflation = free
        observed = self.cost >= 0.0
        result[observed & (inflated_grid < 1.0)] = 0
        # Cells with inflation cost
        has_cost = observed & (inflated_grid >= 1.0)
        result[has_cost] = np.clip(
            inflated_grid[has_cost], 1, 100).astype(np.int8)
        return result

    def project_to_2d_inflated(self, inflated_grid, height_m, band_m):
        z_min = height_m - band_m / 2.0 + self.size_z / 2.0
        z_max = height_m + band_m / 2.0 + self.size_z / 2.0
        iz_min = max(0, int(z_min / self.resolution))
        iz_max = min(self.nz, int(z_max / self.resolution) + 1)
        if iz_min >= iz_max:
            return np.full((self.nx, self.ny), -1, dtype=np.int8)
        slice_3d = inflated_grid[:, :, iz_min:iz_max]
        max_cost = np.max(slice_3d, axis=2)
        raw_slice = self.cost[:, :, iz_min:iz_max]
        has_any_data = np.any(raw_slice >= 0.0, axis=2)
        result = np.full((self.nx, self.ny), -1, dtype=np.int8)
        result[has_any_data & (max_cost < 1.0)] = 0
        has_cost = has_any_data & (max_cost >= 1.0)
        result[has_cost] = np.clip(max_cost[has_cost], 1, 100).astype(np.int8)
        return result


# ── Main Node ────────────────────────────────────────────────────────

class LocalCostmap3DNode(Node):

    def __init__(self):
        super().__init__('local_costmap_node')

        # ── Parameters ──
        self.declare_parameter('costmap.size_x_m', 40.0)
        self.declare_parameter('costmap.size_y_m', 40.0)
        self.declare_parameter('costmap.size_z_m', 20.0)
        self.declare_parameter('costmap.resolution_m', 0.3)
        self.declare_parameter('fusion.obstacle_persistence_s', 2.0)
        self.declare_parameter('fusion.decay_check_hz', 2.0)
        self.declare_parameter('fusion.max_ghost_distance_m', 3.0)
        self.declare_parameter('fusion.raycasting_enabled', True)
        self.declare_parameter('inflation.radius_m', 2.5)
        self.declare_parameter('inflation.cost_scaling_factor', 5.0)
        self.declare_parameter('publish.rate_hz', 10.0)
        self.declare_parameter('publish.costmap_topic', '/pegasus/local_costmap')
        self.declare_parameter('publish.inflated_topic', '/pegasus/local_costmap_inflated')
        self.declare_parameter('publish.marker_topic', '/pegasus/local_costmap_markers')
        self.declare_parameter('publish.marker_use_latest_tf', True)
        self.declare_parameter('publish.marker_empty_hold_s', 0.5)
        self.declare_parameter('publish.publish_2d_projection', True)
        self.declare_parameter('publish.projection_topic', '/pegasus/local_costmap_2d')
        self.declare_parameter('publish.projection_height_m', 0.0)
        self.declare_parameter('publish.metadata_topic', '/pegasus/costmap_metadata')
        self.declare_parameter('publish.publish_3d_grid', True)
        self.declare_parameter('publish.grid_3d_topic', '/pegasus/local_costmap_3d_grid')
        self.declare_parameter('publish.grid_3d_rate_hz', 5.0)

        size_x = self.get_parameter('costmap.size_x_m').value
        size_y = self.get_parameter('costmap.size_y_m').value
        size_z = self.get_parameter('costmap.size_z_m').value
        resolution = self.get_parameter('costmap.resolution_m').value
        self.base_persistence = self.get_parameter('fusion.obstacle_persistence_s').value
        decay_hz = self.get_parameter('fusion.decay_check_hz').value
        self.max_ghost_dist = self.get_parameter('fusion.max_ghost_distance_m').value
        self.raycasting_enabled = self.get_parameter('fusion.raycasting_enabled').value
        self.inflation_radius = self.get_parameter('inflation.radius_m').value
        self.inflation_scaling = self.get_parameter('inflation.cost_scaling_factor').value
        pub_hz = self.get_parameter('publish.rate_hz').value
        costmap_topic = self.get_parameter('publish.costmap_topic').value
        inflated_topic = self.get_parameter('publish.inflated_topic').value
        marker_topic = self.get_parameter('publish.marker_topic').value
        self.marker_use_latest_tf = self.get_parameter('publish.marker_use_latest_tf').value
        self.marker_empty_hold_s = self.get_parameter('publish.marker_empty_hold_s').value
        self.pub_2d = self.get_parameter('publish.publish_2d_projection').value
        proj_topic = self.get_parameter('publish.projection_topic').value
        self.proj_height = self.get_parameter('publish.projection_height_m').value
        metadata_topic = self.get_parameter('publish.metadata_topic').value
        self.pub_3d_grid = self.get_parameter('publish.publish_3d_grid').value
        grid_3d_topic = self.get_parameter('publish.grid_3d_topic').value
        grid_3d_hz = self.get_parameter('publish.grid_3d_rate_hz').value

        # ── Voxel Grid ──
        self.voxel_grid = VoxelGrid3D(size_x, size_y, size_z, resolution)
        self.lock = threading.Lock()

        self.get_logger().info(
            f'3D Costmap v2.4: {self.voxel_grid.nx}x{self.voxel_grid.ny}x'
            f'{self.voxel_grid.nz} voxels @ {resolution}m '
            f'({size_x}x{size_y}x{size_z}m) '
            f'raycast={self.raycasting_enabled} '
            f'pub_3d_grid={self.pub_3d_grid}')

        # ── Sensor state ──
        self.lidar_healthy = False
        self.zed_healthy = False
        self.sensor_mode = "all_degraded"
        self.current_speed = 0.0
        self.lidar_origin = None
        self.zed_origin = None
        self.last_marker_points = np.empty((0, 3), dtype=np.float32)
        self.last_marker_sources = np.empty(0, dtype=np.uint8)
        self.last_marker_update_s = 0.0

        # ── Subscribers ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(
            PointCloud2, '/pegasus/lidar_obstacles',
            self.lidar_obstacle_callback, sensor_qos)
        self.create_subscription(
            PointCloud2, '/pegasus/zed_obstacles',
            self.zed_obstacle_callback, sensor_qos)
        self.create_subscription(
            PointStamped, '/pegasus/lidar_origin',
            self.lidar_origin_callback, 10)
        self.create_subscription(
            PointStamped, '/pegasus/zed_origin',
            self.zed_origin_callback, 10)
        self.create_subscription(
            Bool, '/pegasus/lidar_health', self.lidar_health_callback, 10)
        self.create_subscription(
            Bool, '/pegasus/zed_health', self.zed_health_callback, 10)
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # ── Publishers ──
        self.costmap_pub = self.create_publisher(PointCloud2, costmap_topic, 10)
        self.inflated_pub = self.create_publisher(PointCloud2, inflated_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.status_pub = self.create_publisher(String, '/pegasus/sensor_status', 10)
        self.metadata_pub = self.create_publisher(String, metadata_topic, 10)

        if self.pub_2d:
            self.grid2d_pub = self.create_publisher(OccupancyGrid, proj_topic, 10)

        # NEW v2.4: 3D grid publisher for D* Lite
        if self.pub_3d_grid:
            self.grid3d_pub = self.create_publisher(
                Int8MultiArray, grid_3d_topic, 10)
            self.grid3d_timer = self.create_timer(
                1.0 / grid_3d_hz, self.publish_3d_grid)

        # ── Timers ──
        self.publish_timer = self.create_timer(1.0 / pub_hz, self.publish_costmap)
        self.decay_timer = self.create_timer(1.0 / decay_hz, self.decay_stale_voxels)
        self.status_timer = self.create_timer(0.5, self.publish_sensor_status)
        self.metadata_timer = self.create_timer(1.0, self.publish_metadata)

        self.get_logger().info('3D Local Costmap Node v2.4 initialized')

    # ── Sensor origin callbacks ──────────────────────────────────

    def lidar_origin_callback(self, msg: PointStamped):
        self.lidar_origin = np.array([
            msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    def zed_origin_callback(self, msg: PointStamped):
        self.zed_origin = np.array([
            msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    # ── Obstacle insertion + raycasting ──────────────────────────

    def lidar_obstacle_callback(self, msg: PointCloud2):
        points = pointcloud2_to_xyz(msg)
        if points.shape[0] == 0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if self.raycasting_enabled and self.lidar_origin is not None:
                self.voxel_grid.mark_free_along_rays(
                    self.lidar_origin, points, now)
            self.voxel_grid.insert_obstacles(points, now, source_id=1)

    def zed_obstacle_callback(self, msg: PointCloud2):
        points = pointcloud2_to_xyz(msg)
        if points.shape[0] == 0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if self.raycasting_enabled and self.zed_origin is not None:
                self.voxel_grid.mark_free_along_rays(
                    self.zed_origin, points, now)
            self.voxel_grid.insert_obstacles(points, now, source_id=2)

    # ── Health + velocity tracking ───────────────────────────────

    def lidar_health_callback(self, msg: Bool):
        self.lidar_healthy = msg.data

    def zed_health_callback(self, msg: Bool):
        self.zed_healthy = msg.data

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.current_speed = math.sqrt(vx*vx + vy*vy + vz*vz)

    def publish_sensor_status(self):
        if self.lidar_healthy and self.zed_healthy:
            self.sensor_mode = "nominal"
        elif self.lidar_healthy:
            self.sensor_mode = "lidar_only"
        elif self.zed_healthy:
            self.sensor_mode = "camera_only"
        else:
            self.sensor_mode = "all_degraded"
        msg = String()
        msg.data = self.sensor_mode
        self.status_pub.publish(msg)
        if self.sensor_mode != "nominal":
            self.get_logger().warn(
                f'Sensor mode: {self.sensor_mode}',
                throttle_duration_sec=2.0)

    # ── Speed-adaptive decay ─────────────────────────────────────

    def decay_stale_voxels(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.current_speed > 0.5:
            speed_persistence = self.max_ghost_dist / self.current_speed
            effective_persistence = max(0.1, min(
                speed_persistence, self.base_persistence))
        else:
            effective_persistence = self.base_persistence
        with self.lock:
            self.voxel_grid.decay_stale(now, effective_persistence)

    # ── Publishing ───────────────────────────────────────────────

    def publish_costmap(self):
        stamp = self.get_clock().now().to_msg()

        with self.lock:
            points, sources = self.voxel_grid.get_occupied_with_source()
            inflated_grid = self.voxel_grid.compute_inflation(
                self.inflation_radius, self.inflation_scaling)

        # 1. Raw occupied voxels
        if points.shape[0] > 0:
            self.costmap_pub.publish(
                xyz_to_pointcloud2(points, 'base_link', stamp))

        # 2. Inflated costmap as PointCloud2
        self._publish_inflated_cloud(inflated_grid, stamp)

        # 3. RViz markers (always publish, even when empty)
        now_s = self.get_clock().now().nanoseconds / 1e9
        marker_points = points
        marker_sources = sources
        if points.shape[0] > 0:
            self.last_marker_points = points.copy()
            self.last_marker_sources = sources.copy()
            self.last_marker_update_s = now_s
        elif (self.last_marker_points.shape[0] > 0 and
              (now_s - self.last_marker_update_s) <= self.marker_empty_hold_s):
            marker_points = self.last_marker_points
            marker_sources = self.last_marker_sources

        self._publish_markers(marker_points, marker_sources, stamp)

        # 4. 2D inflated projection
        if self.pub_2d:
            self._publish_2d_inflated(inflated_grid, stamp)

    def publish_3d_grid(self):
        """
        NEW v2.4: Publish the full 3D inflated voxel grid as Int8MultiArray.

        Layout: flat row-major array of shape (nx, ny, nz).
        Index mapping: grid[ix, iy, iz] = data[ix * ny * nz + iy * nz + iz]

        Values: -1=unknown, 0=free, 1-100=cost (inflation gradient / occupied)

        D* Lite subscribes to this topic and uses it as its 3D search graph.
        Grid metadata (dimensions, resolution, origin offsets) is published
        on /pegasus/costmap_metadata as JSON.
        """
        with self.lock:
            inflated_grid = self.voxel_grid.compute_inflation(
                self.inflation_radius, self.inflation_scaling)
            grid_int8 = self.voxel_grid.get_inflated_as_int8(inflated_grid)

        msg = Int8MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='x', size=self.voxel_grid.nx,
                                stride=self.voxel_grid.nx * self.voxel_grid.ny * self.voxel_grid.nz),
            MultiArrayDimension(label='y', size=self.voxel_grid.ny,
                                stride=self.voxel_grid.ny * self.voxel_grid.nz),
            MultiArrayDimension(label='z', size=self.voxel_grid.nz,
                                stride=self.voxel_grid.nz),
        ]
        msg.layout.data_offset = 0
        msg.data = grid_int8.ravel().tolist()

        self.grid3d_pub.publish(msg)

    def _publish_inflated_cloud(self, inflated_grid, stamp):
        threshold = 10.0
        inflated_voxels = np.argwhere(inflated_grid > threshold)
        if inflated_voxels.shape[0] == 0:
            return
        res = self.voxel_grid.resolution
        centers = np.column_stack([
            (inflated_voxels[:, 0] + 0.5) * res - self.voxel_grid.size_x / 2.0,
            (inflated_voxels[:, 1] + 0.5) * res - self.voxel_grid.size_y / 2.0,
            (inflated_voxels[:, 2] + 0.5) * res - self.voxel_grid.size_z / 2.0,
        ]).astype(np.float32)
        costs = inflated_grid[
            inflated_voxels[:, 0],
            inflated_voxels[:, 1],
            inflated_voxels[:, 2],
        ]
        self.inflated_pub.publish(
            xyz_cost_to_pointcloud2(centers, costs, 'base_link', stamp))

    def _publish_markers(self, points, sources, stamp):
        marker_array = MarkerArray()

        marker = Marker()
        marker_stamp = Time() if self.marker_use_latest_tf else stamp
        marker.header = Header(stamp=marker_stamp, frame_id='base_link')
        marker.ns = 'local_costmap_3d'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        res = self.voxel_grid.resolution
        marker.scale.x = res * 0.9
        marker.scale.y = res * 0.9
        marker.scale.z = res * 0.9
        if points.shape[0] > 0:
            z_min = points[:, 2].min()
            z_max = points[:, 2].max()
            z_range = max(z_max - z_min, 0.1)
            for i in range(points.shape[0]):
                marker.points.append(Point(
                    x=float(points[i, 0]), y=float(points[i, 1]),
                    z=float(points[i, 2])))
                alpha = 0.4 + 0.6 * (points[i, 2] - z_min) / z_range
                src = sources[i]
                if src == 1:
                    c = ColorRGBA(r=1.0, g=0.2, b=0.2, a=float(alpha))
                elif src == 2:
                    c = ColorRGBA(r=0.2, g=0.4, b=1.0, a=float(alpha))
                elif src == 3:
                    c = ColorRGBA(r=0.2, g=1.0, b=0.2, a=float(alpha))
                else:
                    c = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)
                marker.colors.append(c)

        # Keep marker persistent and update it every publish cycle.
        # Empty CUBE_LIST clears the display without DELETEALL-induced flicker.
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _publish_empty_markers(self, stamp):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id='base_link')
        marker.ns = 'local_costmap_3d'
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _publish_2d_inflated(self, inflated_grid, stamp):
        with self.lock:
            grid_2d = self.voxel_grid.project_to_2d_inflated(
                inflated_grid, self.proj_height, band_m=2.0)
        occ_msg = OccupancyGrid()
        occ_msg.header = Header(stamp=stamp, frame_id='base_link')
        occ_msg.info.resolution = self.voxel_grid.resolution
        occ_msg.info.width = self.voxel_grid.ny
        occ_msg.info.height = self.voxel_grid.nx
        occ_msg.info.origin = Pose()
        occ_msg.info.origin.position.x = -self.voxel_grid.size_x / 2.0
        occ_msg.info.origin.position.y = -self.voxel_grid.size_y / 2.0
        occ_msg.info.origin.position.z = float(self.proj_height)
        occ_msg.info.origin.orientation.w = 1.0
        occ_msg.data = grid_2d.T.ravel().tolist()
        self.grid2d_pub.publish(occ_msg)

    # ── Metadata ─────────────────────────────────────────────────

    def publish_metadata(self):
        vg = self.voxel_grid
        occupied_count = int((vg.cost >= 100.0).sum())
        free_count = int((vg.cost == 0.0).sum())
        unknown_count = int((vg.cost < 0.0).sum())
        metadata = {
            "size_x_m": vg.size_x,
            "size_y_m": vg.size_y,
            "size_z_m": vg.size_z,
            "resolution_m": vg.resolution,
            "grid_nx": vg.nx,
            "grid_ny": vg.ny,
            "grid_nz": vg.nz,
            "frame_id": "base_link",
            "inflation_radius_m": self.inflation_radius,
            "sensor_mode": self.sensor_mode,
            "current_speed_m_s": round(self.current_speed, 2),
            "occupied_voxels": occupied_count,
            "free_voxels": free_count,
            "unknown_voxels": unknown_count,
            "raycasting_enabled": self.raycasting_enabled,
            "publish_3d_grid": self.pub_3d_grid,
        }
        msg = String()
        msg.data = json.dumps(metadata)
        self.metadata_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmap3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()