#!/usr/bin/env python3
"""
Pegasus 3D D* Lite Local Replanner Node
-----------------------------------------
Operates on the full 3D voxel grid (not the 2D projection) so the UAV
can plan paths that go over low obstacles, under overhangs, and through
gaps at different altitudes.

The node subscribes to the 3D inflated grid published by local_costmap_node
as Int8MultiArray on /pegasus/local_costmap_3d_grid, and to the grid
metadata on /pegasus/costmap_metadata for dimensions/resolution.

Search graph: each voxel is a node with 26 neighbors (6 face + 12 edge +
8 corner adjacent). D* Lite runs backward from goal to start. When cells
change cost, only affected vertices are re-queued.

Pipeline:
    A* (global_planner_node) → global_path (2D seed)
        ↓
    D* Lite 3D (THIS NODE) → local_path (true 3D waypoints)
        ↓
    MPC (mpc_trajectory_node) → smooth trajectory → PX4

Publishes:
    /pegasus/path_planner/local_path       (nav_msgs/Path — 3D adjusted path)
    /pegasus/path_planner/local_status     (std_msgs/String — JSON status)

Subscribes:
    /pegasus/path_planner/global_path      (nav_msgs/Path — from A*)
    /pegasus/local_costmap_3d_grid         (std_msgs/Int8MultiArray — 3D grid)
    /pegasus/costmap_metadata              (std_msgs/String — JSON grid info)
    /odom                                  (nav_msgs/Odometry — current pose)

Author: Team Pegasus — Cal Poly Pomona
"""

import heapq
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String, Int8MultiArray


# ═══════════════════════════════════════════════════════════════════
#  3D D* Lite Implementation
# ═══════════════════════════════════════════════════════════════════

class DStarLite3D:
    """
    D* Lite on a 3D voxel grid with 26-connected neighbors.

    Uses flat-index addressing for memory efficiency:
      flat_idx = ix * ny * nz + iy * nz + iz

    The g and rhs arrays are stored as flat numpy arrays (nx*ny*nz).
    """

    INF = float('inf')

    def __init__(self, nx, ny, nz, lethal_threshold=90,
                 cost_penalty_factor=10.0):
        self.nx = nx
        self.ny = ny
        self.nz = nz
        self.n_total = nx * ny * nz
        self.lethal = lethal_threshold
        self.cost_factor = cost_penalty_factor

        self.g = np.full(self.n_total, self.INF, dtype=np.float64)
        self.rhs = np.full(self.n_total, self.INF, dtype=np.float64)
        self.grid = np.zeros(self.n_total, dtype=np.int8)

        self.km = 0.0
        self.start_idx = -1
        self.goal_idx = -1
        self.open_set = []       # min-heap of (key, flat_idx)
        self.in_open = set()
        self._initialized = False

        # Pre-compute 26 neighbor offsets and their move costs
        self._neighbor_offsets = []
        self._neighbor_costs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    self._neighbor_offsets.append((dx, dy, dz))
                    # Euclidean distance: face=1, edge=sqrt(2), corner=sqrt(3)
                    self._neighbor_costs.append(
                        math.sqrt(dx*dx + dy*dy + dz*dz))

    def _to_flat(self, ix, iy, iz):
        return ix * self.ny * self.nz + iy * self.nz + iz

    def _from_flat(self, idx):
        ix = idx // (self.ny * self.nz)
        rem = idx % (self.ny * self.nz)
        iy = rem // self.nz
        iz = rem % self.nz
        return ix, iy, iz

    def _heuristic(self, a_idx, b_idx):
        """3D octile distance."""
        ax, ay, az = self._from_flat(a_idx)
        bx, by, bz = self._from_flat(b_idx)
        dx = abs(ax - bx)
        dy = abs(ay - by)
        dz = abs(az - bz)
        vals = sorted([dx, dy, dz])
        # 3D octile: min*sqrt(3) + (mid-min)*sqrt(2) + (max-mid)*1
        return (vals[0] * 1.732 + (vals[1] - vals[0]) * 1.414
                + (vals[2] - vals[1]) * 1.0)

    def _calc_key(self, s_idx):
        g_val = self.g[s_idx]
        rhs_val = self.rhs[s_idx]
        mn = min(g_val, rhs_val)
        return (mn + self._heuristic(self.start_idx, s_idx) + self.km, mn)

    def _traverse_cost(self, flat_idx):
        v = self.grid[flat_idx]
        if v >= self.lethal:
            return self.INF
        if v < 0:
            return 3.0  # unknown — traversable with penalty
        if v > 0:
            return 1.0 + (v / 100.0) * self.cost_factor
        return 1.0

    def _neighbors(self, flat_idx):
        """Yield (neighbor_flat_idx, move_cost) for 26-connected neighbors."""
        ix, iy, iz = self._from_flat(flat_idx)
        for i, (dx, dy, dz) in enumerate(self._neighbor_offsets):
            nx_ = ix + dx
            ny_ = iy + dy
            nz_ = iz + dz
            if 0 <= nx_ < self.nx and 0 <= ny_ < self.ny and 0 <= nz_ < self.nz:
                n_idx = self._to_flat(nx_, ny_, nz_)
                tc = self._traverse_cost(n_idx)
                if tc < self.INF:
                    yield n_idx, self._neighbor_costs[i] * tc
                else:
                    yield n_idx, self.INF

    def _update_vertex(self, u_idx):
        if u_idx != self.goal_idx:
            min_rhs = self.INF
            for nbr, cost in self._neighbors(u_idx):
                val = self.g[nbr] + cost
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u_idx] = min_rhs

        self.in_open.discard(u_idx)

        if self.g[u_idx] != self.rhs[u_idx]:
            key = self._calc_key(u_idx)
            heapq.heappush(self.open_set, (key, u_idx))
            self.in_open.add(u_idx)

    def initialize(self, grid_flat, start_ixyz, goal_ixyz):
        """Initialize a fresh 3D D* Lite search."""
        self.grid = grid_flat.copy()
        self.start_idx = self._to_flat(*start_ixyz)
        self.goal_idx = self._to_flat(*goal_ixyz)
        self.km = 0.0

        self.g[:] = self.INF
        self.rhs[:] = self.INF
        self.open_set = []
        self.in_open = set()

        self.rhs[self.goal_idx] = 0.0
        key = self._calc_key(self.goal_idx)
        heapq.heappush(self.open_set, (key, self.goal_idx))
        self.in_open.add(self.goal_idx)

        self._compute_shortest_path()
        self._initialized = True

    def _compute_shortest_path(self, max_expansions=100000):
        expansions = 0
        start_key = self._calc_key(self.start_idx)

        while self.open_set:
            top_key, u_idx = self.open_set[0]

            if top_key >= start_key and self.g[self.start_idx] == self.rhs[self.start_idx]:
                break

            heapq.heappop(self.open_set)
            self.in_open.discard(u_idx)

            if expansions >= max_expansions:
                break
            expansions += 1

            old_key = top_key
            new_key = self._calc_key(u_idx)

            if old_key < new_key:
                heapq.heappush(self.open_set, (new_key, u_idx))
                self.in_open.add(u_idx)
            elif self.g[u_idx] > self.rhs[u_idx]:
                self.g[u_idx] = self.rhs[u_idx]
                for nbr, _ in self._neighbors(u_idx):
                    self._update_vertex(nbr)
            else:
                self.g[u_idx] = self.INF
                self._update_vertex(u_idx)
                for nbr, _ in self._neighbors(u_idx):
                    self._update_vertex(nbr)

            start_key = self._calc_key(self.start_idx)

        return expansions

    def update_start(self, new_start_ixyz):
        new_idx = self._to_flat(*new_start_ixyz)
        if self.start_idx >= 0 and new_idx != self.start_idx:
            self.km += self._heuristic(self.start_idx, new_idx)
        self.start_idx = new_idx

    def update_cells(self, changed_cells):
        """
        Update grid cells that changed cost.
        changed_cells: list of (flat_idx, new_value) tuples.
        """
        if not self._initialized:
            return 0

        for flat_idx, new_val in changed_cells:
            old_val = self.grid[flat_idx]
            if old_val != new_val:
                self.grid[flat_idx] = new_val
                self._update_vertex(flat_idx)
                for nbr, _ in self._neighbors(flat_idx):
                    self._update_vertex(nbr)

        return self._compute_shortest_path()

    def extract_path(self, max_steps=10000):
        if not self._initialized:
            return []
        if self.g[self.start_idx] >= self.INF:
            return []

        path = [self._from_flat(self.start_idx)]
        current = self.start_idx
        visited = {current}

        for _ in range(max_steps):
            if current == self.goal_idx:
                break

            best_nbr = -1
            best_cost = self.INF

            for nbr, move_cost in self._neighbors(current):
                total = self.g[nbr] + move_cost
                if total < best_cost and nbr not in visited:
                    best_cost = total
                    best_nbr = nbr

            if best_nbr < 0 or best_cost >= self.INF:
                return []

            path.append(self._from_flat(best_nbr))
            visited.add(best_nbr)
            current = best_nbr

        return path


# ═══════════════════════════════════════════════════════════════════
#  ROS 2 Node
# ═══════════════════════════════════════════════════════════════════

class DStarLite3DNode(Node):

    def __init__(self):
        super().__init__('dstar_lite_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('lethal_cost_threshold', 90)
        self.declare_parameter('cost_penalty_factor', 15.0)
        self.declare_parameter('path_simplification', True)
        self.declare_parameter('simplification_tolerance_m', 0.5)

        update_hz = self.get_parameter('update_rate_hz').value
        self.lethal = self.get_parameter('lethal_cost_threshold').value
        self.cost_factor = self.get_parameter('cost_penalty_factor').value
        self.simplify = self.get_parameter('path_simplification').value
        self.simp_tol = self.get_parameter('simplification_tolerance_m').value

        # ── Grid metadata (from costmap_metadata JSON) ──────
        self.grid_nx = 0
        self.grid_ny = 0
        self.grid_nz = 0
        self.grid_resolution = 0.3
        self.grid_size_x = 40.0
        self.grid_size_y = 40.0
        self.grid_size_z = 20.0
        self.metadata_received = False

        # ── State ───────────────────────────────────────────
        self.dstar = None
        self.current_grid_flat = None
        self.prev_grid_flat = None
        self.current_odom = None
        self.global_path = None
        self.goal_pose = None
        self.planner_state = "WAITING_FOR_GLOBAL_PATH"
        self.replan_count = 0

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            Path, '/pegasus/path_planner/global_path',
            self._global_path_cb, 10)
        self.create_subscription(
            Int8MultiArray, '/pegasus/local_costmap_3d_grid',
            self._grid3d_cb, 10)
        self.create_subscription(
            String, '/pegasus/costmap_metadata',
            self._metadata_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── Publishers ──────────────────────────────────────
        self.local_path_pub = self.create_publisher(
            Path, '/pegasus/path_planner/local_path', 10)
        self.status_pub = self.create_publisher(
            String, '/pegasus/path_planner/local_status', 10)

        # ── Timers ──────────────────────────────────────────
        self.create_timer(1.0 / update_hz, self._update_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'3D D* Lite: rate={update_hz}Hz, lethal={self.lethal}, '
            f'cost_factor={self.cost_factor}, 26-connected')

    # ── Callbacks ────────────────────────────────────────────

    def _global_path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            self.global_path = None
            self.dstar = None
            self.planner_state = "WAITING_FOR_GLOBAL_PATH"
            return
        self.global_path = msg
        self.goal_pose = msg.poses[-1]
        self.planner_state = "INITIALIZING"
        self.get_logger().info(
            f'New global path: {len(msg.poses)} waypoints — init 3D D* Lite')

    def _grid3d_cb(self, msg: Int8MultiArray):
        """Receive the full 3D inflated voxel grid."""
        if not self.metadata_received:
            return
        expected = self.grid_nx * self.grid_ny * self.grid_nz
        if len(msg.data) != expected:
            self.get_logger().warn(
                f'3D grid size mismatch: got {len(msg.data)}, '
                f'expected {expected}', throttle_duration_sec=5.0)
            return
        self.current_grid_flat = np.array(msg.data, dtype=np.int8)

    def _metadata_cb(self, msg: String):
        """Parse costmap metadata JSON for grid dimensions."""
        try:
            meta = json.loads(msg.data)
            self.grid_nx = meta['grid_nx']
            self.grid_ny = meta['grid_ny']
            self.grid_nz = meta['grid_nz']
            self.grid_resolution = meta['resolution_m']
            self.grid_size_x = meta['size_x_m']
            self.grid_size_y = meta['size_y_m']
            self.grid_size_z = meta['size_z_m']
            if not self.metadata_received:
                self.metadata_received = True
                self.get_logger().info(
                    f'Grid metadata: {self.grid_nx}x{self.grid_ny}x{self.grid_nz} '
                    f'@ {self.grid_resolution}m')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Bad metadata: {e}',
                                   throttle_duration_sec=5.0)

    def _odom_cb(self, msg: Odometry):
        self.current_odom = msg

    # ── Coordinate conversions ───────────────────────────────

    def _world_to_voxel(self, wx, wy, wz):
        """World (base_link frame) to voxel (ix, iy, iz)."""
        ix = int((wx + self.grid_size_x / 2.0) / self.grid_resolution)
        iy = int((wy + self.grid_size_y / 2.0) / self.grid_resolution)
        iz = int((wz + self.grid_size_z / 2.0) / self.grid_resolution)
        if (0 <= ix < self.grid_nx and 0 <= iy < self.grid_ny
                and 0 <= iz < self.grid_nz):
            return (ix, iy, iz)
        return None

    def _voxel_to_world(self, ix, iy, iz):
        """Voxel (ix, iy, iz) to world (base_link frame) center."""
        wx = (ix + 0.5) * self.grid_resolution - self.grid_size_x / 2.0
        wy = (iy + 0.5) * self.grid_resolution - self.grid_size_y / 2.0
        wz = (iz + 0.5) * self.grid_resolution - self.grid_size_z / 2.0
        return wx, wy, wz

    # ── Main update loop ─────────────────────────────────────

    def _update_tick(self):
        if (self.current_grid_flat is None or self.current_odom is None
                or self.global_path is None or not self.metadata_received):
            return

        # Current robot position → voxel
        start_voxel = self._world_to_voxel(
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y,
            self.current_odom.pose.pose.position.z)
        if start_voxel is None:
            return

        # Goal → voxel
        gp = self.goal_pose.pose.position
        goal_voxel = self._world_to_voxel(gp.x, gp.y, gp.z)
        if goal_voxel is None:
            return

        # Initialize D* Lite if needed
        if self.dstar is None or self.planner_state == "INITIALIZING":
            self.dstar = DStarLite3D(
                self.grid_nx, self.grid_ny, self.grid_nz,
                lethal_threshold=self.lethal,
                cost_penalty_factor=self.cost_factor)

            t0 = time.monotonic()
            self.dstar.initialize(
                self.current_grid_flat, start_voxel, goal_voxel)
            elapsed = (time.monotonic() - t0) * 1000

            self.prev_grid_flat = self.current_grid_flat.copy()
            self.planner_state = "ACTIVE"
            self.get_logger().info(
                f'3D D* Lite initialized: {self.grid_nx}x{self.grid_ny}x'
                f'{self.grid_nz} grid ({self.dstar.n_total} voxels) '
                f'in {elapsed:.0f}ms')

        # Update robot position
        self.dstar.update_start(start_voxel)

        # Find changed cells (flat index comparison)
        if (self.prev_grid_flat is not None
                and self.prev_grid_flat.shape == self.current_grid_flat.shape):
            diff_mask = self.prev_grid_flat != self.current_grid_flat
            if diff_mask.any():
                changed_indices = np.where(diff_mask)[0]
                changed_cells = [
                    (int(idx), int(self.current_grid_flat[idx]))
                    for idx in changed_indices
                ]
                t0 = time.monotonic()
                expansions = self.dstar.update_cells(changed_cells)
                elapsed = (time.monotonic() - t0) * 1000
                self.replan_count += 1

                if len(changed_cells) > 50:
                    self.get_logger().info(
                        f'3D D* Lite repair: {len(changed_cells)} voxels, '
                        f'{expansions} expanded, {elapsed:.1f}ms',
                        throttle_duration_sec=1.0)

        self.prev_grid_flat = self.current_grid_flat.copy()

        # Extract 3D path
        path_voxels = self.dstar.extract_path()

        if not path_voxels:
            self.planner_state = "NO_LOCAL_PATH"
            self._publish_empty_path()
            return

        self.planner_state = "ACTIVE"

        # Simplify
        if self.simplify:
            path_voxels = self._rdp_simplify_3d(path_voxels)

        # Convert to ROS path with true 3D waypoints
        path_msg = self._voxels_to_path(path_voxels)
        self.local_path_pub.publish(path_msg)

    def _rdp_simplify_3d(self, voxels, tolerance=None):
        """Ramer-Douglas-Peucker in 3D voxel space."""
        if tolerance is None:
            tolerance = max(1, int(
                self.simp_tol / self.grid_resolution))
        if len(voxels) <= 2:
            return voxels

        start = np.array(voxels[0], dtype=np.float64)
        end = np.array(voxels[-1], dtype=np.float64)
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-6:
            return [voxels[0], voxels[-1]]

        line_unit = line_vec / line_len
        max_dist = 0.0
        max_idx = 0
        for i in range(1, len(voxels) - 1):
            pt = np.array(voxels[i], dtype=np.float64)
            proj = np.dot(pt - start, line_unit)
            proj = max(0, min(line_len, proj))
            closest = start + proj * line_unit
            dist = np.linalg.norm(pt - closest)
            if dist > max_dist:
                max_dist = dist
                max_idx = i

        if max_dist > tolerance:
            left = self._rdp_simplify_3d(voxels[:max_idx+1], tolerance)
            right = self._rdp_simplify_3d(voxels[max_idx:], tolerance)
            return left[:-1] + right
        else:
            return [voxels[0], voxels[-1]]

    def _voxels_to_path(self, voxels) -> Path:
        """Convert list of (ix, iy, iz) to nav_msgs/Path with 3D positions."""
        path_msg = Path()
        path_msg.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='base_link')

        for i, (ix, iy, iz) in enumerate(voxels):
            wx, wy, wz = self._voxel_to_world(ix, iy, iz)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = wz

            # Yaw from direction to next waypoint (3D)
            if i < len(voxels) - 1:
                nwx, nwy, _ = self._voxel_to_world(*voxels[i+1])
                yaw = math.atan2(nwy - wy, nwx - wx)
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        return path_msg

    def _publish_empty_path(self):
        path_msg = Path()
        path_msg.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        self.local_path_pub.publish(path_msg)

    def _publish_status(self):
        status = {
            "state": self.planner_state,
            "has_global_path": self.global_path is not None,
            "has_3d_grid": self.current_grid_flat is not None,
            "has_odom": self.current_odom is not None,
            "metadata_received": self.metadata_received,
            "grid_dimensions": f"{self.grid_nx}x{self.grid_ny}x{self.grid_nz}",
            "total_voxels": self.grid_nx * self.grid_ny * self.grid_nz,
            "replan_count": self.replan_count,
            "dstar_initialized": self.dstar is not None
                                 and self.dstar._initialized,
            "search_type": "3D_26_connected",
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DStarLite3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()