#!/usr/bin/env python3
"""
Pegasus A* Global Planner Node
-------------------------------
Computes an initial global path from the UAV's current position to a
goal waypoint using weighted A* on either the RTAB-Map occupancy grid
(when available) or the local costmap 2D projection.

Architecture context:
    A* (this node)  →  global path (nav_msgs/Path)
            ↓
    D* Lite (local replanner)  →  dynamically adjusted path
            ↓
    MPC (trajectory smoother)  →  kinodynamically feasible commands
            ↓
    PX4 Offboard Interface  →  TrajectorySetpoint over XRCE-DDS

Publishes:
    /pegasus/path_planner/global_path   (nav_msgs/Path)
    /pegasus/path_planner/status        (std_msgs/String — JSON status)
    /pegasus/path_planner/visited       (nav_msgs/OccupancyGrid — debug)

Subscribes:
    /rtabmap/grid_map                   (nav_msgs/OccupancyGrid — SLAM map)
    /pegasus/local_costmap_2d           (nav_msgs/OccupancyGrid — local)
    /odom                               (nav_msgs/Odometry — current pose)
    /pegasus/autonomy/target_waypoint   (geometry_msgs/PoseStamped — goal)
    /pegasus/path_planner/replan_request (std_msgs/Bool — external replan)

Author: Team Pegasus — Cal Poly Pomona
"""

import heapq
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header, Bool, String


# ═══════════════════════════════════════════════════════════════════
#  A* Search Implementation
# ═══════════════════════════════════════════════════════════════════

class AStarResult:
    """Container for A* search results."""
    __slots__ = ('path', 'cost', 'nodes_expanded', 'elapsed_ms', 'success')

    def __init__(self, path=None, cost=0.0, nodes_expanded=0,
                 elapsed_ms=0.0, success=False):
        self.path = path or []
        self.cost = cost
        self.nodes_expanded = nodes_expanded
        self.elapsed_ms = elapsed_ms
        self.success = success


def astar_search(grid: np.ndarray,
                 start: tuple,
                 goal: tuple,
                 heuristic_weight: float = 1.0,
                 diagonal: bool = True,
                 max_iterations: int = 50000,
                 lethal_threshold: int = 90,
                 cost_penalty_factor: float = 2.0) -> AStarResult:
    """
    Weighted A* on a 2D occupancy grid.

    Args:
        grid:              (H, W) int8 array.  -1=unknown, 0=free, 1-100=cost
        start:             (row, col) grid indices
        goal:              (row, col) grid indices
        heuristic_weight:  epsilon for weighted A* (1.0=optimal, >1=greedy)
        diagonal:          allow 8-connected movement
        max_iterations:    hard cap on node expansions
        lethal_threshold:  cells >= this are impassable
        cost_penalty_factor: multiplier for traversing high-cost cells

    Returns:
        AStarResult with path as list of (row, col) grid cells.
    """
    t0 = time.monotonic()
    rows, cols = grid.shape

    # Validate start / goal
    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return AStarResult(success=False)
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return AStarResult(success=False)
    if grid[start[0], start[1]] >= lethal_threshold:
        return AStarResult(success=False)
    if grid[goal[0], goal[1]] >= lethal_threshold:
        return AStarResult(success=False)

    # Movement directions
    SQRT2 = 1.414
    if diagonal:
        directions = [
            (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
            (-1, -1, SQRT2), (-1,  1, SQRT2), ( 1, -1, SQRT2), ( 1,  1, SQRT2),
        ]
    else:
        directions = [
            (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
        ]

    # Heuristic: octile distance (admissible for 8-connected)
    def heuristic(r, c):
        dr = abs(r - goal[0])
        dc = abs(c - goal[1])
        if diagonal:
            return (max(dr, dc) + (SQRT2 - 1) * min(dr, dc)) * heuristic_weight
        return (dr + dc) * heuristic_weight

    # Open list: (f_cost, counter, row, col)
    counter = 0
    open_list = []
    heapq.heappush(open_list, (heuristic(start[0], start[1]), counter, start[0], start[1]))
    counter += 1

    g_cost = np.full((rows, cols), np.inf, dtype=np.float64)
    g_cost[start[0], start[1]] = 0.0

    came_from = {}
    closed = np.zeros((rows, cols), dtype=bool)

    nodes_expanded = 0

    while open_list and nodes_expanded < max_iterations:
        f, _, r, c = heapq.heappop(open_list)

        if closed[r, c]:
            continue
        closed[r, c] = True
        nodes_expanded += 1

        # Goal reached
        if r == goal[0] and c == goal[1]:
            # Reconstruct path
            path = []
            cr, cc = r, c
            while (cr, cc) in came_from:
                path.append((cr, cc))
                cr, cc = came_from[(cr, cc)]
            path.append(start)
            path.reverse()

            elapsed = (time.monotonic() - t0) * 1000.0
            return AStarResult(
                path=path,
                cost=g_cost[r, c],
                nodes_expanded=nodes_expanded,
                elapsed_ms=elapsed,
                success=True)

        for dr, dc, move_cost in directions:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if closed[nr, nc]:
                continue

            cell_val = grid[nr, nc]

            # Impassable
            if cell_val >= lethal_threshold:
                continue

            # Unknown cells: treat as traversable but penalised
            if cell_val < 0:
                traverse_cost = move_cost * 1.5  # mild penalty for unknown
            elif cell_val == 0:
                traverse_cost = move_cost
            else:
                # Cost cells (inflation zone): penalise proportionally
                traverse_cost = move_cost * (1.0 + cost_penalty_factor * cell_val / 100.0)

            tentative_g = g_cost[r, c] + traverse_cost
            if tentative_g < g_cost[nr, nc]:
                g_cost[nr, nc] = tentative_g
                came_from[(nr, nc)] = (r, c)
                f_new = tentative_g + heuristic(nr, nc)
                heapq.heappush(open_list, (f_new, counter, nr, nc))
                counter += 1

    elapsed = (time.monotonic() - t0) * 1000.0
    return AStarResult(nodes_expanded=nodes_expanded, elapsed_ms=elapsed, success=False)


# ═══════════════════════════════════════════════════════════════════
#  ROS 2 Node
# ═══════════════════════════════════════════════════════════════════

class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('global_planner_node')

        # ── Declare parameters ──────────────────────────────
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('diagonal_movement', True)
        self.declare_parameter('max_iterations', 50000)
        self.declare_parameter('cost_penalty_factor', 2.0)
        self.declare_parameter('lethal_cost_threshold', 90)
        self.declare_parameter('unknown_cell_cost', 50)
        self.declare_parameter('goal_tolerance_m', 2.0)
        self.declare_parameter('replan_frequency_hz', 1.0)
        self.declare_parameter('path_deviation_threshold_m', 5.0)
        self.declare_parameter('min_altitude_m', 5.0)
        self.declare_parameter('max_altitude_m', 120.0)
        self.declare_parameter('min_clearance_m', 3.0)
        self.declare_parameter('prefer_local_costmap', True)
        self.declare_parameter('publish_visited_grid', False)

        # Read parameters
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        self.diagonal = self.get_parameter('diagonal_movement').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.cost_penalty_factor = self.get_parameter('cost_penalty_factor').value
        self.lethal_threshold = self.get_parameter('lethal_cost_threshold').value
        self.unknown_cost = self.get_parameter('unknown_cell_cost').value
        self.goal_tolerance_m = self.get_parameter('goal_tolerance_m').value
        self.replan_hz = self.get_parameter('replan_frequency_hz').value
        self.deviation_threshold = self.get_parameter('path_deviation_threshold_m').value
        self.min_alt = self.get_parameter('min_altitude_m').value
        self.max_alt = self.get_parameter('max_altitude_m').value
        self.min_clearance = self.get_parameter('min_clearance_m').value
        self.prefer_local = self.get_parameter('prefer_local_costmap').value
        self.pub_visited = self.get_parameter('publish_visited_grid').value

        # ── State ───────────────────────────────────────────
        self.current_odom = None       # Odometry
        self.goal_pose = None          # PoseStamped
        self.slam_grid = None          # OccupancyGrid from RTAB-Map
        self.local_grid = None         # OccupancyGrid from local costmap 2D
        self.current_path = None       # nav_msgs/Path (last published)
        self.planner_state = "IDLE"    # IDLE | PLANNING | ACTIVE | NO_PATH
        self.replan_requested = False

        # ── Subscribers ─────────────────────────────────────
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.slam_grid_sub = self.create_subscription(
            OccupancyGrid, '/rtabmap/grid_map',
            self._slam_grid_cb, map_qos)

        self.local_grid_sub = self.create_subscription(
            OccupancyGrid, '/pegasus/local_costmap_2d',
            self._local_grid_cb, map_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/pegasus/autonomy/target_waypoint',
            self._goal_cb, 10)

        self.replan_sub = self.create_subscription(
            Bool, '/pegasus/path_planner/replan_request',
            self._replan_cb, 10)

        # ── Publishers ──────────────────────────────────────
        self.path_pub = self.create_publisher(
            Path, '/pegasus/path_planner/global_path', 10)

        self.status_pub = self.create_publisher(
            String, '/pegasus/path_planner/status', 10)

        if self.pub_visited:
            self.visited_pub = self.create_publisher(
                OccupancyGrid, '/pegasus/path_planner/visited', 1)
        else:
            self.visited_pub = None

        # ── Timers ──────────────────────────────────────────
        self.create_timer(1.0 / self.replan_hz, self._planning_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Global Planner: A* (w={self.heuristic_weight}, '
            f'diag={self.diagonal}, max_iter={self.max_iterations})')

    # ── Callbacks ────────────────────────────────────────────

    def _slam_grid_cb(self, msg: OccupancyGrid):
        self.slam_grid = msg

    def _local_grid_cb(self, msg: OccupancyGrid):
        self.local_grid = msg

    def _odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def _goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.replan_requested = True
        self.planner_state = "PLANNING"
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.1f}, '
            f'{msg.pose.position.y:.1f}, {msg.pose.position.z:.1f})')

    def _replan_cb(self, msg: Bool):
        if msg.data:
            self.replan_requested = True
            self.get_logger().info('External replan request received')

    # ── Planning loop ────────────────────────────────────────

    def _planning_tick(self):
        """Main planning loop, runs at replan_frequency_hz."""

        # Need odometry and a goal to plan
        if self.current_odom is None or self.goal_pose is None:
            return

        # Check if a replan is needed
        needs_replan = self.replan_requested
        if not needs_replan and self.current_path is not None:
            needs_replan = self._check_path_deviation()

        if not needs_replan and self.planner_state == "ACTIVE":
            # Path is still valid, check if goal reached
            if self._goal_reached():
                self.planner_state = "IDLE"
                self.goal_pose = None
                self.current_path = None
                self.get_logger().info('Goal reached!')
                self._publish_empty_path()
            return

        if not needs_replan:
            return

        self.replan_requested = False

        # Select the best available grid
        grid_msg = self._select_grid()
        if grid_msg is None:
            self.planner_state = "NO_PATH"
            self.get_logger().warn(
                'No occupancy grid available — cannot plan',
                throttle_duration_sec=5.0)
            return

        # Convert to numpy
        grid_np = self._grid_to_numpy(grid_msg)

        # Get start and goal in grid coordinates
        start_rc = self._world_to_grid(
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y,
            grid_msg)
        goal_rc = self._world_to_grid(
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
            grid_msg)

        if start_rc is None or goal_rc is None:
            self.planner_state = "NO_PATH"
            self.get_logger().warn(
                'Start or goal is outside the grid',
                throttle_duration_sec=2.0)
            return

        # Clamp start to free space if it lands on an obstacle
        # (can happen due to costmap inflation overlapping the UAV)
        start_rc = self._find_nearest_free(grid_np, start_rc)
        if start_rc is None:
            self.planner_state = "NO_PATH"
            self.get_logger().warn('Cannot find free cell near start')
            return

        # Run A*
        result = astar_search(
            grid=grid_np,
            start=start_rc,
            goal=goal_rc,
            heuristic_weight=self.heuristic_weight,
            diagonal=self.diagonal,
            max_iterations=self.max_iterations,
            lethal_threshold=self.lethal_threshold,
            cost_penalty_factor=self.cost_penalty_factor)

        if result.success:
            path_msg = self._grid_path_to_ros(result.path, grid_msg)
            self.path_pub.publish(path_msg)
            self.current_path = path_msg
            self.planner_state = "ACTIVE"
            self.get_logger().info(
                f'A* path found: {len(result.path)} cells, '
                f'{result.nodes_expanded} expanded, '
                f'{result.elapsed_ms:.1f}ms, cost={result.cost:.1f}')
        else:
            self.planner_state = "NO_PATH"
            self._publish_empty_path()
            self.get_logger().warn(
                f'A* failed: {result.nodes_expanded} nodes expanded in '
                f'{result.elapsed_ms:.1f}ms')

    # ── Grid selection ───────────────────────────────────────

    def _select_grid(self) -> OccupancyGrid:
        """
        Select the best available occupancy grid.

        Strategy:
          - If prefer_local_costmap is True AND local costmap is available,
            use the local costmap (faster updates, immediate obstacles).
          - Otherwise fall back to RTAB-Map SLAM grid (larger coverage,
            loop-closure-corrected).
          - If neither is available, return None.

        During first-time flight with no stored map, the SLAM grid may be
        sparse or empty, so the local costmap is usually better early in
        a mission.
        """
        if self.prefer_local and self.local_grid is not None:
            return self.local_grid
        if self.slam_grid is not None:
            return self.slam_grid
        if self.local_grid is not None:
            return self.local_grid
        return None

    # ── Coordinate conversions ───────────────────────────────

    def _grid_to_numpy(self, msg: OccupancyGrid) -> np.ndarray:
        """Convert OccupancyGrid to (H, W) int8 numpy array."""
        return np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _world_to_grid(self, wx, wy, msg: OccupancyGrid):
        """Convert world (x, y) to grid (row, col). Returns None if OOB."""
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if 0 <= row < msg.info.height and 0 <= col < msg.info.width:
            return (row, col)
        return None

    def _grid_to_world(self, row, col, msg: OccupancyGrid):
        """Convert grid (row, col) to world (x, y)."""
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        wx = ox + (col + 0.5) * res
        wy = oy + (row + 0.5) * res
        return wx, wy

    def _find_nearest_free(self, grid_np, start_rc, search_radius=10):
        """
        If start cell is lethal, spiral outward to find nearest free cell.
        Returns (row, col) or None.
        """
        r, c = start_rc
        if grid_np[r, c] < self.lethal_threshold:
            return start_rc

        rows, cols = grid_np.shape
        for radius in range(1, search_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue  # only check perimeter
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if grid_np[nr, nc] < self.lethal_threshold:
                            return (nr, nc)
        return None

    # ── Path conversion ──────────────────────────────────────

    def _grid_path_to_ros(self, grid_path, grid_msg) -> Path:
        """
        Convert a list of (row, col) cells to a nav_msgs/Path.

        The z coordinate is taken from the goal altitude (for UAV flight).
        Heading (yaw) is computed from the direction to the next waypoint.
        """
        path_msg = Path()
        path_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=grid_msg.header.frame_id)

        # Use the goal altitude for all waypoints
        goal_z = self.goal_pose.pose.position.z if self.goal_pose else 0.0
        # Clamp to altitude constraints
        goal_z = max(self.min_alt, min(self.max_alt, goal_z))

        for i, (r, c) in enumerate(grid_path):
            wx, wy = self._grid_to_world(r, c, grid_msg)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = goal_z

            # Compute yaw toward next waypoint
            if i < len(grid_path) - 1:
                nr, nc = grid_path[i + 1]
                nwx, nwy = self._grid_to_world(nr, nc, grid_msg)
                yaw = math.atan2(nwy - wy, nwx - wx)
            elif i > 0:
                # Last point: keep previous heading
                pr, pc = grid_path[i - 1]
                pwx, pwy = self._grid_to_world(pr, pc, grid_msg)
                yaw = math.atan2(wy - pwy, wx - pwx)
            else:
                yaw = 0.0

            # Quaternion from yaw (roll=0, pitch=0)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        return path_msg

    # ── Path monitoring ──────────────────────────────────────

    def _check_path_deviation(self) -> bool:
        """Check if the UAV has drifted too far from the planned path."""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return False
        if self.current_odom is None:
            return False

        px = self.current_odom.pose.pose.position.x
        py = self.current_odom.pose.pose.position.y

        # Find closest point on path
        min_dist = float('inf')
        for pose in self.current_path.poses:
            dx = pose.pose.position.x - px
            dy = pose.pose.position.y - py
            dist = math.sqrt(dx * dx + dy * dy)
            min_dist = min(min_dist, dist)

        return min_dist > self.deviation_threshold

    def _goal_reached(self) -> bool:
        """Check if the UAV is within tolerance of the goal."""
        if self.current_odom is None or self.goal_pose is None:
            return False

        dx = self.goal_pose.pose.position.x - self.current_odom.pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_odom.pose.pose.position.y
        dz = self.goal_pose.pose.position.z - self.current_odom.pose.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz) < self.goal_tolerance_m

    # ── Publishing helpers ───────────────────────────────────

    def _publish_empty_path(self):
        """Publish an empty path (signals no valid plan)."""
        path_msg = Path()
        path_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map')
        self.path_pub.publish(path_msg)

    def _publish_status(self):
        """Publish planner status as JSON."""
        status = {
            "state": self.planner_state,
            "has_slam_grid": self.slam_grid is not None,
            "has_local_grid": self.local_grid is not None,
            "has_odom": self.current_odom is not None,
            "has_goal": self.goal_pose is not None,
            "path_length": len(self.current_path.poses) if self.current_path else 0,
        }

        if self.goal_pose is not None:
            status["goal_x"] = round(self.goal_pose.pose.position.x, 2)
            status["goal_y"] = round(self.goal_pose.pose.position.y, 2)
            status["goal_z"] = round(self.goal_pose.pose.position.z, 2)

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()