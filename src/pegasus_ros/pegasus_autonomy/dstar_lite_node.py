#!/usr/bin/env python3
import heapq
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool, String


class DStarLitePlannerNode(Node):
    def __init__(self):
        super().__init__('path_planning_sim_dstar_lite_core_node')

        self.declare_parameter('diagonal_movement', True)
        self.declare_parameter('cost_penalty_factor', 2.0)
        self.declare_parameter('lethal_cost_threshold', 90)
        self.declare_parameter('goal_tolerance_m', 2.0)
        self.declare_parameter('replan_frequency_hz', 3.0)
        self.declare_parameter('path_deviation_threshold_m', 5.0)
        self.declare_parameter('min_altitude_m', 5.0)
        self.declare_parameter('max_altitude_m', 120.0)
        self.declare_parameter('prefer_local_costmap', True)
        self.declare_parameter('max_dstar_iterations', 150000)
        self.declare_parameter('max_changed_cells_for_incremental', 1500)

        self.diagonal = self.get_parameter('diagonal_movement').value
        self.cost_penalty_factor = self.get_parameter('cost_penalty_factor').value
        self.lethal_threshold = self.get_parameter('lethal_cost_threshold').value
        self.goal_tolerance_m = self.get_parameter('goal_tolerance_m').value
        self.replan_hz = self.get_parameter('replan_frequency_hz').value
        self.deviation_threshold = self.get_parameter('path_deviation_threshold_m').value
        self.min_altitude_m = self.get_parameter('min_altitude_m').value
        self.max_altitude_m = self.get_parameter('max_altitude_m').value
        self.prefer_local = self.get_parameter('prefer_local_costmap').value
        self.max_dstar_iterations = self.get_parameter('max_dstar_iterations').value
        self.max_changed_cells_for_incremental = self.get_parameter('max_changed_cells_for_incremental').value

        self.current_odom = None
        self.goal_pose = None
        self.slam_grid = None
        self.local_grid = None
        self.current_path = None
        self.planner_state = 'IDLE'
        self.replan_requested = False

        self.previous_grid_np = None
        self.previous_grid_signature = None
        self.last_planning_grid = None

        self.g_values = {}
        self.rhs_values = {}
        self.open_heap = []
        self.open_lookup = {}
        self.km = 0.0
        self.initialized = False
        self.goal_state = None
        self.start_state = None
        self.last_start_state = None

        self.last_compute_ms = 0.0
        self.last_compute_iterations = 0
        self.last_changed_cells = 0
        self.last_replan_reason = 'none'
        self.last_update_mode = 'none'
        self.incremental_update_count = 0
        self.full_reset_count = 0

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self._slam_grid_cb, map_qos)
        self.create_subscription(OccupancyGrid, '/pegasus/local_costmap_2d', self._local_grid_cb, map_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/pegasus/autonomy/target_waypoint', self._goal_cb, 10)
        self.create_subscription(Bool, '/pegasus/path_planner/replan_request', self._replan_cb, 10)

        self.path_pub = self.create_publisher(Path, '/pegasus/path_planner/global_path', 10)
        self.status_pub = self.create_publisher(String, '/pegasus/path_planner/status', 10)

        self.create_timer(1.0 / self.replan_hz, self._planning_tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'D* Lite incremental core active (diag={self.diagonal}, cost_penalty_factor={self.cost_penalty_factor})'
        )

    def _slam_grid_cb(self, message: OccupancyGrid):
        self.slam_grid = message

    def _local_grid_cb(self, message: OccupancyGrid):
        self.local_grid = message

    def _odom_cb(self, message: Odometry):
        self.current_odom = message

    def _goal_cb(self, message: PoseStamped):
        self.goal_pose = message
        self.replan_requested = True
        self.planner_state = 'PLANNING'
        self.get_logger().info(
            f'New goal received: ({message.pose.position.x:.1f}, {message.pose.position.y:.1f}, {message.pose.position.z:.1f})'
        )

    def _replan_cb(self, message: Bool):
        if message.data:
            self.replan_requested = True

    def _planning_tick(self):
        if self.current_odom is None or self.goal_pose is None:
            return

        selected_grid = self._select_grid()
        if selected_grid is None:
            self.planner_state = 'NO_PATH'
            self.last_replan_reason = 'no_grid'
            return

        grid_signature = self._grid_signature(selected_grid)
        if self.previous_grid_signature is not None and grid_signature != self.previous_grid_signature:
            self._reset_incremental_state()
            self.last_update_mode = 'full_reset_grid_signature'
            self.full_reset_count += 1

        current_grid_np = self._grid_to_numpy(selected_grid)
        changed_states = self._compute_changed_states(current_grid_np)

        start_state = self._world_to_grid(
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y,
            selected_grid,
        )
        goal_state = self._world_to_grid(
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
            selected_grid,
        )

        if start_state is None or goal_state is None:
            self.planner_state = 'NO_PATH'
            self._publish_empty_path()
            return

        start_state = self._find_nearest_traversable(current_grid_np, start_state)
        goal_state = self._find_nearest_traversable(current_grid_np, goal_state)
        if start_state is None or goal_state is None:
            self.planner_state = 'NO_PATH'
            self._publish_empty_path()
            return

        if (not self.initialized) or (self.goal_state != goal_state):
            self._initialize_incremental(current_grid_np, start_state, goal_state)
            self.last_update_mode = 'full_init'
            self.full_reset_count += 1
            start_state_moved = False
        else:
            start_state_moved = False
            if self.last_start_state is not None and start_state != self.start_state:
                self.km += self._heuristic(self.last_start_state, start_state)
                start_state_moved = True
            self.start_state = start_state
            self.last_start_state = start_state

            if changed_states:
                map_update_mode = self._apply_changed_states(current_grid_np, changed_states)
                self.last_update_mode = map_update_mode
                if map_update_mode.startswith('incremental'):
                    self.incremental_update_count += 1
                elif map_update_mode.startswith('full_reset'):
                    self.full_reset_count += 1

        replan_reasons = []
        if self.replan_requested:
            replan_reasons.append('manual_or_goal')
        if bool(changed_states):
            replan_reasons.append('map_changed')
        if start_state_moved:
            replan_reasons.append('start_moved')

        needs_replan = bool(replan_reasons)
        if not needs_replan and self.current_path is not None:
            if self._check_path_deviation():
                needs_replan = True
                replan_reasons.append('deviation')

        self.last_replan_reason = '+'.join(replan_reasons) if replan_reasons else 'none'

        if not needs_replan and self.planner_state == 'ACTIVE':
            if self._goal_reached():
                self.planner_state = 'IDLE'
                self.goal_pose = None
                self.current_path = None
                self._publish_empty_path()
                self.get_logger().info('Goal reached!')
            self._cache_grid_snapshot(current_grid_np, grid_signature, selected_grid)
            return

        self.replan_requested = False

        compute_started = time.monotonic()
        terminated_early = self._compute_shortest_path(current_grid_np)
        self.last_compute_ms = (time.monotonic() - compute_started) * 1000.0

        if self.last_update_mode == 'none' and needs_replan:
            self.last_update_mode = 'incremental_recompute'

        path_states = self._extract_path_states(current_grid_np)
        if path_states:
            path_message = self._grid_path_to_ros(path_states, selected_grid)
            self.path_pub.publish(path_message)
            self.current_path = path_message
            self.planner_state = 'ACTIVE'
            self.get_logger().info(
                f'D* Lite path found: {len(path_states)} cells, iterations={self.last_compute_iterations}, '
                f'elapsed={self.last_compute_ms:.1f}ms, changed_cells={self.last_changed_cells}, '
                f'mode={self.last_update_mode}, reason={self.last_replan_reason}'
            )
        else:
            self.current_path = None
            self.planner_state = 'NO_PATH'
            self._publish_empty_path()
            if terminated_early:
                self.get_logger().warn(
                    f'D* Lite terminated at iteration cap ({self.max_dstar_iterations}) with no path'
                )
            else:
                self.get_logger().warn('D* Lite reports no traversable path')

        self._cache_grid_snapshot(current_grid_np, grid_signature, selected_grid)

    def _select_grid(self):
        if self.prefer_local and self.local_grid is not None:
            return self.local_grid
        if self.slam_grid is not None:
            return self.slam_grid
        return self.local_grid

    def _grid_signature(self, grid_message: OccupancyGrid):
        origin = grid_message.info.origin.position
        return (
            grid_message.info.width,
            grid_message.info.height,
            float(grid_message.info.resolution),
            float(origin.x),
            float(origin.y),
            grid_message.header.frame_id,
        )

    def _grid_to_numpy(self, grid_message: OccupancyGrid) -> np.ndarray:
        return np.array(grid_message.data, dtype=np.int16).reshape(grid_message.info.height, grid_message.info.width)

    def _world_to_grid(self, world_x: float, world_y: float, grid_message: OccupancyGrid):
        resolution = grid_message.info.resolution
        origin_x = grid_message.info.origin.position.x
        origin_y = grid_message.info.origin.position.y

        col_index = int((world_x - origin_x) / resolution)
        row_index = int((world_y - origin_y) / resolution)

        if 0 <= row_index < grid_message.info.height and 0 <= col_index < grid_message.info.width:
            return (row_index, col_index)
        return None

    def _grid_to_world(self, row_index: int, col_index: int, grid_message: OccupancyGrid):
        resolution = grid_message.info.resolution
        origin_x = grid_message.info.origin.position.x
        origin_y = grid_message.info.origin.position.y
        return (
            origin_x + (col_index + 0.5) * resolution,
            origin_y + (row_index + 0.5) * resolution,
        )

    def _find_nearest_traversable(self, grid_np: np.ndarray, state, search_radius: int = 12):
        row_index, col_index = state
        if self._is_traversable_state(grid_np, state):
            return state

        rows_count, cols_count = grid_np.shape
        for radius in range(1, search_radius + 1):
            for row_delta in range(-radius, radius + 1):
                for col_delta in range(-radius, radius + 1):
                    if abs(row_delta) != radius and abs(col_delta) != radius:
                        continue
                    candidate_row = row_index + row_delta
                    candidate_col = col_index + col_delta
                    if 0 <= candidate_row < rows_count and 0 <= candidate_col < cols_count:
                        candidate_state = (candidate_row, candidate_col)
                        if self._is_traversable_state(grid_np, candidate_state):
                            return candidate_state
        return None

    def _cache_grid_snapshot(self, grid_np: np.ndarray, grid_signature, selected_grid: OccupancyGrid):
        self.previous_grid_np = grid_np.copy()
        self.previous_grid_signature = grid_signature
        self.last_planning_grid = selected_grid

    def _compute_changed_states(self, current_grid_np: np.ndarray):
        if self.previous_grid_np is None:
            self.last_changed_cells = int(current_grid_np.size)
            return []
        if current_grid_np.shape != self.previous_grid_np.shape:
            self.last_changed_cells = int(current_grid_np.size)
            return []

        changed_indices = np.argwhere(current_grid_np != self.previous_grid_np)
        self.last_changed_cells = int(len(changed_indices))
        return [tuple(int(index_value) for index_value in state_index) for state_index in changed_indices]

    def _reset_incremental_state(self):
        self.g_values.clear()
        self.rhs_values.clear()
        self.open_heap.clear()
        self.open_lookup.clear()
        self.km = 0.0
        self.initialized = False
        self.goal_state = None
        self.start_state = None
        self.last_start_state = None

    def _initialize_incremental(self, grid_np: np.ndarray, start_state, goal_state):
        self._reset_incremental_state()
        self.start_state = start_state
        self.goal_state = goal_state
        self.last_start_state = start_state
        self._set_rhs(goal_state, 0.0)
        self._push_open(goal_state, self._calculate_key(goal_state))
        self.initialized = True

    def _is_traversable_state(self, grid_np: np.ndarray, state) -> bool:
        row_index, col_index = state
        if row_index < 0 or col_index < 0 or row_index >= grid_np.shape[0] or col_index >= grid_np.shape[1]:
            return False
        return int(grid_np[row_index, col_index]) < self.lethal_threshold

    def _neighbor_states(self, grid_np: np.ndarray, state):
        row_index, col_index = state
        base_moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.diagonal:
            base_moves.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        neighbor_list = []
        for row_delta, col_delta in base_moves:
            candidate_state = (row_index + row_delta, col_index + col_delta)
            if 0 <= candidate_state[0] < grid_np.shape[0] and 0 <= candidate_state[1] < grid_np.shape[1]:
                neighbor_list.append(candidate_state)
        return neighbor_list

    def _heuristic(self, from_state, to_state):
        row_delta = abs(from_state[0] - to_state[0])
        col_delta = abs(from_state[1] - to_state[1])
        if self.diagonal:
            diagonal_steps = min(row_delta, col_delta)
            straight_steps = max(row_delta, col_delta) - diagonal_steps
            return diagonal_steps * math.sqrt(2.0) + straight_steps
        return float(row_delta + col_delta)

    def _state_cost_penalty(self, grid_np: np.ndarray, state):
        cell_cost = int(grid_np[state[0], state[1]])
        if cell_cost < 0:
            return 1.5
        if cell_cost == 0:
            return 1.0
        return 1.0 + self.cost_penalty_factor * cell_cost / 100.0

    def _transition_cost(self, grid_np: np.ndarray, from_state, to_state):
        if (not self._is_traversable_state(grid_np, from_state)) or (not self._is_traversable_state(grid_np, to_state)):
            return math.inf

        row_delta = abs(from_state[0] - to_state[0])
        col_delta = abs(from_state[1] - to_state[1])
        if row_delta == 1 and col_delta == 1:
            base_distance = math.sqrt(2.0)
        else:
            base_distance = 1.0
        return base_distance * self._state_cost_penalty(grid_np, to_state)

    def _get_g(self, state):
        return self.g_values.get(state, math.inf)

    def _set_g(self, state, value: float):
        if math.isinf(value):
            self.g_values.pop(state, None)
        else:
            self.g_values[state] = value

    def _get_rhs(self, state):
        return self.rhs_values.get(state, math.inf)

    def _set_rhs(self, state, value: float):
        if math.isinf(value):
            self.rhs_values.pop(state, None)
        else:
            self.rhs_values[state] = value

    def _calculate_key(self, state):
        min_cost = min(self._get_g(state), self._get_rhs(state))
        return (
            min_cost + self._heuristic(self.start_state, state) + self.km,
            min_cost,
        )

    def _key_less(self, left_key, right_key):
        epsilon = 1e-9
        if left_key[0] < right_key[0] - epsilon:
            return True
        if abs(left_key[0] - right_key[0]) <= epsilon and left_key[1] < right_key[1] - epsilon:
            return True
        return False

    def _push_open(self, state, key):
        self.open_lookup[state] = key
        heapq.heappush(self.open_heap, (key[0], key[1], state))

    def _peek_open(self):
        while self.open_heap:
            top_key_first, top_key_second, top_state = self.open_heap[0]
            current_key = self.open_lookup.get(top_state)
            if current_key is None or abs(current_key[0] - top_key_first) > 1e-9 or abs(current_key[1] - top_key_second) > 1e-9:
                heapq.heappop(self.open_heap)
                continue
            return current_key, top_state
        return None, None

    def _pop_open(self):
        while self.open_heap:
            top_key_first, top_key_second, top_state = heapq.heappop(self.open_heap)
            current_key = self.open_lookup.get(top_state)
            if current_key is None:
                continue
            if abs(current_key[0] - top_key_first) <= 1e-9 and abs(current_key[1] - top_key_second) <= 1e-9:
                del self.open_lookup[top_state]
                return current_key, top_state
        return None, None

    def _remove_open(self, state):
        self.open_lookup.pop(state, None)

    def _update_vertex(self, grid_np: np.ndarray, state):
        if state != self.goal_state:
            min_rhs = math.inf
            for neighbor_state in self._neighbor_states(grid_np, state):
                transition_cost = self._transition_cost(grid_np, state, neighbor_state)
                if math.isinf(transition_cost):
                    continue
                min_rhs = min(min_rhs, transition_cost + self._get_g(neighbor_state))
            self._set_rhs(state, min_rhs)

        self._remove_open(state)
        if abs(self._get_g(state) - self._get_rhs(state)) > 1e-9:
            self._push_open(state, self._calculate_key(state))

    def _apply_changed_states(self, grid_np: np.ndarray, changed_states):
        if not changed_states:
            return 'none'

        if len(changed_states) > self.max_changed_cells_for_incremental:
            if self.start_state is not None and self.goal_state is not None:
                self._initialize_incremental(grid_np, self.start_state, self.goal_state)
            return 'full_reset_large_change'

        affected_states = set()
        for changed_state in changed_states:
            affected_states.add(changed_state)
            for predecessor_state in self._neighbor_states(grid_np, changed_state):
                affected_states.add(predecessor_state)

        for affected_state in affected_states:
            self._update_vertex(grid_np, affected_state)
        return 'incremental_local_update'

    def _compute_shortest_path(self, grid_np: np.ndarray):
        self.last_compute_iterations = 0
        iteration_count = 0

        while True:
            top_key, _ = self._peek_open()
            start_key = self._calculate_key(self.start_state)
            start_consistent = abs(self._get_rhs(self.start_state) - self._get_g(self.start_state)) <= 1e-9

            queue_requires_work = top_key is not None and self._key_less(top_key, start_key)
            if (not queue_requires_work) and start_consistent:
                break

            if iteration_count >= self.max_dstar_iterations:
                self.last_compute_iterations = iteration_count
                return True

            popped_key, popped_state = self._pop_open()
            if popped_state is None:
                break

            recalculated_key = self._calculate_key(popped_state)
            if self._key_less(popped_key, recalculated_key):
                self._push_open(popped_state, recalculated_key)
                iteration_count += 1
                continue

            if self._get_g(popped_state) > self._get_rhs(popped_state):
                self._set_g(popped_state, self._get_rhs(popped_state))
                for predecessor_state in self._neighbor_states(grid_np, popped_state):
                    self._update_vertex(grid_np, predecessor_state)
            else:
                self._set_g(popped_state, math.inf)
                self._update_vertex(grid_np, popped_state)
                for predecessor_state in self._neighbor_states(grid_np, popped_state):
                    self._update_vertex(grid_np, predecessor_state)

            iteration_count += 1

        self.last_compute_iterations = iteration_count
        return False

    def _extract_path_states(self, grid_np: np.ndarray):
        if self.start_state is None or self.goal_state is None:
            return []
        if math.isinf(self._get_g(self.start_state)):
            return []

        path_states = [self.start_state]
        visited_states = {self.start_state}
        current_state = self.start_state
        max_path_len = grid_np.shape[0] * grid_np.shape[1]

        for _ in range(max_path_len):
            if current_state == self.goal_state:
                return path_states

            best_neighbor = None
            best_total_cost = math.inf
            for neighbor_state in self._neighbor_states(grid_np, current_state):
                transition_cost = self._transition_cost(grid_np, current_state, neighbor_state)
                if math.isinf(transition_cost):
                    continue
                total_cost = transition_cost + self._get_g(neighbor_state)
                if total_cost < best_total_cost:
                    best_total_cost = total_cost
                    best_neighbor = neighbor_state

            if best_neighbor is None or math.isinf(best_total_cost):
                return []
            if best_neighbor in visited_states and best_neighbor != self.goal_state:
                return []

            path_states.append(best_neighbor)
            visited_states.add(best_neighbor)
            current_state = best_neighbor

        return []

    def _grid_path_to_ros(self, path_states, grid_message: OccupancyGrid):
        path_message = Path()
        path_message.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=grid_message.header.frame_id,
        )

        target_altitude = self.goal_pose.pose.position.z if self.goal_pose else 0.0
        target_altitude = max(self.min_altitude_m, min(self.max_altitude_m, target_altitude))

        for index, (row_index, col_index) in enumerate(path_states):
            world_x, world_y = self._grid_to_world(row_index, col_index, grid_message)
            pose_message = PoseStamped()
            pose_message.header = path_message.header
            pose_message.pose.position.x = world_x
            pose_message.pose.position.y = world_y
            pose_message.pose.position.z = target_altitude

            if index < len(path_states) - 1:
                next_row, next_col = path_states[index + 1]
                next_world_x, next_world_y = self._grid_to_world(next_row, next_col, grid_message)
                yaw_angle = math.atan2(next_world_y - world_y, next_world_x - world_x)
            elif index > 0:
                prev_row, prev_col = path_states[index - 1]
                prev_world_x, prev_world_y = self._grid_to_world(prev_row, prev_col, grid_message)
                yaw_angle = math.atan2(world_y - prev_world_y, world_x - prev_world_x)
            else:
                yaw_angle = 0.0

            pose_message.pose.orientation.z = math.sin(yaw_angle / 2.0)
            pose_message.pose.orientation.w = math.cos(yaw_angle / 2.0)
            path_message.poses.append(pose_message)

        return path_message

    def _check_path_deviation(self):
        if self.current_path is None or not self.current_path.poses or self.current_odom is None:
            return False

        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y

        minimum_distance = float('inf')
        for pose_stamped in self.current_path.poses:
            distance_x = pose_stamped.pose.position.x - current_x
            distance_y = pose_stamped.pose.position.y - current_y
            minimum_distance = min(minimum_distance, math.sqrt(distance_x * distance_x + distance_y * distance_y))

        return minimum_distance > self.deviation_threshold

    def _goal_reached(self):
        if self.current_odom is None or self.goal_pose is None:
            return False

        delta_x = self.goal_pose.pose.position.x - self.current_odom.pose.pose.position.x
        delta_y = self.goal_pose.pose.position.y - self.current_odom.pose.pose.position.y
        delta_z = self.goal_pose.pose.position.z - self.current_odom.pose.pose.position.z
        return math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) < self.goal_tolerance_m

    def _publish_empty_path(self):
        path_message = Path()
        path_message.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map',
        )
        self.path_pub.publish(path_message)

    def _publish_status(self):
        changed_ratio = 0.0
        if self.previous_grid_np is not None and self.previous_grid_np.size > 0:
            changed_ratio = float(self.last_changed_cells) / float(self.previous_grid_np.size)

        status_payload = {
            'planner': 'DStarLiteIncremental',
            'state': self.planner_state,
            'has_slam_grid': self.slam_grid is not None,
            'has_local_grid': self.local_grid is not None,
            'has_odom': self.current_odom is not None,
            'has_goal': self.goal_pose is not None,
            'path_length': len(self.current_path.poses) if self.current_path else 0,
            'dstar_initialized': self.initialized,
            'dstar_queue_size': len(self.open_lookup),
            'dstar_last_iterations': self.last_compute_iterations,
            'dstar_last_elapsed_ms': round(self.last_compute_ms, 3),
            'dstar_changed_cells': self.last_changed_cells,
            'dstar_changed_ratio': round(changed_ratio, 6),
            'dstar_last_replan_reason': self.last_replan_reason,
            'dstar_last_update_mode': self.last_update_mode,
            'dstar_incremental_updates': self.incremental_update_count,
            'dstar_full_resets': self.full_reset_count,
            'dstar_incremental_change_cap': int(self.max_changed_cells_for_incremental),
        }

        if self.goal_pose is not None:
            status_payload['goal_x'] = round(self.goal_pose.pose.position.x, 2)
            status_payload['goal_y'] = round(self.goal_pose.pose.position.y, 2)
            status_payload['goal_z'] = round(self.goal_pose.pose.position.z, 2)

        status_message = String()
        status_message.data = json.dumps(status_payload)
        self.status_pub.publish(status_message)


def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
