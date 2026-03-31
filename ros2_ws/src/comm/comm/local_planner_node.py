import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np


OCCUPANCY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class LocalPlannerNode(Node):
    """
    Gap-based local planner for obstacle avoidance.

    Subscribes
    ----------
    occupancy_node/occupancy_grid  - local 2-D grid from OccupancyNode
    mavros/local_position/pose     - current drone pose (map/ENU frame)
    rob498_drone_8/cmd_pose        - PoseStamped target from CommNode.
                                     CommNode auto-routes here when this node
                                     is subscribed (see _timer_callback in CommNode).

    Publishes
    ---------
    mavros/setpoint_position/local - PoseStamped forwarded to MAVROS.
                                     Passthrough when path clear; intermediate
                                     position setpoint substituted when blocked.
                                     Altitude (z) and orientation are always taken
                                     from CommNode's target pose.

    Algorithm
    ---------
    Each timer tick:
    1. Derive intended heading from current_pose -> desired_pose direction vector.
       Project a point at lookahead_dist in that direction -> target_col in grid.
    2. Build a per-column clearance profile: forward distance (m) to nearest
       occupied cell per column (inf = fully clear).
    3. Inflate clearance inward by safety_radius - rejects corridors too narrow
       for the drone body.
    4. If the corridor around target_col is clear: passthrough desired_pose
       unchanged (preserves altitude and orientation from CommNode).
    5. Otherwise find contiguous passable gaps (width >= min_gap_width), pick the
       one closest to the intended direction, compute intermediate position setpoint
       at lookahead_dist ahead offset laterally toward the gap.
    6. No gap -> hold current XY, maintain desired altitude (drone stops laterally).
    """

    def __init__(self):
        super().__init__('local_planner_node')

        self.declare_parameter('lookahead_dist', 0.5)   # m - intermediate waypoint distance
        self.declare_parameter('safety_radius', 0.05)   # m - narrower gaps ignored (5 cm each side)
        self.declare_parameter('min_gap_width', 0.30)   # m - body clearance required
        self.declare_parameter('obstacle_thresh', 50)   # occupancy value = occupied
        self.declare_parameter('planner_rate', 20.0)    # Hz - match CommNode timer
        self.declare_parameter('debug_logging', True)
        self.declare_parameter('debug_log_period_s', 1.0)

        self.lookahead = self.get_parameter('lookahead_dist').value
        self.safety_r = self.get_parameter('safety_radius').value
        self.min_gap_w = self.get_parameter('min_gap_width').value
        self.occ_thresh = self.get_parameter('obstacle_thresh').value
        rate = self.get_parameter('planner_rate').value
        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        self.debug_log_period_s = float(self.get_parameter('debug_log_period_s').value)

        self.grid_msg = None
        self.current_pose = None
        self.desired_pose = None   # Latest PoseStamped target from CommNode.
        self.last_passthrough_reason = None
        self.last_debug_log_time = None
        self.last_plan_mode = None
        self.enabled = False

        self.occ_sub = self.create_subscription(
            OccupancyGrid, 'occupancy_node/occupancy_grid', self._occ_cb, OCCUPANCY_QOS)
        self.enable_sub = self.create_subscription(
            Bool, '/local_planner/enabled', self._enable_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self._pose_cb, qos_profile_sensor_data)
        # Subscribing here triggers CommNode's routing logic:
        # _timer_callback checks get_subscription_count() on rob498_drone_8/cmd_pose.
        self.cmd_sub = self.create_subscription(
            PoseStamped, 'rob498_drone_8/cmd_pose', self._cmd_cb, 10)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        self.timer = self.create_timer(1.0 / rate, self._plan)
        self.get_logger().info('LocalPlannerNode ready.')

    # ------------------------------------------------------------------ callbacks
    def _occ_cb(self, msg: OccupancyGrid):
        self.grid_msg = msg

    def _enable_cb(self, msg: Bool):
        self.enabled = msg.data

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _cmd_cb(self, msg: PoseStamped):
        self.desired_pose = msg
        if not self.enabled:
            return
        # If planner inputs are not ready yet, keep MAVROS fed with the raw target.
        if self.grid_msg is None or self.current_pose is None:
            self._publish_passthrough(msg)

    # ------------------------------------------------------------------ main loop
    def _plan(self):
        if not self.enabled:
            return

        if self.desired_pose is None:
            return

        if self.grid_msg is None:
            self._log_plan('missing_inputs', self.desired_pose, self.desired_pose, reason='occupancy_grid_missing')
            self._publish_passthrough(self.desired_pose)
            return

        if self.current_pose is None:
            self._log_plan('missing_inputs', self.desired_pose, self.desired_pose, reason='vehicle_pose_missing')
            self._publish_passthrough(self.desired_pose)
            return

        self.last_passthrough_reason = None

        info = self.grid_msg.info
        res = info.resolution
        W, D = info.width, info.height
        safety_cells = max(1, int(self.safety_r / res))
        min_gap_cells = max(1, int(self.min_gap_w / res))

        # --- 1. Intended heading -> grid column ----------------------------
        target_col, target_fwd = self._target_in_grid(W, res)

        # --- 2. Per-column clearance profile ------------------------------
        clearance = self._column_clearance(W, D, res)
        closest_dist_m, closest_offset_m, closest_side = self._summarize_closest_obstacle(
            clearance, W, res)

        # --- 3. Inflate by safety_radius (sliding window minimum) ---------
        inflated = np.array([
            clearance[max(0, c - safety_cells): min(W, c + safety_cells + 1)].min()
            for c in range(W)
        ])

        # --- 4. Direct path check -----------------------------------------
        c0 = max(0, int(target_col) - safety_cells)
        c1 = min(W - 1, int(target_col) + safety_cells)
        direct_clear = bool(np.all(inflated[c0: c1 + 1] >= self.lookahead))

        if direct_clear and target_fwd > 0:
            self._log_plan(
                'passthrough', self.desired_pose, self.desired_pose,
                closest_dist_m=closest_dist_m,
                closest_offset_m=closest_offset_m,
                closest_side=closest_side,
                direct_clear=direct_clear,
                best_col=None,
            )
            self.setpoint_pub.publish(self.desired_pose)
            return

        # --- 5. Gap finding -----------------------------------------------
        passable = inflated >= self.lookahead
        best_col = _best_gap_center(passable, W, target_col, min_gap_cells)

        if best_col is not None:
            right_m = (best_col - W / 2.0) * res
            out = self._intermediate_position(self.lookahead, right_m)
            mode = 'avoid'
        else:
            # --- 6. No gap - hold XY, maintain desired altitude -----------
            self.get_logger().warn('No passable gap found - stopping.')
            out = self._hold_position()
            mode = 'stop'

        self._log_plan(
            mode, self.desired_pose, out,
            closest_dist_m=closest_dist_m,
            closest_offset_m=closest_offset_m,
            closest_side=closest_side,
            direct_clear=direct_clear,
            best_col=best_col,
        )
        self.setpoint_pub.publish(out)

    def _publish_passthrough(self, cmd: PoseStamped):
        # Keep the original target intact when the planner cannot safely modify it yet.
        passthrough = PoseStamped()
        passthrough.header.stamp = self.get_clock().now().to_msg()
        passthrough.header.frame_id = cmd.header.frame_id
        passthrough.pose = cmd.pose
        self.setpoint_pub.publish(passthrough)

    def _log_passthrough(self, reason: str):
        if self.last_passthrough_reason == reason:
            return
        self.get_logger().warn(
            f'Planner input missing ({reason}); forwarding commands directly to MAVROS.'
        )
        self.last_passthrough_reason = reason

    def _should_log_debug(self, mode: str):
        if not self.debug_logging:
            return False

        now = self.get_clock().now()
        mode_changed = mode != self.last_plan_mode
        if self.last_debug_log_time is None:
            self.last_debug_log_time = now
            self.last_plan_mode = mode
            return True

        elapsed_s = (now - self.last_debug_log_time).nanoseconds / 1e9
        if mode_changed or elapsed_s >= self.debug_log_period_s:
            self.last_debug_log_time = now
            self.last_plan_mode = mode
            return True

        return False

    def _log_plan(self, mode, input_pose, output_pose, closest_dist_m=float('inf'),
                  closest_offset_m=0.0, closest_side='none', direct_clear=None,
                  best_col=None, reason=''):
        if mode == 'missing_inputs':
            self._log_passthrough(reason or 'missing_inputs')

        if not self._should_log_debug(mode):
            return

        grid_age_ms = self._grid_age_ms()
        grid_age_text = f'{grid_age_ms:.0f}' if grid_age_ms is not None else 'n/a'
        closest_dist_text = f'{closest_dist_m:.2f}' if math.isfinite(closest_dist_m) else 'inf'
        best_col_text = f'{best_col:.1f}' if best_col is not None else 'n/a'
        direct_clear_text = str(direct_clear) if direct_clear is not None else 'n/a'

        self.get_logger().info(
            '[PLAN] '
            f'mode={mode} '
            f'grid_age_ms={grid_age_text} '
            f'in={self._format_pose(input_pose)} '
            f'out={self._format_pose(output_pose)} '
            f'closest={closest_side}@{closest_dist_text}m '
            f'offset={closest_offset_m:.2f} '
            f'direct_clear={direct_clear_text} '
            f'best_col={best_col_text} '
            f'reason={reason or "n/a"}'
        )

    # ------------------------------------------------------------------ helpers
    def _format_pose(self, pose: PoseStamped):
        p = pose.pose.position
        return f'pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})'

    def _grid_age_ms(self):
        if self.grid_msg is None:
            return None

        stamp = self.grid_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None

        msg_time = Time.from_msg(stamp)
        now = self.get_clock().now()
        return (now - msg_time).nanoseconds / 1e6

    def _summarize_closest_obstacle(self, clearance: np.ndarray, width: int, res: float):
        finite_cols = np.where(np.isfinite(clearance))[0]
        if finite_cols.size == 0:
            return float('inf'), 0.0, 'none'

        closest_dist_m = float(np.min(clearance[finite_cols]))
        closest_candidates = finite_cols[np.isclose(clearance[finite_cols], closest_dist_m)]
        center = width / 2.0
        chosen_col = closest_candidates[np.argmin(np.abs(closest_candidates - center))]
        closest_offset_m = (float(chosen_col) - center) * res
        return closest_dist_m, closest_offset_m, self._side_from_offset(closest_offset_m, res)

    def _side_from_offset(self, offset_m: float, res: float):
        if abs(offset_m) <= (res / 2.0):
            return 'center'
        return 'left' if offset_m < 0.0 else 'right'

    def _column_clearance(self, W: int, D: int, res: float) -> np.ndarray:
        """For each column: forward distance (m) to nearest occupied cell, inf if clear."""
        data = np.array(self.grid_msg.data, dtype=np.int8).reshape((D, W))
        occupied = data >= self.occ_thresh
        has_obs = np.any(occupied, axis=0)
        nearest = np.argmax(occupied, axis=0)   # 0 when no obstacle, masked below
        return np.where(has_obs, nearest * res, np.inf)

    def _target_in_grid(self, W: int, res: float):
        """
        Derive target grid column + forward component from the direction to desired_pose.
        Projects a point at lookahead_dist in the heading direction.
        Returns (col: float, fwd_body: float).
        """
        dx = self.desired_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.desired_pose.pose.position.y - self.current_pose.pose.position.y
        dist = math.hypot(dx, dy)

        if dist < 1e-3:
            return W / 2.0, 0.0   # Already at target -> check straight ahead.

        yaw = _yaw_from_quat(self.current_pose.pose.orientation)

        # Rotate map-frame direction into drone body frame.
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        right = dx * math.sin(yaw) - dy * math.cos(yaw)

        col = W / 2.0 + (right / dist) * (self.lookahead / res)
        return col, fwd

    def _intermediate_position(self, fwd_m: float, right_m: float) -> PoseStamped:
        """
        Compute an intermediate PoseStamped lookahead_dist ahead, offset laterally
        toward the chosen gap. Altitude and orientation preserved from desired_pose.
        """
        yaw = _yaw_from_quat(self.current_pose.pose.orientation)

        # Body (fwd, right) -> map frame (same transform as original _override_velocity).
        map_dx = fwd_m * math.cos(yaw) + right_m * math.sin(yaw)
        map_dy = fwd_m * math.sin(yaw) - right_m * math.cos(yaw)

        dist = math.hypot(map_dx, map_dy)
        if dist > 1e-3:
            scale = self.lookahead / dist
            map_dx *= scale
            map_dy *= scale

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.desired_pose.header.frame_id
        out.pose.position.x = self.current_pose.pose.position.x + map_dx
        out.pose.position.y = self.current_pose.pose.position.y + map_dy
        out.pose.position.z = self.desired_pose.pose.position.z   # Preserve target altitude.
        out.pose.orientation = self.desired_pose.pose.orientation  # Preserve target yaw.
        return out

    def _hold_position(self) -> PoseStamped:
        """Hold current XY, maintain desired altitude."""
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.desired_pose.header.frame_id
        out.pose.position.x = self.current_pose.pose.position.x
        out.pose.position.y = self.current_pose.pose.position.y
        out.pose.position.z = self.desired_pose.pose.position.z
        out.pose.orientation = self.desired_pose.pose.orientation
        return out


# ------------------------------------------------------------------ module-level
def _best_gap_center(passable: np.ndarray, W: int, target_col: float,
                     min_gap_cells: int):
    """
    Scan passable left-to-right for contiguous True runs.
    Return the centre of the qualifying run closest to target_col, or None.
    """
    best_col = None
    best_dist = np.inf
    in_gap = False
    gap_start = 0

    for c in range(W + 1):
        if c < W and passable[c]:
            if not in_gap:
                gap_start = c
                in_gap = True
        else:
            if in_gap:
                gap_end = c
                if (gap_end - gap_start) >= min_gap_cells:
                    centre = (gap_start + gap_end - 1) / 2.0
                    dist = abs(centre - target_col)
                    if dist < best_dist:
                        best_dist = dist
                        best_col = centre
                in_gap = False

    return best_col


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
