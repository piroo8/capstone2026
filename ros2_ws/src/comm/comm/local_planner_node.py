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
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import OccupancyGrid
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
    rob498_drone_8/cmd_vel         - PositionTarget from CommNode
                                     CommNode auto-routes here when this node
                                     is subscribed (see _publish_velocity_command).

    Publishes
    ---------
    mavros/setpoint_raw/local      - PositionTarget forwarded to MAVROS.
                                     Passthrough when clear; avoidance vx/vy
                                     substituted when blocked. yaw and vz are
                                     always taken from CommNode's command.

    Algorithm
    ---------
    Each timer tick:
    1. Derive intended heading from CommNode's vx/vy.
       Project a point at lookahead_dist in that direction -> target_col in grid.
    2. Build a per-column clearance profile: forward distance (m) to nearest
       occupied cell per column (inf = fully clear).
    3. Inflate clearance inward by safety_radius - rejects corridors too narrow
       for the drone body.
    4. If the corridor around target_col is clear: passthrough CommNode's
       PositionTarget unchanged (preserves yaw, vz, type_mask).
    5. Otherwise find contiguous passable gaps (width >= min_gap_width), pick the
       one closest to the intended direction, override vx/vy only.
    6. No gap -> zero vx/vy, hold yaw/vz from CommNode (drone stops laterally).
    """

    def __init__(self):
        super().__init__('local_planner_node')

        self.declare_parameter('lookahead_dist', 2.0)   # m - how far ahead to aim
        self.declare_parameter('safety_radius', 0.5)    # m - body clearance required
        self.declare_parameter('min_gap_width', 1.0)    # m - narrower gaps ignored
        self.declare_parameter('obstacle_thresh', 50)   # occupancy value = occupied
        self.declare_parameter('planner_rate', 20.0)    # Hz - match CommNode timer
        self.declare_parameter('max_speed', 0.25)       # m/s cap (match CommNode MAX_SPEED)
        self.declare_parameter('debug_logging', True)
        self.declare_parameter('debug_log_period_s', 1.0)

        self.lookahead = self.get_parameter('lookahead_dist').value
        self.safety_r = self.get_parameter('safety_radius').value
        self.min_gap_w = self.get_parameter('min_gap_width').value
        self.occ_thresh = self.get_parameter('obstacle_thresh').value
        self.max_speed = self.get_parameter('max_speed').value
        rate = self.get_parameter('planner_rate').value
        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        self.debug_log_period_s = float(self.get_parameter('debug_log_period_s').value)

        self.grid_msg = None
        self.current_pose = None
        self.desired_cmd = None   # Latest PositionTarget from CommNode.
        self.last_passthrough_reason = None
        self.last_debug_log_time = None
        self.last_plan_mode = None

        self.occ_sub = self.create_subscription(
            OccupancyGrid, 'occupancy_node/occupancy_grid', self._occ_cb, OCCUPANCY_QOS)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self._pose_cb, qos_profile_sensor_data)
        # Subscribing here triggers CommNode's routing logic:
        # _publish_velocity_command checks get_subscription_count() on this topic.
        self.cmd_sub = self.create_subscription(
            PositionTarget, 'rob498_drone_8/cmd_vel', self._cmd_cb, 10)

        self.setpoint_pub = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', 10)

        self.timer = self.create_timer(1.0 / rate, self._plan)
        self.get_logger().info('LocalPlannerNode ready.')

    # ------------------------------------------------------------------ callbacks
    def _occ_cb(self, msg: OccupancyGrid):
        self.grid_msg = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _cmd_cb(self, msg: PositionTarget):
        self.desired_cmd = msg
        # If planner inputs are not ready yet, keep MAVROS fed with the raw command.
        if self.grid_msg is None or self.current_pose is None:
            self._publish_passthrough(msg)

    # ------------------------------------------------------------------ main loop
    def _plan(self):
        if self.desired_cmd is None:
            return

        if self.grid_msg is None:
            self._log_plan('missing_inputs', self.desired_cmd, self.desired_cmd, reason='occupancy_grid_missing')
            self._publish_passthrough(self.desired_cmd)
            return

        if self.current_pose is None:
            self._log_plan('missing_inputs', self.desired_cmd, self.desired_cmd, reason='vehicle_pose_missing')
            self._publish_passthrough(self.desired_cmd)
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
                'passthrough', self.desired_cmd, self.desired_cmd,
                closest_dist_m=closest_dist_m,
                closest_offset_m=closest_offset_m,
                closest_side=closest_side,
                direct_clear=direct_clear,
                best_col=None,
            )
            self.setpoint_pub.publish(self.desired_cmd)
            return

        # --- 5. Gap finding -----------------------------------------------
        passable = inflated >= self.lookahead
        best_col = _best_gap_center(passable, W, target_col, min_gap_cells)

        cmd_speed = math.hypot(self.desired_cmd.velocity.x, self.desired_cmd.velocity.y)
        avoid_speed = min(cmd_speed or self.max_speed, self.max_speed)

        if best_col is not None:
            right_m = (best_col - W / 2.0) * res
            out = self._override_velocity(self.lookahead, right_m, avoid_speed)
            mode = 'avoid'
        else:
            # --- 6. No gap - stop laterally, hold yaw/vz ------------------
            self.get_logger().warn('No passable gap found - stopping.')
            out = self._override_velocity(0.0, 0.0, 0.0)
            mode = 'stop'

        self._log_plan(
            mode, self.desired_cmd, out,
            closest_dist_m=closest_dist_m,
            closest_offset_m=closest_offset_m,
            closest_side=closest_side,
            direct_clear=direct_clear,
            best_col=best_col,
        )
        self.setpoint_pub.publish(out)

    def _publish_passthrough(self, cmd: PositionTarget):
        # Keep the original command intact when the planner cannot safely modify it yet.
        passthrough = PositionTarget()
        passthrough.header.stamp = self.get_clock().now().to_msg()
        passthrough.coordinate_frame = cmd.coordinate_frame
        passthrough.type_mask = cmd.type_mask
        passthrough.position = cmd.position
        passthrough.velocity = cmd.velocity
        passthrough.acceleration_or_force = cmd.acceleration_or_force
        passthrough.yaw = cmd.yaw
        passthrough.yaw_rate = cmd.yaw_rate
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

    def _log_plan(self, mode, input_cmd, output_cmd, closest_dist_m=float('inf'),
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
            f'in={self._format_cmd(input_cmd)} '
            f'out={self._format_cmd(output_cmd)} '
            f'closest={closest_side}@{closest_dist_text}m '
            f'offset={closest_offset_m:.2f} '
            f'direct_clear={direct_clear_text} '
            f'best_col={best_col_text} '
            f'reason={reason or "n/a"}'
        )

    # ------------------------------------------------------------------ helpers
    def _format_cmd(self, cmd: PositionTarget):
        return (
            f'vel=({cmd.velocity.x:.2f},{cmd.velocity.y:.2f},{cmd.velocity.z:.2f}) '
            f'yaw={cmd.yaw:.2f}'
        )

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
        Derive target grid column + forward component from CommNode's velocity.
        Projects a point at lookahead_dist in the commanded direction.
        Returns (col: float, fwd_body: float).
        """
        vx = self.desired_cmd.velocity.x
        vy = self.desired_cmd.velocity.y
        speed = math.hypot(vx, vy)

        if speed < 1e-3:
            return W / 2.0, 0.0   # No horizontal motion -> check straight ahead.

        yaw = _yaw_from_quat(self.current_pose.pose.orientation)

        # Rotate map-frame velocity into drone body frame.
        fwd = vx * math.cos(yaw) + vy * math.sin(yaw)
        right = vx * math.sin(yaw) - vy * math.cos(yaw)

        col = W / 2.0 + (right / speed) * (self.lookahead / res)
        return col, fwd

    def _override_velocity(self, fwd: float, right: float, speed: float) -> PositionTarget:
        """
        Copy CommNode's PositionTarget, overriding only vx/vy.
        yaw, vz, type_mask, and coordinate_frame are preserved exactly.
        """
        yaw = _yaw_from_quat(self.current_pose.pose.orientation)

        # Body (fwd, right) -> map frame.
        dx = fwd * math.cos(yaw) + right * math.sin(yaw)
        dy = fwd * math.sin(yaw) - right * math.cos(yaw)
        dist = math.hypot(dx, dy)

        if dist > 1e-3:
            vx = dx / dist * speed
            vy = dy / dist * speed
        else:
            vx = vy = 0.0

        out = PositionTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.coordinate_frame = self.desired_cmd.coordinate_frame
        out.type_mask = self.desired_cmd.type_mask
        out.velocity.x = vx
        out.velocity.y = vy
        out.velocity.z = self.desired_cmd.velocity.z   # Preserve altitude control.
        out.yaw = self.desired_cmd.yaw                 # Preserve rate-limited yaw.
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
