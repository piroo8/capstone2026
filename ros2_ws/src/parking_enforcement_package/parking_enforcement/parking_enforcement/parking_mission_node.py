#!/usr/bin/env python3
"""First-pass parking mission node.

This node keeps the same service interface as the lab mission code, but it now
loads hardcoded parking waypoints from the shared config, uses the sector-based
avoidance result from `/avoid_dir`, scans for plates at each waypoint, and logs
confirmed plates to disk for the later revisit pass.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from parking_enforcement.config import (
    AVOID_FORWARD_STEP,
    AVOID_STEP,
    AVOID_TIMEOUT_SEC,
    AVOID_TOPIC,
    BACK_UP_STEP,
    DRONE_NS,
    FIRST_PASS_LOG_PATH,
    FRAME_ID,
    LAUNCH_ALT,
    LOCAL_POSE_TOPIC,
    MAVROS_ARM_TOPIC,
    MAVROS_SET_MODE_TOPIC,
    MAVROS_STATE_TOPIC,
    MISSION_RATE_HZ,
    PARKING_WAYPOINTS,
    PLATE_CONFIRMED_TOPIC,
    PLATE_ENABLED_TOPIC,
    POSITION_HOLD_RADIUS,
    SCAN_AMPLITUDE,
    SCAN_PERIOD_SEC,
    SCAN_TIMEOUT_SEC,
    SCAN_YAW_RIGHT_DEG,
    SETPOINT_TOPIC,
    TARGET_RADIUS,
    WAYPOINT_SETTLE_TIME,
    Z_OFFSET,
)
from parking_enforcement.mission_utils import (
    TemporaryTarget,
    copy_pose,
    copy_quaternion,
    quaternion_from_yaw,
    serialize_waypoints,
    write_json,
    wrap_angle,
    yaw_from_quaternion,
)

STATE_IDLE = 'IDLE'
STATE_TAKEOFF = 'TAKEOFF'
STATE_WAIT_FOR_TEST = 'WAIT_FOR_TEST'
STATE_NAVIGATE = 'NAVIGATE'
STATE_AVOID = 'AVOID'
STATE_SCAN_YAW = 'SCAN_YAW'
STATE_SCAN_VERTICAL = 'SCAN_VERTICAL'
STATE_LANDING = 'LANDING'


class ParkingMissionNode(Node):
    """Fly the first parking-enforcement pass and save confirmed plates to disk."""

    def __init__(self, node_name: str | None = None):
        super().__init__(node_name or f'{DRONE_NS}_mission')

        self.current_pos = PoseStamped()
        self.current_state = State()
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = FRAME_ID
        self.home_pose = None
        self.pose_received = False

        # Mission state and command flags follow the same pattern as the lab code:
        # service callbacks only set flags, while the timer loop does the work.
        self.state = STATE_IDLE
        self.launch_requested = False
        self.test_requested = False
        self.land_requested = False
        self.abort_requested = False
        self.auto_land_sent = False

        # The parking spots are defined once in config.py for this demo phase.
        self.waypoints = [tuple(map(float, waypoint)) for waypoint in PARKING_WAYPOINTS]
        self.current_wp_index = 0
        self.current_wp_arrival_time = None

        # Avoidance and scan state.
        self.avoid_dir = 'BLOCKED'
        self.avoid_target = None
        self.scan_center_pose = None
        self.scan_yaw = 0.0
        self.scan_start_time = None
        self.confirmed_plate = ''

        # First-pass log data is kept in memory and written to disk whenever it changes.
        self.plate_log = {}
        self.mission_start_time = None

        self.srv_launch = self.create_service(Trigger, f'{DRONE_NS}/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, f'{DRONE_NS}/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, f'{DRONE_NS}/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, f'{DRONE_NS}/comm/abort', self.callback_abort)

        self.arming_client = self.create_client(CommandBool, MAVROS_ARM_TOPIC)
        self.set_mode_client = self.create_client(SetMode, MAVROS_SET_MODE_TOPIC)

        self.create_subscription(State, MAVROS_STATE_TOPIC, self._state_callback, 10)
        self.create_subscription(PoseStamped, LOCAL_POSE_TOPIC, self._pos_callback, qos_profile_sensor_data)
        self.create_subscription(String, AVOID_TOPIC, self._avoid_callback, 10)
        self.create_subscription(String, PLATE_CONFIRMED_TOPIC, self._plate_confirmed_callback, 10)

        self.local_pos_pub = self.create_publisher(PoseStamped, SETPOINT_TOPIC, 10)
        self.plate_enable_pub = self.create_publisher(Bool, PLATE_ENABLED_TOPIC, 10)

        self.timer = self.create_timer(1.0 / MISSION_RATE_HZ, self._timer_callback)
        self.last_target_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        self.last_pos_log_time = self.get_clock().now()
        self.last_nav_log_time = self.get_clock().now()

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self._log_loaded_waypoints()
        self.get_logger().info('First-pass parking mission node ready')

    def callback_launch(self, request, response):
        """Arm and climb to the configured launch altitude."""

        self.launch_requested = True
        response.success = True
        response.message = 'Launch flag set'
        return response

    def callback_test(self, request, response):
        """Start the waypoint mission after takeoff is complete."""

        self.test_requested = True
        response.success = len(self.waypoints) > 0
        response.message = 'Mission start flag set' if self.waypoints else 'No waypoints configured'
        return response

    def callback_land(self, request, response):
        """Normal operator-requested land uses PX4 AUTO.LAND."""

        self.land_requested = True
        response.success = True
        response.message = 'Autonomous landing flag set'
        return response

    def callback_abort(self, request, response):
        """Abort also uses AUTO.LAND instead of an in-air kill for this demo."""

        self.abort_requested = True
        response.success = True
        response.message = 'Abort flag set'
        return response

    def _state_callback(self, msg: State):
        """Track the current FCU state for operator visibility."""

        self.current_state = msg
        now = self.get_clock().now()
        if (now - self.last_state_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f'[STATE] connected={msg.connected} armed={msg.armed} mode={msg.mode} mission={self.state}'
            )
            self.last_state_log_time = now

    def _pos_callback(self, msg: PoseStamped):
        """Initialize home pose from the first valid local-position sample."""

        self.current_pos = msg
        self.current_pos.pose.position.z += Z_OFFSET
        self.pose_received = True

        if self.home_pose is None:
            self.home_pose = PoseStamped()
            self.home_pose.header = msg.header
            self.home_pose.pose = copy_pose(msg.pose)
            self.target_pose.pose = copy_pose(msg.pose)
            self.get_logger().info(
                '[HOME] Initialized at '
                f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
            )

        now = self.get_clock().now()
        if (now - self.last_pos_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f'[POSE] x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} z={msg.pose.position.z:.2f}'
            )
            self.last_pos_log_time = now

    def _avoid_callback(self, msg: String):
        """Receive the discrete left/center/right avoidance result from the costmap."""

        self.avoid_dir = msg.data

    def _plate_confirmed_callback(self, msg: String):
        """Store the latest confirmed plate from the TensorRT plate reader."""

        self.confirmed_plate = msg.data

    def _timer_callback(self):
        """Run the mission state machine and continuously stream MAVROS setpoints."""

        if not self.pose_received:
            return

        self._handle_command_flags()

        if self.state == STATE_TAKEOFF:
            self._update_takeoff()
        elif self.state == STATE_WAIT_FOR_TEST:
            self._update_wait_for_test()
        elif self.state == STATE_NAVIGATE:
            self._update_navigation()
        elif self.state == STATE_AVOID:
            self._update_avoidance()
        elif self.state == STATE_SCAN_YAW:
            self._update_scan_yaw()
        elif self.state == STATE_SCAN_VERTICAL:
            self._update_scan_vertical()
        elif self.state == STATE_LANDING:
            pass
        elif self.home_pose is not None:
            self.target_pose.pose = copy_pose(self.home_pose.pose)

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = FRAME_ID
        self.local_pos_pub.publish(self.target_pose)
        self._publish_plate_enable(self.state in (STATE_SCAN_YAW, STATE_SCAN_VERTICAL))

        now = self.get_clock().now()
        if (now - self.last_target_log_time).nanoseconds > 1e9:
            position = self.target_pose.pose.position
            self.get_logger().info(
                f'[TARGET] x={position.x:.2f} y={position.y:.2f} z={position.z:.2f} state={self.state}'
            )
            self.last_target_log_time = now

    def _handle_command_flags(self):
        """Translate service flags into one-time mission state changes."""

        if self.abort_requested:
            self.abort_requested = False
            self.get_logger().warn('[ABORT] Switching to AUTO.LAND')
            self._command_auto_land()
            return

        if self.land_requested:
            self.land_requested = False
            self.get_logger().warn('[LAND] Switching to AUTO.LAND')
            self._command_auto_land()
            return

        if self.launch_requested:
            self.launch_requested = False
            self._start_takeoff()

        if self.test_requested and self.state == STATE_WAIT_FOR_TEST and self.waypoints:
            self.test_requested = False
            self.current_wp_index = 0
            self.current_wp_arrival_time = None
            self.confirmed_plate = ''
            self.mission_start_time = self._now_sec()
            self.state = STATE_NAVIGATE
            self.get_logger().info('[MISSION] Test started, entering NAVIGATE state')

    def _start_takeoff(self):
        """Enter OFFBOARD, arm, and climb straight up from home."""

        if self.home_pose is None:
            self.get_logger().warn('[LAUNCH] No pose yet, cannot launch')
            return

        self.target_pose.pose = copy_pose(self.current_pos.pose)
        self.target_pose.pose.position.z = self.home_pose.pose.position.z + LAUNCH_ALT
        self.state = STATE_TAKEOFF
        self.auto_land_sent = False
        self._set_mode('OFFBOARD')
        self._arming(True)
        self.get_logger().info(f'[LAUNCH] Takeoff target z={self.target_pose.pose.position.z:.2f}')

    def _update_takeoff(self):
        """Hold XY at home while climbing to the launch altitude."""

        target_z = self.home_pose.pose.position.z + LAUNCH_ALT
        self.target_pose.pose.position.x = self.home_pose.pose.position.x
        self.target_pose.pose.position.y = self.home_pose.pose.position.y
        self.target_pose.pose.position.z = target_z
        self.target_pose.pose.orientation = copy_quaternion(self.home_pose.pose.orientation)

        if abs(self.current_pos.pose.position.z - target_z) < 0.12:
            self.state = STATE_WAIT_FOR_TEST
            self.get_logger().info('[LAUNCH] Takeoff complete, waiting for TEST command')

    def _update_wait_for_test(self):
        """Hold a steady hover above home until the mission starts."""

        self.target_pose.pose.position.x = self.home_pose.pose.position.x
        self.target_pose.pose.position.y = self.home_pose.pose.position.y
        self.target_pose.pose.position.z = self.home_pose.pose.position.z + LAUNCH_ALT
        self.target_pose.pose.orientation = copy_quaternion(self.home_pose.pose.orientation)

    def _update_navigation(self):
        """Fly toward the active waypoint unless the local costmap requests avoidance."""

        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('[MISSION] All waypoints done, landing in place')
            self._command_auto_land()
            return

        waypoint = self.waypoints[self.current_wp_index]
        yaw = yaw_from_quaternion(self.current_pos.pose.orientation)

        self.target_pose.pose.position.x = waypoint[0]
        self.target_pose.pose.position.y = waypoint[1]
        self.target_pose.pose.position.z = waypoint[2]
        self.target_pose.pose.orientation = quaternion_from_yaw(yaw)

        dist = self._distance_to_target(*waypoint)
        now = self.get_clock().now()
        if (now - self.last_nav_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f'[NAV] waypoint={self.current_wp_index + 1} dist={dist:.2f} avoid={self.avoid_dir}'
            )
            self.last_nav_log_time = now

        if self.avoid_dir in ('LEFT', 'RIGHT', 'BLOCKED') and dist > POSITION_HOLD_RADIUS:
            self._start_avoidance(self.avoid_dir)
            return

        if dist < TARGET_RADIUS:
            if self.current_wp_arrival_time is None:
                self.current_wp_arrival_time = self._now_sec()
                self.get_logger().info(f'[WAYPOINT] Reached waypoint {self.current_wp_index + 1}, settling')
            elif self._now_sec() - self.current_wp_arrival_time > WAYPOINT_SETTLE_TIME:
                self._start_scan()
        else:
            self.current_wp_arrival_time = None

    def _start_avoidance(self, direction: str):
        """Generate a temporary sidestep or back-up target from the sector decision."""

        current = self.current_pos.pose.position
        yaw = yaw_from_quaternion(self.current_pos.pose.orientation)

        if direction == 'LEFT':
            forward = AVOID_FORWARD_STEP
            lateral = AVOID_STEP
        elif direction == 'RIGHT':
            forward = AVOID_FORWARD_STEP
            lateral = -AVOID_STEP
        else:
            # BLOCKED means no safe left/right sector is known, so back up and
            # let the costmap re-evaluate before committing laterally.
            forward = -BACK_UP_STEP
            lateral = 0.0

        x = current.x + forward * math.cos(yaw) - lateral * math.sin(yaw)
        y = current.y + forward * math.sin(yaw) + lateral * math.cos(yaw)
        z = current.z

        self.avoid_target = TemporaryTarget(x=x, y=y, z=z, yaw=yaw, created_time=self._now_sec())
        self.state = STATE_AVOID
        self.get_logger().warn(
            f'[AVOID] direction={direction} temporary target=({x:.2f}, {y:.2f}, {z:.2f})'
        )

    def _update_avoidance(self):
        """Fly to the temporary avoid target, then resume normal navigation."""

        if self.avoid_target is None:
            self.state = STATE_NAVIGATE
            return

        self.target_pose.pose.position.x = self.avoid_target.x
        self.target_pose.pose.position.y = self.avoid_target.y
        self.target_pose.pose.position.z = self.avoid_target.z
        self.target_pose.pose.orientation = quaternion_from_yaw(self.avoid_target.yaw)

        dist = self._distance_to_target(self.avoid_target.x, self.avoid_target.y, self.avoid_target.z)
        timed_out = (self._now_sec() - self.avoid_target.created_time) > AVOID_TIMEOUT_SEC
        if self.avoid_dir == 'CLEAR' or dist < TARGET_RADIUS or timed_out:
            self.state = STATE_NAVIGATE
            self.avoid_target = None
            self.current_wp_arrival_time = None
            self.get_logger().info('[AVOID] Returning to waypoint navigation')

    def _start_scan(self):
        """Freeze over the parking spot, yaw right, and enable the plate reader."""

        self.scan_center_pose = PoseStamped()
        self.scan_center_pose.header = self.current_pos.header
        self.scan_center_pose.pose = copy_pose(self.current_pos.pose)
        current_yaw = yaw_from_quaternion(self.current_pos.pose.orientation)
        self.scan_yaw = wrap_angle(current_yaw - math.radians(SCAN_YAW_RIGHT_DEG))
        self.scan_start_time = None
        self.confirmed_plate = ''
        self.state = STATE_SCAN_YAW
        self.get_logger().info(f'[SCAN] Turning right at waypoint {self.current_wp_index + 1}')

    def _update_scan_yaw(self):
        """Rotate in place before starting the small vertical scan motion."""

        if self.scan_center_pose is None:
            self.state = STATE_NAVIGATE
            return

        position = self.scan_center_pose.pose.position
        self.target_pose.pose.position.x = position.x
        self.target_pose.pose.position.y = position.y
        self.target_pose.pose.position.z = position.z
        self.target_pose.pose.orientation = quaternion_from_yaw(self.scan_yaw)

        yaw_now = yaw_from_quaternion(self.current_pos.pose.orientation)
        if abs(wrap_angle(yaw_now - self.scan_yaw)) < 0.15:
            self.scan_start_time = self._now_sec()
            self.state = STATE_SCAN_VERTICAL
            self.get_logger().info('[SCAN] Yaw reached, scanning vertically')

    def _update_scan_vertical(self):
        """Oscillate in Z until a plate is confirmed or the scan times out."""

        if self.scan_center_pose is None or self.scan_start_time is None:
            self.state = STATE_NAVIGATE
            return

        elapsed = self._now_sec() - self.scan_start_time
        position = self.scan_center_pose.pose.position
        z_cmd = position.z + SCAN_AMPLITUDE * math.sin(2.0 * math.pi * elapsed / SCAN_PERIOD_SEC)

        self.target_pose.pose.position.x = position.x
        self.target_pose.pose.position.y = position.y
        self.target_pose.pose.position.z = z_cmd
        self.target_pose.pose.orientation = quaternion_from_yaw(self.scan_yaw)

        if self.confirmed_plate:
            self._record_plate_for_current_waypoint(self.confirmed_plate)
            self._advance_waypoint()
            return

        if elapsed > SCAN_TIMEOUT_SEC:
            self.get_logger().warn(f'[SCAN] Timeout at waypoint {self.current_wp_index + 1}, moving on')
            self._advance_waypoint()

    def _record_plate_for_current_waypoint(self, plate: str):
        """Store the confirmed plate and write the first-pass log file to disk."""

        waypoint = self.waypoints[self.current_wp_index]
        self.plate_log[str(self.current_wp_index + 1)] = {
            'waypoint_index': self.current_wp_index + 1,
            'position': [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])],
            'plate': plate,
            'first_seen_time_sec': round(self._now_sec(), 2),
        }
        self.get_logger().info(f'[PLATE] waypoint={self.current_wp_index + 1} plate={plate}')
        self._write_first_pass_log()

    def _advance_waypoint(self):
        """Leave the current parking spot and continue to the next mission target."""

        self.confirmed_plate = ''
        self.scan_center_pose = None
        self.scan_start_time = None
        self.current_wp_arrival_time = None
        self.current_wp_index += 1
        self.state = STATE_NAVIGATE
        if self.current_wp_index < len(self.waypoints):
            self.get_logger().info(f'[MISSION] Moving to waypoint {self.current_wp_index + 1}')
        else:
            self.get_logger().info('[MISSION] Final waypoint complete')
            self._write_first_pass_log()

    def _command_auto_land(self):
        """Switch PX4 into AUTO.LAND once and keep the node in LANDING state."""

        if self.auto_land_sent:
            self.state = STATE_LANDING
            return

        self.auto_land_sent = True
        self.state = STATE_LANDING
        self._write_first_pass_log()
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self._auto_land_callback)

    def _auto_land_callback(self, future):
        """Report whether PX4 accepted AUTO.LAND."""

        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().warn('[LAND] AUTO.LAND sent to PX4')
            else:
                self.get_logger().error('[LAND] PX4 rejected AUTO.LAND')
        except Exception as exc:
            self.get_logger().error(f'[LAND] AUTO.LAND service call failed: {exc}')

    def _arming(self, value: bool):
        """Send the MAVROS arm or disarm command."""

        req = CommandBool.Request()
        req.value = value
        future = self.arming_client.call_async(req)
        future.add_done_callback(lambda f: self._arming_callback(f, value))

    def _arming_callback(self, future, value: bool):
        """Log the result of the arm/disarm request."""

        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'[ARM] set to {value}')
            else:
                self.get_logger().warn(f'[ARM] command failed for {value}')
        except Exception as exc:
            self.get_logger().error(f'[ARM] service call failed: {exc}')

    def _set_mode(self, mode_string: str):
        """Send a PX4 mode change request through MAVROS."""

        req = SetMode.Request()
        req.custom_mode = mode_string
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._set_mode_callback(f, mode_string))

    def _set_mode_callback(self, future, mode_string: str):
        """Log the result of the mode change request."""

        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info(f'[MODE] {mode_string}')
            else:
                self.get_logger().warn(f'[MODE] failed to enter {mode_string}')
        except Exception as exc:
            self.get_logger().error(f'[MODE] service call failed: {exc}')

    def _publish_plate_enable(self, enabled: bool):
        """Only run TensorRT plate inference during the scan states."""

        msg = Bool()
        msg.data = enabled
        self.plate_enable_pub.publish(msg)

    def _distance_to_target(self, x: float, y: float, z: float) -> float:
        """Euclidean distance from the current local pose to a target setpoint."""

        dx = float(x) - self.current_pos.pose.position.x
        dy = float(y) - self.current_pos.pose.position.y
        dz = float(z) - self.current_pos.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _now_sec(self) -> float:
        """Current ROS clock time in seconds."""

        return self.get_clock().now().nanoseconds * 1e-9

    def _log_loaded_waypoints(self):
        """Print the hardcoded parking waypoint list at startup for quick review."""

        if not self.waypoints:
            self.get_logger().warn('No parking waypoints are configured in config.py')
            return

        for index, waypoint in enumerate(self.waypoints, start=1):
            self.get_logger().info(
                f'[WAYPOINT LOAD] {index}: x={waypoint[0]:.2f} y={waypoint[1]:.2f} z={waypoint[2]:.2f}'
            )

    def _write_first_pass_log(self):
        """Persist the first-pass mission results for the separate revisit node."""

        payload = {
            'drone_ns': DRONE_NS,
            'mission_start_time_sec': None if self.mission_start_time is None else round(self.mission_start_time, 2),
            'waypoints': serialize_waypoints(self.waypoints),
            'entries': [self.plate_log[key] for key in sorted(self.plate_log, key=lambda item: int(item))],
        }
        write_json(FIRST_PASS_LOG_PATH, payload)


def main(args=None):
    """Spin the first-pass mission node until shutdown."""

    rclpy.init(args=args)
    node = ParkingMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
