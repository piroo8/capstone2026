import math
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from stereo_msgs.msg import DisparityImage
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, String
import numpy as np

MODES = ['OFFBOARD', 'ALTCTL', 'STABILIZED']

Z_OFFSET = -0.125
RADIUS = 0.2

YAW_RADIUS = 0.5

# Scanning parameters
TRANSIT_ALT = 0.5
SCAN_ALT = 0.8
ASCEND_DESCEND_TOL = 0.15
SCAN_TIMEOUT_SEC = 15.0
FRESHNESS_TIMEOUT_SEC = 1.0
WAIT_LOG_PERIOD_SEC = 1.0

# FSM States
STATE_IDLE = 'IDLE'
STATE_TRANSIT = 'TRANSIT'
STATE_DESCEND = 'DESCEND'
STATE_SCAN = 'SCAN'
STATE_ASCEND = 'ASCEND'


class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_8/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_8/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_8/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_8/comm/abort', self.callback_abort)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.command_client = self.create_client(CommandLong, 'mavros/cmd/command')

        # Subs and Pubs
        self.state_sub = self.create_subscription(State, 'mavros/state', self._state_callback, 10)
        self.pos_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self._pos_callback, qos_profile_sensor_data)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.waypoint_sub = self.create_subscription(PoseArray,'rob498_drone_8/comm/waypoints',self._waypoint_callback, 10)
        # Routed to local_planner_node when it is running; planner then publishes to MAVROS.
        self.cmd_pose_pub = self.create_publisher(PoseStamped, 'rob498_drone_8/cmd_pose', 10)

        # Plate reader gating
        self.plate_confirmed_sub = self.create_subscription(String, '/plate_reader/confirmed', self._plate_confirmed_callback, 10,)
        self.plate_ready_sub = self.create_subscription(Bool, '/plate_reader/ready', self._plate_ready_callback, 10)
        self.plate_enable_pub = self.create_publisher(Bool, '/plate_reader/enabled', 10)

        # Compute gating for planner stack
        self.planner_enable_pub = self.create_publisher(Bool, '/local_planner/enabled', 10)
        self.occupancy_enable_pub = self.create_publisher(Bool, '/occupancy_node/enabled', 10)
        self.depth_enable_pub = self.create_publisher(Bool, '/perception/depth_enabled', 10)

        # Freshness monitors used as safety interlock before resuming transit
        self.depth_fresh_sub = self.create_subscription(DisparityImage, 'disparity', self._depth_fresh_callback, qos_profile_sensor_data)
        self.occupancy_fresh_sub = self.create_subscription(OccupancyGrid, 'occupancy_node/occupancy_grid', self._occupancy_fresh_callback, qos_profile_sensor_data)
        self.planner_fresh_sub = self.create_subscription(PoseStamped, 'mavros/setpoint_position/local', self._planner_fresh_callback, 10)

        # Timer
        self.timer = self.create_timer(0.05, self._timer_callback)
        self.last_target_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        self.last_pos_log_time = self.get_clock().now()
        self.last_nav_log_time = self.get_clock().now()
        self.last_scan_wait_log_time = self.get_clock().now()
        self.last_stack_wait_log_time = self.get_clock().now()

        # Poses
        self.current_pos = PoseStamped()
        self.target_pose = PoseStamped()
        self.current_state = State()
        self.home_pose = None ## Needed like this for launch to store correctly

        # Waypoints
        self.waypoints = []
        self.waypoints_received = False
        self.current_wp_index = 0
        self.target_radius = RADIUS
        self.test_active = False
        self.return_home_active = False
        self.prev_waypoint = None

        # FSM State
        self.state = STATE_IDLE

        # License plate scanning
        self.confirmed_plate = None
        
        # Scan state tracking
        self.scan_start_time = None
        self.scan_wp_index = None
        self.scan_hold_orientation = None
        self.plate_reader_ready = False
        self.plate_reader_enabled = False
        self.nav_stack_enabled = False
        self.waiting_for_stack_ready = False
        self.last_depth_msg_time = None
        self.last_occupancy_msg_time = None
        self.last_planner_msg_time = None

        # Live mission logging (JSON)
        self.mission_log_path = None
        self.mission_log_entries = []
        self.logged_plate_events = set()
        self.mission_log_fail_count = 0
        self._MISSION_LOG_MAX_FAILS = 3

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.command_client.wait_for_service()
        self.get_logger().info('Initialization complete!')

        # Keep heavy stack idle until mission test starts.
        self._set_plate_reader_enabled(False)
        self._set_nav_stack_enabled(False)


    def callback_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Takeoff in place to TRANSIT_ALT"""
        if self.home_pose is None:
            self.home_pose = PoseStamped()
            self.home_pose.pose = self.current_pos.pose
            self.get_logger().info(f"[HOME SET] X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")

        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + TRANSIT_ALT
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        self.get_logger().info(f"[LAUNCH]: Takeoff to target | X: {self.target_pose.pose.position.x:.3f} | Y: {self.target_pose.pose.position.y:.3f} | Z: {self.target_pose.pose.position.z:.3f}")

        self.set_mode('OFFBOARD')

        self.arming(True)
        
        self.get_logger().info("[LAUNCH]: Takeoff initiated.")

        response.success = True
        response.message = f"Launch initiated. Drone is taking off to {self.target_pose.pose.position.z} m."
        return response

    def callback_test(self, request, response):

        if not self.waypoints_received:
            self.get_logger().warn("[TEST]: Cannot start test — no waypoints received")
            response.success = False
            response.message = "No waypoints received"
            return response

        self.prev_waypoint = np.array([self.current_pos.pose.position.x, self.current_pos.pose.position.y, self.current_pos.pose.position.z])
        self.current_wp_index = 0
        self.test_active = True
        self.state = STATE_TRANSIT
        self.confirmed_plate = None
        self.waiting_for_stack_ready = False
        self._set_plate_reader_enabled(False)
        self._set_nav_stack_enabled(True)
        self.logged_plate_events.clear()
        self._start_mission_log()

        self.get_logger().info("[TEST]: Starting waypoint navigation")

        response.success = True
        response.message = "Waypoint test started"
        return response

    def callback_land(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """RTL and land"""
        # Set target to home pose exactly
        """Return toward home XY/heading, then auto-land."""

        if self.home_pose is None:
            response.success = False
            response.message = "No home pose available yet."
            return response
        
        if not self.return_home_active:
            response.success = False
            response.message = "Return home not active"
            return response

        # Step 1: Lock target XY and yaw to home, but keep current Z
        self.target_pose.pose.position.x = self.home_pose.pose.position.x
        self.target_pose.pose.position.y = self.home_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z

        self.target_pose.pose.orientation = self.home_pose.pose.orientation
        
        dx = self.home_pose.pose.position.x - self.current_pos.pose.position.x
        dy = self.home_pose.pose.position.y - self.current_pos.pose.position.y

        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.target_radius:

            self.get_logger().info("[MISSION]: Home reached. Initiating landing.")

            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.get_logger().info("Requesting AUTO.LAND...")
            self.set_mode_client.call_async(req)
            response.success = True
            response.message = "Sent AUTO.LAND command to MAVROS."

            self.return_home_active = False
            self.state = STATE_IDLE
            self._set_plate_reader_enabled(False)
            self._set_nav_stack_enabled(False)
            self._append_mission_log({
                'event': 'home_reached',
                'timestamp_s': round(self.get_clock().now().nanoseconds / 1e9, 3),
                'state': self.state,
            })

            return response

        response.success = True
        response.message = 'Returning to home position.'
        return response

    def _start_mission_log(self):
        log_dir = Path(__file__).resolve().parent
        self.mission_log_path = log_dir / 'confirmed_plates.json'
        self.mission_log_entries = []
        self._append_mission_log({
            'event': 'mission_start',
            'timestamp_s': round(self.get_clock().now().nanoseconds / 1e9, 3),
            'state': self.state,
        })
        self.get_logger().info(f'[LOG]: Writing mission plate log to {self.mission_log_path}')

    def _append_mission_log(self, entry):
        if self.mission_log_path is None:
            return
        self.mission_log_entries.append(entry)
        payload = {
            'entries': self.mission_log_entries,
        }
        try:
            with self.mission_log_path.open('w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2)
            self.mission_log_fail_count = 0
        except OSError as exc:
            self.mission_log_fail_count += 1
            self.get_logger().error(
                f'[LOG]: Failed to write mission log ({self.mission_log_fail_count}/'
                f'{self._MISSION_LOG_MAX_FAILS}): {exc}'
            )
            if self.mission_log_fail_count >= self._MISSION_LOG_MAX_FAILS:
                self.get_logger().error('[LOG]: Disabling mission log after repeated failures.')
                self.mission_log_path = None

    def _set_plate_reader_enabled(self, enabled: bool):
        if self.plate_reader_enabled == enabled:
            return
        self.plate_reader_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.plate_enable_pub.publish(msg)

    def _set_nav_stack_enabled(self, enabled: bool, reset_freshness: bool = False):
        if self.nav_stack_enabled != enabled:
            msg = Bool()
            msg.data = enabled
            self.planner_enable_pub.publish(msg)
            self.occupancy_enable_pub.publish(msg)
            self.depth_enable_pub.publish(msg)
            self.nav_stack_enabled = enabled

        if enabled and reset_freshness:
            self.last_depth_msg_time = None
            self.last_occupancy_msg_time = None
            self.last_planner_msg_time = None

    def _publish_control_heartbeat(self):
        """Republish current control states so late subscribers converge quickly."""
        plate_msg = Bool()
        plate_msg.data = self.plate_reader_enabled
        self.plate_enable_pub.publish(plate_msg)

        nav_msg = Bool()
        nav_msg.data = self.nav_stack_enabled
        self.planner_enable_pub.publish(nav_msg)
        self.occupancy_enable_pub.publish(nav_msg)
        self.depth_enable_pub.publish(nav_msg)

    def _plate_ready_callback(self, msg: Bool):
        self.plate_reader_ready = msg.data

    def _depth_fresh_callback(self, msg: DisparityImage):
        self.last_depth_msg_time = self.get_clock().now()

    def _occupancy_fresh_callback(self, msg: OccupancyGrid):
        self.last_occupancy_msg_time = self.get_clock().now()

    def _planner_fresh_callback(self, msg: PoseStamped):
        if self.nav_stack_enabled:
            self.last_planner_msg_time = self.get_clock().now()

    def _is_fresh(self, timestamp, now):
        if timestamp is None:
            return False
        return (now - timestamp).nanoseconds / 1e9 <= FRESHNESS_TIMEOUT_SEC

    def _planner_stack_ready(self, now):
        return (
            self._is_fresh(self.last_depth_msg_time, now)
            and self._is_fresh(self.last_occupancy_msg_time, now)
            and self._is_fresh(self.last_planner_msg_time, now)
        )


    def callback_abort(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Land using PX4 land mode"""
        self._set_plate_reader_enabled(False)
        self._set_nav_stack_enabled(False)
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.get_logger().info("Requesting AUTO.LAND...")
        self.set_mode_client.call_async(req)
        response.success = True
        response.message = "Sent AUTO.LAND command to MAVROS."
        return response

    def update_waypoint_navigation(self, wp_index, target_altitude, hold_orientation=None):
        """Navigate to a waypoint at a requested altitude and return nav metrics."""
        wp = self.waypoints[wp_index]
        yaw = 0.0

        # Set target position
        self.target_pose.pose.position.x = wp[0]
        self.target_pose.pose.position.y = wp[1]
        self.target_pose.pose.position.z = target_altitude

        # Calculate distances
        dx = wp[0] - self.current_pos.pose.position.x
        dy = wp[1] - self.current_pos.pose.position.y
        dz = target_altitude - self.current_pos.pose.position.z

        xy_dist = np.sqrt(dx**2 + dy**2)
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        # Set orientation
        if hold_orientation is not None:
            self.target_pose.pose.orientation = hold_orientation
        else:
            # Calculate yaw toward waypoint
            yaw = math.atan2(wp[1] - self.prev_waypoint[1], wp[0] - self.prev_waypoint[0])

            if xy_dist > YAW_RADIUS:
                self.target_pose.pose.orientation.x = 0.0
                self.target_pose.pose.orientation.y = 0.0
                self.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
                self.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        return dist, xy_dist, dz, yaw

    def arming(self, val: bool):
        arm_req = CommandBool.Request()
        arm_req.value = val
        self.get_logger().info(f"[ARMING]: Request arming set to {val}...")
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(lambda f: self._arming_callback(f, val))


    def _arming_callback(self, future, val):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"[ARMING]: Set to {val}.")
            else:
                self.get_logger().error(f"[ARMING] Failed to set to {val}!")
        except Exception as e:
            self.get_logger().error(f"[ARMING] Failed to set to {val}!\n{e}")


    def set_mode(self, mode_string: str):
        """Set flight mode"""
        if mode_string not in MODES:
            self.get_logger().error(f"Invalid mode: {mode_string}")
            return

        req = SetMode.Request()
        req.custom_mode = mode_string

        self.get_logger().info(f"[MODE]: Changing mode to {mode_string}...")
        
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._set_mode_callback(f, mode_string))


    def _set_mode_callback(self, future, mode_string):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"[MODE]: {mode_string}")
            else:
                self.get_logger().warn(f"[MODE]: Failed to enter {mode_string} mode.")
        except Exception as e:
            self.get_logger().error(f"[MODE]: Failed to enter {mode_string} mode.\n{e}")

    
    def kill_switch(self, request, response):
        """Emergency Force Kill (Disarm in air)"""
        kill_req = CommandLong.Request()
        kill_req.command = 400      # MAV_CMD_COMPONENT_ARM_DISARM
        kill_req.param1 = 0.0       # 0 = Disarm
        kill_req.param2 = 21196.0   # Force Disarm (Kill)
        
        self.get_logger().warn("[KILL SWITCH]: !!! Requesting kill switch !!!")
        
        future = self.command_client.call_async(kill_req)
        future.add_done_callback(lambda f: self._killswitch_callback(f))

        response.success = True
        response.message = "Kill command sent to FCU."
        return response


    def _killswitch_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().fatal("[KILL SWITCH]: Command ACCEPTED. Drone disarmed.")
            else:
                self.get_logger().error("[KILL SWITCH]: Command REJECTED by FCU!")
        except Exception as e:
            self.get_logger().error(f"[KILL SWITCH]: Kill service call failed: {e}")


    def _state_callback(self, msg: State) -> None:
        self.current_state = msg

        now = self.get_clock().now()
        if (now - self.last_state_log_time).nanoseconds > 1e9:
            self.get_logger().info(f'[STATE]: Connected: {msg.connected} | Armed: {msg.armed} | Mode: {msg.mode}')
            self.last_state_log_time = now


    def _pos_callback(self, msg):
        """Get current position + orientation"""
        self.current_pos = msg
        self.current_pos.pose.position.z += Z_OFFSET # hard code height offset of vicon

        now = self.get_clock().now()
        if (now - self.last_pos_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[POSITION]: X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")
            self.last_pos_log_time = now


    def _timer_callback(self):
        """Heartbeat for the drone"""
        # WAYPOINT NAVIGATION
        if self.test_active:
            # FSM state dispatch
            if self.state == STATE_TRANSIT:
                self._update_transit()
            elif self.state == STATE_DESCEND:
                self._update_descend()
            elif self.state == STATE_SCAN:
                self._update_scan()
            elif self.state == STATE_ASCEND:
                self._update_ascend()

        # RETURN HOME
        elif self.return_home_active:   
            # create dummy service objects to call the function
            req = Trigger.Request()
            res = Trigger.Response()

            self.callback_land(req, res)

        # Keep enable-state topics fresh for nodes that (re)subscribe mid-mission.
        self._publish_control_heartbeat()

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        # Route through local_planner_node when planner enabled; otherwise go direct to MAVROS.
        if self.nav_stack_enabled and self.cmd_pose_pub.get_subscription_count() > 0:
            self.cmd_pose_pub.publish(self.target_pose)
        else:
            self.local_pos_pub.publish(self.target_pose)
        
        now = self.get_clock().now()
        if (now - self.last_target_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[TARGET]: X: {self.target_pose.pose.position.x:.3f} | Y: {self.target_pose.pose.position.y:.3f} | Z: {self.target_pose.pose.position.z:.3f}")
            self.last_target_log_time = now
    
    def _waypoint_callback(self, msg):
        if self.waypoints_received:
            return

        self.get_logger().info("[WAYPOINTS]: Waypoints received from ground control")

        for i, pose in enumerate(msg.poses):
            wp = np.array([pose.position.x, pose.position.y, pose.position.z])

            self.waypoints.append(wp)

            self.get_logger().info(f"[WAYPOINT {i+1}]: "f"X {wp[0]:.3f} | Y {wp[1]:.3f} | Z {wp[2]:.3f}")

        self.waypoints_received = True

        self.get_logger().info(f"[WAYPOINTS]: Stored {len(self.waypoints)} waypoints successfully")

    def _plate_confirmed_callback(self, msg: String):
        """Receive confirmed license plate from plate_reader_node"""
        plate = msg.data.strip()
        if plate and self.state == STATE_SCAN:
            self.confirmed_plate = plate
            self.get_logger().info(f"[PLATE CONFIRMED]: {self.confirmed_plate}")
            wp_index = self.scan_wp_index + 1 if self.scan_wp_index is not None else self.current_wp_index + 1
            event_key = (wp_index, self.confirmed_plate)
            if event_key not in self.logged_plate_events:
                self.logged_plate_events.add(event_key)
                self._append_mission_log({
                    'event': 'plate_confirmed',
                    'timestamp_s': round(self.get_clock().now().nanoseconds / 1e9, 3),
                    'waypoint_index': wp_index,
                    'plate': self.confirmed_plate,
                    'state': self.state,
                })

    def _update_transit(self):
        """Navigate waypoints at TRANSIT_ALT with planner enabled"""
        self._set_plate_reader_enabled(False)
        self._set_nav_stack_enabled(True)

        # Waypoint have already been checked that they exist in Test service callback.
        if self.current_wp_index >= len(self.waypoints):
            # All waypoints done, return home
            self.test_active = False
            self.return_home_active = True
            self._set_plate_reader_enabled(False)
            self._append_mission_log({
                'event': 'all_waypoints_complete',
                'timestamp_s': round(self.get_clock().now().nanoseconds / 1e9, 3),
                'state': self.state,
            })
            self.get_logger().info("[MISSION]: All waypoints reached, returning to home")
            return

        # Navigate to waypoint at transit altitude
        dist, xy_dist, dz, yaw = self.update_waypoint_navigation(self.current_wp_index, TRANSIT_ALT)

        now = self.get_clock().now()
        if (now - self.last_nav_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[NAV]: WP {self.current_wp_index+1} | Dist {dist:.3f} m | Yaw {math.degrees(yaw):.1f}°")
            self.last_nav_log_time = now

        # Waypoint reached, descend for scan
        if dist < self.target_radius:
            self.get_logger().info(f"[TRANSIT]: Waypoint {self.current_wp_index+1} reached")
            self.scan_wp_index = self.current_wp_index
            self.scan_hold_orientation = self.target_pose.pose.orientation
            self.scan_start_time = None
            self.plate_reader_ready = False
            self._set_nav_stack_enabled(False)
            self._set_plate_reader_enabled(True)
            self.state = STATE_DESCEND

    def _update_descend(self):
        """Descend to SCAN_ALT before enabling camera"""
        self._set_nav_stack_enabled(False)
        self._set_plate_reader_enabled(True)

        # Navigate to scan altitude, holding orientation from transit phase
        dist, xy_dist, dz, yaw = self.update_waypoint_navigation(self.scan_wp_index, SCAN_ALT, self.scan_hold_orientation)
        
        if abs(dz) < ASCEND_DESCEND_TOL:  # Reached scan altitude
            if self.plate_reader_ready:
                self.state = STATE_SCAN
                self.scan_start_time = self.get_clock().now()
                self.confirmed_plate = None
                self.get_logger().info(f"[SCAN]: At scan altitude {SCAN_ALT} m, camera enabled")
            else:
                now = self.get_clock().now()
                if (now - self.last_scan_wait_log_time).nanoseconds / 1e9 > WAIT_LOG_PERIOD_SEC:
                    self.get_logger().warn('[SCAN]: Waiting for plate_reader ready before starting timeout')
                    self.last_scan_wait_log_time = now

    def _update_scan(self):
        """Hold scanning phase at current waypoint"""
        self._set_nav_stack_enabled(False)
        self._set_plate_reader_enabled(True)

        # Navigate to hold position at scan altitude
        dist, xy_dist, dz, yaw = self.update_waypoint_navigation(self.scan_wp_index, SCAN_ALT, self.scan_hold_orientation)

        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now()
        
        now = self.get_clock().now()
        scan_elapsed = (now - self.scan_start_time).nanoseconds / 1e9
        
        # Exit SCAN on either successful read or timeout, then run one shared ascent transition.
        timed_out = scan_elapsed > SCAN_TIMEOUT_SEC
        if self.confirmed_plate or timed_out:
            if timed_out:
                self.get_logger().warn(f"[SCAN]: Timeout at waypoint {self.current_wp_index+1}, ascending")

            self._set_plate_reader_enabled(False)
            self.state = STATE_ASCEND
            return

    def _update_ascend(self):
        """Ascend back to TRANSIT_ALT and enable planner"""
        self._set_plate_reader_enabled(False)

        # Navigate to transit altitude, holding orientation from transit phase
        dist, xy_dist, dz, yaw = self.update_waypoint_navigation(self.scan_wp_index, TRANSIT_ALT, self.scan_hold_orientation)
        
        if abs(dz) < ASCEND_DESCEND_TOL:  # Reached transit altitude
            if not self.waiting_for_stack_ready:
                self._set_nav_stack_enabled(True, reset_freshness=True)
                self.waiting_for_stack_ready = True

            now = self.get_clock().now()
            if not self._planner_stack_ready(now):
                if (now - self.last_stack_wait_log_time).nanoseconds / 1e9 > WAIT_LOG_PERIOD_SEC:
                    self.get_logger().warn('[ASCEND]: Holding for fresh depth/occupancy/planner publishes before transit')
                    self.last_stack_wait_log_time = now
                return

            self.waiting_for_stack_ready = False
            self.current_wp_index += 1
            wp = self.waypoints[self.scan_wp_index]
            self.prev_waypoint = np.array([wp[0], wp[1], TRANSIT_ALT])
            self.state = STATE_TRANSIT
            if self.confirmed_plate:
                self.get_logger().info(f"[WAYPOINT {self.scan_wp_index+1}]: Plate scanned: {self.confirmed_plate}")
            self.get_logger().info(f"[ASCEND]: Reached transit altitude, moving to waypoint {self.current_wp_index+1}")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
