import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from scipy.spatial.transform import Rotation as R

MODES = ['OFFBOARD', 'ALTCTL', 'STABILIZED']

LAUNCH_ALT = 0.5
Z_OFFSET = 0.0
RADIUS = 0.2
MAX_SPEED = 0.5         # m/s horizontal
MAX_SPEED_Z = 0.3       # m/s vertical
APPROACH_SLOWDOWN = 0.5  # start slowing within this distance of waypoint


class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')

        # Services
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_8/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_8/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_8/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_8/comm/abort', self.callback_abort)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.command_client = self.create_client(CommandLong, 'mavros/cmd/command')

        # Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self._state_callback, 10)
        self.pos_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self._pos_callback, qos_profile_sensor_data)
        self.waypoint_sub = self.create_subscription(PoseArray, 'rob498_drone_8/comm/waypoints', self._waypoint_callback, 10)

        # Publishers
        # Position setpoint — used for launch and hover
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        # Velocity setpoint — used during waypoint navigation (avoidance node intercepts this)
        self.vel_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10) # mavros/setpoint_raw/local rob498_drone_8/cmd_vel
        # Waypoint publisher
        self.waypoint_pub = self.create_publisher(PoseArray, 'rob498_drone_8/comm/waypoints', 10)

        # Timer at 20Hz
        self.timer = self.create_timer(0.05, self._timer_callback)
        self.last_target_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        self.last_pos_log_time = self.get_clock().now()
        self.last_nav_log_time = self.get_clock().now()

        # State
        self.current_pos = PoseStamped()
        self.current_yaw = 0.0
        self.target_pose = PoseStamped()
        self.current_state = State()
        self.home_pose = None

        # Waypoints
        self.waypoints = []
        self.waypoints_received = False
        self.current_wp_index = 0
        self.target_radius = RADIUS
        self.test_active = False
        self.return_home_active = False

        # Control mode
        self.use_velocity = False  # True during waypoint nav, False during launch/hover/land

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.command_client.wait_for_service()
        self.get_logger().info('Initialization complete!')

    # ── Services ──────────────────────────────────────────────

    def callback_launch(self, request, response):
        """Takeoff in place to LAUNCH_ALT using position control."""
        if self.home_pose is None:
            self.home_pose = PoseStamped()
            self.home_pose.pose = self.current_pos.pose
            self.get_logger().info(
                f"[HOME SET] X: {self.current_pos.pose.position.x:.3f} | "
                f"Y: {self.current_pos.pose.position.y:.3f} | "
                f"Z: {self.current_pos.pose.position.z:.3f}")

        self.use_velocity = False

        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        self.get_logger().info(
            f"[LAUNCH]: Takeoff to target | "
            f"X: {self.target_pose.pose.position.x:.3f} | "
            f"Y: {self.target_pose.pose.position.y:.3f} | "
            f"Z: {self.target_pose.pose.position.z:.3f}")

        self.set_mode('OFFBOARD')
        self.arming(True)

        self.get_logger().info("[LAUNCH]: Takeoff initiated.")
        response.success = True
        response.message = f"Launch initiated. Drone is taking off to {self.target_pose.pose.position.z:.2f} m."
        return response

    def callback_test(self, request, response):
        if not self.waypoints_received:
            self.get_logger().warn("[TEST]: Cannot start test — no waypoints received")
            response.success = False
            response.message = "No waypoints received"
            return response

        # Publish a hold command immediately to prevent OFFBOARD dropout
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        cmd.yaw = self.current_yaw
        self.vel_pub.publish(cmd)

        self.current_wp_index = 0
        self.test_active = True
        self.use_velocity = True

        self.get_logger().info("[TEST]: Starting waypoint navigation (velocity mode)")
        response.success = True
        response.message = "Waypoint test started"
        return response

    def callback_land(self, request, response):
        """Return home and land."""
        if self.home_pose is None:
            response.success = False
            response.message = "No home pose available yet."
            return response

        if not self.return_home_active:
            response.success = False
            response.message = "Return home not active"
            return response

        dx = self.home_pose.pose.position.x - self.current_pos.pose.position.x
        dy = self.home_pose.pose.position.y - self.current_pos.pose.position.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.target_radius:
            self.get_logger().info("[MISSION]: Home reached. Initiating landing.")
            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.set_mode_client.call_async(req)
            self.return_home_active = False
            self.use_velocity = False
            response.success = True
            response.message = "Sent AUTO.LAND command."
        else:
            response.success = False
            response.message = f"Still {dist:.2f}m from home"

        return response

    def callback_abort(self, request, response):
        """Emergency land."""
        self.test_active = False
        self.return_home_active = False
        self.use_velocity = False

        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.get_logger().info("Requesting AUTO.LAND...")
        self.set_mode_client.call_async(req)
        response.success = True
        response.message = "Sent AUTO.LAND command to MAVROS."
        return response

    # ── Waypoint Navigation (Velocity Control) ───────────────

    def _compute_velocity_to_target(self, target_x, target_y, target_z):
        """Compute velocity vector toward a target position.
        Returns a PositionTarget message with velocity setpoint."""
        dx = target_x - self.current_pos.pose.position.x
        dy = target_y - self.current_pos.pose.position.y
        dz = target_z - self.current_pos.pose.position.z

        dist_xy = np.sqrt(dx**2 + dy**2)
        dist_z = abs(dz)

        # Horizontal velocity — slow down near waypoint
        if dist_xy < 0.01:
            vx, vy = 0.0, 0.0
        else:
            speed_xy = min(MAX_SPEED, dist_xy / APPROACH_SLOWDOWN * MAX_SPEED)
            speed_xy = np.clip(speed_xy, 0.0, MAX_SPEED)
            vx = (dx / dist_xy) * speed_xy
            vy = (dy / dist_xy) * speed_xy

        # Vertical velocity
        if dist_z < 0.01:
            vz = 0.0
        else:
            speed_z = min(MAX_SPEED_Z, dist_z / APPROACH_SLOWDOWN * MAX_SPEED_Z)
            speed_z = np.clip(speed_z, 0.0, MAX_SPEED_Z)
            vz = np.sign(dz) * speed_z

        # Compute yaw toward target
        yaw = np.arctan2(dy, dx)

        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.yaw = yaw

        return cmd

    def update_waypoint_navigation(self):
        if not self.test_active:
            return

        if self.current_wp_index >= len(self.waypoints):
            self.test_active = False
            self.return_home_active = True
            self.use_velocity = True
            self.get_logger().info("[MISSION]: All waypoints reached. Returning home.")
            return

        wp = self.waypoints[self.current_wp_index]

        # Check distance to waypoint
        dx = wp[0] - self.current_pos.pose.position.x
        dy = wp[1] - self.current_pos.pose.position.y
        dz = wp[2] - self.current_pos.pose.position.z
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        now = self.get_clock().now()
        if (now - self.last_nav_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f"[NAV]: WP {self.current_wp_index + 1} | Dist {dist:.3f} m")
            self.last_nav_log_time = now

        if dist < self.target_radius:
            self.get_logger().info(
                f"[WAYPOINT]: {self.current_wp_index + 1} reached (distance {dist:.3f} m)")
            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints):
                next_wp = self.waypoints[self.current_wp_index]
                self.get_logger().info(
                    f"[NAV]: Moving to waypoint {self.current_wp_index + 1} | "
                    f"X {next_wp[0]:.3f} | Y {next_wp[1]:.3f} | Z {next_wp[2]:.3f}")

    def update_return_home(self):
        if not self.return_home_active:
            return

        dx = self.home_pose.pose.position.x - self.current_pos.pose.position.x
        dy = self.home_pose.pose.position.y - self.current_pos.pose.position.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.target_radius:
            self.get_logger().info("[MISSION]: Home reached. Initiating landing.")
            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.set_mode_client.call_async(req)
            self.return_home_active = False
            self.use_velocity = False

    # ── Timer ─────────────────────────────────────────────────

    def _timer_callback(self):
        if self.test_active:
            self.update_waypoint_navigation()

            # Publish velocity toward current waypoint
            if self.current_wp_index < len(self.waypoints):
                wp = self.waypoints[self.current_wp_index]
                cmd = self._compute_velocity_to_target(wp[0], wp[1], wp[2])
                self.vel_pub.publish(cmd)

        elif self.return_home_active:
            self.update_return_home()

            # Publish velocity toward home
            if self.home_pose is not None:
                cmd = self._compute_velocity_to_target(
                    self.home_pose.pose.position.x,
                    self.home_pose.pose.position.y,
                    self.current_pos.pose.position.z)
                self.vel_pub.publish(cmd)

        else:
            # Launch / hover — use position control
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.local_pos_pub.publish(self.target_pose)

        now = self.get_clock().now()
        if (now - self.last_target_log_time).nanoseconds > 1e9:
            if self.use_velocity and self.test_active and self.current_wp_index < len(self.waypoints):
                wp = self.waypoints[self.current_wp_index]
                self.get_logger().info(
                    f"[TARGET]: WP {self.current_wp_index + 1} | "
                    f"X: {wp[0]:.3f} | Y: {wp[1]:.3f} | Z: {wp[2]:.3f}")
            else:
                self.get_logger().info(
                    f"[TARGET]: X: {self.target_pose.pose.position.x:.3f} | "
                    f"Y: {self.target_pose.pose.position.y:.3f} | "
                    f"Z: {self.target_pose.pose.position.z:.3f}")
            self.last_target_log_time = now

    # ── Callbacks ─────────────────────────────────────────────

    def _state_callback(self, msg):
        self.current_state = msg
        now = self.get_clock().now()
        if (now - self.last_state_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f'[STATE]: Connected: {msg.connected} | Armed: {msg.armed} | Mode: {msg.mode}')
            self.last_state_log_time = now

    def _pos_callback(self, msg):
        self.current_pos = msg
        self.current_pos.pose.position.z += Z_OFFSET

        # Extract yaw
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny, cosy)

    def _waypoint_callback(self, msg):
        if self.waypoints_received:
            return

        self.get_logger().info("[WAYPOINTS]: Waypoints received from ground control")
        for i, pose in enumerate(msg.poses):
            wp = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints.append(wp)
            self.get_logger().info(
                f"[WAYPOINT {i + 1}]: X {wp[0]:.3f} | Y {wp[1]:.3f} | Z {wp[2]:.3f}")

        self.waypoints_received = True
        self.get_logger().info(f"[WAYPOINTS]: Stored {len(self.waypoints)} waypoints successfully")

    # ── Arming / Mode ─────────────────────────────────────────

    def arming(self, val):
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

    def set_mode(self, mode_string):
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
        kill_req = CommandLong.Request()
        kill_req.command = 400
        kill_req.param1 = 0.0
        kill_req.param2 = 21196.0
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


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()