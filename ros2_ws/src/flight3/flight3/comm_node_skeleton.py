import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
import numpy as np

STATES = ['LAUNCH, ARMING']
MODES = ['OFFBOARD', 'ALTCTL', 'STABILIZED']
#Vicon
LAUNCH_ALT = 0.5 - 0.185

#Camera
#LAUNCH_ALT = 0.5


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
        # Flight test 3
        self.waypoint_pub = self.create_publisher(PoseArray,'rob498_drone_8/comm/waypoints', 10)
        self.waypoint_sub = self.create_subscription(PoseArray,'rob498_drone_8/comm/waypoints',self._waypoint_callback, 10)

        # Timer
        self.timer = self.create_timer(0.05, self._timer_callback)
        self.last_target_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        self.last_pos_log_time = self.get_clock().now()
        self.last_nav_log_time = self.get_clock().now()

        # Poses
        self.current_pos = PoseStamped()
        self.target_pose = PoseStamped()
        self.current_state = State()
        # Flight test 3
        self.home_pose = PoseStamped()

        # Flight test 3
        # Waypoints
        self.waypoints = []
        self.waypoints_received = False
        self.current_wp_index = 0
        self.target_radius = 0.3  # 40 cm radius test # changed to 0.3
        self.test_active = False
        self.return_home_active = False

        # From Flight 2 Needed.
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.command_client.wait_for_service()
        self.get_logger().info('Initialization complete!')


    def callback_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Takeoff in place to LAUNCH_ALT"""
        if self.home_pose is None:
            self.home_pose.pose = self.current_pos.pose
            self.get_logger().info(f"[HOME SET] X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")

        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        self.get_logger().info(f"[LAUNCH]: Takeoff to target | X: {self.target_pose.pose.position.x:.3f} | Y: {self.target_pose.pose.position.y:.3f} | Z: {self.target_pose.pose.position.z:.3f}")

        self.set_mode('OFFBOARD')

        self.arming(True)
        
        self.get_logger().info("[LAUNCH]: Takeoff initiated.")

        response.success = True
        response.message = f"Launch initiated. Drone is taking off to {self.target_pose.pose.position.z} m."
        return response

    def callback_test(self, request, response):
        
        publish_waypoints()

        if not self.waypoints_received:
            self.get_logger().warn("[TEST]: Cannot start test — no waypoints received")

            response.success = False
            response.message = "No waypoints received"
            return response

        self.current_wp_index = 0
        self.test_active = True

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

            return response


    def callback_abort(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Land using PX4 land mode"""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.get_logger().info("Requesting AUTO.LAND...")
        self.set_mode_client.call_async(req)
        response.success = True
        response.message = "Sent AUTO.LAND command to MAVROS."
        return response

    def update_waypoint_navigation(self):
        if not self.test_active:
            return

        if self.current_wp_index >= len(self.waypoints):
            self.test_active = False
            self.return_home_active = True
            self.get_logger().info("[MISSION]: All waypoints reached. Returning home.")
            return

        wp = self.waypoints[self.current_wp_index]

        # Set target
        self.target_pose.pose.position.x = wp[0]
        self.target_pose.pose.position.y = wp[1]
        self.target_pose.pose.position.z = wp[2]
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        # Distance to waypoint
        dx = wp[0] - self.current_pos.pose.position.x
        dy = wp[1] - self.current_pos.pose.position.y
        dz = wp[2] - self.current_pos.pose.position.z

        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        now = self.get_clock().now()

        if (now - self.last_nav_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[NAV]: WP {self.current_wp_index+1} | "f"Dist {dist:.3f} m")
            self.last_nav_log_time = now

        if dist < self.target_radius:

            self.get_logger().info(f"[WAYPOINT]: {self.current_wp_index+1} reached "f"(distance {dist:.3f} m)")

            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints):

                next_wp = self.waypoints[self.current_wp_index]

                self.get_logger().info(f"[NAV]: Moving to waypoint {self.current_wp_index+1} | "f"X {next_wp[0]:.3f} | Y {next_wp[1]:.3f} | Z {next_wp[2]:.3f}")

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

        now = self.get_clock().now()
        if (now - self.last_pos_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[POSITION]: X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")
            self.last_pos_log_time = now


    def _timer_callback(self):
        """Heartbeat for the drone"""
        # WAYPOINT NAVIGATION
        if self.test_active:
            self.update_waypoint_navigation()

        # RETURN HOME
        elif self.return_home_active:   
            # create dummy service objects to call the function
            req = Trigger.Request()
            res = Trigger.Response()

            self.callback_land(req, res)

        self.target_pose.header.stamp = self.get_clock().now().to_msg()
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

    
    def publish_waypoints(self):
        wp1 = Pose()
        wp2 = Pose()
        wp3 = Pose()
        wp4 = Pose()

        wp1.position.x = 2
        wp1.position.y = 0
        wp1.position.z = 1

        wp2.position.x = 2
        wp2.position.y = 2
        wp2.position.z = 1

        wp3.position.x = 0
        wp3.position.y = 2
        wp3.position.z = 1

        wp4.position.x = 0
        wp4.position.y = 0
        wp4.position.z = 1

        waypoints = PoseArray()

        waypoints.append(wp1)
        waypoints.append(wp2)
        waypoints.append(wp3)
        waypoints.append(wp4)

        self.waypoint_pub.publish(waypoints)

        return

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()