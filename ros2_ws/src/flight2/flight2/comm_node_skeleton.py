import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


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
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        self.pos_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_callback, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_timer_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        self.last_pos_log_time = self.get_clock().now()

        # Poses
        self.current_pos = PoseStamped()
        self.target_pose = PoseStamped()
        self.current_state = State()

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.command_client.wait_for_service()
        self.get_logger().info('Initialization complete!')


    def callback_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Takeoff in place to LAUNCH_ALT"""
        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.orientation = self.current_pos.pose.orientation
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT

        self.get_logger().info(f"[LAUNCH]: Takeoff to target | X: {self.target_pose.pose.position.x:.3f} | Y: {self.target_pose.pose.position.y:.3f} | Z: {self.target_pose.pose.position.z:.3f}")

        self.set_mode('OFFBOARD')

        self.arming(True)
        
        self.get_logger().info("[LAUNCH]: Takeoff initiated.")

        response.success = True
        response.message = f"Launch initiated. Drone is taking off to {self.target_pose.pose.position.z} m."
        return response


    def callback_test(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """hover"""
        # lock XY to current pose
        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT

        self.get_logger().info(f"[TEST]: X {self.current_pos.pose.position.x}")
        self.get_logger().info(f"[TEST]: Y: {self.current_pos.pose.position.y}")
        self.get_logger().info(f"[TEST]: Z: {self.current_pos.pose.position.z}")

        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        self.set_mode('OFFBOARD')
        response.success = True
        response.message = f"Drone is hovering."
        return response


    def callback_land(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """RTL and land"""
        #self.target_pose.pose.position.x = 0
        #self.target_pose.pose.position.y = 0
        #self.target_pose.pose.position.z = 0
        #self.target_pose.pose.orientation = self.current_pos.pose.orientation
        #
        #dist = (self.target_pose.pose.position.x - self.current_pos.pose.position.x)**2 + (self.target_pose.pose.position.y - self.current_pos.pose.position.y)**2
        #
        #while dist > 0.1:
        #    self.target_pose.header.stamp = self.get_clock().now().to_msg()
        #    self.local_pos_pub.publish(self.target_pose)
        #    dist = (self.target_pose.pose.position.x - self.current_pos.pose.position.x)**2 + (self.target_pose.pose.position.y - self.current_pos.pose.position.y)**2

        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.get_logger().info("Requesting AUTO.LAND...")
        self.set_mode_client.call_async(req)
        response.success = True
        response.message = "Sent AUTO.LAND command to MAVROS."
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


    def state_callback(self, msg: State) -> None:
        self.current_state = msg

        now = self.get_clock().now()
        if (now - self.last_state_log_time).nanoseconds > 1e9:
            self.get_logger().info(f'[STATE]: Connected: {msg.connected} | Armed: {msg.armed} | Mode: {msg.mode}')
            self.last_state_log_time = now


    def pos_callback(self, msg):
        """Get current position + orientation"""
        self.current_pos = msg

        now = self.get_clock().now()
        if (now - self.last_pos_log_time).nanoseconds > 1e9:
            #self.get_logger().info(f"[POSITION]: X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")
            self.get_logger().info("[POSITION]: TEST")
            self.last_pos_log_time = now


    def timer_callback(self):
        """Heartbeat for the drone"""
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)
        
        now = self.get_clock().now()
        if (now - self.last_timer_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"[POSITION]: X: {self.current_pos.pose.position.x:.3f} | Y: {self.current_pos.pose.position.y:.3f} | Z: {self.current_pos.pose.position.z:.3f}")
            self.get_logger().info(f"[TARGET]: X: {self.target_pose.pose.position.x:.3f} | Y: {self.target_pose.pose.position.y:.3f} | Z: {self.target_pose.pose.position.z:.3f}")
            self.last_timer_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()