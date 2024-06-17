import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


MODES = ['OFFBOARD', 'ALTCTL', 'STABILIZED']
#Vicon
LAUNCH_ALT = 0.5 - 0.185
RETURN_ALT = 2.0

#Camera
#LAUNCH_ALT = 0.5 - 0.085


class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_08')

        self.group = ReentrantCallbackGroup()

        self.srv_launch = self.create_service(Trigger, 'rob498_drone_08/comm/launch', self.callback_launch, callback_group=self.group)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_08/comm/test', self.callback_test, callback_group=self.group)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_08/comm/land', self.callback_land, callback_group=self.group)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_08/comm/abort', self.callback_abort, callback_group=self.group)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=self.group)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=self.group)
        self.command_client = self.create_client(CommandLong, 'mavros/cmd/command', callback_group=self.group)

        # Subs and Pubs 
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10, callback_group=self.group)
        self.pos_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_callback, 10, callback_group=self.group)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback, callback_group=self.group)

        # Poses
        self.current_pos = PoseStamped()
        self.target_pose = PoseStamped()
        self.home_pos = PoseStamped()

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.command_client.wait_for_service()

        self.get_logger().info('Initializing Home Position... waiting for MAVROS data.')
    
        # This loop runs until current_pos is no longer empty
        while rclpy.ok() and self.current_pos.header.stamp.sec == 0:
            # Spin the node for a tiny fraction of a second to allow callbacks to run
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 3. Snapshot the current position as Home
        self.home_pos.header = self.current_pos.header
        self.home_pos.pose.position.x = self.current_pos.pose.position.x
        self.home_pos.pose.position.y = self.current_pos.pose.position.y
        self.home_pos.pose.position.z = self.current_pos.pose.position.z
        self.home_pos.pose.orientation = self.current_pos.pose.orientation
        
        self.get_logger().info(f'Home set at: x={self.home_pos.pose.position.x:.2f}, '
                            f'y={self.home_pos.pose.position.y:.2f}, '
                            f'z={self.home_pos.pose.position.z:.2f}')

        self.get_logger().info('Initialization complete!')


    # Service callbacks

    async def callback_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Takeoff sequence: Set Mode -> Arm -> Set Altitude"""
        self.get_logger().info("Launch triggered. Switching to OFFBOARD...")
        
        mode_success = await self.set_mode_async('OFFBOARD')
        if not mode_success:
            response.success = False
            response.message = "[LAUNCH]: Failed to enter OFFBOARD mode."
            return response

        arm_success = await self.arm_drone_async(True)
        if not arm_success:
            response.success = False
            response.message = "[LAUNCH]: Arming rejected."
            return response

        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        response.success = True
        response.message = f"[LAUNCH]: Armed and taking off to {LAUNCH_ALT}m"
        return response
        # # lock XY to current pose
        # self.target_pose.pose.position.x = self.current_pos.pose.position.x
        # self.target_pose.pose.position.y = self.current_pos.pose.position.y
        # self.target_pose.pose.orientation = self.current_pos.pose.orientation

        # self.target_pose.pose.position.z = self.current_pos.pose.position.z + LAUNCH_ALT
        # self.set_mode('OFFBOARD')
        
        # response.success = True
        # response.message = f"Drone is taking off to {LAUNCH_ALT} m."
        # return response
    

    async def callback_test(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Hover at current position"""
        self.target_pose.pose.position = self.current_pos.pose.position
        self.target_pose.pose.orientation = self.current_pos.pose.orientation
        await self.set_mode_async('OFFBOARD')
        response.success = True
        response.message = "Hovering."
        return response

    
    async def callback_land(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Fly to Home XY at RETURN_ALT altitude, then land."""

        self.get_logger().info("[LAND]: Phase 1 - Returning to Home XY...")

        self.target_pose.pose.position.x = self.home_pos.pose.position.x
        self.target_pose.pose.position.y = self.home_pos.pose.position.y
        self.target_pose.pose.position.z = self.home_pos.pose.position.z + LAUNCH_ALT

        arrival_threshold = 0.2
        while rclpy.ok():
            dx = self.current_pos.pose.position.x - self.home_pos.pose.position.x
            dy = self.current_pos.pose.position.y - self.home_pos.pose.position.y
            distance_to_home_xy = (dx**2 + dy**2)**0.5

            if distance_to_home_xy < arrival_threshold:
                self.get_logger().info("[LAND]: Arrived at Home XY. Initiating Phase 2 (AUTO.LAND)...")
                break
            
            # This small sleep allows other callbacks (like pos_callback) to run
            await rclpy.task.sleep(0.1)

        # 3. Phase 2 - Trigger PX4 Landing
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        
        try:
            result = await self.set_mode_client.call_async(req)
            response.success = result.mode_sent
            response.message = "[LAND]: Return-to-Home and Land sequence complete."
        except Exception as e:
            response.success = False
            response.message = f"[LAND]: Service call failed: {e}"
            
        return response
        

    async def callback_abort(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Land immediately"""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        try:
            result = await self.set_mode_client.call_async(req)
            response.success = result.mode_sent
            response.message = "[ABORT]: Emergency landing initiated."
        except Exception as e:
            response.success = False
            response.message = f"[ABORT]: Abort mission failed: {e}"
        return response
    

    async def kill_switch(self):
        """Kill switch."""
        kill_req = CommandLong.Request()
        kill_req.command = 400
        kill_req.param1 = 0.0
        kill_req.param2 = 21196.0
        
        kill_result = await self.command_client.call_async(kill_req)
        
        if kill_result.success:
            self.get_logger().info(f"[KILL SWITCH]: Success.")
        else:
            self.get_logger().info(f"[KILL SWITCH]: Failed.")
        
    
    # Helper functions

    async def arm_drone_async(self, val: bool) -> bool:
        req = CommandBool.Request()
        req.value = val
        try:
            result = await self.arming_client.call_async(req)
            self.get_logger().info(f"[ARMING]: Arming state set to: {val}")
            return result.success
        except Exception as e:
            self.get_logger().error(f"[ARMING]: Failed to set drone state to: {val}\n{e}")
            return False
        
    async def set_mode_async(self, mode_string: str) -> bool:
        if mode_string not in MODES and 'AUTO' not in mode_string:
            return False
        req = SetMode.Request()
        req.custom_mode = mode_string
        try:
            result = await self.set_mode_client.call_async(req)
            self.get_logger().info(f"[MODE]: Flight mode set to {mode_string}")
            return result.mode_sent
        except Exception as e:
            self.get_logger().error(f"[MODE]: Failed to set flight mode to: {mode_string}\n{e}")
            return False


    # standard callbacks

    def state_callback(self, msg: State) -> None:
        """State subscriber callback"""
        if self.get_clock().now().nanoseconds % 5000000000 == 0:
            self.get_logger().info(f'Connected: {msg.connected} | Armed: {msg.armed} | Current Mode: {msg.mode}')

    
    def pos_callback(self, msg):
        """Get current position + orientation"""
        self.current_pos = msg


    def timer_callback(self):
        """Heartbeat for the drone"""
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)


def main(args=None):
    rclpy.init(args=args)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
