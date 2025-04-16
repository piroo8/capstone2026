import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


MODES = ['OFFBOARD', 'ALTCTL', 'STABILIZED']
#Vicon
#LAUNCH_ALT = 1.5 - 0.185

#Camera
LAUNCH_ALT = 1.5 - 0.085


class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_08')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_08/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_08/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_08/comm/land', self.callback_land)
        #self.srv_abort = self.create_service(Trigger, 'rob498_drone_08/comm/abort', self.callback_abort)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        #self.command_client = self.create_client(CommandLong, 'mavros/cmd/command')

        # Subs and Pubs 
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        self.pos_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_callback, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Poses
        self.current_pos = PoseStamped()
        self.target_pose = PoseStamped()

        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        #self.command_client.wait_for_service()
        self.get_logger().info('Initialization complete!')


    def callback_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Takeoff in place to LAUNCH_ALT"""
        # lock XY to current pose
        self.target_pose.pose.position.x = self.current_pos.pose.position.x
        self.target_pose.pose.position.y = self.current_pos.pose.position.y
        self.target_pose.pose.orientation = self.current_pos.pose.orientation

        self.target_pose.pose.position.z = LAUNCH_ALT
        self.set_mode('OFFBOARD')
        
        # arm drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(lambda f: self.universal_callback(f, "ARMING"))
        
        response.success = True
        response.message = f"Drone is taking off to {LAUNCH_ALT} m."
        return response
    

    def callback_test(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        pass


    def callback_land(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Land using PX4 land mode"""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.get_logger().info("Requesting AUTO.LAND...")
        self.set_mode_client.call_async(req)
        
        response.success = True
        response.message = "Sent AUTO.LAND command to MAVROS."
        return response
    

    # def callback_abort(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
    #     """Kill switch."""
    #     kill_req = CommandLong.Request()
    #     kill_req.command = 400
    #     kill_req.param1 = 0.0
    #     kill_req.param2 = 21196.0
        
    #     future = self.command_client.call_async(kill_req)
    #     future.add_done_callback(lambda f: self.universal_callback(f, "FORCE KILL"))
        
    #     response.success = True
    #     response.message = "Kill switch engaged."
    #     return response
    

    def set_mode(self, mode_string: str):
        """Set flight mode"""
        if mode_string not in MODES:
            self.get_logger().error(f"Invalid mode requested: {mode_string}")
            return
        
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_string

        self.get_logger().info(f"Requesting mode change to: {mode_string}")
        
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.universal_callback(f, f'SETMODE:{mode_string}'))

    def state_callback(self, msg: State) -> None:
        """State subscriber callback"""
        if self.get_clock().now().nanoseconds % 1000000000 == 0: # Roughly once per second
            self.get_logger().info(f'Connected: {msg.connected} | Armed: {msg.armed} | Current Mode: {msg.mode}')

    
    def pos_callback(self, msg):
        """Get current position + orientation"""
        self.current_pos = msg


    def timer_callback(self):
        """Heartbeat for the drone"""
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)

    
    def universal_callback(self, future, cmd_name):
        """
        Enhanced handler that logs the specific command name and its result.
        """
        try:
            response = future.result()
            
            # 1. Handle SetMode (returns 'mode_sent')
            if hasattr(response, 'mode_sent'):
                success = response.mode_sent
            # 2. Handle CommandBool/CommandLong/Trigger (returns 'success')
            elif hasattr(response, 'success'):
                success = response.success
            else:
                success = False

            if success:
                self.get_logger().info(f"SUCCESS: [{cmd_name}] command accepted.")
            else:
                self.get_logger().error(f"REJECTED: [{cmd_name}] failed.")

        except Exception as e:
            self.get_logger().error(f"CRITICAL: [{cmd_name}] service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
