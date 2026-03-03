#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class VisionBridge(Node):

    def __init__(self):
        super().__init__('vision_bdg')

        # Parameters
        self.declare_parameter('use_vicon', False)
        self.declare_parameter('use_realsense', True)

        self.use_vicon = self.get_parameter('use_vicon').value
        self.use_realsense = self.get_parameter('use_realsense').value

        # Publisher (EKF2 vision pipeline)
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        # Subscribers
        if self.use_vicon:
            self.get_logger().info("[VisionBdg] Using Vicon")
            self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone',
                self.vicon_callback,
                10
            )

        if self.use_realsense:
            self.get_logger().info("[VisionBdg] Using Realsense")
            self.create_subscription(
                Odometry,
                '/camera/pose/sample',
                self.realsense_callback,
                rclpy.qos.qos_profile_system_default
            )

    # ---------------- VICON ----------------
    # def vicon_callback(self, msg: PoseStamped):

    #     out_msg = PoseStamped()
    #     out_msg.header.stamp = msg.header.stamp
    #     out_msg.header.frame_id = 'map'
    #     out_msg.pose = msg.pose

    #     self.publisher_.publish(out_msg)
    
    # def vicon_callback(self, msg: PoseStamped):
    #     out_msg = PoseStamped()
    #     out_msg.header.stamp = msg.header.stamp
    #     out_msg.header.frame_id = 'map'

    #     # --- Position Transform ---
    #     # Flip X and Z, keep Y
    #     out_msg.pose.position.x = -msg.pose.position.x
    #     out_msg.pose.position.y =  msg.pose.position.y
    #     out_msg.pose.position.z = -msg.pose.position.z

    #     # --- Orientation Transform (180 deg about Y) ---
    #     # Simplified Hamilton product for q_orig * [0, 1, 0, 0]
    #     ox = msg.pose.orientation.x
    #     oy = msg.pose.orientation.y
    #     oz = msg.pose.orientation.z
    #     ow = msg.pose.orientation.w

    #     out_msg.pose.orientation.x =  oz
    #     out_msg.pose.orientation.y =  ow
    #     out_msg.pose.orientation.z = -ox
    #     out_msg.pose.orientation.w = -oy

    #     self.publisher_.publish(out_msg)
    def vicon_callback(self, msg: PoseStamped):
        out_msg = PoseStamped()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'map'

        # Current values from Vicon
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ox, oy, oz, ow = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w

        # --- Step 1: Position Transform ---
        # Combination of 180-Y and -90-Z results in:
        out_msg.pose.position.x = -py
        out_msg.pose.position.y = -px
        out_msg.pose.position.z = -pz

        # --- Step 2: Orientation Transform ---
        # Constant for 90-degree rotations (sin(45) = cos(45))
        c = 0.70710678118 

        # This represents the combined rotation (180-Y followed by -90-Z)
        # The resulting quaternion math simplifies to:
        out_msg.pose.orientation.x =  c * (oz - ow)
        out_msg.pose.orientation.y =  c * (oz + ow)
        out_msg.pose.orientation.z =  c * (ox - oy)
        out_msg.pose.orientation.w = -c * (ox + oy)

        self.publisher_.publish(out_msg)
    # ---------------- REALSENSE ----------------
    def realsense_callback(self, msg: Odometry):
        out_msg = PoseStamped()

        # Use the same timestamp from camera
        #out_msg.header.stamp = msg.header.stamp
        
        # Use local node time (stable for EKF2)
        out_msg.header.stamp = self.get_clock().now().to_msg()

        # MAVROS expects ENU; it converts internally to NED
        out_msg.header.frame_id = 'map'

        # Extract pose from Odometry
        out_msg.pose = msg.pose.pose

        self.publisher_.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()