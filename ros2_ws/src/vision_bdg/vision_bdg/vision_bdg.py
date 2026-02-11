#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

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
    def vicon_callback(self, msg: PoseStamped):

        out_msg = PoseStamped()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'map'
        out_msg.pose = msg.pose

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