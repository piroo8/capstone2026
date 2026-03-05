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
    def vicon_callback(self, msg: PoseStamped):
        out = PoseStamped()

        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        # ---- position transform ----
        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = msg.pose.position.y
        out.pose.position.z = msg.pose.position.z

        # ---- orientation transform (180° about Z) ----
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # quaternion for 180° rotation about Z
        rx, ry, rz, rw = 0.0, 0.0, 1.0, 0.0

        out.pose.orientation.x = rw*qx + rx*qw + ry*qz - rz*qy
        out.pose.orientation.y = rw*qy - rx*qz + ry*qw + rz*qx
        out.pose.orientation.z = rw*qz + rx*qy - ry*qx + rz*qw
        out.pose.orientation.w = rw*qw - rx*qx - ry*qy - rz*qz

        self.publisher_.publish(out)
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