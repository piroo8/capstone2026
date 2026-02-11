#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')

        # ROS2 parameters to select sources
        self.declare_parameter('use_vicon', True)
        self.declare_parameter('use_realsense', False)

        self.use_vicon = self.get_parameter('use_vicon').value
        self.use_realsense = self.get_parameter('use_realsense').value

        # Subscribers
        if self.use_vicon:
            self.get_logger().info("VisionBridge: Subscribing to Vicon pose")
            self.vicon_sub = self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone',
                self.handle_pose,
                10
            )

        if self.use_realsense:
            self.get_logger().info("VisionBridge: Subscribing to Realsense pose")
            self.realsense_sub = self.create_subscription(
                PoseStamped,
                '/camera/pose/sample',
                self.handle_pose,
                10
            )

        # Publisher
        self.vision_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10
        )

    def handle_pose(self, msg: PoseStamped):
        # IMPORTANT: set the frame_id expected by MAVROS/PX4
        msg.header.frame_id = 'map'
        self.vision_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
