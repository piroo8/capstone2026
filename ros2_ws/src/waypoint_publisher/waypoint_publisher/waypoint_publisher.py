import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray

WAYPOINT_TOPIC = 'rob498_drone_8/comm/waypoints'
WAYPOINT_SCAN_FLAGS_TOPIC = 'rob498_drone_8/comm/waypoint_scan_flags'

# Edit these waypoints for your space
# WAYPOINTS = [
#     {'x': 0.0, 'y': 0.0, 'z': 1.0, 'scan_plate': False},
#     {'x': 1.0, 'y': 0.0, 'z': 1.0, 'scan_plate': True},
#     {'x': 1.0, 'y': -1.0, 'z': 1.0, 'scan_plate': True},
# ]

# Bigger square pattern for testing
# WAYPOINTS = [
#     {'x': 0.0, 'y': 0.0, 'z': 1.0, 'scan_plate': False},
#     {'x': -1.0, 'y': 1.0, 'z': 1.0, 'scan_plate': False},
#     {'x': -1.0, 'y': -1.0, 'z': 1.0, 'scan_plate': True},
#     {'x': 1.0, 'y': -1.0, 'z': 1.0, 'scan_plate': True},
#     {'x': 1.0, 'y': 1.0, 'z': 1.0, 'scan_plate': True},
#     {'x': 1.0, 'y': 1.0, 'z': 1.0, 'scan_plate': False},
# ]

WAYPOINTS = [
    {'x': 0.2, 'y': 0.0, 'z': 2.0, 'scan_plate': False},
    {'x': 3.0, 'y': 0.0, 'z': 2.0, 'scan_plate': False},
    {'x': 3.0, 'y': -0.5, 'z': 2.0, 'scan_plate': False},
    {'x': 3.0, 'y': -1.0, 'z': 2.0, 'scan_plate': False},
]

# WAYPOINTS = [
#     {'x': 0.2, 'y': 0.0, 'z': 0.5, 'scan_plate': False},
#     {'x': 1.5, 'y': 0.0, 'z': 0.5, 'scan_plate': True},
# ]

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.waypoint_pub = self.create_publisher(PoseArray, WAYPOINT_TOPIC, 10)
        self.scan_flags_pub = self.create_publisher(Int32MultiArray, WAYPOINT_SCAN_FLAGS_TOPIC, 10)

        # Publish once after 2 seconds to give comm node time to start
        self.timer = self.create_timer(2.0, self._publish)
        self.published = False

        self.get_logger().info(f'Will publish {len(WAYPOINTS)} waypoints in 2 seconds...')

    def _publish(self):
        if self.published:
            return

        waypoint_msg = PoseArray()
        scan_flags_msg = Int32MultiArray()
        scan_flags = []

        for waypoint in WAYPOINTS:
            p = Pose()
            p.position.x = waypoint['x']
            p.position.y = waypoint['y']
            p.position.z = waypoint['z']
            waypoint_msg.poses.append(p)
            scan_flags.append(1 if waypoint['scan_plate'] else 0)

        scan_flags_msg.data = scan_flags

        self.waypoint_pub.publish(waypoint_msg)
        self.scan_flags_pub.publish(scan_flags_msg)
        self.published = True

        self.get_logger().info(
            f'Published {len(WAYPOINTS)} waypoints and {len(scan_flags)} scan flags:'
        )
        for i, waypoint in enumerate(WAYPOINTS):
            waypoint_type = 'scan' if waypoint['scan_plate'] else 'nav-only'
            self.get_logger().info(
                f"  WP {i + 1}: ({waypoint['x']}, {waypoint['y']}, {waypoint['z']}) [{waypoint_type}]"
            )

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
