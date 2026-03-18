import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


# Edit these waypoints for your space
WAYPOINTS = [
    (0.5, 0.0, 1.0),
    (3.0, 0.0, 1.0),
]


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.pub = self.create_publisher(
            PoseArray, 'rob498_drone_8/comm/waypoints', 10)

        # Publish once after 2 seconds to give comm node time to start
        self.timer = self.create_timer(2.0, self._publish)
        self.published = False

        self.get_logger().info(f'Will publish {len(WAYPOINTS)} waypoints in 2 seconds...')

    def _publish(self):
        if self.published:
            return

        msg = PoseArray()
        for x, y, z in WAYPOINTS:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            msg.poses.append(p)

        self.pub.publish(msg)
        self.published = True

        self.get_logger().info(f'Published {len(WAYPOINTS)} waypoints:')
        for i, (x, y, z) in enumerate(WAYPOINTS):
            self.get_logger().info(f'  WP {i + 1}: ({x}, {y}, {z})')

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()