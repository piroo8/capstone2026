import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from cv_bridge import CvBridge
import numpy as np

# Avoidance parameters
OBSTACLE_STOP_DIST = 0.8     # full stop below this (meters)
OBSTACLE_SLOW_DIST = 2.0     # start slowing at this distance
MAX_SPEED = 0.5              # m/s
MIN_DEPTH = 0.2              # ignore depths closer (noise)
MAX_DEPTH = 3.0              # ignore depths farther (unreliable)
NUM_SECTORS = 8              # horizontal angular sectors
FOV_DEG = 160.0              # fisheye horizontal FOV
AVOIDANCE_GAIN = 0.3         # how hard to steer away
TIMEOUT_SEC = 0.5            # stop if no velocity command received


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_cmd = None
        self.latest_cmd_time = None
        self.current_yaw = 0.0

        # Subscribe to depth
        self.create_subscription(
            Image, '/stereo/depth',
            self._depth_cb, qos_profile_sensor_data)

        # Subscribe to desired velocity from comm node
        self.create_subscription(
            PositionTarget, 'rob498_drone_8/cmd_vel',
            self._cmd_cb, 10)

        # Subscribe to pose for yaw
        self.create_subscription(
            PoseStamped, 'mavros/local_position/pose',
            self._pose_cb, qos_profile_sensor_data)

        # Publish to MAVROS
        self.cmd_pub = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', 10)

        # Run at 20Hz to match comm node
        self.create_timer(0.05, self._avoidance_loop)

        self.get_logger().info('Obstacle avoidance node ready')

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def _cmd_cb(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _pose_cb(self, msg):
        q = msg.pose.orientation
        # Extract yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny, cosy)

    def _depth_to_sectors(self, depth):
        """Collapse depth into horizontal sectors using middle rows only."""
        h, w = depth.shape
        y_start = int(h * 0.2)
        y_end = int(h * 0.8)
        depth_cropped = depth[y_start:y_end, :]

        sector_width = w // NUM_SECTORS
        sectors = np.full(NUM_SECTORS, MAX_DEPTH)

        for i in range(NUM_SECTORS):
            col_start = i * sector_width
            col_end = (i + 1) * sector_width
            region = depth_cropped[:, col_start:col_end]
            valid = region[(region > MIN_DEPTH) & (region < MAX_DEPTH)]
            if len(valid) > 20:
                sectors[i] = np.percentile(valid, 10)

        return sectors

    def _compute_avoidance(self, sectors, vx_desired, vy_desired):
        """Modify velocity based on obstacles.
        Input velocity is in local NED frame.
        Sectors are in camera frame (forward-facing)."""

        # Map sectors to angles in body frame
        fov = np.radians(FOV_DEG)
        angles = np.linspace(-fov / 2, fov / 2, NUM_SECTORS)

        min_dist = np.min(sectors)

        # Nothing close — pass through
        if min_dist > OBSTACLE_SLOW_DIST:
            return vx_desired, vy_desired

        # Speed scaling
        if min_dist < OBSTACLE_STOP_DIST:
            speed_scale = 0.0
        else:
            speed_scale = (min_dist - OBSTACLE_STOP_DIST) / (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
            speed_scale = np.clip(speed_scale, 0.0, 1.0)

        # Compute repulsive vector in body frame (x=forward, y=left)
        repulse_fwd = 0.0
        repulse_left = 0.0

        for i in range(NUM_SECTORS):
            if sectors[i] < OBSTACLE_SLOW_DIST:
                strength = 1.0 / max(sectors[i], 0.3) - 1.0 / OBSTACLE_SLOW_DIST
                obs_fwd = np.cos(angles[i])
                obs_left = np.sin(angles[i])
                repulse_fwd -= strength * obs_fwd
                repulse_left -= strength * obs_left

        # Normalize
        repulse_mag = np.sqrt(repulse_fwd**2 + repulse_left**2)
        if repulse_mag > 0.01:
            repulse_fwd /= repulse_mag
            repulse_left /= repulse_mag

        # Rotate repulsive vector from body frame to local NED frame
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        repulse_x_ned = repulse_fwd * cos_yaw - repulse_left * sin_yaw
        repulse_y_ned = repulse_fwd * sin_yaw + repulse_left * cos_yaw

        # Blend
        vx_out = speed_scale * vx_desired + AVOIDANCE_GAIN * repulse_x_ned
        vy_out = speed_scale * vy_desired + AVOIDANCE_GAIN * repulse_y_ned

        # Clamp speed
        speed = np.sqrt(vx_out**2 + vy_out**2)
        if speed > MAX_SPEED:
            vx_out = vx_out / speed * MAX_SPEED
            vy_out = vy_out / speed * MAX_SPEED

        return vx_out, vy_out

    def _avoidance_loop(self):
        # Safety: if no command received recently, publish zero velocity
        if self.latest_cmd is None or self.latest_cmd_time is None:
            return

        elapsed = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if elapsed > TIMEOUT_SEC:
            # Timeout — send zero velocity to hold position
            cmd = PositionTarget()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            cmd.type_mask = (
                PositionTarget.IGNORE_PX |
                PositionTarget.IGNORE_PY |
                PositionTarget.IGNORE_PZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            cmd.velocity.x = 0.0
            cmd.velocity.y = 0.0
            cmd.velocity.z = 0.0
            cmd.yaw = self.current_yaw
            self.cmd_pub.publish(cmd)
            return

        vx = self.latest_cmd.velocity.x
        vy = self.latest_cmd.velocity.y
        vz = self.latest_cmd.velocity.z
        yaw = self.latest_cmd.yaw

        # Apply avoidance if depth data available
        if self.latest_depth is not None:
            sectors = self._depth_to_sectors(self.latest_depth)
            vx, vy = self._compute_avoidance(sectors, vx, vy)

            min_dist = np.min(sectors)
            if min_dist < OBSTACLE_SLOW_DIST:
                self.get_logger().info(
                    f'AVOIDING - closest: {min_dist:.2f}m, '
                    f'output: ({vx:.2f}, {vy:.2f})')

        # Publish modified command
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.yaw = yaw

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()