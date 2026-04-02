import cv2
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import PoseStamped
import numpy as np
from cv_bridge import CvBridge
from scipy.ndimage import binary_erosion


OCCUPANCY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

MIN_VALID_PX = 8
PERCENTILE = 90

class OccupancyNode(Node):
    """
    Converts a stereo disparity image into a 2-D occupancy grid (bird's-eye view).

    Grid layout
    -----------
    row 0    -> Z = 0        (drone / camera position)
    row D-1  -> Z = max_depth
    col 0    -> X = -(W/2)*res  (left)
    col W/2  -> X = 0           (straight ahead)
    col W-1  -> X = +(W/2)*res  (right)

    Algorithm (noise-robust)
    ------------------------
    1. Crop to a horizontal band of rows centred on the image midpoint.
       This isolates the height range that corresponds to drone-level obstacles.
    2. Per column, take the 90th-percentile of valid disparities in that band.
       A single percentile is far more robust to speckle noise than per-pixel
       back-projection, because outlier pixels get outvoted.
    3. Convert each column's representative disparity -> depth via Z = f*B/d,
       lateral offset via X = (col - cx)*Z/f.
    4. Vote the (X, Z) point into the occupancy grid.
    """

    def __init__(self):
        super().__init__('occupancy_node')

        self.declare_parameter('grid_resolution', 0.1)   # m/cell
        self.declare_parameter('grid_width_cells', 65)  # lateral  (6.5 m)
        self.declare_parameter('grid_depth_cells', 65)  # forward  (6.5 m)
        self.declare_parameter('max_depth', 4.0)         # ignore beyond this (m)
        self.declare_parameter('height_band_frac', 0.25) # +/- frac of H centred on midrow
        self.declare_parameter('left_crop_pixels', 62)   # invalid left disparity strip
        self.declare_parameter('debug_logging', True)
        self.declare_parameter('debug_log_period_s', 1.0)

        self.res = self.get_parameter('grid_resolution').value
        self.grid_w = self.get_parameter('grid_width_cells').value
        self.grid_d = self.get_parameter('grid_depth_cells').value
        self.max_depth = self.get_parameter('max_depth').value
        self.hband = self.get_parameter('height_band_frac').value
        self.left_crop = self.get_parameter('left_crop_pixels').value
        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        self.debug_log_period_s = float(self.get_parameter('debug_log_period_s').value)

        self.bridge = CvBridge()
        self.cx = None   # Set from CameraInfo; falls back to W/2 until received.
        self.frame_count = 0
        self.last_debug_log_time = None
        self.last_occ_pub_time = None
        self.last_occ_rate_hz = None
        self.current_pose = None
        self.target_pose = None

        self.cam_info_sub = self.create_subscription(
            CameraInfo, 'disparity/camera_info', self._cam_info_cb, qos_profile_sensor_data)
        self.disparity_sub = self.create_subscription(
            DisparityImage, 'disparity', self.disparity_callback, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self._pose_cb, qos_profile_sensor_data)
        self.target_sub = self.create_subscription(
            PoseStamped, 'rob498_drone_8/cmd_pose', self._target_cb, 10)

        # Latest-only QoS keeps the planner reacting to the newest local obstacle view.
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, 'occupancy_node/occupancy_grid', OCCUPANCY_QOS)
        self.viz_pub = self.create_publisher(
            Image, 'occupancy_node/viz', qos_profile_sensor_data)

        self.get_logger().info('OccupancyNode ready.')

    # ------------------------------------------------------------------
    def _cam_info_cb(self, msg: CameraInfo):
        # K is row-major: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.cx = msg.k[2]

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _target_cb(self, msg: PoseStamped):
        self.target_pose = msg

    # ------------------------------------------------------------------
    def disparity_callback(self, msg: DisparityImage):
        f = float(msg.f)
        B = abs(float(msg.t))
        if f == 0.0 or B == 0.0:
            self.get_logger().warn('Invalid camera parameters in disparity message.')
            return

        disp = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        H, W = disp.shape
        cx = self.cx if self.cx is not None else W / 2.0

        # 1. Height-band ROI.
        half = int(H * self.hband)
        cy = H // 2
        roi = disp[max(0, cy - half): min(H, cy + half), :].astype(np.float32)
        roi[(roi <= msg.min_disparity) | ~np.isfinite(roi)] = np.nan
        # Zero out the invalid left disparity strip.
        roi[:, :self.left_crop] = np.nan

        # 2. Per-column Xth-percentile disparity.
        # Suppress RuntimeWarning for all-NaN columns in the invalid strip.
        with np.errstate(all='ignore'):
            col_disp = np.nanpercentile(roi, PERCENTILE, axis=0)
        col_count = np.sum(np.isfinite(roi), axis=0)
        valid_cols = np.where(np.isfinite(col_disp) & (col_disp > msg.min_disparity) & (col_count >= MIN_VALID_PX))[0]

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info(f'First disparity received: {W}x{H}, f={f:.1f}, B={B:.4f}')
        if self.frame_count % 50 == 0:
            self.get_logger().info(f'Frame {self.frame_count}: valid_cols={valid_cols.size}/{W}')

        grid = np.zeros((self.grid_d, self.grid_w), dtype=np.int8)

        if valid_cols.size > 0:
            d = col_disp[valid_cols]

            # 3. Back-project -> camera-frame (X, Z).
            Z = f * B / d
            X = (valid_cols - cx) * Z / f

            # 4. Vote into grid.
            mask = (Z > 0.0) & (Z < self.max_depth)
            col_idx = (X[mask] / self.res + self.grid_w / 2.0).astype(int)
            row_idx = (Z[mask] / self.res).astype(int)

            in_bounds = (
                (col_idx >= 0) & (col_idx < self.grid_w) &
                (row_idx >= 0) & (row_idx < self.grid_d)
            )
            grid[row_idx[in_bounds], col_idx[in_bounds]] = 100

        # 5. Erode to remove isolated noise cells.
        #    A cell survives only if its 3x3 neighbourhood is fully occupied.
        #occupied = grid == 100
        #grid = np.where(binary_erosion(occupied), np.int8(100), np.int8(0))

        now = self.get_clock().now()
        if self.last_occ_pub_time is not None:
            period_s = (now - self.last_occ_pub_time).nanoseconds / 1e9
            if period_s > 0.0:
                self.last_occ_rate_hz = 1.0 / period_s
        self.last_occ_pub_time = now

        self.occupancy_pub.publish(self._build_msg(grid, msg))
        _, viz_msg = self._build_viz(grid, msg, self.current_pose, self.target_pose)
        self.viz_pub.publish(viz_msg)

        self._log_occupancy_summary(grid, valid_cols.size, W)

    # ------------------------------------------------------------------
    def _log_occupancy_summary(self, grid: np.ndarray, valid_col_count: int, image_width: int):
        if not self._should_log_debug():
            return

        occupied_cells, nearest_dist_m, nearest_offset_m, nearest_side = self._grid_summary(grid)
        nearest_dist_text = f'{nearest_dist_m:.2f}' if np.isfinite(nearest_dist_m) else 'inf'
        hz_text = f'{self.last_occ_rate_hz:.2f}' if self.last_occ_rate_hz is not None else 'n/a'
        self.get_logger().info(
            '[OCC] '
            f'hz={hz_text} '
            f'valid_cols={valid_col_count}/{image_width} '
            f'occ_cells={occupied_cells} '
            f'closest={nearest_side}@{nearest_dist_text}m '
            f'offset={nearest_offset_m:.2f}'
        )

    def _should_log_debug(self):
        if not self.debug_logging:
            return False

        now = self.get_clock().now()
        if self.last_debug_log_time is None:
            self.last_debug_log_time = now
            return True

        elapsed_s = (now - self.last_debug_log_time).nanoseconds / 1e9
        if elapsed_s >= self.debug_log_period_s:
            self.last_debug_log_time = now
            return True

        return False

    def _grid_summary(self, grid: np.ndarray):
        # Scan the already-built grid once to keep debug overhead low.
        occ_rows, occ_cols = np.where(grid == 100)
        occupied_cells = int(occ_rows.size)
        if occupied_cells == 0:
            return occupied_cells, float('inf'), 0.0, 'none'

        nearest_row = int(np.min(occ_rows))
        nearest_cols = occ_cols[occ_rows == nearest_row]
        center = self.grid_w / 2.0
        chosen_col = nearest_cols[np.argmin(np.abs(nearest_cols - center))]

        nearest_dist_m = (nearest_row + 0.5) * self.res
        nearest_offset_m = (float(chosen_col) - center) * self.res
        nearest_side = self._side_from_offset(nearest_offset_m)
        return occupied_cells, nearest_dist_m, nearest_offset_m, nearest_side

    def _side_from_offset(self, offset_m: float):
        if abs(offset_m) <= (self.res / 2.0):
            return 'center'
        return 'left' if offset_m < 0.0 else 'right'

    # ------------------------------------------------------------------
    def _build_viz(self, grid: np.ndarray, src: DisparityImage,
                   current_pose: PoseStamped, target_pose: PoseStamped):
        """
        Bird's-eye view image of the occupancy grid.
        - Dark grey : free space
        - Red       : occupied cell
        - Green     : drone position (bottom-centre)
        - Cyan X    : next waypoint projected into drone body frame
        - Grey      : centre line (straight ahead)
        Grid is flipped so the drone is at the bottom and far distance at top.
        Each cell is drawn as CELL_PX x CELL_PX pixels for readability.
        """
        CELL_PX = 5

        H = self.grid_d * CELL_PX
        W = self.grid_w * CELL_PX
        # Dark grey background - distinguishable from "no image" black.
        img = np.full((H, W, 3), 40, dtype=np.uint8)

        # Occupied cells -> red  (grid row 0 = near = image bottom after flip).
        occ_rows, occ_cols = np.where(grid == 100)
        for r, c in zip(occ_rows, occ_cols):
            y0 = r * CELL_PX
            x0 = c * CELL_PX
            img[y0:y0 + CELL_PX, x0:x0 + CELL_PX] = (0, 0, 220)

        # Flip so drone (row 0) is at the bottom.
        img = cv2.flip(img, 0)

        # Centre line (straight ahead).
        cx_px = (self.grid_w // 2) * CELL_PX
        cv2.line(img, (cx_px, 0), (cx_px, H - 1), (80, 80, 80), 1)

        # Drone marker - filled green circle at bottom-centre.
        drone_x = (self.grid_w // 2) * CELL_PX
        drone_y = H - CELL_PX - 1
        cv2.circle(img, (drone_x, drone_y), CELL_PX, (0, 220, 0), -1)

        # Waypoint marker - cyan X projected into drone body frame.
        if current_pose is not None and target_pose is not None:
            yaw = _yaw_from_quat(current_pose.pose.orientation)
            dx = target_pose.pose.position.x - current_pose.pose.position.x
            dy = target_pose.pose.position.y - current_pose.pose.position.y
            # Rotate map-frame delta into drone body frame (same convention as planner).
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            right = dx * math.sin(yaw) - dy * math.cos(yaw)
            # Convert body-frame offsets to grid indices.
            wp_grid_row = int(fwd / self.res)
            wp_grid_col = int(self.grid_w / 2.0 + right / self.res)
            if 0 <= wp_grid_row < self.grid_d and 0 <= wp_grid_col < self.grid_w:
                # After flip: image_y = H - 1 - (grid_row * CELL_PX + CELL_PX//2)
                wp_img_x = wp_grid_col * CELL_PX + CELL_PX // 2
                wp_img_y = H - 1 - (wp_grid_row * CELL_PX + CELL_PX // 2)
                cv2.drawMarker(img, (wp_img_x, wp_img_y),
                               (220, 220, 0), cv2.MARKER_CROSS, CELL_PX * 4, 2)

        # Text overlay: obstacle cell count and camera info status.
        n_occ = int(np.sum(grid == 100))
        cx_status = f'cx={self.cx:.1f}' if self.cx is not None else 'cx=W/2'
        label = f'obs:{n_occ}  {cx_status}'
        cv2.putText(img, label, (4, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        viz_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        viz_msg.header = src.header
        return img, viz_msg

    # ------------------------------------------------------------------
    def _build_msg(self, grid: np.ndarray, src: DisparityImage) -> OccupancyGrid:
        occ = OccupancyGrid()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = src.header.frame_id
        occ.info.resolution = self.res
        occ.info.width = self.grid_w
        occ.info.height = self.grid_d
        occ.info.origin.position.x = -(self.grid_w / 2.0) * self.res
        occ.info.origin.position.y = 0.0
        occ.info.origin.position.z = 0.0
        occ.info.origin.orientation.w = 1.0
        occ.data = grid.flatten().tolist()
        return occ


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
