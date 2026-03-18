#!/usr/bin/env python3
"""Build a local 2D occupancy grid from stereo depth.

This node keeps the full 2D grid for RViz/debugging, but the flight decision is
based only on three forward sectors: left, center, and right.  Vertical up/down
structure is ignored outside a fixed body-height slice.
"""

from __future__ import annotations

from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from parking_enforcement.config import (
    AVOID_TOPIC,
    CENTER_HALF_WIDTH,
    CENTER_MIN_KNOWN_RATIO,
    CENTER_OCC_THRESHOLD,
    GRID_FRAME,
    GRID_RESOLUTION,
    GRID_X_MAX,
    GRID_X_MIN,
    GRID_Y_MAX,
    GRID_Y_MIN,
    HYSTERESIS_LEN,
    LOCAL_GRID_TOPIC,
    POINT_MAX_DEPTH,
    POINT_MIN_DEPTH,
    POINT_STRIDE,
    POINT_Z_MAX,
    POINT_Z_MIN,
    SECTOR_X_MAX,
    SECTOR_X_MIN,
    SIDE_HALF_WIDTH,
    SIDE_MIN_KNOWN_RATIO,
    SIDE_OCC_THRESHOLD,
    DEPTH_INFO_TOPIC,
    DEPTH_TOPIC,
)


class LocalCostmapNode(Node):
    """Convert cropped stereo depth into a rolling occupancy grid and sector decision."""

    def __init__(self):
        super().__init__('rob498_drone_8_local_costmap')
        self.bridge = CvBridge()

        # Camera intrinsics come from the cropped depth camera_info topic.
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Precompute grid geometry once so callbacks stay lightweight.
        self.grid_width = int(round((GRID_X_MAX - GRID_X_MIN) / GRID_RESOLUTION))
        self.grid_height = int(round((GRID_Y_MAX - GRID_Y_MIN) / GRID_RESOLUTION))
        self.origin_row = int(round((0.0 - GRID_Y_MIN) / GRID_RESOLUTION))

        # Hysteresis prevents the avoid direction from flipping every frame.
        self.last_cmds = deque(maxlen=HYSTERESIS_LEN)
        self.last_dir = 'BLOCKED'
        self.last_sector_log_time = self.get_clock().now()

        self.grid_pub = self.create_publisher(OccupancyGrid, LOCAL_GRID_TOPIC, 10)
        self.avoid_pub = self.create_publisher(String, AVOID_TOPIC, 10)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._camera_info_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, qos_profile_sensor_data)

        self.get_logger().info('Local costmap node started')

    def _camera_info_cb(self, msg: CameraInfo):
        """Cache the current depth intrinsics for fast back-projection."""

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _depth_cb(self, msg: Image):
        """Convert depth into a 2D grid and a left/center/right avoid command."""

        if self.fx is None:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        pts_body = self._depth_to_body_points(depth)
        grid = self._build_grid(pts_body)
        avoid_dir, center_stats, left_stats, right_stats = self._choose_direction(grid)
        self._publish_grid(grid, msg.header.stamp)
        self._publish_direction(avoid_dir)
        self._log_sector_stats(avoid_dir, center_stats, left_stats, right_stats)

    def _depth_to_body_points(self, depth: np.ndarray) -> np.ndarray:
        """Back-project depth pixels into a body-frame slice used for 2D occupancy.

        With the stereo pair mounted on the front, we treat camera forward as body
        forward and keep only points near the drone body height.
        """

        height, width = depth.shape
        vs, us = np.mgrid[0:height:POINT_STRIDE, 0:width:POINT_STRIDE]

        z = depth[vs, us]
        valid = np.isfinite(z) & (z > POINT_MIN_DEPTH) & (z < POINT_MAX_DEPTH)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        us = us[valid].astype(np.float32)
        vs = vs[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        # OpenCV camera frame is x-right, y-down, z-forward.
        x_cam = (us - self.cx) * z / self.fx
        y_cam = (vs - self.cy) * z / self.fy

        # Convert to a simple body frame: x-forward, y-left, z-up.
        x_body = z
        y_body = -x_cam
        z_body = -y_cam
        pts_body = np.stack([x_body, y_body, z_body], axis=1)

        keep = (
            (pts_body[:, 0] >= GRID_X_MIN) & (pts_body[:, 0] < GRID_X_MAX) &
            (pts_body[:, 1] >= GRID_Y_MIN) & (pts_body[:, 1] < GRID_Y_MAX) &
            (pts_body[:, 2] >= POINT_Z_MIN) & (pts_body[:, 2] <= POINT_Z_MAX)
        )
        return pts_body[keep]

    def _build_grid(self, pts_body: np.ndarray) -> np.ndarray:
        """Rasterize the filtered 3D points into a local 2D occupancy grid.

        Unknown cells remain -1, free cells become 0 through raytracing, and hit
        cells become 100.
        """

        grid = -np.ones((self.grid_height, self.grid_width), dtype=np.int8)
        if pts_body.size == 0:
            return grid

        cols = ((pts_body[:, 0] - GRID_X_MIN) / GRID_RESOLUTION).astype(np.int32)
        rows = ((pts_body[:, 1] - GRID_Y_MIN) / GRID_RESOLUTION).astype(np.int32)
        keep = (
            (rows >= 0) & (rows < self.grid_height) &
            (cols >= 0) & (cols < self.grid_width)
        )
        rows = rows[keep]
        cols = cols[keep]
        if rows.size == 0:
            return grid

        unique_cells = np.unique(np.stack([rows, cols], axis=1), axis=0)
        for row, col in unique_cells:
            # Mark the observed line of sight as free, then the hit cell as occupied.
            ray = self._bresenham(self.origin_row, 0, int(row), int(col))
            for ray_row, ray_col in ray[:-1]:
                if 0 <= ray_row < self.grid_height and 0 <= ray_col < self.grid_width:
                    if grid[ray_row, ray_col] == -1:
                        grid[ray_row, ray_col] = 0
            grid[int(row), int(col)] = 100
        return grid

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int):
        """Return the grid cells along a ray so we can carve free space."""

        points = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr
        row, col = r0, c0

        while True:
            points.append((row, col))
            if row == r1 and col == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                col += sc
            if e2 < dc:
                err += dc
                row += sr
        return points

    def _choose_direction(self, grid: np.ndarray):
        """Use only the left/center/right forward sectors for avoidance decisions."""

        center_stats = self._window_stats(grid, SECTOR_X_MIN, SECTOR_X_MAX, -CENTER_HALF_WIDTH, CENTER_HALF_WIDTH)
        left_stats = self._window_stats(grid, SECTOR_X_MIN, SECTOR_X_MAX, CENTER_HALF_WIDTH, SIDE_HALF_WIDTH)
        right_stats = self._window_stats(grid, SECTOR_X_MIN, SECTOR_X_MAX, -SIDE_HALF_WIDTH, -CENTER_HALF_WIDTH)

        center_clear = (
            center_stats['known_ratio'] >= CENTER_MIN_KNOWN_RATIO
            and center_stats['occ_ratio'] < CENTER_OCC_THRESHOLD
        )
        left_open = (
            left_stats['known_ratio'] >= SIDE_MIN_KNOWN_RATIO
            and left_stats['occ_ratio'] < SIDE_OCC_THRESHOLD
        )
        right_open = (
            right_stats['known_ratio'] >= SIDE_MIN_KNOWN_RATIO
            and right_stats['occ_ratio'] < SIDE_OCC_THRESHOLD
        )

        if center_clear:
            raw = 'CLEAR'
        elif left_open and right_open:
            # When both sides are usable, keep the currently active side if it is
            # still competitive.  This gives smoother sidesteps around obstacles.
            if self.last_dir in ('LEFT', 'RIGHT'):
                if self.last_dir == 'LEFT' and left_stats['occ_ratio'] <= right_stats['occ_ratio'] + 0.02:
                    raw = 'LEFT'
                elif self.last_dir == 'RIGHT' and right_stats['occ_ratio'] <= left_stats['occ_ratio'] + 0.02:
                    raw = 'RIGHT'
                else:
                    raw = 'LEFT' if left_stats['occ_ratio'] <= right_stats['occ_ratio'] else 'RIGHT'
            else:
                raw = 'LEFT' if left_stats['occ_ratio'] <= right_stats['occ_ratio'] else 'RIGHT'
        elif left_open:
            raw = 'LEFT'
        elif right_open:
            raw = 'RIGHT'
        else:
            # If center is not clear and neither side has enough known free space,
            # the mission node will stop or back up before re-evaluating.
            raw = 'BLOCKED'

        self.last_cmds.append(raw)
        if len(self.last_cmds) < self.last_cmds.maxlen:
            self.last_dir = raw
            return raw, center_stats, left_stats, right_stats

        counts = {cmd: self.last_cmds.count(cmd) for cmd in set(self.last_cmds)}
        stable = max(counts, key=counts.get)
        self.last_dir = stable
        return stable, center_stats, left_stats, right_stats

    def _window_stats(self, grid: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> dict[str, float]:
        """Compute how much of a sector is known and how much of it is occupied."""

        c0 = int((x0 - GRID_X_MIN) / GRID_RESOLUTION)
        c1 = int((x1 - GRID_X_MIN) / GRID_RESOLUTION)
        r0 = int((y0 - GRID_Y_MIN) / GRID_RESOLUTION)
        r1 = int((y1 - GRID_Y_MIN) / GRID_RESOLUTION)

        c0 = max(0, min(grid.shape[1], c0))
        c1 = max(0, min(grid.shape[1], c1))
        r0 = max(0, min(grid.shape[0], r0))
        r1 = max(0, min(grid.shape[0], r1))
        if r1 <= r0 or c1 <= c0:
            return {'known_ratio': 0.0, 'occ_ratio': 1.0}

        window = grid[r0:r1, c0:c1]
        total = float(window.size)
        if total <= 0.0:
            return {'known_ratio': 0.0, 'occ_ratio': 1.0}

        known = window != -1
        known_count = float(np.count_nonzero(known))
        known_ratio = known_count / total
        if known_count <= 0.0:
            return {'known_ratio': 0.0, 'occ_ratio': 1.0}

        occ_ratio = float(np.mean(window[known] == 100))
        return {'known_ratio': known_ratio, 'occ_ratio': occ_ratio}

    def _publish_grid(self, grid: np.ndarray, stamp):
        """Publish the local occupancy grid for RViz and debugging."""

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = GRID_FRAME
        msg.info.resolution = GRID_RESOLUTION
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = GRID_X_MIN
        msg.info.origin.position.y = GRID_Y_MIN
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        self.grid_pub.publish(msg)

    def _publish_direction(self, avoid_dir: str):
        """Publish the discrete avoidance result for the mission node."""

        msg = String()
        msg.data = avoid_dir
        self.avoid_pub.publish(msg)

    def _log_sector_stats(self, avoid_dir: str, center_stats: dict[str, float], left_stats: dict[str, float], right_stats: dict[str, float]):
        """Log sector occupancy at a low rate so tuning stays observable in flight tests."""

        now = self.get_clock().now()
        if (now - self.last_sector_log_time).nanoseconds <= 1e9:
            return

        self.get_logger().info(
            '[SECTORS] '
            f'center(k={center_stats["known_ratio"]:.2f}, o={center_stats["occ_ratio"]:.2f}) '
            f'left(k={left_stats["known_ratio"]:.2f}, o={left_stats["occ_ratio"]:.2f}) '
            f'right(k={right_stats["known_ratio"]:.2f}, o={right_stats["occ_ratio"]:.2f}) '
            f'-> {avoid_dir}'
        )
        self.last_sector_log_time = now


def main(args=None):
    """Spin the local costmap node until shutdown."""

    rclpy.init(args=args)
    node = LocalCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
