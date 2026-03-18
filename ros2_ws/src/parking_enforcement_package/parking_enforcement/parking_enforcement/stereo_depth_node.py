#!/usr/bin/env python3
"""Self-contained stereo depth node for the parking package.

This node computes cropped stereo depth directly inside the parking package so
the local costmap can consume `/stereo/depth` and `/stereo/depth/camera_info`
without depending on the separate perception package.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from parking_enforcement.config import (
    DEPTH_INFO_TOPIC,
    DEPTH_TOPIC,
    DISPARITY_VIS_TOPIC,
    DOWNSCALE,
    LEFT_CAMERA_INFO_TOPIC,
    LEFT_CROP_EXTRA_PADDING_PIXELS,
    LEFT_CROP_MIN_OVERLAP_RATIO,
    LEFT_CROP_OVERRIDE_PIXELS,
    LEFT_IMAGE_TOPIC,
    MAX_DEPTH_METERS,
    MIN_DEPTH_METERS,
    MIN_PUBLISHED_WIDTH_PIXELS,
    NUM_DISPARITIES,
    RIGHT_CAMERA_INFO_TOPIC,
    RIGHT_IMAGE_TOPIC,
    SKIP_FRAMES,
    STEREO_FRAME_ID,
    VERTICAL_OFFSET_PIXELS,
)


class StereoDepthNode(Node):
    """Produce cropped stereo depth, disparity visualization, and camera info."""

    def __init__(self):
        super().__init__('rob498_drone_8_stereo_depth')
        self.bridge = CvBridge()

        # Intrinsics and rectification data are filled once the camera drivers and
        # TF tree are online.
        self.K1 = None
        self.D1 = None
        self.K2 = None
        self.D2 = None
        self.image_size = None

        self.left_xmap = None
        self.left_ymap = None
        self.right_xmap = None
        self.right_ymap = None
        self.depth_camera_info = None

        self.scale = DOWNSCALE
        self.vertical_offset = VERTICAL_OFFSET_PIXELS
        self.crop_start_col = 0
        self.output_width = 0
        self.output_height = 0
        self.baseline = None
        self.fx_scaled = None
        self.fy_scaled = None
        self.cx_scaled = None
        self.cy_scaled = None
        self.min_disp = None
        self.max_disp = None
        self.maps_computed = False

        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=NUM_DISPARITIES,
            blockSize=5,
            P1=8 * 3 * 5 * 5,
            P2=32 * 3 * 5 * 5,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=25,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, LEFT_CAMERA_INFO_TOPIC, self._cam1_intrinsics_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, RIGHT_CAMERA_INFO_TOPIC, self._cam2_intrinsics_cb, qos_profile_sensor_data)

        left_sub = Subscriber(self, Image, LEFT_IMAGE_TOPIC, qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(self, Image, RIGHT_IMAGE_TOPIC, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._sync_cb)

        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        self.disp_vis_pub = self.create_publisher(Image, DISPARITY_VIS_TOPIC, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, DEPTH_INFO_TOPIC, 10)

        self.setup_timer = self.create_timer(0.5, self._try_compute_maps)
        self.frame_count = 0
        self.process_every_n = SKIP_FRAMES

        self.get_logger().info('Stereo depth node waiting for camera info and TF...')

    def _cam1_intrinsics_cb(self, msg: CameraInfo):
        """Cache the left fisheye intrinsics the first time they arrive."""

        if self.K1 is not None:
            return
        self.K1 = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D1 = np.array(msg.d[:4], dtype=np.float64)
        self.image_size = (msg.width, msg.height)
        self.get_logger().info('Got left fisheye camera info')

    def _cam2_intrinsics_cb(self, msg: CameraInfo):
        """Cache the right fisheye intrinsics the first time they arrive."""

        if self.K2 is not None:
            return
        self.K2 = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D2 = np.array(msg.d[:4], dtype=np.float64)
        self.get_logger().info('Got right fisheye camera info')

    def _try_compute_maps(self):
        """Build rectification maps and the cropped output camera model once TF is ready."""

        if self.maps_computed or self.K1 is None or self.K2 is None or self.image_size is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                STEREO_FRAME_ID,
                'camera_fisheye2_optical_frame',
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warn(f'TF not ready yet: {exc}')
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        T = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        R = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()

        self.baseline = float(abs(T[0]))
        if self.baseline < 1e-6:
            self.get_logger().error('Baseline is zero. Check stereo TF before flying.')
            return

        half_rotvec = Rotation.from_matrix(R).as_rotvec() / 2.0
        R1 = Rotation.from_rotvec(half_rotvec).as_matrix()
        R2 = Rotation.from_rotvec(-half_rotvec).as_matrix()

        K1_scaled = self.K1.copy()
        K2_scaled = self.K2.copy()
        for K in (K1_scaled, K2_scaled):
            K[0, 0] *= self.scale
            K[1, 1] *= self.scale
            K[0, 2] *= self.scale
            K[1, 2] *= self.scale

        scaled_size = (int(self.image_size[0] * self.scale), int(self.image_size[1] * self.scale))
        self.left_xmap, self.left_ymap = cv2.fisheye.initUndistortRectifyMap(
            self.K1, self.D1, R1, K1_scaled, scaled_size, cv2.CV_32FC1
        )
        self.right_xmap, self.right_ymap = cv2.fisheye.initUndistortRectifyMap(
            self.K2, self.D2, R2, K2_scaled, scaled_size, cv2.CV_32FC1
        )

        self.crop_start_col = self._estimate_left_crop(scaled_size)
        self.output_width = scaled_size[0] - self.crop_start_col
        self.output_height = scaled_size[1]
        if self.output_width < MIN_PUBLISHED_WIDTH_PIXELS:
            self.get_logger().error('Stereo crop removed too much width. Reduce the crop override.')
            return

        self.fx_scaled = float(K1_scaled[0, 0])
        self.fy_scaled = float(K1_scaled[1, 1])
        self.cx_scaled = float(K1_scaled[0, 2]) - float(self.crop_start_col)
        self.cy_scaled = float(K1_scaled[1, 2])
        self.min_disp = (self.fx_scaled * self.baseline) / MAX_DEPTH_METERS
        self.max_disp = (self.fx_scaled * self.baseline) / MIN_DEPTH_METERS
        self.warp_M = np.float32([[1, 0, 0], [0, 1, self.vertical_offset]])

        self.depth_camera_info = CameraInfo()
        self.depth_camera_info.header.frame_id = STEREO_FRAME_ID
        self.depth_camera_info.width = self.output_width
        self.depth_camera_info.height = self.output_height
        self.depth_camera_info.distortion_model = 'plumb_bob'
        self.depth_camera_info.d = [0.0] * 5
        self.depth_camera_info.k = [
            self.fx_scaled, 0.0, self.cx_scaled,
            0.0, self.fy_scaled, self.cy_scaled,
            0.0, 0.0, 1.0,
        ]
        self.depth_camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        self.depth_camera_info.p = [
            self.fx_scaled, 0.0, self.cx_scaled, 0.0,
            0.0, self.fy_scaled, self.cy_scaled, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.depth_camera_info.roi.x_offset = self.crop_start_col
        self.depth_camera_info.roi.y_offset = 0
        self.depth_camera_info.roi.width = self.output_width
        self.depth_camera_info.roi.height = self.output_height
        self.depth_camera_info.roi.do_rectify = True

        self.maps_computed = True
        self.setup_timer.cancel()
        self.get_logger().info(
            f'Stereo depth ready. Baseline={self.baseline:.4f} m, '
            f'crop_left={self.crop_start_col} px, output={self.output_width}x{self.output_height}'
        )

    def _estimate_left_crop(self, scaled_size: tuple[int, int]) -> int:
        """Estimate the invalid left strip created by fisheye rectification."""

        width, _ = scaled_size
        if LEFT_CROP_OVERRIDE_PIXELS is not None:
            return max(0, min(int(LEFT_CROP_OVERRIDE_PIXELS), width - MIN_PUBLISHED_WIDTH_PIXELS))

        valid_left = (
            (self.left_xmap >= 0.0) & (self.left_xmap < self.image_size[0] - 1) &
            (self.left_ymap >= 0.0) & (self.left_ymap < self.image_size[1] - 1)
        )
        valid_right = (
            (self.right_xmap >= 0.0) & (self.right_xmap < self.image_size[0] - 1) &
            (self.right_ymap >= 0.0) & (self.right_ymap < self.image_size[1] - 1)
        )
        overlap_ratio = np.mean(valid_left & valid_right, axis=0)
        good_cols = np.where(overlap_ratio >= LEFT_CROP_MIN_OVERLAP_RATIO)[0]
        if good_cols.size == 0:
            return 0

        crop = int(good_cols[0])
        if crop > 0:
            crop += LEFT_CROP_EXTRA_PADDING_PIXELS
        return max(0, min(crop, width - MIN_PUBLISHED_WIDTH_PIXELS))

    def _sync_cb(self, left_msg: Image, right_msg: Image):
        """Rectify, crop, compute disparity/depth, and publish the cropped outputs."""

        if not self.maps_computed:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        left_rect = cv2.remap(left, self.left_xmap, self.left_ymap, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.right_xmap, self.right_ymap, cv2.INTER_LINEAR)
        right_rect = cv2.warpAffine(right_rect, self.warp_M, (right_rect.shape[1], right_rect.shape[0]))

        left_rect = self.clahe.apply(left_rect)
        right_rect = self.clahe.apply(right_rect)

        if self.crop_start_col > 0:
            left_rect = left_rect[:, self.crop_start_col:]
            right_rect = right_rect[:, self.crop_start_col:]

        disparity = self.stereo.compute(left_rect, right_rect).astype(np.float32) / 16.0
        valid_disp = (disparity > self.min_disp) & (disparity < self.max_disp)
        disparity = np.where(valid_disp, disparity, 0.0)

        depth = np.where(disparity > 0.0, (self.fx_scaled * self.baseline) / disparity, 0.0).astype(np.float32)
        depth = np.clip(depth, 0.0, MAX_DEPTH_METERS)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = left_msg.header
        depth_msg.header.frame_id = STEREO_FRAME_ID
        self.depth_pub.publish(depth_msg)

        info_msg = CameraInfo()
        info_msg.header = depth_msg.header
        info_msg.width = self.depth_camera_info.width
        info_msg.height = self.depth_camera_info.height
        info_msg.distortion_model = self.depth_camera_info.distortion_model
        info_msg.d = list(self.depth_camera_info.d)
        info_msg.k = list(self.depth_camera_info.k)
        info_msg.r = list(self.depth_camera_info.r)
        info_msg.p = list(self.depth_camera_info.p)
        info_msg.roi = self.depth_camera_info.roi
        self.depth_info_pub.publish(info_msg)

        if self.disp_vis_pub.get_subscription_count() > 0:
            disp_vis = np.clip(disparity, 0.0, NUM_DISPARITIES)
            disp_vis = (disp_vis / NUM_DISPARITIES * 255.0).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            disp_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding='bgr8')
            disp_msg.header = depth_msg.header
            self.disp_vis_pub.publish(disp_msg)


def main(args=None):
    """Spin the stereo depth node until shutdown."""

    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
