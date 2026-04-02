#!/usr/bin/env python3

import numpy as np
import warnings

# Avoid Jetson/OpenCV/NumPy subnormal warning spam
np.finfo(np.float32)
np.finfo(np.float64)
warnings.filterwarnings(
    "ignore",
    message="The value of the smallest subnormal for <class 'numpy.float(32|64)'> type is zero."
)

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

MIN_DEPTH = 0.1   # meters
MAX_DEPTH = 5.0   # meters
SKIP_FRAMES = 3   # process every Nth frame for performance
NUM_DISPARITIES = 64
LEFT_CROP_PIXELS = 62


class DepthNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.bridge = CvBridge()

        if not hasattr(cv2, "cuda"):
            raise RuntimeError("OpenCV was built without cv2.cuda")
        if cv2.cuda.getCudaEnabledDeviceCount() < 1:
            raise RuntimeError("No CUDA-enabled device found by OpenCV")
        if not hasattr(cv2.cuda, "createStereoSGM"):
            raise RuntimeError("cv2.cuda.createStereoSGM not available")

        cv2.cuda.setDevice(0)

        # Intrinsics
        self.K1 = None
        self.D1 = None
        self.K2 = None
        self.D2 = None
        self.image_size = None

        # Rectification maps
        self.maps_computed = False
        self.left_xmap = None
        self.left_ymap = None
        self.right_xmap = None
        self.right_ymap = None

        # GPU copies of rectification maps
        self.left_xmap_gpu = None
        self.left_ymap_gpu = None
        self.right_xmap_gpu = None
        self.right_ymap_gpu = None

        self.baseline = None
        self.min_disp = None
        self.max_disp = None

        # Keep original CPU resize
        self.scale = 0.5

        # Keep original CPU-style penalties as closely as possible
        block_size = 5
        p1 = 8 * 3 * block_size * block_size
        p2 = 32 * 3 * block_size * block_size

        # Use HH instead of HH4 for a less speckly result
        if hasattr(cv2.cuda, "StereoSGM_MODE_HH"):
            sgm_mode = cv2.cuda.StereoSGM_MODE_HH
        elif hasattr(cv2, "StereoSGBM_MODE_HH"):
            sgm_mode = cv2.StereoSGBM_MODE_HH
        else:
            sgm_mode = None

        if sgm_mode is None:
            self.stereo = cv2.cuda.createStereoSGM(
                0,
                NUM_DISPARITIES,
                p1,
                p2,
                5
            )
        else:
            self.stereo = cv2.cuda.createStereoSGM(
                0,
                NUM_DISPARITIES,
                p1,
                p2,
                5,
                sgm_mode
            )

        # Match original CPU params where available
        if hasattr(self.stereo, "setBlockSize"):
            self.stereo.setBlockSize(block_size)
        if hasattr(self.stereo, "setDisp12MaxDiff"):
            self.stereo.setDisp12MaxDiff(1)
        if hasattr(self.stereo, "setUniquenessRatio"):
            self.stereo.setUniquenessRatio(3)
        if hasattr(self.stereo, "setSpeckleWindowSize"):
            self.stereo.setSpeckleWindowSize(0)
        if hasattr(self.stereo, "setSpeckleRange"):
            self.stereo.setSpeckleRange(0)
        if hasattr(self.stereo, "setPreFilterCap"):
            self.stereo.setPreFilterCap(63)

        # Keep CPU CLAHE exactly like original
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Keep original vertical correction
        self.vertical_offset = -2

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera info subscriptions
        self.create_subscription(
            CameraInfo,
            '/camera/fisheye1/camera_info',
            self._cam1_intrinsics_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            '/camera/fisheye2/camera_info',
            self._cam2_intrinsics_cb,
            qos_profile_sensor_data
        )

        # Synced image subscriptions
        left_sub = Subscriber(
            self,
            Image,
            '/camera/fisheye1/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        right_sub = Subscriber(
            self,
            Image,
            '/camera/fisheye2/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self._sync_cb)

        # Publishers
        self.disp_pub = self.create_publisher(
            DisparityImage,
            'disparity',
            qos_profile_sensor_data
        )
        self.cam_info_pub = self.create_publisher(
            CameraInfo,
            'disparity/camera_info',
            qos_profile_sensor_data
        )
        self.disp_vis_pub = self.create_publisher(
            Image,
            '/stereo/disp_vis',
            10
        )

        # Timer to poll for TF readiness
        self.setup_timer = self.create_timer(0.5, self._try_compute_maps)

        # Frame skip
        self.frame_count = 0
        self.process_every_n = SKIP_FRAMES
        self.crop_start_col = 0

        self.get_logger().info(
            f'Using OpenCV CUDA on Jetson, devices={cv2.cuda.getCudaEnabledDeviceCount()}'
        )
        self.get_logger().info('Waiting for camera info and TF...')

    def _cam1_intrinsics_cb(self, msg):
        if self.K1 is not None:
            return
        self.K1 = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D1 = np.array(msg.d[:4], dtype=np.float64)
        self.image_size = (msg.width, msg.height)
        self.get_logger().info('Got fisheye1 camera info')

    def _cam2_intrinsics_cb(self, msg):
        if self.K2 is not None:
            return
        self.K2 = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D2 = np.array(msg.d[:4], dtype=np.float64)
        self.get_logger().info('Got fisheye2 camera info')

    def _try_compute_maps(self):
        if self.maps_computed:
            return

        if self.K1 is None or self.K2 is None or self.image_size is None:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'camera_fisheye1_optical_frame',
                'camera_fisheye2_optical_frame',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return

        trans = t.transform.translation
        rot = t.transform.rotation
        T = np.array([trans.x, trans.y, trans.z], dtype=np.float64)
        R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

        self.baseline = abs(T[0])

        # Split rotation evenly between both cameras for rectification
        r = Rotation.from_matrix(R)
        half_angle = r.as_rotvec() / 2.0
        R1 = Rotation.from_rotvec(half_angle).as_matrix()
        R2 = Rotation.from_rotvec(-half_angle).as_matrix()

        # Scaled intrinsics for downscaled images
        K1_scaled = self.K1.copy()
        K1_scaled[0, 0] *= self.scale
        K1_scaled[1, 1] *= self.scale
        K1_scaled[0, 2] *= self.scale
        K1_scaled[1, 2] *= self.scale

        K2_scaled = self.K2.copy()
        K2_scaled[0, 0] *= self.scale
        K2_scaled[1, 1] *= self.scale
        K2_scaled[0, 2] *= self.scale
        K2_scaled[1, 2] *= self.scale

        scaled_size = (
            int(self.image_size[0] * self.scale),
            int(self.image_size[1] * self.scale),
        )

        self.left_xmap, self.left_ymap = cv2.fisheye.initUndistortRectifyMap(
            self.K1, self.D1, R1, K1_scaled, scaled_size, cv2.CV_32FC1
        )
        self.right_xmap, self.right_ymap = cv2.fisheye.initUndistortRectifyMap(
            self.K2, self.D2, R2, K2_scaled, scaled_size, cv2.CV_32FC1
        )

        # Upload maps once
        self.left_xmap_gpu = cv2.cuda_GpuMat()
        self.left_ymap_gpu = cv2.cuda_GpuMat()
        self.right_xmap_gpu = cv2.cuda_GpuMat()
        self.right_ymap_gpu = cv2.cuda_GpuMat()

        self.left_xmap_gpu.upload(self.left_xmap)
        self.left_ymap_gpu.upload(self.left_ymap)
        self.right_xmap_gpu.upload(self.right_xmap)
        self.right_ymap_gpu.upload(self.right_ymap)

        self.crop_start_col = max(0, min(int(LEFT_CROP_PIXELS), scaled_size[0] - 1))

        self.fx_scaled = float(K1_scaled[0, 0])
        self.fy_scaled = float(K1_scaled[1, 1])
        self.cx_scaled = float(K1_scaled[0, 2])
        self.cy_scaled = float(K1_scaled[1, 2])
        self.scaled_size = scaled_size

        self.min_disp = float((self.fx_scaled * self.baseline) / MAX_DEPTH)
        self.max_disp = float((self.fx_scaled * self.baseline) / MIN_DEPTH)

        self.warp_M = np.array(
            [[1.0, 0.0, 0.0],
             [0.0, 1.0, float(self.vertical_offset)]],
            dtype=np.float32
        )

        self.maps_computed = True
        self.setup_timer.cancel()
        self.get_logger().info(
            f'Rectification maps computed. Baseline: {self.baseline:.4f}m, '
            f'Scale: {self.scale}, Crop left: {self.crop_start_col}px, '
            f'Output: {scaled_size[0]}x{scaled_size[1]}'
        )

    def _sync_cb(self, left_msg, right_msg):
        if not self.maps_computed:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        if left is None or right is None:
            return

        # Upload raw images
        left_gpu = cv2.cuda_GpuMat()
        right_gpu = cv2.cuda_GpuMat()
        left_gpu.upload(left)
        right_gpu.upload(right)

        # GPU rectification
        left_rect_gpu = cv2.cuda.remap(
            left_gpu,
            self.left_xmap_gpu,
            self.left_ymap_gpu,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )
        right_rect_gpu = cv2.cuda.remap(
            right_gpu,
            self.right_xmap_gpu,
            self.right_ymap_gpu,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        # GPU vertical alignment correction
        right_rect_gpu = cv2.cuda.warpAffine(
            right_rect_gpu,
            self.warp_M,
            self.scaled_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        # Use the same CPU CLAHE as original
        left_rect = left_rect_gpu.download()
        right_rect = right_rect_gpu.download()

        left_rect = self.clahe.apply(left_rect)
        right_rect = self.clahe.apply(right_rect)

        # Upload rectified/enhanced images for GPU stereo
        left_rect_gpu = cv2.cuda_GpuMat()
        right_rect_gpu = cv2.cuda_GpuMat()
        left_rect_gpu.upload(left_rect)
        right_rect_gpu.upload(right_rect)

        # GPU stereo matching
        disp_16s_gpu = self.stereo.compute(left_rect_gpu, right_rect_gpu)
        disp_16s = disp_16s_gpu.download()

        # Extra cleanup on raw fixed-point disparity
        #cv2.filterSpeckles(disp_16s, 0, 100, 16)

        # Convert from fixed-point disparity
        disparity = disp_16s.astype(np.float32) / 16.0

        # Same validity filtering as original
        disparity = np.where(
            (disparity > self.min_disp) & (disparity < self.max_disp),
            disparity,
            0.0
        ).astype(np.float32)

        # Build stereo_msgs/DisparityImage
        disp_image_msg = self.bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
        disp_image_msg.header = left_msg.header

        disp_msg = DisparityImage()
        disp_msg.header = left_msg.header
        disp_msg.image = disp_image_msg
        disp_msg.f = self.fx_scaled
        disp_msg.t = float(self.baseline)
        disp_msg.min_disparity = float(self.min_disp)
        disp_msg.max_disparity = float(self.max_disp)
        disp_msg.delta_d = 1.0 / 16.0

        self.disp_pub.publish(disp_msg)

        # Publish rectified camera info
        cam_info = CameraInfo()
        cam_info.header = left_msg.header
        cam_info.width = self.scaled_size[0]
        cam_info.height = self.scaled_size[1]
        cam_info.k = [
            self.fx_scaled, 0.0,            self.cx_scaled,
            0.0,            self.fy_scaled, self.cy_scaled,
            0.0,            0.0,            1.0,
        ]
        self.cam_info_pub.publish(cam_info)

        # Better visualization: show only valid disparity
        if self.disp_vis_pub.get_subscription_count() > 0:
            disp_vis = np.zeros_like(disparity, dtype=np.uint8)
            valid = disparity > 0.0

            if np.any(valid):
                hi = np.percentile(disparity[valid], 99)
                hi = max(hi, self.min_disp + 1.0)

                disp_vis[valid] = np.clip(
                    (disparity[valid] - self.min_disp) / (hi - self.min_disp) * 255.0,
                    0,
                    255
                ).astype(np.uint8)

            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            disp_vis_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding='bgr8')
            disp_vis_msg.header = left_msg.header
            self.disp_vis_pub.publish(disp_vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()