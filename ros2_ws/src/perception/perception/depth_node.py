import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

MIN_DEPTH = 0.1 # meters
MAX_DEPTH = 5.0 # meters
SKIP_FRAMES = 3 # process every Nth frame for performance
NUM_DISPARITIES = 64
# Manual crop for the invalid left disparity strip. Tune this in pixels.
LEFT_CROP_PIXELS = 62

class DepthNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_8')
        self.bridge = CvBridge()

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
        self.baseline = None
        self.min_disp = None
        self.max_disp = None

        # Downscale factor for performance
        self.scale = 0.5

        # SGBM with smaller block size for sharper edges
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=NUM_DISPARITIES,
            blockSize=5,
            P1=8 * 3 * 5 * 5,      # 8 * channels * blockSize^2
            P2=32 * 3 * 5 * 5,     # 32 * channels * blockSize^2
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=25,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        # CLAHE for adaptive contrast (better than equalizeHist)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Vertical offset correction (tune this)
        self.vertical_offset = -2

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera info subscriptions
        self.create_subscription(
            CameraInfo, '/camera/fisheye1/camera_info',
            self._cam1_intrinsics_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/fisheye2/camera_info',
            self._cam2_intrinsics_cb, qos_profile_sensor_data)

        # Synced image subscriptions
        left_sub = Subscriber(self, Image, '/camera/fisheye1/image_raw',
                              qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(self, Image, '/camera/fisheye2/image_raw',
                               qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._sync_cb)

        # Publishers
        self.disp_pub = self.create_publisher(
            DisparityImage, 'disparity', qos_profile_sensor_data)
        self.cam_info_pub = self.create_publisher(
            CameraInfo, 'disparity/camera_info', qos_profile_sensor_data)
        self.disp_vis_pub = self.create_publisher(
            Image, '/stereo/disp_vis', 10)

        # Timer to poll for TF readiness
        self.setup_timer = self.create_timer(0.5, self._try_compute_maps)

        # Frame skip — process every Nth frame
        self.frame_count = 0
        self.process_every_n = SKIP_FRAMES
        self.crop_start_col = 0

        self.get_logger().info('Waiting for camera info and TF...')

    def _cam1_intrinsics_cb(self, msg):
        if self.K1 is not None:
            return
        self.K1 = np.array(msg.k).reshape(3, 3)
        self.D1 = np.array(msg.d[:4])
        self.image_size = (msg.width, msg.height)
        self.get_logger().info('Got fisheye1 camera info')

    def _cam2_intrinsics_cb(self, msg):
        if self.K2 is not None:
            return
        self.K2 = np.array(msg.k).reshape(3, 3)
        self.D2 = np.array(msg.d[:4])
        self.get_logger().info('Got fisheye2 camera info')

    def _try_compute_maps(self):
        if self.maps_computed:
            return

        if self.K1 is None or self.K2 is None:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'camera_fisheye1_optical_frame',
                'camera_fisheye2_optical_frame',
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return

        trans = t.transform.translation
        rot = t.transform.rotation
        T = np.array([trans.x, trans.y, trans.z])
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
            self.K1, self.D1, R1, K1_scaled, scaled_size, cv2.CV_32FC1)
        self.right_xmap, self.right_ymap = cv2.fisheye.initUndistortRectifyMap(
            self.K2, self.D2, R2, K2_scaled, scaled_size, cv2.CV_32FC1)

        self.crop_start_col = max(0, min(int(LEFT_CROP_PIXELS), scaled_size[0] - 1))

        self.fx_scaled = K1_scaled[0, 0]
        self.fy_scaled = K1_scaled[1, 1]
        self.cx_scaled = K1_scaled[0, 2]
        self.cy_scaled = K1_scaled[1, 2]
        self.scaled_size = scaled_size

        self.min_disp = (self.fx_scaled * self.baseline) / MAX_DEPTH
        self.max_disp = (self.fx_scaled * self.baseline) / MIN_DEPTH

        # Precompute vertical offset warp matrix
        self.warp_M = np.float32([[1, 0, 0], [0, 1, self.vertical_offset]])

        self.maps_computed = True
        self.setup_timer.cancel()
        self.get_logger().info(
            f'Rectification maps computed. Baseline: {self.baseline:.4f}m, '
            f'Scale: {self.scale}, Crop left: {self.crop_start_col}px, '
            f'Output: {scaled_size[0] - self.crop_start_col}x{scaled_size[1]}')

    def _sync_cb(self, left_msg, right_msg):
        if not self.maps_computed:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        # Undistort + rectify + downscale in one remap
        left_rect = cv2.remap(left, self.left_xmap, self.left_ymap, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.right_xmap, self.right_ymap, cv2.INTER_LINEAR)

        # Vertical alignment correction
        right_rect = cv2.warpAffine(
            right_rect, self.warp_M,
            (right_rect.shape[1], right_rect.shape[0]))

        # CLAHE for better contrast
        left_rect = self.clahe.apply(left_rect)
        right_rect = self.clahe.apply(right_rect)

        # Compute disparity (int16 in 1/16 pixel units → float32 pixels)
        # Invalid pixels (no stereo match) come out as minDisparity-1 = -1/16 ≈ -0.0625.
        # Zero them out here so occupancy_node's min_disparity filter drops them cleanly.
        # We do NOT spatially crop the left invalid strip: cropping would shift cx away
        # from W/2 and break occupancy_node's back-projection. Zeroing is sufficient.
        disparity = self.stereo.compute(left_rect, right_rect).astype(np.float32) / 16.0
        disparity = np.where((disparity > self.min_disp) & (disparity < self.max_disp), disparity, 0.0)

        # Build stereo_msgs/DisparityImage
        disp_image_msg = self.bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
        disp_image_msg.header = left_msg.header

        disp_msg = DisparityImage()
        disp_msg.header        = left_msg.header
        disp_msg.image         = disp_image_msg
        disp_msg.f             = self.fx_scaled   # focal length of rectified image (pixels)
        disp_msg.t             = self.baseline     # baseline (metres); occupancy_node takes abs()
        disp_msg.min_disparity = float(self.min_disp)
        disp_msg.max_disparity = float(self.max_disp)
        disp_msg.delta_d       = 1.0 / 16.0       # SGBM sub-pixel resolution

        self.disp_pub.publish(disp_msg)

        # Publish rectified camera info so downstream nodes get the true cx/cy
        # rather than assuming the principal point is at image centre.
        cam_info = CameraInfo()
        cam_info.header = left_msg.header
        cam_info.width  = self.scaled_size[0]
        cam_info.height = self.scaled_size[1]
        cam_info.k = [
            self.fx_scaled, 0.0,            self.cx_scaled,
            0.0,            self.fy_scaled, self.cy_scaled,
            0.0,            0.0,            1.0,
        ]
        self.cam_info_pub.publish(cam_info)

        # Publish disparity visualization
        if self.disp_vis_pub.get_subscription_count() > 0:
            disp_vis = np.clip(disparity, 0, NUM_DISPARITIES)
            disp_vis = (disp_vis / NUM_DISPARITIES * 255).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            disp_vis_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding='bgr8')
            disp_vis_msg.header = left_msg.header
            self.disp_vis_pub.publish(disp_vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
