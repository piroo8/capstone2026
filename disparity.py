import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Configure pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.fisheye, 1, rs.format.y8)
cfg.enable_stream(rs.stream.fisheye, 2, rs.format.y8)

# Start pipeline
profile = pipe.start(cfg)

# Get streams
fisheye1_stream = profile.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()
fisheye2_stream = profile.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()

# Get intrinsics and extrinsics
fisheye1_intrinsics = fisheye1_stream.get_intrinsics()
fisheye2_intrinsics = fisheye2_stream.get_intrinsics()
fisheye_extrinsics = fisheye1_stream.get_extrinsics_to(fisheye2_stream)

# Build camera matrices and distortion coefficients
def intrinsics_to_K(intr):
    return np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ])

K1 = intrinsics_to_K(fisheye1_intrinsics)
K2 = intrinsics_to_K(fisheye2_intrinsics)
D1 = np.array(fisheye1_intrinsics.coeffs[:4])
D2 = np.array(fisheye2_intrinsics.coeffs[:4])

# Rotation and translation from extrinsics
R = np.array(fisheye_extrinsics.rotation).reshape(3, 3)
T = np.array(fisheye_extrinsics.translation)

img_size = (fisheye1_intrinsics.width, fisheye1_intrinsics.height)

# Stereo rectification using fisheye model
R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K1, D1,
    K2, D2,
    img_size,
    R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    balance=0.0,
    fov_scale=1.0
)[:5]

# Compute undistort/rectify maps
map1_left, map2_left = cv2.fisheye.initUndistortRectifyMap(
    K1, D1, R1, P1, img_size, cv2.CV_32FC1
)
map1_right, map2_right = cv2.fisheye.initUndistortRectifyMap(
    K2, D2, R2, P2, img_size, cv2.CV_32FC1
)

# Configure stereo matcher
# Window size must be odd, between 5 and 255
window_size = 5
num_disparities = 128  # Must be divisible by 16
min_disparity = 0

stereo = cv2.StereoSGBM_create(
    minDisparity=min_disparity,
    numDisparities=num_disparities,
    blockSize=window_size,
    P1=8 * 1 * window_size * window_size,
    P2=32 * 1 * window_size * window_size,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# WLS filter for cleaner disparity
right_matcher = cv2.ximgproc.createRightMatcher(stereo)
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(8000)
wls_filter.setSigmaColor(1.5)

print("Capturing frames... Press 'q' to quit live view, or 's' to save and plot.")

try:
    # Let auto-exposure settle
    for _ in range(30):
        pipe.wait_for_frames()

    while True:
        frames = pipe.wait_for_frames()
        f1 = frames.get_fisheye_frame(1)
        f2 = frames.get_fisheye_frame(2)

        if not f1 or not f2:
            continue

        left_img = np.asanyarray(f1.get_data())
        right_img = np.asanyarray(f2.get_data())

        # Rectify images
        left_rect = cv2.remap(left_img, map1_left, map2_left, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_img, map1_right, map2_right, cv2.INTER_LINEAR)

        # Compute disparity (left and right for WLS filtering)
        disp_left = stereo.compute(left_rect, right_rect)
        disp_right = right_matcher.compute(right_rect, left_rect)

        # Apply WLS filter
        disparity_filtered = wls_filter.filter(
            disp_left, left_rect, None, disp_right
        ).astype(np.float32) / 16.0

        # Also compute raw disparity for comparison
        disparity_raw = disp_left.astype(np.float32) / 16.0

        # Normalize for display
        disp_vis = cv2.normalize(disparity_filtered, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        # Show live view
        cv2.imshow("Left Rectified", left_rect)
        cv2.imshow("Right Rectified", right_rect)
        cv2.imshow("Disparity (WLS Filtered)", disp_color)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save and plot with matplotlib
            fig, axes = plt.subplots(2, 2, figsize=(16, 10))

            axes[0, 0].imshow(left_rect, cmap='gray')
            axes[0, 0].set_title("Left Fisheye (Rectified)")
            axes[0, 0].axis('off')

            axes[0, 1].imshow(right_rect, cmap='gray')
            axes[0, 1].set_title("Right Fisheye (Rectified)")
            axes[0, 1].axis('off')

            axes[1, 0].imshow(disparity_raw, cmap='jet')
            axes[1, 0].set_title("Raw Disparity")
            axes[1, 0].axis('off')
            axes[1, 0].figure.colorbar(
                axes[1, 0].images[0], ax=axes[1, 0], fraction=0.046
            )

            axes[1, 1].imshow(disparity_filtered, cmap='jet')
            axes[1, 1].set_title("WLS Filtered Disparity")
            axes[1, 1].axis('off')
            axes[1, 1].figure.colorbar(
                axes[1, 1].images[0], ax=axes[1, 1], fraction=0.046
            )

            plt.tight_layout()
            plt.savefig("t265_disparity_output.png", dpi=150)
            print("Saved plot to t265_disparity_output.png")
            plt.show()

finally:
    pipe.stop()
    cv2.destroyAllWindows()
    print("Pipeline stopped.")