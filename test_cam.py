import pyrealsense2 as rs
import numpy as np
import cv2

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.fisheye, 1)  # Left fisheye
cfg.enable_stream(rs.stream.fisheye, 2)  # Right fisheye

pipe.start(cfg)

try:
    while True:
        frames = pipe.wait_for_frames()
        left = frames.get_fisheye_frame(1)
        right = frames.get_fisheye_frame(2)

        if not left or not right:
            continue

        left_img = np.asanyarray(left.get_data())
        right_img = np.asanyarray(right.get_data())

        # Display side by side
        combined = np.hstack((left_img, right_img))
        cv2.imshow("T265 Fisheye (Left | Right)", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()