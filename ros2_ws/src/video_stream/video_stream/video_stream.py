import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class HUDStreamer(Node):
    def __init__(self):
        super().__init__('hud_streamer')
        self.bridge = CvBridge()
        self.roll = 0.0
        self.pitch = 0.0

        # Subscribe to IMU or MAVROS attitude
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, 10)

        # Capture pipeline (CSI camera)
        capture_pipeline = (
            "nvarguscamerasrc sensor-mode=4 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true"
        )

        # Stream pipeline to QGC
        stream_pipeline = (
            "appsrc ! "
            "videoconvert ! "
            "video/x-raw, format=BGRx ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM), format=NV12 ! "
            "nvv4l2h264enc bitrate=2000000 insert-sps-pps=true iframeinterval=60 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=100.66.14.64 port=5600"
        )

        self.cap = cv2.VideoCapture(capture_pipeline, cv2.CAP_GSTREAMER)
        self.out = cv2.VideoWriter(stream_pipeline, cv2.CAP_GSTREAMER, 0, 60, (1280, 720))

        self.create_timer(1.0 / 60.0, self.stream_frame)

    def imu_cb(self, msg):
        # Convert quaternion to roll/pitch
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))

    def draw_hud(self, frame):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        roll_deg  = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)

        overlay = frame.copy()

        # ---------- Artificial horizon ----------
        horizon_radius = 180
        pixels_per_deg = 4

        # Sky/ground split rotated by roll
        for dy in range(-horizon_radius, horizon_radius):
            pitch_offset = int(pitch_deg * pixels_per_deg)
            x_offset = int(dy * math.tan(self.roll))
            y_pos = cy + dy + pitch_offset

            # Ground (brown) vs sky (blue)
            color = (34, 85, 139) if dy < 0 else (42, 100, 55)
            x1 = cx - horizon_radius + x_offset
            x2 = cx + horizon_radius + x_offset
            if 0 <= y_pos < h:
                cv2.line(overlay, (max(0, x1), y_pos), (min(w, x2), y_pos), color, 1)

        # Horizon line
        angle_rad = self.roll
        dx = int(horizon_radius * math.cos(angle_rad))
        dy = int(horizon_radius * math.sin(angle_rad))
        pitch_offset = int(pitch_deg * pixels_per_deg)
        cv2.line(overlay, (cx - dx, cy + dy + pitch_offset),
                           (cx + dx, cy - dy + pitch_offset), (255, 255, 255), 2)

        # Blend overlay with original frame
        frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

        # ---------- Roll indicator arc ----------
        cv2.ellipse(frame, (cx, cy), (120, 120), 0, 200, 340, (255, 255, 255), 1)
        # Tick marks at 0, ±10, ±20, ±30, ±60 degrees
        for tick_deg in [-60, -30, -20, -10, 0, 10, 20, 30, 60]:
            tick_rad = math.radians(tick_deg - 90)
            inner = 115
            outer = 125 if tick_deg % 30 == 0 else 120
            x1 = int(cx + inner * math.cos(tick_rad))
            y1 = int(cy + inner * math.sin(tick_rad))
            x2 = int(cx + outer * math.cos(tick_rad))
            y2 = int(cy + outer * math.sin(tick_rad))
            cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)

        # Roll pointer triangle
        ptr_rad = math.radians(roll_deg - 90)
        px = int(cx + 110 * math.cos(ptr_rad))
        py = int(cy + 110 * math.sin(ptr_rad))
        pts = np.array([[px, py],
                         [px + int(8 * math.cos(ptr_rad + 2.5)), py + int(8 * math.sin(ptr_rad + 2.5))],
                         [px + int(8 * math.cos(ptr_rad - 2.5)), py + int(8 * math.sin(ptr_rad - 2.5))]],
                        np.int32)
        cv2.fillPoly(frame, [pts], (255, 255, 0))

        # ---------- Fixed aircraft reference ----------
        cv2.line(frame, (cx - 60, cy), (cx - 20, cy), (255, 255, 0), 3)
        cv2.line(frame, (cx + 20, cy), (cx + 60, cy), (255, 255, 0), 3)
        cv2.circle(frame, (cx, cy), 4, (255, 255, 0), -1)

        # ---------- Pitch ladder ----------
        for p in range(-30, 31, 5):
            if p == 0:
                continue
            y_pos = cy + int((p - pitch_deg) * pixels_per_deg)
            bar_w = 40 if p % 10 == 0 else 20
            color = (255, 255, 255)
            cv2.line(frame, (cx - bar_w, y_pos), (cx - 10, y_pos), color, 1)
            cv2.line(frame, (cx + 10, y_pos), (cx + bar_w, y_pos), color, 1)
            if p % 10 == 0:
                cv2.putText(frame, str(p), (cx - bar_w - 25, y_pos + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # ---------- Text readouts ----------
        cv2.putText(frame, f"Roll:  {roll_deg:+.1f} deg", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Pitch: {pitch_deg:+.1f} deg", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return frame

    def stream_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = self.draw_hud(frame)
        self.out.write(frame)


def main():
    rclpy.init()
    node = HUDStreamer()
    rclpy.spin(node)
    node.cap.release()
    node.out.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()