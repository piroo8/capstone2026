# web_viewer.py
import cv2
from flask import Flask, Response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

app = Flask(__name__)
bridge = CvBridge()
latest_frame = None

class Viewer(Node):
    def __init__(self):
        super().__init__('web_viewer')
        self.create_subscription(Image, '/stereo/disp_vis', self.cb, 10)
    def cb(self, msg):
        global latest_frame
        latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def gen():
    while True:
        if latest_frame is not None:
            _, jpg = cv2.imencode('.jpg', latest_frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')

@app.route('/')
def video():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros_thread():
    rclpy.init()
    node = Viewer()
    rclpy.spin(node)

threading.Thread(target=ros_thread, daemon=True).start()
app.run(host='0.0.0.0', port=5000)