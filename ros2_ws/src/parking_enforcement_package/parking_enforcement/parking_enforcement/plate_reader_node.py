#!/usr/bin/env python3
"""ROS wrapper for the TensorRT license-plate pipeline.

This node stays idle until the mission node enables scanning. During scans it
runs the detector/recognizer, applies multi-frame voting, and publishes the
current candidate plus the confirmed 7-character plate string.
"""

from __future__ import annotations

from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from parking_enforcement.config import (
    BLANK,
    CHARS,
    DETECT_EVERY,
    DRONE_NS,
    LPD_ENGINE_PATH,
    LPD_H,
    LPD_W,
    LPR_ENGINE_PATH,
    LPR_H,
    LPR_W,
    MIN_DET_CONF,
    MIN_READ_CONF,
    MIN_VOTES,
    PAD_BOTTOM,
    PAD_LEFT,
    PAD_RIGHT,
    PAD_TOP,
    PLATE_CONFIRMED_TOPIC,
    PLATE_CURRENT_TOPIC,
    PLATE_DEBUG_IMAGE_TOPIC,
    PLATE_ENABLED_TOPIC,
    RGB_IMAGE_TOPIC,
    VOTE_WINDOW,
)

try:
    import tensorrt as trt
    import pycuda.autoinit  # noqa: F401  # Creates the CUDA context once at import time.
    import pycuda.driver as cuda
except Exception as exc:  # pragma: no cover - depends on Jetson runtime.
    trt = None
    cuda = None
    TRT_IMPORT_ERROR = exc
else:
    TRT_IMPORT_ERROR = None

TRT_LOGGER = trt.Logger(trt.Logger.WARNING) if trt is not None else None


class TRTModel:
    """Minimal TensorRT runner for the static-shape engines used in the demo."""

    def __init__(self, engine_path: str):
        with open(engine_path, 'rb') as handle:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(handle.read())
        if self.engine is None:
            raise RuntimeError(f'Could not deserialize TensorRT engine: {engine_path}')

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f'Could not create TensorRT execution context: {engine_path}')

        self.inputs = []
        self.outputs = []
        self.bindings = []

        for binding in self.engine:
            shape = tuple(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(int(trt.volume(shape)), dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(device_mem))

            payload = {'host': host_mem, 'device': device_mem, 'shape': shape}
            if self.engine.binding_is_input(binding):
                self.inputs.append(payload)
            else:
                self.outputs.append(payload)

    def infer(self, *input_arrays):
        """Copy inputs to the device, execute once, and return reshaped outputs."""

        for index, arr in enumerate(input_arrays):
            np.copyto(self.inputs[index]['host'], arr.ravel())
            cuda.memcpy_htod(self.inputs[index]['device'], self.inputs[index]['host'])

        self.context.execute_v2(self.bindings)

        results = []
        for output in self.outputs:
            cuda.memcpy_dtoh(output['host'], output['device'])
            results.append(output['host'].reshape(output['shape']))
        return results


class PlateVoter:
    """Confirm a plate only after it has been seen repeatedly in a short window."""

    def __init__(self, min_votes: int = MIN_VOTES, window: int = VOTE_WINDOW):
        self.min_votes = min_votes
        self.window = deque(maxlen=window)

    def clear(self):
        """Reset the vote history when a new scan starts."""

        self.window.clear()

    def update(self, plate_text: str, det_conf: float, read_conf: float):
        """Return a confirmed plate once the same reading has enough votes."""

        if len(plate_text) != 7:
            return None
        if det_conf < MIN_DET_CONF or read_conf < MIN_READ_CONF:
            return None

        self.window.append(plate_text)
        counts = Counter(self.window)
        best_plate, best_count = counts.most_common(1)[0]
        if best_count >= self.min_votes:
            return best_plate
        return None


class PlateReaderNode(Node):
    """Run the plate detector/recognizer only when the mission enters a scan state."""

    def __init__(self):
        super().__init__(f'{DRONE_NS}_plate_reader')
        self.bridge = CvBridge()
        self.enabled = False
        self.frame_id = 0
        self.last_detections = []
        self.last_confirmed_plate = ''
        self.voter = PlateVoter()

        self.confirmed_pub = self.create_publisher(String, PLATE_CONFIRMED_TOPIC, 10)
        self.current_pub = self.create_publisher(String, PLATE_CURRENT_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, PLATE_DEBUG_IMAGE_TOPIC, 10)

        self.create_subscription(Image, RGB_IMAGE_TOPIC, self._image_cb, 10)
        self.create_subscription(Bool, PLATE_ENABLED_TOPIC, self._enable_cb, 10)

        self.models_ready = False
        if trt is None or cuda is None:
            self.get_logger().error(f'TensorRT/PyCUDA import failed: {TRT_IMPORT_ERROR}')
            return

        try:
            self.get_logger().info(f'Loading LPD engine from {LPD_ENGINE_PATH}')
            self.lpd_model = TRTModel(LPD_ENGINE_PATH)
            self.get_logger().info(f'Loading LPR engine from {LPR_ENGINE_PATH}')
            self.lpr_model = TRTModel(LPR_ENGINE_PATH)
            self.models_ready = True
            self.get_logger().info('Plate reader ready')
        except Exception as exc:
            self.get_logger().error(f'Failed to initialize plate reader: {exc}')

    def _enable_cb(self, msg: Bool):
        """Turn the reader on only during mission scan phases."""

        if msg.data == self.enabled:
            return

        self.enabled = msg.data
        self.frame_id = 0
        self.last_detections = []
        self.last_confirmed_plate = ''
        self.voter.clear()
        state = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f'Plate reader {state}')

    def _image_cb(self, msg: Image):
        """Process a color frame when the mission node has enabled plate scanning."""

        if not self.enabled or not self.models_ready:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_id += 1

        # Run the detector every few frames, then re-read the strongest recent box.
        if self.frame_id % DETECT_EVERY == 0:
            lpd_tensor, scale_x, scale_y = preprocess_lpd(frame)
            lpd_out = self.lpd_model.infer(lpd_tensor)
            self.last_detections = decode_lpd(lpd_out[0], lpd_out[1], scale_x, scale_y)

        overlay = frame.copy()
        current_plate = ''

        if self.last_detections:
            x1, y1, x2, y2, det_conf = self.last_detections[0]
            height, width = frame.shape[:2]
            x1 = max(0, min(width - 1, x1))
            y1 = max(0, min(height - 1, y1))
            x2 = max(0, min(width, x2))
            y2 = max(0, min(height, y2))

            crop = frame[
                max(0, y1 - PAD_TOP):min(height, y2 + PAD_BOTTOM),
                max(0, x1 - PAD_LEFT):min(width, x2 + PAD_RIGHT),
            ]
            if crop.size > 0:
                lpr_tensor = preprocess_lpr(crop)
                lpr_out = self.lpr_model.infer(lpr_tensor)
                pred = lpr_out[1][0].astype(int)
                plate_text = ctc_decode(pred)
                char_confs = lpr_out[0][0].astype(np.float32)
                read_conf = float(char_confs.mean())
                current_plate = plate_text

                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f'{plate_text} d={det_conf:.2f} r={read_conf:.2f}',
                    (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                confirmed = self.voter.update(plate_text, det_conf, read_conf)
                if confirmed is not None and confirmed != self.last_confirmed_plate:
                    confirmed_msg = String()
                    confirmed_msg.data = confirmed
                    self.confirmed_pub.publish(confirmed_msg)
                    self.last_confirmed_plate = confirmed
                    self.get_logger().info(f'Confirmed plate: {confirmed}')

        current_msg = String()
        current_msg.data = current_plate
        self.current_pub.publish(current_msg)

        if self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def ctc_decode(pred):
    """Collapse repeated class IDs and remove the CTC blank token."""

    plate = []
    prev = BLANK
    for token in pred:
        if token != prev and token != BLANK and 0 <= token < len(CHARS):
            plate.append(CHARS[token])
        prev = token
    return ''.join(plate)


def preprocess_lpd(frame):
    """Resize and normalize an RGB image for the plate detector engine."""

    orig_h, orig_w = frame.shape[:2]
    scale_x = orig_w / LPD_W
    scale_y = orig_h / LPD_H
    img = cv2.resize(frame, (LPD_W, LPD_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))
    x = np.expand_dims(x, 0)
    return x.astype(np.float32), scale_x, scale_y


def decode_lpd(cov, bbox, scale_x, scale_y, conf_thresh=0.50):
    """Decode the detector heatmap into pixel-space boxes, then apply NMS."""

    cov = cov.squeeze()
    bbox = bbox.squeeze()
    if bbox.shape[-1] == 4:
        bbox = np.transpose(bbox, (2, 0, 1))

    grid_h, grid_w = cov.shape
    cell_w = LPD_W / grid_w
    cell_h = LPD_H / grid_h
    bbox_scale_x = 35.0
    bbox_scale_y = 20.0

    raw_boxes = []
    for gy in range(grid_h):
        for gx in range(grid_w):
            conf = float(cov[gy, gx])
            if conf < conf_thresh:
                continue
            cx = (gx + 0.5) * cell_w
            cy = (gy + 0.5) * cell_h
            x1 = int((cx - bbox[0, gy, gx] * bbox_scale_x) * scale_x)
            y1 = int((cy - bbox[1, gy, gx] * bbox_scale_y) * scale_y)
            x2 = int((cx + bbox[2, gy, gx] * bbox_scale_x) * scale_x)
            y2 = int((cy + bbox[3, gy, gx] * bbox_scale_y) * scale_y)
            if x2 <= x1 or y2 <= y1:
                continue
            raw_boxes.append((x1, y1, x2, y2, conf))

    if not raw_boxes:
        return []

    boxes_xywh = [[x1, y1, x2 - x1, y2 - y1] for x1, y1, x2, y2, _ in raw_boxes]
    scores = [score for *_, score in raw_boxes]
    indices = cv2.dnn.NMSBoxes(boxes_xywh, scores, conf_thresh, 0.4)

    result = []
    if len(indices) > 0:
        for idx in indices.flatten():
            result.append(raw_boxes[idx])
    result.sort(key=lambda box: box[4], reverse=True)
    return result


def preprocess_lpr(crop_bgr):
    """Resize and normalize a plate crop for the LPR engine."""

    img = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (LPR_W, LPR_H))
    x = img.astype(np.float32) / 255.0
    x = (x - 0.5) / 0.5
    x = np.transpose(x, (2, 0, 1))
    return np.expand_dims(x, 0).astype(np.float32)


def main(args=None):
    """Spin the plate reader node until shutdown."""

    rclpy.init(args=args)
    node = PlateReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
