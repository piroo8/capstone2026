#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
import numpy as np
import pycuda.autoinit  # noqa: F401  (creates CUDA context)
import pycuda.driver as cuda
import tensorrt as trt


# ── User settings: edit these at the top only ────────────────────────────────
LPD_ENGINE_PATH = "/home/jetson/engines/lpdnet_fp32.engine"
LPR_ENGINE_PATH = "/home/jetson/engines/lprnet_fp32.engine"

CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30
SENSOR_MODE   = 4
FLIP_METHOD   = 2

CONF_THRESH  = 0.50
DETECT_EVERY = 15

PAD_LEFT   = 25
PAD_RIGHT  = 5
PAD_TOP    = 30
PAD_BOTTOM = 30

DEBUG = False

# ── Model / decoding constants ────────────────────────────────────────────────
LPD_W, LPD_H = 640, 480
CHARS = "0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ"
BLANK = 35
NUM_CLASSES = len(CHARS) + 1  # + CTC blank
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


# ── TensorRT wrapper ──────────────────────────────────────────────────────────
class TRTModel:
    """Simple TensorRT runner for static-shape engines (Jetson Nano / TRT 8 style)."""

    def __init__(self, engine_path: str):
        self.engine_path = engine_path
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        if self.engine is None:
            raise RuntimeError(f"Failed to deserialize engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f"Failed to create execution context: {engine_path}")

        self.bindings = [None] * self.engine.num_bindings
        self.input_bindings = []
        self.output_bindings = []

        for binding_idx in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(binding_idx)
            shape = tuple(self.engine.get_binding_shape(binding_idx))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding_idx))
            is_input = self.engine.binding_is_input(binding_idx)

            if any(dim < 0 for dim in shape):
                raise RuntimeError(
                    f"Dynamic shape binding not supported by this helper. "
                    f"Binding '{name}' has shape {shape}"
                )

            size = int(trt.volume(shape))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings[binding_idx] = int(device_mem)

            meta = {
                "index": binding_idx,
                "name": name,
                "shape": shape,
                "dtype": dtype,
                "host": host_mem,
                "device": device_mem,
            }

            if is_input:
                self.input_bindings.append(meta)
            else:
                self.output_bindings.append(meta)

    def infer(self, *input_arrays):
        if len(input_arrays) != len(self.input_bindings):
            raise ValueError(
                f"Engine expects {len(self.input_bindings)} input(s) but got {len(input_arrays)}"
            )

        # Copy host -> device
        for meta, arr in zip(self.input_bindings, input_arrays):
            arr = np.ascontiguousarray(arr.astype(meta["dtype"], copy=False))
            expected = int(np.prod(meta["shape"]))
            got = int(arr.size)
            if got != expected:
                raise ValueError(
                    f"Input '{meta['name']}' size mismatch: expected {meta['shape']} ({expected} vals), "
                    f"got {arr.shape} ({got} vals)"
                )
            np.copyto(meta["host"], arr.ravel())
            cuda.memcpy_htod(meta["device"], meta["host"])

        # Run inference
        ok = self.context.execute_v2(self.bindings)
        if not ok:
            raise RuntimeError(f"TensorRT execution failed for {self.engine_path}")

        # Copy device -> host
        outputs = {}
        for meta in self.output_bindings:
            cuda.memcpy_dtoh(meta["host"], meta["device"])
            outputs[meta["name"]] = meta["host"].reshape(meta["shape"])

        return outputs


# ── CTC decoding ──────────────────────────────────────────────────────────────
def ctc_decode(pred_ids):
    plate = []
    prev = BLANK
    for p in pred_ids:
        p = int(p)
        if p != prev and p != BLANK and 0 <= p < len(CHARS):
            plate.append(CHARS[p])
        prev = p
    return "".join(plate)


# ── Preprocessing ─────────────────────────────────────────────────────────────
def preprocess_lpd(frame_bgr):
    orig_h, orig_w = frame_bgr.shape[:2]
    scale_x = orig_w / LPD_W
    scale_y = orig_h / LPD_H

    img = cv2.resize(frame_bgr, (LPD_W, LPD_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))
    x = np.expand_dims(x, 0)
    return x, scale_x, scale_y


def preprocess_lpr(crop_bgr):
    img = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (96, 48))
    x = img.astype(np.float32) / 255.0
    x = (x - 0.5) / 0.5   # TAO / LPRNet normalization to [-1, 1]
    x = np.transpose(x, (2, 0, 1))
    x = np.expand_dims(x, 0)
    return x.astype(np.float32)


# ── Output picking helpers ────────────────────────────────────────────────────
def _shape_str(arr):
    return "x".join(str(s) for s in arr.shape)


def pick_lpd_outputs(outputs_dict):
    """Find coverage/confidence and bbox tensors by shape, not output order."""
    arrays = list(outputs_dict.items())
    cov = None
    bbox = None

    for name, arr in arrays:
        shape = arr.shape

        # Coverage / confidence examples:
        # (1, 30, 40), (1, 1, 30, 40), (30, 40)
        if arr.ndim in (2, 3, 4):
            if shape[-2:] == (30, 40):
                if arr.ndim == 2:
                    cov = arr
                elif arr.ndim == 3 and shape[0] == 1:
                    cov = arr
                elif arr.ndim == 4 and shape[1] == 1:
                    cov = arr

        # Bounding boxes examples:
        # (1, 4, 30, 40), (4, 30, 40), (30, 40, 4)
        if arr.ndim in (3, 4):
            if arr.ndim == 3:
                if shape[0] == 4 and shape[1:] == (30, 40):
                    bbox = arr
                elif shape[-1] == 4 and shape[:2] == (30, 40):
                    bbox = arr
            elif arr.ndim == 4:
                if shape[1] == 4 and shape[-2:] == (30, 40):
                    bbox = arr
                elif shape[-1] == 4 and shape[1:3] == (30, 40):
                    bbox = arr

    if cov is None or bbox is None:
        names_shapes = ", ".join(f"{name}:{_shape_str(arr)}" for name, arr in arrays)
        raise RuntimeError(f"Could not identify LPD outputs. Saw: {names_shapes}")

    return cov, bbox


def pick_lpr_prediction(outputs_dict):
    """
    Supports either:
      1) deployable engine with [max_values, argmax_ids]
      2) logits/probabilities tensor needing argmax over class dimension
    Returns: pred_ids_1d (T,), read_conf (float)
    """
    arrays = list(outputs_dict.items())

    # First try to find a direct argmax/ID tensor.
    id_tensor = None
    conf_tensor = None

    for name, arr in arrays:
        if arr.ndim == 2 and arr.shape[0] == 1:
            # Likely (1, 24) ids or confidences
            flat = arr[0]
            if flat.size >= 8:
                # If values look like class IDs, prefer this tensor.
                looks_like_ids = np.all(np.isfinite(flat)) and np.all(np.abs(flat - np.round(flat)) < 1e-5)
                in_range = np.all((flat >= 0) & (flat < NUM_CLASSES))
                if looks_like_ids and in_range:
                    id_tensor = flat.astype(np.int32)
                else:
                    conf_tensor = flat.astype(np.float32)

    if id_tensor is not None:
        if conf_tensor is None:
            read_conf = 0.0
        else:
            non_blank = id_tensor != BLANK
            read_conf = float(conf_tensor[non_blank].mean()) if np.any(non_blank) else float(conf_tensor.mean())
        return id_tensor, read_conf

    # Fallback: find logits/probs tensor and argmax over class axis.
    for name, arr in arrays:
        if arr.ndim == 3 and arr.shape[0] == 1:
            shape = arr.shape
            # Find the class axis by looking for the 36-class dimension.
            class_axis = None
            for axis, dim in enumerate(shape):
                if dim == NUM_CLASSES:
                    class_axis = axis
                    break
            if class_axis is None:
                continue

            logits = arr[0]  # remove batch => 2D
            class_axis_2d = class_axis - 1
            pred_ids = np.argmax(logits, axis=class_axis_2d).astype(np.int32)
            confs = np.max(logits, axis=class_axis_2d).astype(np.float32)
            pred_ids = pred_ids.ravel()
            confs = confs.ravel()
            non_blank = pred_ids != BLANK
            read_conf = float(confs[non_blank].mean()) if np.any(non_blank) else float(confs.mean())
            return pred_ids, read_conf

    names_shapes = ", ".join(f"{name}:{_shape_str(arr)}" for name, arr in arrays)
    raise RuntimeError(f"Could not identify LPR outputs. Saw: {names_shapes}")


# ── LPD decoder ───────────────────────────────────────────────────────────────
def decode_lpd(cov, bbox, scale_x, scale_y, conf_thresh=0.5, debug=False):
    """
    Decode DetectNet_v2-style outputs.

    Expected shapes after squeeze handling:
      cov  -> (30, 40)
      bbox -> (4, 30, 40)  or (30, 40, 4)
    """
    cov = np.squeeze(cov)
    bbox = np.squeeze(bbox)

    if cov.ndim != 2:
        raise RuntimeError(f"Unexpected cov shape after squeeze: {cov.shape}")

    if bbox.ndim != 3:
        raise RuntimeError(f"Unexpected bbox shape after squeeze: {bbox.shape}")

    if bbox.shape[-1] == 4:
        bbox = np.transpose(bbox, (2, 0, 1))
    elif bbox.shape[0] != 4:
        raise RuntimeError(f"Could not interpret bbox tensor shape: {bbox.shape}")

    grid_h, grid_w = cov.shape
    cell_w = LPD_W / grid_w
    cell_h = LPD_H / grid_h

    # These are the same scales you were using before.
    bbox_scale_x = 35.0
    bbox_scale_y = 20.0

    if debug:
        gy, gx = np.unravel_index(np.argmax(cov), cov.shape)
        print(
            f"BEST CELL conf={cov[gy, gx]:.3f} raw bbox: "
            f"x1={bbox[0, gy, gx]:.3f} y1={bbox[1, gy, gx]:.3f} "
            f"x2={bbox[2, gy, gx]:.3f} y2={bbox[3, gy, gx]:.3f}"
        )

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
    scores = [conf for *_, conf in raw_boxes]
    indices = cv2.dnn.NMSBoxes(boxes_xywh, scores, conf_thresh, 0.4)

    result = []
    if indices is not None and len(indices) > 0:
        for i in np.array(indices).flatten():
            result.append(raw_boxes[int(i)])

    result.sort(key=lambda b: b[4], reverse=True)
    return result


# ── Camera / drawing helpers ──────────────────────────────────────────────────
def build_gstreamer_pipeline(width=1280, height=720, framerate=30, sensor_mode=4, flip_method=2):
    return (
        f"nvarguscamerasrc sensor-mode={sensor_mode} ! "
        f"video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def clamp_box(x1, y1, x2, y2, w, h):
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w, x2))
    y2 = max(0, min(h, y2))
    return x1, y1, x2, y2


def draw_crop_inset(display, crop, plate_text, read_conf):
    if crop is None or crop.size == 0:
        return

    h, w = display.shape[:2]
    inset_w = min(320, w // 3)
    inset_h = int(inset_w * 0.5)
    resized = cv2.resize(crop, (inset_w, inset_h))

    x0 = w - inset_w - 20
    y0 = 20
    display[y0:y0 + inset_h, x0:x0 + inset_w] = resized
    cv2.rectangle(display, (x0, y0), (x0 + inset_w, y0 + inset_h), (255, 255, 255), 2)

    caption = f"LPR crop: {plate_text}" if plate_text else "LPR crop"
    if read_conf is not None:
        caption += f"  ({read_conf:.2f})"
    cv2.putText(
        display,
        caption,
        (x0, max(18, y0 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


# ── Main app ──────────────────────────────────────────────────────────────────
def main():
    if not Path(LPD_ENGINE_PATH).exists():
        raise FileNotFoundError(f"LPD engine not found: {LPD_ENGINE_PATH}")
    if not Path(LPR_ENGINE_PATH).exists():
        raise FileNotFoundError(f"LPR engine not found: {LPR_ENGINE_PATH}")

    print("Loading LPD engine...")
    lpd_model = TRTModel(LPD_ENGINE_PATH)
    print("Loading LPR engine...")
    lpr_model = TRTModel(LPR_ENGINE_PATH)
    print("Both TensorRT engines loaded")

    gst_pipeline = build_gstreamer_pipeline(
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        framerate=CAMERA_FPS,
        sensor_mode=SENSOR_MODE,
        flip_method=FLIP_METHOD,
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError("ERROR: Camera failed to open")

    print("Running live visualizer — press ESC to quit")

    frame_id = 0
    detections = []
    plate_text = ""
    last_read_conf = None
    last_crop = None
    last_detect_stats = ""

    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera read failed")
                break

            now = time.time()
            dt = max(now - prev_time, 1e-6)
            fps = 1.0 / dt
            prev_time = now

            frame_id += 1
            orig_h, orig_w = frame.shape[:2]
            display = frame.copy()

            # ── Detect on the first frame, then every N frames ──────────────
            if frame_id == 1 or frame_id % max(1, DETECT_EVERY) == 0:
                lpd_tensor, scale_x, scale_y = preprocess_lpd(frame)
                lpd_out = lpd_model.infer(lpd_tensor)
                cov, bbox = pick_lpd_outputs(lpd_out)

                cov_squeezed = np.squeeze(cov)
                last_detect_stats = (
                    f"LPD max={float(np.max(cov_squeezed)):.3f}  "
                    f"mean={float(np.mean(cov_squeezed)):.3f}"
                )
                if DEBUG:
                    print(last_detect_stats)

                detections = decode_lpd(
                    cov,
                    bbox,
                    scale_x,
                    scale_y,
                    conf_thresh=CONF_THRESH,
                    debug=DEBUG,
                )

            active_crop = None
            active_conf = None
            active_det_conf = None

            # ── Draw all NMS detections for visualization ───────────────────
            for i, (x1, y1, x2, y2, conf) in enumerate(detections):
                x1, y1, x2, y2 = clamp_box(x1, y1, x2, y2, orig_w, orig_h)
                color = (0, 255, 0) if i == 0 else (0, 255, 255)
                thickness = 2 if i == 0 else 1
                cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
                cv2.putText(
                    display,
                    f"det {i}: {conf:.2f}",
                    (x1, max(20, y1 - 8 - 18 * i)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            # ── Recognize using top detection ────────────────────────────────
            if detections:
                x1, y1, x2, y2, det_conf = detections[0]
                x1, y1, x2, y2 = clamp_box(x1, y1, x2, y2, orig_w, orig_h)

                pad_x1 = max(0, x1 - PAD_LEFT)
                pad_y1 = max(0, y1 - PAD_TOP)
                pad_x2 = min(orig_w, x2 + PAD_RIGHT)
                pad_y2 = min(orig_h, y2 + PAD_BOTTOM)

                cv2.rectangle(display, (pad_x1, pad_y1), (pad_x2, pad_y2), (255, 0, 0), 2)

                crop = frame[pad_y1:pad_y2, pad_x1:pad_x2]
                if crop.size > 0:
                    lpr_tensor = preprocess_lpr(crop)
                    lpr_out = lpr_model.infer(lpr_tensor)
                    pred_ids, read_conf = pick_lpr_prediction(lpr_out)
                    plate_text = ctc_decode(pred_ids)
                    last_read_conf = read_conf
                    last_crop = crop.copy()
                    active_crop = crop
                    active_conf = read_conf
                    active_det_conf = det_conf

                    label_text = plate_text if plate_text else "<blank>"
                    label_y = max(25, y1 - 35)
                    cv2.putText(
                        display,
                        label_text,
                        (x1, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

                    print(
                        f"Plate: {plate_text or '<blank>'}  "
                        f"det_conf={det_conf:.2f}  read_conf={read_conf:.2f}  "
                        f"box=({x1},{y1},{x2},{y2})      ",
                        end="\r",
                    )
                else:
                    print("Top detection crop was empty                          ", end="\r")
            else:
                print("No plate detected                                     ", end="\r")

            # ── Overlays ─────────────────────────────────────────────────────
            label = f"Plate: {plate_text}" if plate_text else "No plate detected"
            label_color = (0, 255, 0) if plate_text else (0, 0, 255)
            cv2.putText(
                display,
                label,
                (30, orig_h - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                label_color,
                3,
                cv2.LINE_AA,
            )

            line2 = f"FPS: {fps:.1f}   detect_every: {DETECT_EVERY}"
            if active_det_conf is not None:
                line2 += f"   det_conf: {active_det_conf:.2f}"
            if active_conf is not None:
                line2 += f"   read_conf: {active_conf:.2f}"
            cv2.putText(
                display,
                line2,
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if last_detect_stats:
                cv2.putText(
                    display,
                    last_detect_stats,
                    (30, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            draw_crop_inset(display, active_crop if active_crop is not None else last_crop, plate_text, last_read_conf)

            cv2.imshow("LPDNet + LPRNet (TensorRT)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
