import cv2
import numpy as np
import onnxruntime as ort

# ── Model paths ───────────────────────────────────────────────────────────────
LPD_PATH = "/home/jetson/lpdnet/lpdnet.onnx"
LPR_PATH = "/home/jetson/lprnet_vdeployable_onnx_v1.1/us_lprnet_baseline18_deployable.onnx"

# ── Load models ───────────────────────────────────────────────────────────────
lpd = ort.InferenceSession(LPD_PATH)
lpr = ort.InferenceSession(LPR_PATH)

lpd_input = lpd.get_inputs()[0].name
lpr_input = lpr.get_inputs()[0].name

# LPDNet expects (N, 3, 480, 640)
LPD_W, LPD_H = 640, 480

# ── LPRNet character set ──────────────────────────────────────────────────────
CHARS = "0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ"
BLANK = 35

# ── CTC decoder ───────────────────────────────────────────────────────────────
def ctc_decode(pred):
    plate, prev = [], BLANK
    for p in pred:
        if p != prev and p != BLANK:
            plate.append(CHARS[p])
        prev = p
    return "".join(plate)

# ── LPDNet preprocessing ──────────────────────────────────────────────────────
def preprocess_lpd(frame):
    """
    Resize to 640x480, normalize to [0,1].
    DetectNet_v2 does NOT use [-1,1] — just 0-1 with no mean subtraction.
    Returns tensor (1,3,480,640) and the scale factors to map boxes back.
    """
    orig_h, orig_w = frame.shape[:2]
    scale_x = orig_w / LPD_W
    scale_y = orig_h / LPD_H

    img = cv2.resize(frame, (LPD_W, LPD_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    x = img.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))        # HWC -> CHW
    x = np.expand_dims(x, 0)              # add batch

    return x, scale_x, scale_y

# ── LPDNet output decoder ─────────────────────────────────────────────────────
def decode_lpd(cov, bbox, scale_x, scale_y, conf_thresh=0.5):
    """
    cov  shape: (1, 30, 40)  — confidence per grid cell
    bbox shape: (4, 30, 40)  — absolute x1,y1,x2,y2 in 640x480 space

    DetectNet_v2 bbox values are already absolute pixel coords in input space.
    We just scale them back to the original camera resolution.
    """
    cov = cov.squeeze()
    bbox = bbox.squeeze()
    if bbox.shape[-1] == 4:
        bbox = np.transpose(bbox, (2,0,1))
    
    grid_h, grid_w = cov.shape

    # find the highest confidence cell and print its bbox values raw
    idx = np.unravel_index(cov.argmax(), cov.shape)
    gy, gx = idx
    print(f"\nBEST CELL conf={cov[gy,gx]:.3f} raw bbox: x1={bbox[0,gy,gx]:.3f} y1={bbox[1,gy,gx]:.3f} x2={bbox[2,gy,gx]:.3f} y2={bbox[3,gy,gx]:.3f}")

    raw_boxes = []

    for gy in range(grid_h):
        for gx in range(grid_w):
            conf = float(cov[gy, gx])
            if conf < conf_thresh:
                continue

            # DetectNet_v2 GridBox format:
            # bbox values are offsets from grid cell in units of grid cell size
            cell_w = LPD_W / grid_w   # 640/40 = 16 pixels per cell
            cell_h = LPD_H / grid_h   # 480/30 = 16 pixels per cell

            # cell center in 640x480 space
            cx = (gx + 0.5) * cell_w
            cy = (gy + 0.5) * cell_h

            BBOX_SCALE_X = 35.0
            BBOX_SCALE_Y = 20.0   # plates are wide not tall
            x1 = int((cx - bbox[0, gy, gx] * BBOX_SCALE_X) * scale_x)
            y1 = int((cy - bbox[1, gy, gx] * BBOX_SCALE_Y) * scale_y)
            x2 = int((cx + bbox[2, gy, gx] * BBOX_SCALE_X) * scale_x)
            y2 = int((cy + bbox[3, gy, gx] * BBOX_SCALE_Y) * scale_y)

            # Sanity check — skip degenerate boxes
            if x2 <= x1 or y2 <= y1:
                continue

            raw_boxes.append((x1, y1, x2, y2, conf))

    if not raw_boxes:
        return []

    # NMS — remove overlapping boxes, keep highest confidence
    boxes_xywh = [[x1, y1, x2 - x1, y2 - y1] for x1, y1, x2, y2, _ in raw_boxes]
    scores     = [conf for *_, conf in raw_boxes]

    indices = cv2.dnn.NMSBoxes(
        bboxes          = boxes_xywh,
        scores          = scores,
        score_threshold = conf_thresh,
        nms_threshold   = 0.4
    )

    result = []
    if len(indices) > 0:
        for i in indices.flatten():
            result.append(raw_boxes[i])

    # Sort by confidence, best first
    result.sort(key=lambda b: b[4], reverse=True)
    return result

# ── LPRNet preprocessing ──────────────────────────────────────────────────────
def preprocess_lpr(crop_bgr):
    """Crop -> RGB -> 96x48 -> normalize to [-1,1] -> tensor."""
    img = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (96, 48))
    x   = img.astype(np.float32) / 255.0
    x   = (x - 0.5) / 0.5                 # TAO expects [-1, 1]
    x   = np.transpose(x, (2, 0, 1))
    return np.expand_dims(x, 0)

# ── GStreamer pipeline ─────────────────────────────────────────────────────────
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=4 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! "
    "appsink max-buffers=1 drop=true"
)
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Camera failed to open")
    exit()

print("Running LPDNet + LPRNet — press ESC to quit")

plate_text = ""   # keep last good read on screen

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    orig_h, orig_w = frame.shape[:2]
    display = frame.copy()

    # ── Step 1: detect plate with LPDNet ──────────────────────────────────────
    lpd_tensor, scale_x, scale_y = preprocess_lpd(frame)
    lpd_out  = lpd.run(None, {lpd_input: lpd_tensor})

    cov  = lpd_out[0]   # (1, 30, 40)
    bbox = lpd_out[1]   # (4, 30, 40)

    ##
    cov_squeezed = lpd_out[0].squeeze()
    print(f"max conf: {cov_squeezed.max():.3f}  mean: {cov_squeezed.mean():.3f}", end="\r")

    detections = decode_lpd(cov, bbox, scale_x, scale_y, conf_thresh=0.5)

    if detections:
        # Take highest-confidence detection
        x1, y1, x2, y2, conf = detections[0]

        # Clamp to frame
        x1 = max(0, min(orig_w-1, x1))
        y1 = max(0, min(orig_h-1, y1))
        x2 = max(0, min(orig_w, x2))
        y2 = max(0, min(orig_h, y2))

        #if x2 - x1 < 10 or y2 - y1 < 10:
        #    continue

        #crop = frame[y1:y2, x1:x2]

        pad_left = 15
        pad_right = 0
        pad_top = 10
        pad_bottom = 10
        crop = frame[max(0,y1-pad_top):min(orig_h,y2+pad_bottom),
                    max(0,x1-pad_left):min(orig_w,x2+pad_right)]

        print("crop shape:", crop.shape)

        if crop.size > 0:
            # ── Step 2: read characters with LPRNet ───────────────────────
            lpr_tensor = preprocess_lpr(crop)
            lpr_out    = lpr.run(None, {lpr_input: lpr_tensor})
            plate_text = ctc_decode(lpr_out[0][0])

            # Green box around plate
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Confidence above box
            cv2.putText(display,
                        f"{conf:.2f}",
                        (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Draw the padded crop box that goes into LPRNet in blue
            cv2.rectangle(display,
                          (max(0, x1 - pad_left), max(0, y1 - pad_top)),
                          (min(orig_w, x2 + pad_right), min(orig_h, y2 + pad_bottom)),
                          (255, 0, 0), 2)

        print(f"Plate: {plate_text}  conf={conf:.2f}  box=({x1},{y1},{x2},{y2})", end="\r")
    else:
        print("No plate detected                                    ", end="\r")

    # ── Plate text overlay — always shows last good read ──────────────────────
    label = f"Plate: {plate_text}" if plate_text else "No plate detected"
    color = (0, 255, 0) if plate_text else (0, 0, 255)
    cv2.putText(display, label,
                (30, orig_h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 3, cv2.LINE_AA)

    cv2.imshow("LPDNet + LPRNet", display)

    if cv2.waitKey(1) == 27:   # ESC
        break

cap.release()
cv2.destroyAllWindows()