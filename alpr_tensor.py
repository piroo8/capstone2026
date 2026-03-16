import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ── Engine paths ──────────────────────────────────────────────────────────────
LPD_ENGINE_PATH = "/home/jetson/engines/lpdnet_fp32.engine"  # FP32
LPR_ENGINE_PATH = "/home/jetson/engines/lprnet_fp32.engine"  # FP32

LPD_W, LPD_H = 640, 480
CHARS = "0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ"
BLANK = 35

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

# ── TensorRT engine loader ────────────────────────────────────────────────────
class TRTModel:
    """Loads a TensorRT engine and runs inference."""

    def __init__(self, engine_path):
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine  = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # Allocate host + device buffers for each binding
        self.inputs  = []
        self.outputs = []
        self.bindings = []

        for binding in self.engine:
            shape = self.engine.get_binding_shape(binding)
            size  = trt.volume(shape)
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))

            host_mem   = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(device_mem))

            if self.engine.binding_is_input(binding):
                self.inputs.append( {"host": host_mem, "device": device_mem, "shape": shape})
            else:
                self.outputs.append({"host": host_mem, "device": device_mem, "shape": shape})

    def infer(self, *input_arrays):
        # Copy inputs to device
        for i, arr in enumerate(input_arrays):
            np.copyto(self.inputs[i]["host"], arr.ravel())
            cuda.memcpy_htod(self.inputs[i]["device"], self.inputs[i]["host"])

        # Run inference
        self.context.execute_v2(self.bindings)

        # Copy outputs back to host
        results = []
        for out in self.outputs:
            cuda.memcpy_dtoh(out["host"], out["device"])
            results.append(out["host"].reshape(out["shape"]))

        return results

# ── Load both engines ─────────────────────────────────────────────────────────
print("Loading LPD engine...")
lpd_model = TRTModel(LPD_ENGINE_PATH)

print("Loading LPR engine...")
lpr_model = TRTModel(LPR_ENGINE_PATH)

print("Both engines loaded")

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
    orig_h, orig_w = frame.shape[:2]
    scale_x = orig_w / LPD_W
    scale_y = orig_h / LPD_H
    img = cv2.resize(frame, (LPD_W, LPD_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))
    x = np.expand_dims(x, 0)
    return x, scale_x, scale_y

# ── LPDNet output decoder ─────────────────────────────────────────────────────
def decode_lpd(cov, bbox, scale_x, scale_y, conf_thresh=0.5):
    cov  = cov.squeeze()
    bbox = bbox.squeeze()
    if bbox.shape[-1] == 4:
        bbox = np.transpose(bbox, (2, 0, 1))

    grid_h, grid_w = cov.shape
    cell_w = LPD_W / grid_w
    cell_h = LPD_H / grid_h
    BBOX_SCALE_X = 35.0
    BBOX_SCALE_Y = 20.0

    raw_boxes = []
    for gy in range(grid_h):
        for gx in range(grid_w):
            conf = float(cov[gy, gx])
            if conf < conf_thresh:
                continue
            cx = (gx + 0.5) * cell_w
            cy = (gy + 0.5) * cell_h
            x1 = int((cx - bbox[0, gy, gx] * BBOX_SCALE_X) * scale_x)
            y1 = int((cy - bbox[1, gy, gx] * BBOX_SCALE_Y) * scale_y)
            x2 = int((cx + bbox[2, gy, gx] * BBOX_SCALE_X) * scale_x)
            y2 = int((cy + bbox[3, gy, gx] * BBOX_SCALE_Y) * scale_y)
            if x2 <= x1 or y2 <= y1:
                continue
            raw_boxes.append((x1, y1, x2, y2, conf))

    if not raw_boxes:
        return []

    boxes_xywh = [[x1, y1, x2-x1, y2-y1] for x1, y1, x2, y2, _ in raw_boxes]
    scores     = [c for *_, c in raw_boxes]
    indices    = cv2.dnn.NMSBoxes(boxes_xywh, scores, conf_thresh, 0.4)

    result = []
    if len(indices) > 0:
        for i in indices.flatten():
            result.append(raw_boxes[i])
    result.sort(key=lambda b: b[4], reverse=True)
    return result

# ── LPRNet preprocessing ──────────────────────────────────────────────────────
def preprocess_lpr(crop_bgr):
    img = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (96, 48))
    x   = img.astype(np.float32) / 255.0
    x   = (x - 0.5) / 0.5
    x   = np.transpose(x, (2, 0, 1))
    return np.expand_dims(x, 0).astype(np.float32)

# ── Camera ────────────────────────────────────────────────────────────────────
gst_pipeline = (
    "nvarguscamerasrc sensor-mode=4 ! "
    "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, "
    "format=(string)NV12, framerate=(fraction)30/1 ! "
    "nvvidconv flip-method=2 ! "
    "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Camera failed to open")
    exit()

print("Running — press Ctrl+C to quit")

DETECT_EVERY = 15
frame_id     = 0
detections   = []

PAD_LEFT   = 15
PAD_RIGHT  = 0
PAD_TOP    = 10
PAD_BOTTOM = 10

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    orig_h, orig_w = frame.shape[:2]
    frame_id += 1

    # ── Detect every N frames ─────────────────────────────────────────────────
    if frame_id % DETECT_EVERY == 0:
        lpd_tensor, scale_x, scale_y = preprocess_lpd(frame)
        lpd_out    = lpd_model.infer(lpd_tensor)
        detections = decode_lpd(lpd_out[0], lpd_out[1], scale_x, scale_y)

    # ── Recognise using last known detection ──────────────────────────────────
    if detections:
        x1, y1, x2, y2, conf = detections[0]

        x1 = max(0, min(orig_w-1, x1))
        y1 = max(0, min(orig_h-1, y1))
        x2 = max(0, min(orig_w,   x2))
        y2 = max(0, min(orig_h,   y2))

        crop = frame[max(0, y1-PAD_TOP)  : min(orig_h, y2+PAD_BOTTOM),
                     max(0, x1-PAD_LEFT) : min(orig_w,  x2+PAD_RIGHT)]

        if crop.size > 0:
            lpr_tensor = preprocess_lpr(crop)
            lpr_out    = lpr_model.infer(lpr_tensor)

            # LPR engine outputs: [max_values (1,24), argmax_ids (1,24)]
            # argmax is the second output — character indices
            pred       = lpr_out[1][0].astype(int)
            
            plate_text = ctc_decode(pred)
            # lpr_out[0] is per-character confidence scores (1, 24)
            # take mean of non-blank positions as overall read confidence
            char_confs  = lpr_out[0][0]          # shape (24,)
            read_conf   = float(char_confs.mean())
            print(f"Plate: {plate_text}  det_conf={conf:.2f}  read_conf={read_conf:.2f}")
    else:
        print("No plate detected", end="\r")