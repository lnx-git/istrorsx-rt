import time
import cv2 as cv
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

logger = None

# TensorRT engine state (all set up once in model_init(), used by every
# model_predict() call). No class wrapper -- kept as free functions/module
# globals to match visionn_model.py's plain-function interface exactly, same
# as the SegFormer TRT server (script/visionn_segfb0-trt_server/visionn_model.py).
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
engine = None
context = None
stream = None

input_tensor_name = None
input_dtype = None
d_input = None

# YOLO11-seg has TWO outputs (unlike SegFormer's one): a 3-D "detections"
# tensor (boxes + class score + mask coefficients per candidate) and a 4-D
# "prototypes" tensor (shared mask basis). Tensor NAMES are not hardcoded the
# way SegFormer's "input"/"logits" are below in model_init() -- this engine's
# output order is not guaranteed, so both are discovered generically and
# told apart by how many dimensions they have. See
# ~/pablo_trenovanie_NN-mix/road-seg-yolo-20260826-win/infer_trt.py's
# TrtSegmenter.__init__, which this is ported from.
det_tensor_name = None
d_det = None
h_det = None
proto_tensor_name = None
d_proto = None
h_proto = None

# Fixed TensorRT engine input size (H, W) -- like segfb0-trt, this engine was
# built with explicit (not dynamic) shapes. Unlike segfb0-trt, this is NOT a
# hardcoded module constant: it is read back from the engine's own input
# shape in model_init() (ishape[2], ishape[3]) so it can never silently drift
# out of sync with whichever .engine file MODEL_PATH actually points at.
MODEL_H = None
MODEL_W = None

# Confidence / NMS IoU thresholds, ported from infer_trt.py's own CLI
# defaults (--conf 0.10, --iou 0.7). 0.10 is the lowest threshold at which the
# reference project's own validation split has zero fully-missed images (see
# roadseg-yolo's README).
CONF_THR = 0.10
IOU_THR = 0.7

# NOTE: this .engine file is specific to the exact Jetson + JetPack/TensorRT
# version it was built on (trtexec, run once on-device) -- it is NOT
# portable between machines or TensorRT versions. See doc/ai/01_architecture.md.
MODEL_PATH = "model_roadyolo-trt/road_yolo11s_seg_fp16.engine"

### mtime ###

def mtime_begin():
    return int(time.time() * 1000)

def mtime_delta(t):
    # return time difference in milliseconds
    return int(mtime_begin() - t)

def mtime_delta2(t1, t2):
    return int(t2 - t1)

def mtime_end(ss, t):
    logger.debug('visionn_model::mtime(): m="' + ss + '", dt=' + str(mtime_delta(t)))

### cuda ###

def cuda_check(err):
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError('visionn_model::cuda_check(): msg="CUDA error", err="' + str(err) + '"')

### road segmentation pre/postprocess ###
#
# Ported near-verbatim from roadseg_post.py in
# ~/pablo_trenovanie_NN-mix/road-seg-yolo-20260826-win/ (comments translated
# Slovak->English). That module is shared between an ONNX/PC path and a
# TensorRT/Jetson path in the reference project specifically so the two
# runtimes can never disagree on postprocess; here there is only ever one
# runtime (this file), so the functions are inlined directly into
# visionn_model.py instead of split into a sibling module -- keeps the
# "one self-contained visionn_model.py per variant" convention every other
# variant already follows, rather than introducing a module split that would
# only pay for itself with a second consumer.
#
# Verified (in the reference project) to give a bit-identical result to
# ultralytics's own postprocess (IoU 1.00000) -- the step order in
# postprocess() below is therefore not arbitrary, see its own comments.

def letterbox(bgr, size):
    """Resize to `size` preserving aspect ratio, padding with gray bars.

    `size` is (H, W) of the model input. When the image's aspect ratio
    already matches (e.g. model 480x640 and a 640x480 frame), padding is
    zero and nothing is wasted.
    """
    ih, iw = size
    h, w = bgr.shape[:2]
    r = min(ih / h, iw / w)
    nh, nw = round(h * r), round(w * r)
    top, left = (ih - nh) // 2, (iw - nw) // 2

    out = np.full((ih, iw, 3), 114, dtype=np.uint8)
    out[top:top + nh, left:left + nw] = cv.resize(bgr, (nw, nh), interpolation=cv.INTER_LINEAR)
    return out, r, left, top

def preprocess(bgr, size):
    """BGR frame -> (input tensor 1x3xHxW float32 0..1, r, dx, dy)."""
    lb, r, dx, dy = letterbox(bgr, size)
    x = cv.cvtColor(lb, cv.COLOR_BGR2RGB).astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))[None]
    return np.ascontiguousarray(x), r, dx, dy

def nms(boxes, scores, iou_thr):
    """Pure-numpy NMS over xyxy boxes."""
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][ovr <= iou_thr]
    return np.array(keep, dtype=np.int64)

def postprocess(out0, out1, size, r, dx, dy, orig_shape, conf_thr, iou_thr):
    """Model outputs -> binary road mask at the original resolution.

    `size` is (H, W) of the model input. Returns (mask uint8 0/1, num detections).
    """
    pred = out0[0]                      # (4+nc+32, N)
    protos = out1[0]                    # (32, mh, mw)
    nm = protos.shape[0]
    nc = pred.shape[0] - 4 - nm

    boxes = pred[:4].T                  # (N,4) cxcywh
    scores_all = pred[4:4 + nc].T       # (N,nc)
    coeffs = pred[4 + nc:].T            # (N,32)

    cls = scores_all.argmax(1)
    conf = scores_all.max(1)

    m = conf > conf_thr
    if not m.any():
        return np.zeros(orig_shape[:2], np.uint8), 0
    boxes, conf, cls, coeffs = boxes[m], conf[m], cls[m], coeffs[m]

    # cxcywh -> xyxy
    xy = np.empty_like(boxes)
    xy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    keep = nms(xy, conf, iou_thr)
    xy, coeffs = xy[keep], coeffs[keep]

    # Mask at prototype resolution, still as logits -- thresholding at 0 on
    # the logit is the same as 0.5 after sigmoid, so sigmoid need not be
    # computed at all.
    mh, mw = protos.shape[1:]
    h, w = orig_shape[:2]
    logits = (coeffs @ protos.reshape(nm, -1)).reshape(-1, mh, mw)

    # Step order intentionally matches ultralytics (ops.process_mask_native):
    # first crop the letterbox padding at prototype resolution, then do ONE
    # interpolation straight to the original resolution. Composing this any
    # other way (e.g. via an intermediate resize to imgsz) gives visibly
    # different mask edges.
    gain = min(mh / h, mw / w)
    pad_w = (mw - round(w * gain)) / 2
    pad_h = (mh - round(h * gain)) / 2
    top, left = round(pad_h - 0.1), round(pad_w - 0.1)
    bottom, right = mh - round(pad_h + 0.1), mw - round(pad_w + 0.1)
    logits = logits[:, top:bottom, left:right]

    # boxes from imgsz coords back to original image coords
    boxes = xy.copy()
    boxes[:, [0, 2]] -= dx
    boxes[:, [1, 3]] -= dy
    boxes /= r

    cols = np.arange(w, dtype=np.float32)[None, :]
    rows = np.arange(h, dtype=np.float32)[:, None]

    combined = np.zeros((h, w), dtype=np.uint8)
    for lg, (bx1, by1, bx2, by2) in zip(logits, boxes):
        up = cv.resize(lg, (w, h), interpolation=cv.INTER_LINEAR)
        m = up > 0.0
        # box-crop only at original resolution, with subpixel bounds
        m &= (cols >= bx1) & (cols < bx2) & (rows >= by1) & (rows < by2)
        combined |= m.astype(np.uint8)

    return combined, len(keep)

### model ###

def model_init(log):
    global logger
    global engine, context, stream
    global input_tensor_name, input_dtype, d_input
    global det_tensor_name, d_det, h_det
    global proto_tensor_name, d_proto, h_proto
    global MODEL_H, MODEL_W

    logger = log

    # Load TensorRT engine (roadyolo-trt = YOLO11s-seg road segmentation, see
    # doc/ai/01_architecture.md) and set up a persistent execution context +
    # device buffers, reused by every model_predict() call.
    logger.info('visionn_model::model_init(): msg="load model - start...", path="' + MODEL_PATH + '"')
    with open(MODEL_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()

    # Generic I/O discovery (name/shape/dtype/mode), ported from infer_trt.py's
    # TrtSegmenter.__init__ -- deliberately not hardcoded tensor names like
    # segfb0-trt's model_init() does ("input"/"logits"), since this engine
    # has two outputs whose names/order this file must not assume.
    inputs = []
    outputs = []
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        shape = tuple(engine.get_tensor_shape(name))
        dtype = trt.nptype(engine.get_tensor_dtype(name))
        host = np.empty(shape, dtype=dtype)

        err, dev = cudart.cudaMalloc(host.nbytes)
        cuda_check(err)
        context.set_tensor_address(name, dev)

        entry = {"name": name, "shape": shape, "dtype": dtype, "host": host, "dev": dev}
        if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
            inputs.append(entry)
        else:
            outputs.append(entry)

    if len(inputs) != 1:
        raise RuntimeError(
            'visionn_model::model_init(): msg="expected exactly one input tensor", '
            'count=' + str(len(inputs)))
    if len(outputs) != 2:
        raise RuntimeError(
            'visionn_model::model_init(): msg="expected exactly two output tensors '
            '(detections + prototypes)", count=' + str(len(outputs)))

    # engine output order is not guaranteed -- sort so outputs[0] is always
    # the 3-D detections tensor and outputs[1] the 4-D prototypes tensor.
    outputs.sort(key=lambda o: len(o["shape"]))
    det_entry, proto_entry = outputs[0], outputs[1]
    input_entry = inputs[0]

    input_tensor_name = input_entry["name"]
    input_dtype = input_entry["dtype"]
    d_input = input_entry["dev"]
    MODEL_H, MODEL_W = int(input_entry["shape"][2]), int(input_entry["shape"][3])

    det_tensor_name = det_entry["name"]
    d_det = det_entry["dev"]
    h_det = det_entry["host"]

    proto_tensor_name = proto_entry["name"]
    d_proto = proto_entry["dev"]
    h_proto = proto_entry["host"]

    err, stream = cudart.cudaStreamCreate()
    cuda_check(err)

    logger.info('visionn_model::model_init(): image_size="' + str([MODEL_H, MODEL_W]) + '"')
    logger.info('visionn_model::model_init(): input_tensor_name="' + input_tensor_name
                + '", det_tensor_name="' + det_tensor_name
                + '", proto_tensor_name="' + proto_tensor_name + '"')
    logger.info('visionn_model::model_init(): det_shape="' + str(det_entry["shape"])
                + '", proto_shape="' + str(proto_entry["shape"]) + '"')
    logger.info('visionn_model::model_init(): msg="load model - finished"')
    return context

def model_predict(img):
    global context, stream
    global d_input, input_dtype, d_det, h_det, d_proto, h_proto

    t0 = mtime_begin()
    img_h, img_w = img.shape[:2]
    x, r, dx, dy = preprocess(img, (MODEL_H, MODEL_W))
    if x.dtype != input_dtype:
        x = x.astype(input_dtype)
    x = np.ascontiguousarray(x)

    t1 = mtime_begin()
    err = cudart.cudaMemcpyAsync(
        d_input,
        x.ctypes.data,
        x.nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
        stream,
    )[0]
    cuda_check(err)

    ok = context.execute_async_v3(stream)
    if not ok:
        raise RuntimeError('visionn_model::model_predict(): msg="TensorRT execute_async_v3 failed"')

    err = cudart.cudaMemcpyAsync(
        h_det.ctypes.data,
        d_det,
        h_det.nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
        stream,
    )[0]
    cuda_check(err)
    err = cudart.cudaMemcpyAsync(
        h_proto.ctypes.data,
        d_proto,
        h_proto.nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
        stream,
    )[0]
    cuda_check(err)

    cuda_check(cudart.cudaStreamSynchronize(stream)[0])

    t2 = mtime_begin()
    mask01, ndet = postprocess(h_det, h_proto, (MODEL_H, MODEL_W), r, dx, dy,
                                (img_h, img_w), CONF_THR, IOU_THR)

    pred_img = (mask01 * 255).astype(np.uint8)
    pred_img = np.expand_dims(pred_img, axis=2)    # HxWx1, matches legacy model_predict()'s output shape

    logger.debug('visionn_model::model_predict(): ndet=' + str(ndet)
                 + ', dt1=' + str(mtime_delta2(t0, t1))
                 + ', dt2=' + str(mtime_delta2(t1, t2))
                 + ', dt3=' + str(mtime_delta(t2)))
    return pred_img
