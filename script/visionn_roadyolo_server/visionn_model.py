import time
import cv2 as cv
import numpy as np
import onnxruntime as ort

logger = None

# ONNX Runtime state, set up once in model_init() and reused by every
# model_predict() call. Same plain-function interface as every other variant's
# visionn_model.py (segfb0 = SegFormer/ONNX, roadyolo-trt = this model on
# TensorRT); only the runtime differs.
ort_sess = None
input_tensor_name = None
det_output_name = None      # 3-D: boxes + class scores + mask coefficients
proto_output_name = None    # 4-D: mask prototypes

# ONNX input size (H, W). Read back from the model itself, like
# ../visionn_roadyolo-trt_server reads it from the engine, so a re-export at a
# different resolution needs no edit here. This export is 480x640 (see
# archive/260826_road-seg-yolo-win/export_model.py: rectangular input is both
# more accurate and 27 % faster than the square one it was trained at).
MODEL_H = None
MODEL_W = None

# Same thresholds as the TensorRT variant and as infer_trt.py/infer_onnx.py in
# the reference project. 0.10 rather than the usual 0.25 is the lowest value at
# which no validation image comes back completely empty.
CONF_THR = 0.10
IOU_THR = 0.7

# CPU/ONNX, no GPU needed -- the point of this variant is comparing NN models on
# a plain VM. The Jetson runs ../visionn_roadyolo-trt_server instead, the same
# model as a TensorRT engine. See doc/ai/01_architecture.md.
MODEL_PATH = "model_roadyolo/road_yolo11s_seg.onnx"

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

### road segmentation pre/postprocess ###
#
# Byte-identical to the same block in ../visionn_roadyolo-trt_server/visionn_model.py,
# itself a near-verbatim port of roadseg_post.py in archive/260826_road-seg-yolo-win/
# (comments translated Slovak->English). That module is shared between an ONNX/PC path and a
# TensorRT/Jetson path in the reference project specifically so the two
# runtimes can never disagree on postprocess; here too the functions are inlined directly into
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
    global ort_sess
    global input_tensor_name, det_output_name, proto_output_name
    global MODEL_H, MODEL_W

    logger = log

    logger.info('visionn_model::model_init(): msg="load model - start...", path="' + MODEL_PATH + '"')
    ort_sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

    inp = ort_sess.get_inputs()[0]
    input_tensor_name = inp.name
    MODEL_H, MODEL_W = int(inp.shape[2]), int(inp.shape[3])

    outs = ort_sess.get_outputs()
    if len(outs) != 2:
        raise RuntimeError(
            'visionn_model::model_init(): msg="expected exactly two output tensors '
            '(detections + prototypes)", count=' + str(len(outs)))
    # Told apart by rank, not by position: the export's output order is not
    # something this file should assume (infer_onnx.py in the reference project
    # does assume it; the TensorRT variant sorts the same way this does).
    outs = sorted(outs, key=lambda o: len(o.shape))
    det_output_name, proto_output_name = outs[0].name, outs[1].name

    logger.info('visionn_model::model_init(): image_size="' + str([MODEL_H, MODEL_W]) + '"')
    logger.info('visionn_model::model_init(): input_tensor_name="' + input_tensor_name
                + '", det_tensor_name="' + det_output_name
                + '", proto_tensor_name="' + proto_output_name + '"')
    logger.info('visionn_model::model_init(): det_shape="' + str(tuple(outs[0].shape))
                + '", proto_shape="' + str(tuple(outs[1].shape)) + '"')
    logger.info('visionn_model::model_init(): msg="load model - finished"')
    return ort_sess

def model_predict(img):
    global ort_sess
    global input_tensor_name, det_output_name, proto_output_name

    t0 = mtime_begin()
    img_h, img_w = img.shape[:2]
    x, r, dx, dy = preprocess(img, (MODEL_H, MODEL_W))

    t1 = mtime_begin()
    out0, out1 = ort_sess.run([det_output_name, proto_output_name], {input_tensor_name: x})

    t2 = mtime_begin()
    mask01, ndet = postprocess(out0, out1, (MODEL_H, MODEL_W), r, dx, dy,
                                (img_h, img_w), CONF_THR, IOU_THR)

    pred_img = (mask01 * 255).astype(np.uint8)
    pred_img = np.expand_dims(pred_img, axis=2)    # HxWx1, matches legacy model_predict()'s output shape

    logger.debug('visionn_model::model_predict(): ndet=' + str(ndet)
                 + ', dt1=' + str(mtime_delta2(t0, t1))
                 + ', dt2=' + str(mtime_delta2(t1, t2))
                 + ', dt3=' + str(mtime_delta(t2)))
    return pred_img
