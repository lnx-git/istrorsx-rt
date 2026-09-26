import time
import cv2 as cv
import numpy as np
import onnxruntime as ort

logger = None

ort_sess = None
input_tensor_name = None
output_tensor_name = None

# fixed ONNX input size (H, W) -- export_onnx.py only makes the batch axis
# dynamic, see ~/road-segmentation-20260314-trt/doc/road-segmentation-20260314.md
MODEL_H = 384
MODEL_W = 512

MODEL_PATH = "model_segfb0/segformer_b0_road.onnx"

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

### model ###

def model_init(log):
    global logger
    global ort_sess
    global input_tensor_name
    global output_tensor_name

    logger = log

    # Load ONNX model (segfb0 = SegFormer-B0 road segmentation, see
    # doc/ai/01_architecture.md § "NN Inference Service - SegFormer-B0 Road
    # Segmentation") and create an inference session.
    logger.info('visionn_model::model_init(): msg="load model - start...", path="' + MODEL_PATH + '"')
    ort_sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

    # Discover input/output tensor names from the model itself (mirrors the
    # legacy TF code's own graph_def node-name discovery loop).
    input_tensor_name = ort_sess.get_inputs()[0].name
    output_tensor_name = ort_sess.get_outputs()[0].name

    logger.info('visionn_model::model_init(): image_size="' + str([MODEL_H, MODEL_W]) + '"')
    logger.info('visionn_model::model_init(): input_tensor_name="' + input_tensor_name + '", output_tensor_name="' + output_tensor_name + '"')
    logger.info('visionn_model::model_init(): msg="load model - finished"')
    return ort_sess

def model_predict(img):
    global ort_sess
    global input_tensor_name
    global output_tensor_name

    t0 = mtime_begin()
    img_h, img_w = img.shape[:2]
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    img_rgb = cv.resize(img_rgb, (MODEL_W, MODEL_H), interpolation=cv.INTER_AREA)
    img_np = img_rgb.astype(np.float32) / 255
    img_np = np.transpose(img_np, (2, 0, 1))[None, ...]  # 1x3xMODEL_HxMODEL_W

    t1 = mtime_begin()
    logits = ort_sess.run([output_tensor_name], {input_tensor_name: img_np})[0]    # 1x2xMODEL_HxMODEL_W

    t2 = mtime_begin()
    pred = np.argmax(logits, axis=1)[0].astype(np.uint8)    # MODEL_HxMODEL_W, 0/1

    pred_img = pred * 255
    pred_img = cv.resize(pred_img, (img_w, img_h), interpolation=cv.INTER_NEAREST)    # back to the caller's own image size
    pred_img = np.expand_dims(pred_img, axis=2)    # HxWx1, matches legacy model_predict()'s output shape

    logger.debug('visionn_model::model_predict(): dt1=' + str(mtime_delta2(t0, t1)) + ', dt2=' + str(mtime_delta2(t1, t2)) + ', dt3=' + str(mtime_delta(t2)))
    return pred_img
