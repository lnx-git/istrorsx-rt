import time
import cv2 as cv
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

logger = None

# TensorRT engine state (all set up once in model_init(), used by every
# model_predict() call). No class wrapper -- kept as free functions/module
# globals to match visionn_model.py's plain-function interface exactly, same
# as the ONNX/CPU version (script/visionn_segfb0_server/visionn_model.py).
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
engine = None
context = None
stream = None
d_input = None
d_output = None
h_output = None
input_tensor_name = None
output_tensor_name = None

# fixed TensorRT engine input size (H, W) -- the engine was built with
# explicit (not dynamic) shapes, see
# ~/road-segmentation-20260314-trt/doc/jetson-tensorrt-inference.md
MODEL_H = 384
MODEL_W = 512

# NOTE: this .engine file is specific to the exact Jetson + JetPack/TensorRT
# version it was built on (trtexec, run once on-device) -- it is NOT
# portable between machines or TensorRT versions, unlike the ONNX/CPU
# server's .onnx model. See doc/ai/01_architecture.md.
MODEL_PATH = "model_segfb0-trt/segformer_b0_road_fp16.engine"

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

### model ###

def model_init(log):
    global logger
    global engine, context, stream, d_input, d_output, h_output
    global input_tensor_name, output_tensor_name

    logger = log

    # Load TensorRT engine (segfb0 = SegFormer-B0 road segmentation, see
    # doc/ai/01_architecture.md § "NN Inference Service - SegFormer-B0 Road
    # Segmentation") and set up a persistent execution context + device
    # buffers, reused by every model_predict() call.
    logger.info('visionn_model::model_init(): msg="load model - start...", path="' + MODEL_PATH + '"')
    with open(MODEL_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()

    # Same names as the exported ONNX graph (segformer_b0_road.onnx) -- this
    # engine was built directly from it, see export_onnx.py.
    input_tensor_name = "input"
    output_tensor_name = "logits"
    context.set_input_shape(input_tensor_name, (1, 3, MODEL_H, MODEL_W))

    out_shape = tuple(context.get_tensor_shape(output_tensor_name))
    out_nbytes = int(np.prod(out_shape)) * np.dtype(np.float32).itemsize
    in_nbytes = 1 * 3 * MODEL_H * MODEL_W * np.dtype(np.float32).itemsize

    err, d_input = cudart.cudaMalloc(in_nbytes)
    cuda_check(err)
    err, d_output = cudart.cudaMalloc(out_nbytes)
    cuda_check(err)

    context.set_tensor_address(input_tensor_name, d_input)
    context.set_tensor_address(output_tensor_name, d_output)

    err, stream = cudart.cudaStreamCreate()
    cuda_check(err)

    h_output = np.empty(out_shape, dtype=np.float32)

    logger.info('visionn_model::model_init(): image_size="' + str([MODEL_H, MODEL_W]) + '"')
    logger.info('visionn_model::model_init(): input_tensor_name="' + input_tensor_name + '", output_tensor_name="' + output_tensor_name + '"')
    logger.info('visionn_model::model_init(): msg="load model - finished"')
    return context

def model_predict(img):
    global context, stream, d_input, d_output, h_output

    t0 = mtime_begin()
    img_h, img_w = img.shape[:2]
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    img_rgb = cv.resize(img_rgb, (MODEL_W, MODEL_H), interpolation=cv.INTER_AREA)
    img_np = img_rgb.astype(np.float32) / 255
    img_np = np.transpose(img_np, (2, 0, 1))[None, ...]  # 1x3xMODEL_HxMODEL_W
    img_np = np.ascontiguousarray(img_np)    # must be C-contiguous for cudaMemcpyAsync below

    t1 = mtime_begin()
    err = cudart.cudaMemcpyAsync(
        d_input,
        img_np.ctypes.data,
        img_np.nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
        stream,
    )[0]
    cuda_check(err)

    ok = context.execute_async_v3(stream)
    if not ok:
        raise RuntimeError('visionn_model::model_predict(): msg="TensorRT execute_async_v3 failed"')

    err = cudart.cudaMemcpyAsync(
        h_output.ctypes.data,
        d_output,
        h_output.nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
        stream,
    )[0]
    cuda_check(err)

    cuda_check(cudart.cudaStreamSynchronize(stream)[0])
    logits = h_output    # 1x2xMODEL_HxMODEL_W

    t2 = mtime_begin()
    pred = np.argmax(logits, axis=1)[0].astype(np.uint8)    # MODEL_HxMODEL_W, 0/1

    pred_img = pred * 255
    pred_img = cv.resize(pred_img, (img_w, img_h), interpolation=cv.INTER_NEAREST)    # back to the caller's own image size
    pred_img = np.expand_dims(pred_img, axis=2)    # HxWx1, matches legacy model_predict()'s output shape

    logger.debug('visionn_model::model_predict(): dt1=' + str(mtime_delta2(t0, t1)) + ', dt2=' + str(mtime_delta2(t1, t2)) + ', dt3=' + str(mtime_delta(t2)))
    return pred_img
