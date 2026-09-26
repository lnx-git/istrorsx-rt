from pathlib import Path
import numpy as np
import cv2
import onnxruntime as ort

ONNX_PATH = Path("./model_segfb0/segformer_b0_road.onnx")
IMG_PATH  = Path("./img_input")
OUT_DIR   = Path("./img_output")
OUT_DIR.mkdir(exist_ok=True)

H, W = 384, 512

def preprocess(bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_AREA)
    x = rgb.astype(np.float32) / 255.0
    x = np.transpose(x, (2,0,1))[None, ...]  # 1x3xHxW
    return x, rgb

def main():
    sess = ort.InferenceSession(ONNX_PATH.as_posix(), providers=["CPUExecutionProvider"])

    imgs = sorted(list(Path(IMG_PATH).glob("*.png")))
    if not imgs:
        raise RuntimeError(f"Nenašiel som .png v {IMG_PATH}")

    for p in imgs:
        bgr0 = cv2.imread(str(p), cv2.IMREAD_COLOR)
        x, rgb = preprocess(bgr0)

        logits = sess.run(None, {"input": x})[0]  # 1x2xHxW
        pred = np.argmax(logits, axis=1)[0].astype(np.uint8)  # HxW, 0/1

        # overlay
        overlay = rgb.copy()
        mask = (pred == 1)
        overlay[mask] = (overlay[mask] * 0.5 + np.array([0,255,0]) * 0.5).astype(np.uint8)

        out = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(OUT_DIR / f"{p.stem}_overlay.png"), out)

    print("Saved overlays to:", OUT_DIR)

if __name__ == "__main__":
    main()
