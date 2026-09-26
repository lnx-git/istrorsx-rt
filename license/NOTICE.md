# Third-party material in this repository

The MIT licence in `LICENSE` (a copy is in `license/LICENSE`) covers **the source code and
documentation of this repository only**. Three kinds of file are **not** ours to relicense and keep
the terms of where they came from. They are listed below, with the same note repeated in the
directory each of them sits in.

## Neural-network models trained with Ultralytics (AGPL-3.0)

| File |
|---|
| `script/visionn_roadyolo_server/model_roadyolo/road_yolo11s_seg.onnx` |
| `script/visionn_roadyolo-trt_server/model_roadyolo-trt/road_yolo11s_seg_fp16.engine` |

Both were produced by fine-tuning Ultralytics' pre-trained `yolo11s-seg.pt` checkpoint with the
Ultralytics framework, which is licensed under **AGPL-3.0**. Ultralytics treats models trained with
their code as derived works covered by the same licence, so these two files are distributed under
**AGPL-3.0**, not under this repository's MIT licence.

* Ultralytics: <https://github.com/ultralytics/ultralytics>
* AGPL-3.0 text: `license/AGPL-3.0.txt` in this repository, or <https://www.gnu.org/licenses/agpl-3.0.html>
* Ultralytics licensing, including their commercial option: <https://www.ultralytics.com/license>

The YOLO mask postprocess in `script/visionn_roadyolo*_server/visionn_model.py` reproduces the
algorithm of Ultralytics' `ops.process_mask_native` and was validated against it. If you intend to
reuse that code on its own, consider the same licence question.

## Neural-network models fine-tuned from NVIDIA SegFormer

| File |
|---|
| `script/visionn_segfb0_server/model_segfb0/segformer_b0_road.onnx` |
| `script/visionn_segfb0-trt_server/model_segfb0-trt/segformer_b0_road_fp16.engine` |

Both were fine-tuned from the pre-trained checkpoint
[`nvidia/segformer-b0-finetuned-ade-512-512`](https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512).
NVIDIA publishes those upstream weights under **its own licence, not a permissive one**, and it
restricts use to non-commercial research. Read the model card and
[NVLabs/SegFormer's licence](https://github.com/NVlabs/SegFormer/blob/master/LICENSE) before reusing
these files, and treat them as carrying the same restriction.

## Map data from OpenStreetMap (ODbL)

`conf/*.osm` and the KML files in `doc/kml/` are derived from **OpenStreetMap** data,
© OpenStreetMap contributors, available under the
[Open Database License](https://www.openstreetmap.org/copyright).

## A note on the `.engine` files

The two TensorRT engines were built on one specific Jetson Orin Nano with one specific JetPack and
TensorRT version. They cannot be loaded anywhere else and have to be rebuilt from the matching
`.onnx` on the target device. They are included because they are what the robot actually ran.
