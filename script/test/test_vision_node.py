#!/usr/bin/env python3
# End-to-end test for vision_node: publishes one simulated CameraFrame (a
# real sample JPEG as the color image, a synthetic uniform-distance depth
# image) to /robot/camera_front_data, then listens for vision_node's
# /robot/vision_front_data and /robot/vision_front_debug_data responses.
#
# CameraFrame.msg's byte-array fields can't be built via "ros2 topic pub"
# YAML in any practical way, so unlike this project's other script/test/*.sh
# scripts, this one uses rclpy directly instead of the CLI.
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import CameraFrame
from istrorsx_core.msg import VisionData, VisionDebugData

SAMPLE_IMAGE = "sample/0001032_cesta_oblacno.jpg"


def make_camera_frame():
    color = cv2.imread(SAMPLE_IMAGE, cv2.IMREAD_COLOR)
    if color is None:
        raise RuntimeError(f"could not load {SAMPLE_IMAGE}")

    # Synthetic depth: uniform 2000mm, same size vision_depth.h expects
    # (CAMERA_DEPTH_FRAME_WIDTH/HEIGHT, istrobtx/camera_types.h).
    depth = np.full((480, 848), 2000, dtype=np.uint16)

    msg = CameraFrame()
    msg.color_width = color.shape[1]
    msg.color_height = color.shape[0]
    msg.color_cv_type = 16  # CV_8UC3, matches CameraFrame.msg's own comment
    msg.color_step = int(color.strides[0])
    msg.color_data = color.tobytes()

    msg.depth_width = depth.shape[1]
    msg.depth_height = depth.shape[0]
    msg.depth_cv_type = 2  # CV_16UC1
    msg.depth_step = int(depth.strides[0])
    msg.depth_data = depth.tobytes()

    return msg


class VisionTestNode(Node):
    def __init__(self):
        super().__init__('test_vision_node')
        self.got_data = False
        self.got_debug = False
        self.pub = self.create_publisher(CameraFrame, '/robot/camera_front_data', 10)
        self.create_subscription(VisionData, '/robot/vision_front_data', self.cb_data, 10)
        self.create_subscription(VisionDebugData, '/robot/vision_front_debug_data', self.cb_debug, 10)

    def cb_data(self, msg):
        self.got_data = True
        dmap = list(msg.dmap)
        print(f"VisionData: camera_id={msg.camera_id} angle_min={msg.angle_min} angle_max={msg.angle_max} "
              f"dmap free={dmap.count(1)}/181 obstacle={dmap.count(0)}/181 na={dmap.count(-1)}/181 "
              f"qrscan_lat={msg.qrscan_latitude} qrscan_lon={msg.qrscan_longitude}")

    def cb_debug(self, msg):
        self.got_debug = True
        print(f"VisionDebugData: camera_id={msg.camera_id} "
              f"markers={msg.markers_width}x{msg.markers_height} (cv_type={msg.markers_cv_type}, {len(msg.markers_data)} bytes) "
              f"epweight={msg.epweight_width}x{msg.epweight_height} elweight={msg.elweight_width}x{msg.elweight_height} "
              f"camera_img_pred={msg.camera_img_pred_width}x{msg.camera_img_pred_height} "
              f"angle_min={msg.angle_min} angle_max={msg.angle_max}")


def main():
    rclpy.init()
    node = VisionTestNode()

    # give discovery a moment before publishing (default QoS has no latching)
    time.sleep(1.0)
    node.pub.publish(make_camera_frame())
    print("Published simulated CameraFrame to /robot/camera_front_data")

    deadline = time.time() + 15.0
    while time.time() < deadline and not (node.got_data and node.got_debug):
        rclpy.spin_once(node, timeout_sec=0.5)

    ok = node.got_data and node.got_debug
    print("RESULT:", "OK - both VisionData and VisionDebugData received" if ok
          else f"FAIL - got_data={node.got_data} got_debug={node.got_debug}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
