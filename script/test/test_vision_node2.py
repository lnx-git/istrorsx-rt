#!/usr/bin/env python3
# vision_node test 2/2: same CameraFrame content as test_vision_node.py
# (real sample image + synthetic uniform depth), but with image_number set
# (test_vision_node.py's own message never sets it) and, per test_vision_node2.sh,
# run with save_node ALSO up and subscribed to the same topics.
#
# This exists because test_save_node4.py (camera_node->vision_node->save_node,
# checking for a "vision" SaveEvent tag) was failing even though nothing
# pointed at vision_node itself being broken. Subscribing directly to
# VisionData/VisionDebugData here -- bypassing save_node's SaveEvent layer
# entirely -- isolates the question "does vision_node still publish correctly
# when save_node is also running against the same CameraFrame/image_number"
# from whatever save_node's own cb_vision_debug_front()/saveCameraGroup()
# does with that same data. A pass here means vision_node is healthy and the
# remaining bug (if test_save_node4 still fails) is inside save_node itself,
# not vision_node/visionn_server/ROS discovery timing.
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import CameraFrame
from istrorsx_core.msg import VisionData, VisionDebugData

SAMPLE_IMAGE = "sample/0001032_cesta_oblacno.jpg"
IMAGE_NUMBER = 40001


class TestNode(Node):
    def __init__(self):
        super().__init__('test_vision_node2')
        self.got_data = False
        self.got_debug = False
        self.pub = self.create_publisher(CameraFrame, '/robot/camera_front_data', 10)
        self.create_subscription(VisionData, '/robot/vision_front_data', self.cb_data, 10)
        self.create_subscription(VisionDebugData, '/robot/vision_front_debug_data', self.cb_debug, 10)

    def cb_data(self, msg):
        self.got_data = True
        print(f"VisionData received: angle_min={msg.angle_min} angle_max={msg.angle_max}")

    def cb_debug(self, msg):
        self.got_debug = True
        print(f"VisionDebugData received: markers={msg.markers_width}x{msg.markers_height}")


def main():
    color = cv2.imread(SAMPLE_IMAGE, cv2.IMREAD_COLOR)
    if color is None:
        print(f"could not load {SAMPLE_IMAGE}")
        sys.exit(2)
    # Same uniform-2000mm/848x480 synthetic depth as test_vision_node.py --
    # required for vision_node's own ISTRO_VISION_DEPTH pipeline (see
    # test_save_node4.py's module docstring for why).
    depth = np.full((480, 848), 2000, dtype=np.uint16)

    rclpy.init()
    node = TestNode()
    ok = False
    try:
        time.sleep(3.0)  # let discovery settle (vision_node + save_node both subscribe)

        frame = CameraFrame()
        frame.image_number = IMAGE_NUMBER
        frame.color_width = color.shape[1]
        frame.color_height = color.shape[0]
        frame.color_cv_type = 16  # CV_8UC3
        frame.color_step = int(color.strides[0])
        frame.color_data = color.tobytes()
        frame.depth_width = depth.shape[1]
        frame.depth_height = depth.shape[0]
        frame.depth_cv_type = 2  # CV_16UC1
        frame.depth_step = int(depth.strides[0])
        frame.depth_data = depth.tobytes()
        node.pub.publish(frame)
        print(f"Published CameraFrame (image_number={IMAGE_NUMBER}) to /robot/camera_front_data")

        deadline = time.time() + 60.0
        while time.time() < deadline and not (node.got_data and node.got_debug):
            rclpy.spin_once(node, timeout_sec=0.5)

        ok = node.got_data and node.got_debug
        print("RESULT:", "OK - both VisionData and VisionDebugData received" if ok
              else f"FAIL - got_data={node.got_data} got_debug={node.got_debug}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
