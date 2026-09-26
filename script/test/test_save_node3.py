#!/usr/bin/env python3
# save_node test 3/4: a real image loaded from sample/ (not synthetic noise --
# camera.jpg's own overlay-free save just re-encodes whatever it's given, so
# a real photo makes it easy to visually confirm the right frame landed on
# disk) makes save_node write camera.jpg -- needs only save_node running (see
# test_save_node3.sh). Deliberately no depth (CameraFrame.msg's depth_width
# left at 0, matching camera_node's own "depth capture failed" convention) --
# confirms cdepth.jpg does NOT get written when there's no depth to draw, not
# just that camera.jpg does when there's color.
import sys
import time

import cv2
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import CameraFrame
from istrorsx_core.msg import SaveEvent

SAMPLE_IMAGE = "sample/0001032_cesta_oblacno.jpg"
IMAGE_NUMBER = 30001


class TestNode(Node):
    def __init__(self):
        super().__init__('test_save_node3')
        self.events = []
        self.pub_camera = self.create_publisher(CameraFrame, '/robot/camera_front_data', 10)
        self.create_subscription(SaveEvent, '/robot/save_event', self.cb_event, 10)

    def cb_event(self, msg):
        self.events.append(msg)

    def wait_for_tag(self, tag, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            for ev in self.events:
                for f in ev.files:
                    if f.tag == tag:
                        return f
        return None

    def any_tag(self, tag):
        return any(f.tag == tag for ev in self.events for f in ev.files)


def main():
    color = cv2.imread(SAMPLE_IMAGE, cv2.IMREAD_COLOR)
    if color is None:
        print(f"could not load {SAMPLE_IMAGE}")
        sys.exit(2)

    rclpy.init()
    node = TestNode()
    ok = False
    try:
        time.sleep(1.0)  # let discovery settle

        frame = CameraFrame()
        frame.image_number = IMAGE_NUMBER
        frame.color_width = color.shape[1]
        frame.color_height = color.shape[0]
        frame.color_cv_type = 16  # CV_8UC3
        frame.color_step = int(color.strides[0])
        frame.color_data = color.tobytes()
        # depth_width/height left at 0 -- no depth image for this test.
        node.pub_camera.publish(frame)
        print(f"Published CameraFrame ({SAMPLE_IMAGE}, {color.shape[1]}x{color.shape[0]}, no depth) to /robot/camera_front_data")

        f = node.wait_for_tag("camera", timeout=15.0)
        ok = f is not None
        if f:
            print(f"  camera -> {f.filename}  <-- inspect this one")

        if node.any_tag("cdepth"):
            print("  WARNING: cdepth was saved even though no depth was published!")
            ok = False

        # Keep save_node up a bit longer after the tag is found instead of
        # tearing down immediately -- user-requested, so there's more time to
        # inspect the running process/logs before this script's own .sh
        # wrapper kills it.
        time.sleep(3.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT (camera -> camera.jpg, no depth):", "OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
