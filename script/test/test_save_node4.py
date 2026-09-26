#!/usr/bin/env python3
# save_node test 4/4: the same real sample image, but published for a *real*
# vision_node (backed by a real visionn_server/NN, see test_save_node4.sh) to
# process first -- confirms the full camera_node->vision_node->save_node
# chain produces a real "vision.jpg" overlay image on disk, not just a
# synthetic VisionDebugData message like the earlier ad-hoc testing used.
# save_node itself needs no code change/awareness of this -- it's the same
# /robot/camera_front_data + /robot/vision_front_debug_data subscriptions
# either way, just fed by a real node instead of this script directly.
# No planner_node/wmodel involved.
#
# Unlike tests 1-3, this one DOES send a synthetic depth image -- found via
# testing that vision_node itself (not save_node) segfaults without one:
# ISTRO_VISION_DEPTH is compiled in (system.h), so vision_node's own
# visionTickFront() unconditionally calls VisionDepth::process() on whatever
# CameraFrame.msg.depth_* it received, with no empty-Mat guard (the same
# category of gap fixed in save_node's own Vision::drawOutput() call, but
# this one lives in vision_node, out of scope for this save_node test suite
# to fix). test_vision_node.py already established the workaround: a
# uniform-depth synthetic image is enough to keep vision_node's pipeline
# from touching empty data, whether or not the resulting depth values are
# realistic. This is purely a vision_node-pipeline prerequisite, not
# something save_node's own cdepth.jpg saving is being tested against here.
#
# Expected side effect worth knowing before inspecting vision.jpg: because
# the depth is a perfectly flat/uniform plane (no real road-shaped gradient),
# VisionCore::applyMarkers() (vision.cpp) -- called from vision_node's own
# visionTickFront() to merge the depth-derived classification into the main
# one -- forces every pixel to "not road" (green) wherever the depth-based
# classifier doesn't say "road", which is everywhere here. So vision.jpg's
# overlay comes out fully green even though nnpred.png (the NN's own mask,
# unaffected by this merge) looks correct -- not a save_node/vision_node bug,
# just this test's synthetic depth having no real road geometry for the
# depth classifier to recognize. A real depth camera on the real robot
# wouldn't trigger this.
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import CameraFrame
from istrorsx_core.msg import SaveEvent

SAMPLE_IMAGE = "sample/0001032_cesta_oblacno.jpg"
IMAGE_NUMBER = 40001


class TestNode(Node):
    def __init__(self):
        super().__init__('test_save_node4')
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


def main():
    color = cv2.imread(SAMPLE_IMAGE, cv2.IMREAD_COLOR)
    if color is None:
        print(f"could not load {SAMPLE_IMAGE}")
        sys.exit(2)
    # 640x480, same as test_vision_node.py's own already-verified-working
    # input -- no resize needed (an earlier version of this test resized to
    # 848x480 chasing what turned out to be a different bug, the missing
    # depth image below, not a dimension mismatch).

    # See module docstring: required for vision_node's own ISTRO_VISION_DEPTH
    # pipeline, not something save_node's cdepth.jpg saving is being tested
    # against here. Same uniform-2000mm/848x480 convention as
    # test_vision_node.py's own make_camera_frame().
    depth = np.full((480, 848), 2000, dtype=np.uint16)

    rclpy.init()
    node = TestNode()
    ok = False
    try:
        time.sleep(3.0)  # let discovery settle (vision_node subscribes the same topic)

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
        node.pub_camera.publish(frame)
        print(f"Published CameraFrame ({SAMPLE_IMAGE} + synthetic depth, see module docstring) to /robot/camera_front_data (vision_node + save_node both subscribed)")

        # Generous timeouts -- save_node's own background work (its 50ms
        # saveTick() timer, plus navmap.png/.kml redrawing roughly once a
        # second, see saveTick()'s own comment on why that isn't gated behind
        # camera/lidar) competes for CPU with vision_node's own calibration/
        # NN-eval work on this VM, confirmed via testing to sometimes take
        # noticeably longer than plain test_vision_node.sh's vision_node-only
        # runs (which has no such contention).
        f_cam = node.wait_for_tag("camera", timeout=25.0)
        f_vis = node.wait_for_tag("vision", timeout=60.0)
        ok = f_cam is not None and f_vis is not None
        if f_cam:
            print(f"  camera -> {f_cam.filename}")
        if f_vis:
            print(f"  vision -> {f_vis.filename}  <-- inspect this one for the overlay")

        f_nnpred = node.wait_for_tag("nnpred", timeout=2.0)
        if f_nnpred:
            print(f"  nnpred -> {f_nnpred.filename} (NN prediction mask -- visionn_server was reached)")

        # Keep vision_node/save_node up a bit longer after the tag is found
        # instead of tearing down immediately -- user-requested, so there's
        # more time to inspect the running processes/logs before this
        # script's own .sh wrapper kills them.
        time.sleep(3.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT (vision -> vision.jpg via real vision_node):", "OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
