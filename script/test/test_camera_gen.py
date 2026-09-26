#!/usr/bin/env python3
# Synthetic camera generator -- a stand-in for camera_node where no RealSense
# exists (the cloud dev VM). Publishes CameraFrame.msg at 30 FPS: a still JPEG
# from sample/ as the color half, and a flat-ground depth frame as the depth
# half. Runs until Ctrl+C; no verification logic, just the generator.
#
# It mirrors camera_node's own message exactly, so vision_node cannot tell the
# difference:
#   * color: 640x480 BGR8, CV_8UC3 -- camera.cpp:116 resizes the real 848x480
#     capture down to 640x480 before publishing (conf/vision_mask.png is
#     640x480 too), so the sample/ images are already the right size and are
#     published untouched.
#   * depth: 848x480 CV_16UC1, millimetres -- NOT resized in camera.cpp
#     (that line is commented out), so it keeps the native depth resolution.
#   * image_number is stamped from /robot/image_number, exactly as
#     camera_node's cb_image_number() does. Nothing publishes that topic
#     unless planner_node or test_image_number_gen.sh is running, in which
#     case frames stay stamped 0 -- fine for vision, but save_node pairs on
#     this number, so run one of those alongside if you care about out/.
#
# The depth half deserves its own note. VisionDepth looks for *deviations from
# a flat ground plane* (vision_depth.cpp), so a constant-distance block would
# read as a wall. Instead this reuses the codebase's own reference profile,
# cdepth_col_ref1 ("rovno" = straight/flat) -- the very array VisionDepth::
# drlimit_init() builds drl_dist_ref[] from, i.e. its definition of clear
# ground. It is parsed straight out of vision_depth.cpp rather than copied
# here, so the two can never drift apart. Every column gets the same profile,
# which is how legacy's own VisionDepth::getTestData() builds its test frames.
#
# Sanity-checked against the real thing: medians of cdepth_col_ref1 at the
# four rows that camera_draw_depth_frame() exports, versus 71 frames of a 2025
# park drive (doc/260907_jetson_test/park/camera_depth.park.json) --
#   row  96: 9436 vs 9357 | row 192: 2250 vs 2292
#   row 288: 1284 vs 1311 | row 384:  896 vs  921
# i.e. within ~2% of real flat ground, and by construction obstacle-free.
#
# Parameters (ros2 run ... --ros-args -p name:=value, or via the .sh wrapper):
#   image          "trava" | "cesta" | a filename in sample/ | a full path
#   camera         "front" (-> /robot/camera_front_data) | "rear"
#   fps            frame rate, default 30.0
#   depth_profile  which vision_depth.cpp array to use for depth; default
#                  "cdepth_col_ref1". The others are there for deliberately
#                  provoking an obstacle later: cdepth_col_ref2 (steep
#                  downward tilt), cdepth_col_test (steps), _test2 (hole),
#                  _test3 (patch on the road), _test4 (failed to climb).
import array
import os
import re
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from istrorsx_hw.msg import CameraFrame, ImageNumber

# Literal cv::Mat::type() values, as CameraFrame.msg documents them.
CV_8UC3 = 16
CV_16UC1 = 2

# Repository root, derived from this file's own location (script/test/).
WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
VISION_DEPTH_CPP = os.path.join(
    WS_ROOT, "src", "istrorsx_core", "src", "istrobtx", "vision_depth.cpp")
SAMPLE_DIR = os.path.join(WS_ROOT, "sample")

DEPTH_WIDTH = 848      # CAMERA_DEPTH_FRAME_WIDTH  (camera_defs.h)
DEPTH_HEIGHT = 480     # CAMERA_DEPTH_FRAME_HEIGHT


def load_depth_profile(name):
    """Parse one int[CAMERA_DEPTH_FRAME_HEIGHT] array out of vision_depth.cpp.

    Parsed rather than copied so the mock can never drift from the reference
    the obstacle detector itself uses.
    """
    with open(VISION_DEPTH_CPP) as f:
        src = f.read()

    m = re.search(name + r"\s*\[\s*CAMERA_DEPTH_FRAME_HEIGHT\s*\]\s*=\s*\{(.*?)\}",
                  src, re.S)
    if m is None:
        available = re.findall(r"int\s+(cdepth_col_\w+)\s*\[", src)
        raise SystemExit(
            "depth_profile '%s' not found in %s\navailable: %s"
            % (name, VISION_DEPTH_CPP, ", ".join(available) or "(none)"))

    values = [int(v) for v in re.findall(r"\d+", m.group(1))]
    if len(values) != DEPTH_HEIGHT:
        raise SystemExit("depth_profile '%s' has %d entries, expected %d"
                         % (name, len(values), DEPTH_HEIGHT))
    return values


def build_depth_frame(profile):
    """One depth row per profile entry, constant across all columns.

    Same construction as VisionDepth::getTestData(): the profile is a vertical
    slice through the ground plane, so replicating it horizontally yields a
    flat, featureless floor.
    """
    column = np.asarray(profile, dtype=np.uint16).reshape(DEPTH_HEIGHT, 1)
    return np.repeat(column, DEPTH_WIDTH, axis=1)


# The two default scenes. Pinned to exact filenames because sample/ holds
# several of each ("trava" alone also matches trava_zelena, trava_modra, ...)
# and a mock that silently picks a different scene between runs is worse than
# one that refuses.
SHORT_NAMES = {
    "trava": "0000130_trava.jpg",   # grass -- off-road, vision should see no road
    "cesta": "0000189_cesta.jpg",   # road  -- driveable surface ahead
}


def resolve_image(name):
    """Accept a short key, a full path, or a bare filename in sample/."""
    if name in SHORT_NAMES:
        return os.path.join(SAMPLE_DIR, SHORT_NAMES[name])

    if os.path.isfile(name):
        return name

    candidate = os.path.join(SAMPLE_DIR, name)
    if os.path.isfile(candidate):
        return candidate

    matches = sorted(f for f in os.listdir(SAMPLE_DIR)
                     if name in f and f.lower().endswith((".jpg", ".png")))
    if len(matches) == 1:
        return os.path.join(SAMPLE_DIR, matches[0])
    if not matches:
        raise SystemExit("no image in sample/ matching '%s'" % name)
    raise SystemExit("'%s' is ambiguous, matches: %s" % (name, ", ".join(matches)))


class CameraGenerator(Node):
    def __init__(self):
        super().__init__("test_camera_gen")

        self.declare_parameter("image", "trava")
        self.declare_parameter("camera", "front")
        # Dynamically typed so both "fps:=30" and "fps:=30.0" work -- a
        # plain float default would reject the integer form with a type error.
        self.declare_parameter("fps", 30.0,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("depth_profile", "cdepth_col_ref1")
        # One line per published frame with the timing split. On by default:
        # this generator exists for diagnosis, and the numbers are the point.
        self.declare_parameter("stats", True)

        image_name = self.get_parameter("image").value
        camera = self.get_parameter("camera").value
        fps = float(self.get_parameter("fps").value)
        profile_name = self.get_parameter("depth_profile").value
        self.stats = bool(self.get_parameter("stats").value)

        if camera not in ("front", "rear"):
            raise SystemExit("camera must be 'front' or 'rear', got '%s'" % camera)
        if fps <= 0:
            raise SystemExit("fps must be > 0, got %r" % fps)

        # Topic names come from camera_node's own cameraTopicName().
        topic = "/robot/camera_%s_data" % camera

        image_path = resolve_image(image_name)
        self.color = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if self.color is None:
            raise SystemExit("could not read image: %s" % image_path)
        if not self.color.flags["C_CONTIGUOUS"]:
            self.color = np.ascontiguousarray(self.color)

        self.depth = build_depth_frame(load_depth_profile(profile_name))

        # Serialize both halves ONCE. The scene is a still, so the byte
        # payload never changes -- rebuilding it per frame meant converting
        # ~1.7M values into Python sequences 30x a second, which capped the
        # generator at ~14 FPS. Precomputed, the tick is just an assignment.
        self.color_bytes = array.array("B", self.color.reshape(-1).tobytes())
        self.depth_bytes = array.array("B", self.depth.tobytes())

        self.image_number = 0
        self.frames = 0

        # Per-tick timing state, see tick(). All times are perf_counter
        # seconds; the printed numbers are milliseconds.
        self.prev_start = None
        self.prev_end = None
        self.lo = {}
        self.hi = {}
        self.period_nominal_ms = 1000.0 / fps

        self.pub = self.create_publisher(CameraFrame, topic, 10)
        # Same as camera_node: the virtual clock is stamped, not generated.
        self.create_subscription(
            ImageNumber, "/robot/image_number", self.cb_image_number, 10)
        self.timer = self.create_timer(1.0 / fps, self.tick)

        h, w = self.color.shape[:2]
        self.get_logger().info(
            "publishing %s: color %dx%d from %s, depth %dx%d from %s, %.1f FPS"
            % (topic, w, h, os.path.relpath(image_path, WS_ROOT),
               DEPTH_WIDTH, DEPTH_HEIGHT, profile_name, fps))
        if (w, h) != (640, 480):
            self.get_logger().warn(
                "color is %dx%d; camera_node publishes 640x480 (camera.cpp:116) "
                "and conf/vision_mask.png is 640x480 -- vision may misbehave" % (w, h))

    def cb_image_number(self, msg):
        self.image_number = msg.image_number

    def tick(self):
        # Three numbers per tick, because a stall can hide in any of them and
        # only the split says which. Downstream subscribers were seeing ~2 s
        # gaps on this topic (2026-09-07) and it was not obvious whether the
        # generator was late being called, slow to build the message, or slow
        # to publish it -- so measure all three separately:
        #
        #   period : start-to-start. Should equal 1/fps. If this is 2 s while
        #            prep+pub stay at a couple of ms, the callback simply was
        #            not invoked -- the process was not scheduled, or rclpy's
        #            executor did not get to the timer. Nothing to do with the
        #            message or with ROS 2 transport.
        #   prep   : building the CameraFrame. The payload is precomputed, so
        #            this is only field assignment and should be well under a
        #            millisecond.
        #   pub    : publish() itself, i.e. what ROS 2 costs. Measured at
        #            ~2 ms for this 1.66 MB message on the dev VM.
        t_start = time.perf_counter()
        gap_ms = (t_start - self.prev_end) * 1000.0 if self.prev_end else 0.0
        period_ms = (t_start - self.prev_start) * 1000.0 if self.prev_start else 0.0

        msg = CameraFrame()
        msg.image_number = self.image_number

        msg.color_width = self.color.shape[1]
        msg.color_height = self.color.shape[0]
        msg.color_cv_type = CV_8UC3
        msg.color_step = int(self.color.strides[0])
        msg.color_data = self.color_bytes

        msg.depth_width = DEPTH_WIDTH
        msg.depth_height = DEPTH_HEIGHT
        msg.depth_cv_type = CV_16UC1
        msg.depth_step = int(self.depth.strides[0])
        msg.depth_data = self.depth_bytes

        t_prep = time.perf_counter()
        self.pub.publish(msg)
        t_end = time.perf_counter()

        prep_ms = (t_prep - t_start) * 1000.0
        pub_ms = (t_end - t_prep) * 1000.0

        self.frames += 1
        self.prev_start = t_start
        self.prev_end = t_end

        if self.frames > 1:      # first tick has no previous to compare against
            self.mm("period", period_ms)
            self.mm("idle", gap_ms)
        self.mm("prep", prep_ms)
        self.mm("pub", pub_ms)

        # The first tick has no previous one, so period/idle are undefined and
        # their min/max not yet recorded -- nothing useful to print.
        if self.stats and self.frames > 1:
            # Marked when the tick came in more than twice as late as asked
            # for -- that is the signature the downstream probes were seeing.
            late = period_ms > 2.0 * self.period_nominal_ms
            print("n=%-6d period=%8.1f [%6.1f/%7.1f]  idle=%8.1f  "
                  "prep=%5.2f [%5.2f]  pub=%6.2f [%6.2f]%s"
                  % (self.frames, period_ms,
                     self.lo["period"], self.hi["period"], gap_ms,
                     prep_ms, self.hi["prep"], pub_ms, self.hi["pub"],
                     "   <-- LATE" if late else ""),
                  flush=True)

    def mm(self, key, v):
        """Running min/max, so every line carries the worst seen so far."""
        if key not in self.lo or v < self.lo[key]:
            self.lo[key] = v
        if key not in self.hi or v > self.hi[key]:
            self.hi[key] = v


def main():
    rclpy.init()
    node = CameraGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C and SIGTERM respectively -- both are the normal way this
        # generator ends, neither deserves a traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
