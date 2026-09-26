#!/usr/bin/env python3
# save_node test 1/4: a synthetic GpsData fix makes save_node write
# navmap.png/.kml -- needs only save_node running (see test_save_node1.sh),
# no planner_node/wmodel involved at all (saveNavMap() only needs the GPS
# breadcrumb trail + the static compiled-in route graph, see
# doc/ai/01_architecture.md's save_node section).
#
# Coordinates are the reference point of the currently-active
# ISTRO_MAP_BA_SADJK map (Sad Janka Krala, Bratislava -- navig_data.cpp/
# gpsdev.cpp), so the resulting navmap.png plots a point actually on/near
# the compiled-in route graph.
import sys
import time

import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import GpsData
from istrorsx_core.msg import SaveEvent

SADJK_LATITUDE = 48.1337423
SADJK_LONGITUDE = 17.1112223


class TestNode(Node):
    def __init__(self):
        super().__init__('test_save_node1')
        self.events = []
        self.pub_gps = self.create_publisher(GpsData, '/robot/gps_data', 10)
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
    rclpy.init()
    node = TestNode()
    ok = False
    try:
        time.sleep(3.0)  # let discovery settle (same value test_save_node4.py
        # settled on -- 1.0s occasionally isn't enough with several processes
        # starting near-simultaneously, confirmed via a real discovery miss:
        # cb_gps() never fired for an entire run, so every saveNavMap() tick
        # saw aux_cnt=0 and the resulting navmap.kml had an empty auxPoint
        # folder despite the GpsData message having been published)

        gps = GpsData()
        gps.fix = True
        gps.latitude = SADJK_LATITUDE
        gps.longitude = SADJK_LONGITUDE
        gps.speed = 0.0
        gps.course = 0.0
        node.pub_gps.publish(gps)
        print(f"Published GpsData ({SADJK_LATITUDE}, {SADJK_LONGITUDE}) to /robot/gps_data")

        # saveNavMap() runs on its own SAVE_PERIOD_NAVMAP=1000ms timer,
        # independent of GpsData -- discard any navmap_png/navmap_kml tags
        # already queued from *before* this publish (pre-GPS ticks, which
        # would otherwise satisfy wait_for_tag() below without the file
        # actually reflecting the fix just sent) so only a tag generated
        # after cb_gps() has run counts.
        node.events = []

        # saveNavMap() is throttled to SAVE_PERIOD_NAVMAP=1000ms, so give it
        # enough headroom.
        f_png = node.wait_for_tag("navmap_png", timeout=15.0)
        f_kml = node.wait_for_tag("navmap_kml", timeout=5.0)
        ok = f_png is not None and f_kml is not None
        if f_png:
            print(f"  navmap_png -> {f_png.filename}  <-- inspect this one")
        if f_kml:
            print(f"  navmap_kml -> {f_kml.filename}")

        # Keep save_node up a bit longer after the tag is found instead of
        # tearing down immediately -- user-requested, so there's more time to
        # inspect the running process/logs before this script's own .sh
        # wrapper kills it.
        time.sleep(3.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT (gps -> navmap):", "OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
