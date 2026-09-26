#!/bin/bash
# Synthetic ImageNumber generator: publishes /robot/image_number every 5ms,
# incrementing by 1 from 0, mimicking planner_node's publishImageNumber()
# (planner_node.cpp, PLANNER_TICK_PERIOD=5ms) without needing planner_node's
# GPS/controlboard data-arrival gating. Runs until Ctrl+C -- a standalone
# utility for isolated testing of ImageNumber consumers (e.g. camera_node),
# not a pass/fail test.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec python3 script/test/test_image_number_gen.py
