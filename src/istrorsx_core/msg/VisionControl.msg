# Mode-control input for vision_node -- port of legacy vision_thread's
# process-global flags vision_thread_qrscan_enabled/vision_thread_processRear_enabled
# (istro_rt2025.cpp), toggled today from process_thread's loadarea_process()/
# wrongway_process() helpers. Published by the future planner_node (the port of
# process_thread), subscribed by vision_node. A message, not a service --
# matches every other cross-node control interaction in this project
# (DriveCommand, NavigationData), see doc/ai/01_architecture.md.

bool qrscan_enabled        # mirrors vision_thread_qrscan_enable()/_disable(): run QRScanner::scanGeo() on the front camera image this tick
bool process_rear_enabled  # mirrors vision_thread_processRear_enable()/_disable(): process the rear camera instead of the front camera this tick
