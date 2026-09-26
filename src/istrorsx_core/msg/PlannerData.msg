# Secondary output of planner_node -- the fields of legacy process_writeData()
# (istro_rt2025.cpp) that aren't already covered by DriveCommand.msg
# (angle/velocity/state/stop/yaw). Published alongside DriveCommand.msg every
# plannerTick() -- see doc/ai/01_architecture.md's planner_node section.
#
# process_dir/process_ref/process_x/process_y keep their "process_" prefix
# (unlike DriveCommand.msg dropping it) deliberately, to avoid colliding with
# NavigationData.msg's own ref/x/y fields -- those are a different position
# estimate (GPS-derived, navigation_getXY()), while these are odometry
# dead-reckoning (update_xy()). Mixing the two under the same bare field
# names across two messages would be a real source of confusion.

int64   image_number      # legacy "long" -- virtual clock, incremented once per plannerTick(); used for cross-node log correlation and (once built) save_node's file naming, matching legacy save_thread
int32   process_dir       # DegreeMap::find()'s steering preference (90 = straight ahead); consumed by vision_node once this is wired up (currently hardcoded there, see known gap)
int32   process_ref       # 1 = reference point initialized (wmodel's coordinate system fixed), 0 = not yet -- NOT the same as NavigationData.msg's "ref"
float64 process_x         # odometry-integrated robot position, local XY metres (update_xy()) -- NOT the same as NavigationData.msg's x/y
float64 process_y
int32   process_backward  # 1 = currently reversing (a wrongway maneuver in progress, wrongway_inprogress()), 0 = normal forward driving

int32   coneseek_stop     # ISTRO_VISION_ORANGECONE mode only: 1 = cone close enough to stop and drop payload
int32   coneseek_intlen   # ISTRO_VISION_ORANGECONE mode only: width (degrees) of the detected cone interval
