# planner_node's own virtual clock (legacy "image_number", istro_rt2025.cpp's
# process_thread), republished here purely so camera_node (istrorsx_hw) can
# stamp it onto CameraFrame.msg -- istrorsx_core already depends on
# istrorsx_hw (never the reverse), so camera_node cannot subscribe to
# istrorsx_core's own PlannerData.msg (which carries the same value) without
# creating a circular package dependency. planner_node publishes both this
# and PlannerData.msg's image_number field with the same value, once per
# plannerTick().
int64 image_number
