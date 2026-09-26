# Full lidar scan, one entry per raw hardware sample -- mirrors lidar_data_t
# (lidar.h) as filled by Lidar::getData() (already angle-ascending after its
# internal ascendScanData() call).

# planner_node's own virtual clock (legacy "image_number"), stamped here from
# ImageNumber.msg the same way CameraFrame.msg is -- see that message's own
# comment for why this can't be a direct istrorsx_core dependency instead.
int64 image_number

int32 point_count   # number of valid entries in the arrays below

int32[] sync        # 1 = first sample of a new 360 degree scan, 0 otherwise
float32[] angle      # degrees, 0..360 (0 = right, 90 = front of the robot, counterclockwise)
float32[] distance   # millimeters, 0 = no reading
int32[] quality      # measurement quality, 0-63 (higher = more reliable; see LIDAR_QUALITY_* in lidar.cpp)
