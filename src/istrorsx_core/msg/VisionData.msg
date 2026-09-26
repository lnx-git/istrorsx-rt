# Driving-relevant output of vision_node -- port of legacy vision_thread's
# per-camera obstacle map (istro_rt2025.cpp, vision.cpp/vision_depth.cpp).
# Same message type published on two separate topics, one per camera
# (/robot/vision_front_data, /robot/vision_rear_data) -- same reuse pattern
# as CameraFrame.msg being published by both camera_node instances, rather
# than one message with front_/rear_-prefixed duplicate fields.
# Mirrors DataSet's vision_dmap/vision_angle_min/vision_angle_max (front) and
# vision2_dmap/vision2_angle_min/vision2_angle_max (rear) with the
# "vision_"/"vision2_" prefix dropped -- same convention as
# DriveCommand.msg/NavigationData.msg dropping "process_"/"gps_navp_".
# See doc/ai/01_architecture.md.

int32 camera_id   # 0 = front, 1 = rear -- same semantics as camera_node's conf.cameraDeviceId

# Copied straight from the CameraFrame.msg this tick was computed from (see
# that message's own comment) -- lets a consumer subscribing to multiple
# topics (e.g. the future save_node) confirm which capture this output
# belongs to.
int64 image_number

# DegreeMap (dmap.h, istrobtx), 181 = DEGREE_MAP_COUNT: one entry per degree,
# 0..180 (0=right, 90=in front, 180=left, counterclockwise).
int32[181] dmap   # 1=free, 0=obstacle, -1=not available
int32[181] dist   # distance to obstacle, centimetres, <0 = no obstacle
int32[181] maxd   # max distance an obstacle could be detected at, centimetres

int32 angle_min   # DegreeMap::find()'s widest free interval, start angle, -1 = none found
int32 angle_max   # DegreeMap::find()'s widest free interval, end angle, -1 = none found

# QR-scanned GPS coordinates -- only ever set on the front-camera topic
# (rear camera never runs QR scanning in legacy); ANGLE_NONE (config.h,
# 999999) on the rear topic and whenever no code was scanned this tick.
float64 qrscan_latitude
float64 qrscan_longitude
