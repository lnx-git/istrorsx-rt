# One synchronized RealSense capture (color + depth from the same instant --
# pipe.wait_for_frames() already returns both together, so this stays one
# atomic message instead of two topics a consumer would have to re-sync).
# Each half maps directly onto a cv::Mat with no conversion library needed:
#   cv::Mat color_mat(color_height, color_width, color_cv_type, (void*)color_data.data(), color_step);
#   cv::Mat depth_mat(depth_height, depth_width, depth_cv_type, (void*)depth_data.data(), depth_step);

# planner_node's own virtual clock (legacy "image_number", incremented once
# per plannerTick()) -- stamped here from ImageNumber.msg (see that file's
# own comment for why this is a separate message rather than a direct
# istrorsx_core dependency), the same way legacy's capture_camera_thread()
# read pdata->image_number via capture_readData() before writing a frame.
# Lets vision_node/save_node correlate a raw capture with the VisionData/
# VisionDebugData output computed from it, across two independent topics.
int64 image_number

int32 color_width
int32 color_height
int32 color_cv_type   # literal cv::Mat::type() value, e.g. CV_8UC3=16 for a BGR color frame
int32 color_step      # row stride in bytes (cv::Mat::step)
uint8[] color_data     # raw pixel bytes, row-major, contiguous

int32 depth_width      # 0 if depth capture failed/unavailable this frame
int32 depth_height
int32 depth_cv_type   # literal cv::Mat::type() value, e.g. CV_16UC1=2 for a depth frame in millimeters
int32 depth_step
uint8[] depth_data
