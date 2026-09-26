# Debug/visualization-only output of vision_node -- deliberately kept out of
# VisionData.msg (driving-relevant data only, published every tick for
# planner_node). This carries the intermediate cv::Mat data legacy save_thread
# (future save_node) needs for Vision::drawOutput()'s "vision" overlay image
# plus the raw NN prediction mask -- vision_node publishes it, save_node
# alone decides whether/how often to actually write it to disk (rate-limited
# by disk/OS load in legacy, see doc/ai/01_architecture.md). vision_node must
# not gatekeep this itself.
# Same message type published on two separate topics, one per camera
# (/robot/vision_front_debug_data, /robot/vision_rear_debug_data), same
# reuse pattern as VisionData.msg/CameraFrame.msg.
# Each width / height / cv_type / step / data field group maps directly onto
# a cv::Mat with no conversion library needed, same convention as
# CameraFrame.msg:
#   cv::Mat markers_mat(markers_height, markers_width, markers_cv_type, (void*)markers_data.data(), markers_step);
# markersIM (DataSet/vision.h) is intentionally NOT included here -- it is
# only ever consumed internally by Vision::eval() itself, never by
# drawOutput() or anything save_thread calls.

int32 camera_id   # 0 = front, 1 = rear -- same semantics as camera_node's conf.cameraDeviceId

# Copied straight from the CameraFrame.msg this tick was computed from (see
# that message's own comment). The future save_node needs this specifically
# to pair this message's markers/epweight/elweight with the *matching* raw
# CameraFrame.msg it separately subscribes to (two independent topics, no
# atomicity guarantee the way legacy's single shared DataSet slot had) --
# best-effort pairing by most-recent-of-each, falling back to a logged
# warning and skipping that save if the two image_numbers don't match.
int64 image_number

int32 markers_width
int32 markers_height
int32 markers_cv_type
int32 markers_step
uint8[] markers_data

int32 epweight_width
int32 epweight_height
int32 epweight_cv_type
int32 epweight_step
uint8[] epweight_data

int32 elweight_width
int32 elweight_height
int32 elweight_cv_type
int32 elweight_step
uint8[] elweight_data

# NN-predicted road mask (camera_img_pred, DataSet) -- saved directly by
# save_thread as its own "nnpred"/"rnnpred" image, independent of the
# markers/epweight/elweight overlay above.
int32 camera_img_pred_width
int32 camera_img_pred_height
int32 camera_img_pred_cv_type
int32 camera_img_pred_step
uint8[] camera_img_pred_data

# Duplicated from VisionData.msg (same tick) so save_node can call
# Vision::drawOutput() from this one message alone, without having to
# correlate two separate topics/timestamps for one saved image.
int32 angle_min
int32 angle_max
