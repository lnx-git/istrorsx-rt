#include "istrorsx_hw/camera_node.hpp"

#include <unistd.h>   // _exit()

#include "threads.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "config.h"
#include "istrorsx_hw/istrobtx/camera.h"
#include "istrorsx_hw/istrobtx/istro_main_camera.h"

LOG_DEFINE(loggerIstroCamera2, "istroCamera");

// -camdev selects which physical camera this process binds to (see Config /
// istro_main_camera.cpp) -- since front and rear are two separate camera_node
// processes now (not one process owning both, unlike the legacy app's
// camera/camera2 pair), the ROS node name and topic must also be derived from
// it, otherwise two simultaneously-running instances would collide on both.
static std::string cameraTopicName(void)
{
    if (conf.cameraDeviceId == 0)  return "/robot/camera_front_data";
    if (conf.cameraDeviceId == 1)  return "/robot/camera_rear_data";
    return "/robot/camera_data";
}

static std::string cameraNodeName(void)
{
    if (conf.cameraDeviceId == 0)  return "camera_front_node";
    if (conf.cameraDeviceId == 1)  return "camera_rear_node";
    return "camera_node";
}

CameraNode::CameraNode() : Node(cameraNodeName())
{
    // Setup publisher for camera frames (color + depth together, see CameraFrame.msg)
    pub_camera_ = this->create_publisher<istrorsx_hw::msg::CameraFrame>(cameraTopicName(), 10);

    // Setup a periodic timer for capturing a frame. camera.getFrame()
    // (pipe.wait_for_frames() underneath) blocks until a frame is ready --
    // requested at 30 FPS (~33 ms/frame, see cfg.enable_stream() in
    // camera.cpp), so the timer period here is deliberately shorter than
    // that, the same way lidar_node's is shorter than a full lidar scan --
    // capture ends up back-to-back, paced by the blocking call itself.
    timer_camera_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CameraNode::camera_read_cb, this)
    );

    // See ImageNumber.msg's own comment for why this can't just be a
    // PlannerData.msg subscription (istrorsx_hw cannot depend on
    // istrorsx_core, which depends on istrorsx_hw).
    sub_image_number_ = this->create_subscription<istrorsx_hw::msg::ImageNumber>(
        "/robot/image_number", 10,
        [this](const istrorsx_hw::msg::ImageNumber::SharedPtr msg) { cb_image_number(msg); });
}

void CameraNode::cb_image_number(const istrorsx_hw::msg::ImageNumber::SharedPtr msg)
{
    last_image_number_ = msg->image_number;
}

// NOTE: unlike gps_writeData()/lidar_writeData()/ctrlBoard_writeData(), this
// stores cv::Mat objects directly rather than POD fields -- safe and cheap
// since DataSet lives entirely within this one process (Mat's copy is a
// shallow, reference-counted handle, not a pixel copy), matching how the
// legacy single-process app also just assigned Mats into DataSet directly.
int camera_writeData(const cv::Mat& camera_img, const cv::Mat& camera_depth)
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->camera_img = camera_img;
    pdata->camera_depth = camera_depth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroCamera2, "camera_writeData", "camera_img=" << camera_img.cols << "x" << camera_img.rows
        << ", camera_depth=" << (camera_depth.empty() ? "empty" : "present"));

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::camera_writeData", t);
#endif

    return 0;
}

void CameraNode::publishCameraFrame(const cv::Mat &camera_img, const cv::Mat &camera_depth)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::CameraFrame camera_msg;

    camera_msg.image_number = last_image_number_;

    // total()*elemSize() only equals the exact byte range starting at
    // `data` when the Mat has no per-row padding/gaps (e.g. from an ROI)
    // -- assign() would silently copy the wrong bytes otherwise. If not
    // continuous, leave the whole color_* group at its default zero
    // (uninitialized, "0x0") rather than publishing a width/height that
    // doesn't match the (missing) data.
    if (camera_img.isContinuous()) {
        camera_msg.color_width = camera_img.cols;
        camera_msg.color_height = camera_img.rows;
        camera_msg.color_cv_type = camera_img.type();
        camera_msg.color_step = (int32_t)camera_img.step;
        size_t color_bytes = camera_img.total() * camera_img.elemSize();
        camera_msg.color_data.assign(camera_img.data, camera_img.data + color_bytes);
    } else {
        LOGM_ERROR(loggerIstroCamera2, "publishCameraFrame", "msg=\"camera_img is not continuous, cannot copy into CameraFrame!\"");
    }

    if (!camera_depth.empty() && camera_depth.isContinuous()) {
        camera_msg.depth_width = camera_depth.cols;
        camera_msg.depth_height = camera_depth.rows;
        camera_msg.depth_cv_type = camera_depth.type();
        camera_msg.depth_step = (int32_t)camera_depth.step;
        size_t depth_bytes = camera_depth.total() * camera_depth.elemSize();
        camera_msg.depth_data.assign(camera_depth.data, camera_depth.data + depth_bytes);
    } else {
        if (!camera_depth.empty()) {
            LOGM_ERROR(loggerIstroCamera2, "publishCameraFrame", "msg=\"camera_depth is not continuous, cannot copy into CameraFrame!\"");
        }
        // depth_width left at its default 0 -- same "unavailable" sentinel as a failed depth capture
    }

    pub_camera_->publish(camera_msg);
    timeEnd("istro::camera_node.publishCameraFrame", tpub);
}

// Periodically capture a camera frame (color + depth)
// void *capture_camera_thread(void *parg)
void CameraNode::camera_read_cb()
{
    cv::Mat camera_img;
    cv::Mat camera_depth;

    double t;
    int result = 0;

    //LOG_THREAD_NAME("capture_camera");
    LOGM_INFO(loggerIstroCamera2, "capture_camera_thread", "msg=\"start\"");

    do {
        t = timeBegin();

#if 0  // NOT_YET_MIGRATED: reads process_dir/process_ref/process_x/process_y/process_yaw
       // (vision/navigation pose state) via capture_readData(). planner_node (istro_rt2025.cpp's
       // process_thread) now exists, but deliberately doesn't publish this pose for camera_node to
       // stamp onto captured frames -- see doc/ai/01_architecture.md's planner_node decision #1
       // (simplified pose: planner_node uses its own *current* pose when folding VisionData into
       // wmodel, not a pose captured alongside the image). Not a missing dependency, a deliberate
       // simplification -- revisit only if real-world testing shows it causes meaningful obstacle
       // misplacement.
       // (image_number itself is NOT part of this gap -- it's handled separately, see
       // cb_image_number()/ImageNumber.msg above and publishCameraFrame()'s use of
       // last_image_number_.)
        int    process_dir;
        int    process_ref;
        double process_x;
        double process_y;
        double process_yaw;

        if (capture_readData(process_dir, process_ref, process_x, process_y, process_yaw) < 0) {
            result = -2;
            break;
        }
#endif  // NOT_YET_MIGRATED

        if (camera.getFrame(camera_img) < 0) {
            // Deliberately not fatal, matching the legacy capture_camera_thread(): it also left
            // `result` untouched here and just retried on the next loop iteration (msleep(10) +
            // continue) instead of exiting -- a frame-grab failure is expected to be transient,
            // unlike gps_thread's/capture_lidar_thread's getData() failures. The next timer tick
            // is this port's equivalent retry.
            LOGM_ERROR(loggerIstroCamera2, "capture_camera_thread", "msg=\"camera.getFrame() failed!\"");
            break;
        }
        timeEnd("istro::capture_camera_thread.capture", t);

        t = timeBegin();
        if (camera.getFrameDepth(camera_depth) >= 0) {
            timeEnd("istro::capture_camera_thread.capture_depth", t);
        }
        // No else/error handling here either, matching the legacy code -- a depth-capture
        // failure just leaves camera_depth empty, which both camera_writeData() and the
        // CameraFrame publish below already treat as "depth unavailable this frame".

        if (camera_writeData(camera_img, camera_depth) < 0) {
            result = -4;
            break;
        }

        publishCameraFrame(camera_img, camera_depth);

        LOGM_DEBUG(loggerIstroCamera2, "capture_camera_thread", "msg=\"data captured\"");
    } while(0);

    if (result >= 0) {
        LOGM_INFO(loggerIstroCamera2, "capture_camera_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstroCamera2, "capture_camera_thread", "msg=\"exit(" << result << ")\"");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_camera(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroCamera2, "main", "msg=\"istro_main_camera() failed, exiting\"");
        rclcpp::shutdown();
        // _exit(), not return -- librealsense2.so's own global destructors
        // crash on normal process exit when linked alongside rclcpp/DDS in
        // the same process (confirmed via gdb backtrace: SIGSEGV inside
        // librealsense2's __do_global_dtors_aux, no istrorsx_hw code
        // involved -- reproduced even when Camera::init() was never called
        // at all, and neither a standalone librealsense2-only test nor
        // rs-enumerate-devices crash the same way, so it's specifically the
        // librealsense2 + rclcpp/DDS combination in one process). _exit()
        // skips C++ static/global destructors entirely, avoiding that code
        // path -- see doc/ai/03_progress.md.
        _exit(1);
    }

    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);

    istro_close_camera();

    rclcpp::shutdown();
    _exit(0);
}
