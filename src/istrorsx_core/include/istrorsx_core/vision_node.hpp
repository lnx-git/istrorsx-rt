#pragma once

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "dmap.h"
#include "istrorsx_hw/msg/camera_frame.hpp"
#include "istrorsx_core/msg/vision_data.hpp"
#include "istrorsx_core/msg/vision_debug_data.hpp"
#include "istrorsx_core/msg/vision_control.hpp"
#include "istrorsx_core/msg/planner_data.hpp"

// Vision node: a 1:1 port of istro_rt2025.cpp's vision_thread() -- legacy's
// combined color-clustering (vision.cpp) + NN-mask (visionn.cpp) + depth-jump
// (vision_depth.cpp) + QR-scan (qrscan.cpp) perception pipeline. See
// doc/ai/01_architecture.md § vision_node for the full design writeup.
//
// One node, two independent tick callbacks (cb_camera_front()/cb_camera_rear()),
// not two node instances -- unlike camera_node's front/rear split (hardware
// ownership/crash isolation), vision_node owns no hardware, just reacts to
// camera_node's already-published CameraFrame messages. Both callbacks share
// one set of Vision/VisionDepth/QRScanner/VisioNN instances (istro_main_vision.cpp)
// -- safe without locking since this project's single-threaded rclcpp executor
// serializes all callbacks, matching legacy's own single-thread design (only
// one camera was ever actually processed at a time there too, gated by
// vision_thread_processRear_enabled).
//
// Follows the cross-process DataSet round-trip pattern (05_migration_guide.md)
// for the INPUT side only: cb_camera_front()/cb_camera_rear() write the
// incoming CameraFrame into this process's own DataSet (camera_writeData()/
// camera2_writeData(), mirroring legacy capture_camera_thread/capture_camera2_thread's
// writes into pdata->camera_img/camera2_img), then visionTickFront()/
// visionTickRear() open by reading it straight back out via vision_readData()/
// vision2_readData(). Unlike navigation_node, there is no OUTPUT-side DataSet
// write: legacy's vision_dmap/vision_markers/etc. were only ever read back by
// OTHER threads (process_thread, save_thread), never by vision_thread itself
// -- those don't exist as separate in-process consumers here (planner_node/
// save_node are always separate ROS processes, reached via VisionData.msg/
// VisionDebugData.msg instead), so writing them into DataSet would just be
// dead state nobody reads. publishVisionData()/publishVisionDebugData() take
// their values as plain parameters instead, same as publishNavigationData().
//
// Deliberate simplification vs. legacy: the "pdata2" prefetch optimization
// (send the next already-captured frame to the NN server while still
// finishing the current one's local postprocessing, to overlap network/
// inference latency) does not map cleanly onto ROS's one-callback-per-message
// model and is not ported -- each tick sends its own frame to the NN and
// waits for the reply in place. Revisit if NN round-trip latency turns out to
// matter in practice.
class VisionNode : public rclcpp::Node {
public:
    VisionNode();

private:
    // ---- ROS callbacks -- one call = one legacy vision_thread iteration's
    // front-camera or rear-camera branch ----
    void cb_camera_front(const istrorsx_hw::msg::CameraFrame::SharedPtr msg);
    void cb_camera_rear(const istrorsx_hw::msg::CameraFrame::SharedPtr msg);
    // Port of vision_thread_qrscan_enable()/_disable()/processRear_enable()/
    // _disable() (istro_rt2025.cpp) -- there they were free functions called
    // from process_thread's helpers; here process_thread is a different
    // process (planner_node), so it publishes VisionControl instead.
    void cb_control(const istrorsx_core::msg::VisionControl::SharedPtr msg);
    // Caches planner_node's process_dir for DegreeMap::find()'s "mid"
    // parameter -- see visionTickFront()/visionTickRear(); resolves the gap
    // that was hardcoded to 90 before planner_node existed.
    void cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg);

    // ---- DataSet round-trip, input side only (see class comment). Both
    // *Data() pairs follow legacy's readData/writeData convention: return 0
    // on success, <0 on error -- a caller that gets <0 must not go on to use
    // the (unset) out-params. ----
    int camera_writeData(const cv::Mat &camera_img, const cv::Mat &camera_depth);
    int camera2_writeData(const cv::Mat &camera_img, const cv::Mat &camera_depth);
    int vision_readData(cv::Mat &camera_img, cv::Mat &camera_depth);
    int vision2_readData(cv::Mat &camera_img, cv::Mat &camera_depth);

    // ---- Port of vision_thread's front-camera / rear-camera body ----
    void visionTickFront();
    void visionTickRear();

    // Not a legacy port -- plain publish helpers, so unlike the *Data()
    // methods above they take their values as parameters instead of going
    // through DataSet themselves.
    void publishVisionData(int camera_id, int64_t image_number, const DegreeMap &dmap, int angle_min, int angle_max,
        double qrscan_latitude, double qrscan_longitude);
    void publishVisionDebugData(int camera_id, int64_t image_number, const cv::Mat &markers, const cv::Mat &epweight, const cv::Mat &elweight,
        const cv::Mat &camera_img_pred, int angle_min, int angle_max);

    // ---- Publishers ----
    rclcpp::Publisher<istrorsx_core::msg::VisionData>::SharedPtr pub_vision_front_;
    rclcpp::Publisher<istrorsx_core::msg::VisionData>::SharedPtr pub_vision_rear_;
    rclcpp::Publisher<istrorsx_core::msg::VisionDebugData>::SharedPtr pub_vision_debug_front_;
    rclcpp::Publisher<istrorsx_core::msg::VisionDebugData>::SharedPtr pub_vision_debug_rear_;

    // ---- Subscribers ----
    rclcpp::Subscription<istrorsx_hw::msg::CameraFrame>::SharedPtr sub_camera_front_;
    rclcpp::Subscription<istrorsx_hw::msg::CameraFrame>::SharedPtr sub_camera_rear_;
    rclcpp::Subscription<istrorsx_core::msg::VisionControl>::SharedPtr sub_control_;
    rclcpp::Subscription<istrorsx_core::msg::PlannerData>::SharedPtr sub_planner_data_;

    // ---- vision_thread's process-global flags (istro_rt2025.cpp:
    // vision_thread_qrscan_enabled/vision_thread_processRear_enabled) ->
    // members, updated by cb_control() ----
    bool qrscan_enabled_ = false;
    bool process_rear_enabled_ = false;

    // planner_node's process_dir (DegreeMap::find()'s "mid" parameter,
    // istro_rt2025.cpp's process_thread), cached from PlannerData.msg.
    // Defaults to 90 (straight ahead) until the first message arrives,
    // matching the neutral/no-turn default used throughout istro_rt2025.cpp
    // itself before planner_node existed.
    int process_dir_ = 90;

    // CameraFrame.msg's own image_number field (see its comment), cached by
    // cb_camera_front()/cb_camera_rear() right before calling visionTickFront()/
    // visionTickRear() synchronously -- plain members, not a DataSet round
    // trip, since nothing else in-process needs this (same reasoning as
    // VisionData/VisionDebugData's own OUTPUT-side DataSet skip, see class
    // comment above).
    int64_t camera_image_number_ = 0;
    int64_t camera2_image_number_ = 0;
};
