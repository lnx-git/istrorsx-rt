#pragma once

#include <deque>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_core/istrobtx/navmap.h"

#include "istrorsx_hw/msg/camera_frame.hpp"
#include "istrorsx_hw/msg/lidar_data.hpp"
#include "istrorsx_hw/msg/gps_data.hpp"
#include "istrorsx_core/msg/vision_debug_data.hpp"
#include "istrorsx_core/msg/navigation_data.hpp"
#include "istrorsx_core/msg/navigation_point_set.hpp"
#include "istrorsx_core/msg/navigation_route.hpp"
#include "istrorsx_core/msg/drive_command.hpp"
#include "istrorsx_core/msg/planner_data.hpp"
#include "istrorsx_core/msg/planner_debug_data.hpp"
#include "istrorsx_core/msg/save_event.hpp"
#include "istrorsx_core/msg/save_data.hpp"
#include "istrorsx_core/srv/get_wm_grid_snapshot.hpp"

// Persistence node: a 1:1 port of istro_rt2025.cpp's save_thread -- legacy's
// throttled (~4/sec, far below the perception pipeline's own ~30fps) image/
// map dump to disk, for (a) online visualization (the future telemetry_node,
// see 01_architecture.md), (b) post-hoc driving-problem review, (c) NN
// retraining data. See doc/ai/01_architecture.md's save_node section for the
// full design writeup and the decisions behind it.
//
// legacy's save_thread pulled its input from ONE shared DataSet slot
// (threads.getData(THDATA_STATE_PROCESSED, THDATA_STATE_SAVING) -- a
// blocking wait, itself paced by process_thread's own tick), which
// guaranteed every field it read (camera_img, vision_markers, lidar_data,
// process_angle, ...) came from the exact same fused snapshot. That
// atomicity has no ROS equivalent: save_node instead caches the *latest*
// value from each independent topic (cb_camera_front()/cb_vision_debug_front()/
// cb_lidar()/cb_drive_command()/cb_planner_debug()/...) as plain members --
// no DataSet round trip, matching vision_node's/planner_node's own
// precedent for state with no other in-process consumer.
//
// Deliberately NOT driven by PlannerDebugData.msg's own arrival (unlike an
// earlier version of this design): tying save_node's only tick to another
// node's publish rate would mean it saves nothing at all whenever
// planner_node is down/hasn't started yet, even though camera_node/
// lidar_node data is still arriving and worth persisting. saveTick() is
// instead its own rclcpp::WallTimer, matching drive_node's/planner_node's
// own precedent of a dedicated timer decoupled from any single input's
// cadence -- every *other* callback, including cb_planner_debug(), is
// purely a cache-setter like all the others.
//
// Camera vs. vision-debug pairing is verified explicitly via each message's
// own image_number (CameraFrame.msg/VisionData.msg/VisionDebugData.msg's own
// field, see those messages' comments): saveTick() never resynchronizes --
// it will not pair a camera frame with vision-debug data carrying a
// different image_number, matching the user's own explicit design call (see
// 01_architecture.md).
//
// A single cached "latest camera frame" is NOT enough to reliably find that
// pairing, though (an earlier version of this code only kept one). vision_node
// and save_node are two independent consumers of the same CameraFrame stream,
// each sampling "whatever's freshest" on its own schedule (vision_node:
// whenever its own NN eval finishes; save_node: every SAVE_PERIOD) -- with no
// coordination between them, there is no guarantee those two independently-
// freshest picks ever land on the same image_number, *especially* once
// camera_node publishes faster than vision_node can keep up (a normal,
// steady-state condition when vision eval is slow, not just transient startup
// lag -- confirmed via testing: with only one cached slot, save_node could end
// up picking a camera frame vision_node had already skipped entirely, timing
// out every single time and never saving a single vision.jpg, no matter how
// long the timeout). Fixed with camera_front_buffer_/camera_rear_buffer_: a
// short rolling history of recent camera frames keyed by image_number (see
// CameraFrameEntry below), so the pairing check in saveCameraGroup()/
// saveCamera2Group() can look *backward* for whichever specific frame
// vision_node actually processed, not just compare against whatever's
// currently newest. Entries age out (and get logged, once, as a WARN --
// "camera image discarded, never paired with vision") once older than
// CAMERA_VISION_PAIR_TIMEOUT (save_node.cpp, 1000ms) -- this is also what
// bounds the case where vision_node stops publishing entirely (crashed): a
// frozen vision_front_image_number_ will simply never match anything new
// again, and every buffered frame quietly times out instead of piling up
// forever. The "camera"/"cdepth" tags' own save is unrelated to this buffer
// -- driven by camera_front_img_/camera_front_saved_number_ exactly as
// before, tracked separately from save_camera_ (a pure SAVE_PERIOD-window
// throttle, reset every 250ms) so a frame doesn't get re-saved as "camera"
// again on every throttle-reset.
class SaveNode : public rclcpp::Node {
public:
    SaveNode();

private:
    // ---- ROS callbacks: fast-path cache -- see class comment ----
    void cb_camera_front(const istrorsx_hw::msg::CameraFrame::SharedPtr msg);
    void cb_camera_rear(const istrorsx_hw::msg::CameraFrame::SharedPtr msg);
    void cb_vision_debug_front(const istrorsx_core::msg::VisionDebugData::SharedPtr msg);
    void cb_vision_debug_rear(const istrorsx_core::msg::VisionDebugData::SharedPtr msg);
    void cb_lidar(const istrorsx_hw::msg::LidarData::SharedPtr msg);
    void cb_drive_command(const istrorsx_core::msg::DriveCommand::SharedPtr msg);
    // GPS breadcrumb trail (legacy gps_pt[]/gps_pt_cnt, gps_thread's own
    // "store last N gps raw positions" block) -- save_node's own copy,
    // rebuilt from GpsData.msg rather than shared with gps_node/
    // navigation_node (see 01_architecture.md's "Not an output at all" note
    // on saveNavMap()'s inputs).
    void cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg);
    void cb_navigation(const istrorsx_core::msg::NavigationData::SharedPtr msg);
    // Applies planner_node's QR-scan-resolved navigation point to this
    // process's own navigationPoint[] copy -- needed because saveNavMap()
    // *draws* that array (navmap_draw()/navmap_export_kml()), see the .cpp.
    void cb_navigation_point_set(const istrorsx_core::msg::NavigationPointSet::SharedPtr msg);
    // Applies navigation_node's current NAVMAP_FLAG_ROUTE-flagged segment
    // indices onto this process's own navMapSegment[] copy -- needed because
    // navmap_draw()/navmap_export_kml() only highlight the route using that
    // flag, which navigation_node's navmap_planRouteLL() otherwise only ever
    // sets on ITS OWN copy of the array. See NavigationRoute.msg's own
    // comment for why raw indices are safe to reuse here.
    void cb_navigation_route(const istrorsx_core::msg::NavigationRoute::SharedPtr msg);
    // Caches PlannerDebugData.msg's own fields -- image_number/lidar_angle_min/
    // max/stop/process_angle_min/max -- exactly like every other cb_* here.
    // No longer save_node's tick trigger, see class comment.
    void cb_planner_debug(const istrorsx_core::msg::PlannerDebugData::SharedPtr msg);
    // Not in legacy: samples planner_node's process_x/y into process_pt_ -- the local-position
    // counterpart of cb_gps()'s GPS trail, drawn next to it by saveNavMap().
    void cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg);

    // ---- Tick: own rclcpp::WallTimer (see class comment for why, not
    // PlannerDebugData.msg's arrival). 1:1 port of save_thread's own per-
    // iteration body (throttle-flag reset, diskstats check, the four
    // conf.useCamera/useCamera/useLidar/write_wmodel-gated save groups). ----
    void saveTick(void);
    void reportCameraStats(void);

    // GetWMGridSnapshot.srv response handler -- necessarily async (this
    // project's single-threaded rclcpp::executor convention forbids a
    // blocking service call from inside saveTick(), unlike legacy's
    // same-thread, same-process direct wmodel access). Populates save_node's
    // own wmodel (istro_main_save.h) from the response, then does the same
    // wmgrid/wmgrif/navmap throttle-and-save legacy's save_thread does
    // inline right after computing write_wmodel.
    void cb_wmgrid_snapshot(rclcpp::Client<istrorsx_core::srv::GetWMGridSnapshot>::SharedFuture future);

    // ---- Port of legacy saveImage()/save_thread's own save-group bodies.
    // saveImage() returns the out/-relative filename actually written ("" if
    // the image was empty, matching legacy's own "if (img.empty()) return;"
    // early-out) -- callers append a SaveData entry to pending_saves_ only
    // when non-empty, exactly mirroring legacy's own per-tag save_change bit. ----
    // aux: optional extra key=value text appended to saveImage()'s own log
    // line, for facts that belong WITH the save rather than on a line of
    // their own -- currently the count of camera frames dropped unpaired
    // since the previous save. Keeping it here costs no extra log lines.
    std::string saveImage(int64_t image_number, const std::string &tag, const std::string &ext, const cv::Mat &img,
                          const std::string &aux = "");
    void saveCameraGroup(void);   // front camera+vision+nnpred+cdepth
    void saveCamera2Group(void);  // rear  rcamera+rvision+rnnpred+rcdepth
    void saveLidarGroup(void);
    void saveWmodelGroup(void);   // wmgrid[+wmgrif][+navmap], called from cb_wmgrid_snapshot()
    void saveNavMap(void);

    // Not a legacy port -- publishes whatever accumulated in pending_saves_
    // this tick, see SaveEvent.msg's own comment on why this is an
    // incremental notification, not a full-state snapshot.
    void publishSaveEvent(void);

    // ---- Publishers/Subscribers/Client ----
    rclcpp::Publisher<istrorsx_core::msg::SaveEvent>::SharedPtr pub_save_event_;
    rclcpp::Subscription<istrorsx_hw::msg::CameraFrame>::SharedPtr sub_camera_front_;
    rclcpp::Subscription<istrorsx_hw::msg::CameraFrame>::SharedPtr sub_camera_rear_;
    rclcpp::Subscription<istrorsx_core::msg::VisionDebugData>::SharedPtr sub_vision_debug_front_;
    rclcpp::Subscription<istrorsx_core::msg::VisionDebugData>::SharedPtr sub_vision_debug_rear_;
    rclcpp::Subscription<istrorsx_hw::msg::LidarData>::SharedPtr sub_lidar_;
    rclcpp::Subscription<istrorsx_core::msg::DriveCommand>::SharedPtr sub_drive_command_;
    rclcpp::Subscription<istrorsx_hw::msg::GpsData>::SharedPtr sub_gps_;
    rclcpp::Subscription<istrorsx_core::msg::NavigationData>::SharedPtr sub_navigation_;
    rclcpp::Subscription<istrorsx_core::msg::NavigationPointSet>::SharedPtr sub_navigation_point_set_;
    rclcpp::Subscription<istrorsx_core::msg::NavigationRoute>::SharedPtr sub_navigation_route_;
    rclcpp::Subscription<istrorsx_core::msg::PlannerDebugData>::SharedPtr sub_planner_debug_;
    rclcpp::Subscription<istrorsx_core::msg::PlannerData>::SharedPtr sub_planner_data_;
    rclcpp::Client<istrorsx_core::srv::GetWMGridSnapshot>::SharedPtr client_wmgrid_snapshot_;

    // ---- Timer -- saveTick()'s own pace, decoupled from any single input's
    // publish rate (see class comment). Period (SAVE_TICK_PERIOD, save_node.cpp)
    // is frequent enough to reliably catch the SAVE_PERIOD (250ms) window
    // without adding a second, coarser throttle of its own -- SAVE_PERIOD
    // itself remains the real save-rate limiter, matching legacy 1:1. ----
    rclcpp::TimerBase::SharedPtr timer_;

    // One recent camera frame, kept only long enough for saveCameraGroup()/
    // saveCamera2Group() to find it by image_number when vision_node's own
    // (possibly-delayed) VisionDebugData for that exact frame shows up -- see
    // class comment on why a single cached slot alone can't provide this.
    // Color image only (drawOutput() is all the buffer exists for -- depth
    // is handled independently, see camera_front_depth_ below).
    struct CameraFrameEntry {
        int64_t image_number;
        cv::Mat img;
        double cache_time;  // timeBegin() at push -- see CAMERA_VISION_PAIR_TIMEOUT
    };

    // ---- Cached latest input state (see cb_* above). Default image_numbers
    // deliberately differ across camera/vision pairs (-1 vs. -2) so two
    // still-untouched members don't spuriously "match" before any real
    // message has arrived on either topic. ----
    cv::Mat camera_front_img_, camera_front_depth_;
    int64_t camera_front_image_number_ = -1;
    // image_number the "camera"/"cdepth" tags were last written for --
    // distinct from save_camera_ (a pure SAVE_PERIOD-window throttle, reset
    // every 250ms): this instead survives across those resets so the same
    // frame doesn't get re-saved as "camera" again every time save_camera_
    // clears while camera_node is (briefly) not publishing anything newer.
    int64_t camera_front_saved_number_ = -1;
    // Rolling history for vision pairing -- see CameraFrameEntry/class comment.
    // Pushed to (with a generous hard count cap, a pure safety net -- see
    // save_node.cpp) by cb_camera_front(); searched and time-evicted by
    // saveCameraGroup(), never by the fast-path callback itself.
    std::deque<CameraFrameEntry> camera_front_buffer_;
    cv::Mat camera_rear_img_, camera_rear_depth_;
    int64_t camera_rear_image_number_ = -1;
    int64_t camera_rear_saved_number_ = -1;
    std::deque<CameraFrameEntry> camera_rear_buffer_;

    // Camera frames dropped unpaired since the last successful vision save,
    // one counter per camera. Replaces a per-frame WARN that made up 99% of
    // all warnings in a 2026-09-09 Jetson run (11443 of 11576 lines, ~22/s):
    // camera_node publishes at 30 FPS and vision_node processes at ~12/s, so
    // most frames legitimately never pair, and saying so once per frame
    // buried everything else. The count is reported with the save it precedes
    // instead, which is the number actually worth knowing -- how many were
    // dropped between two frames that made it to disk.
    long camera_front_discarded_ = 0;
    long camera_rear_discarded_ = 0;

    cv::Mat vision_front_markers_, vision_front_epweight_, vision_front_elweight_, vision_front_pred_;
    int64_t vision_front_image_number_ = -2;
    cv::Mat vision_rear_markers_, vision_rear_epweight_, vision_rear_elweight_, vision_rear_pred_;
    int64_t vision_rear_image_number_ = -2;
    int vision_front_angle_min_ = -1, vision_front_angle_max_ = -1;
    int vision_rear_angle_min_ = -1, vision_rear_angle_max_ = -1;

    istrorsx_hw::msg::LidarData::SharedPtr lidar_data_;  // whole message kept -- simplest way to hold a variable-length scan

    int process_angle_ = -1;  // DriveCommand.msg's own "angle" field

    // PlannerDebugData.msg's own fields, cached by cb_planner_debug() --
    // used by saveLidarGroup()/saveWmodelGroup() the next time saveTick()
    // fires, not necessarily the same tick they arrived on (see class
    // comment). Defaults match every other "not found"/"none yet" sentinel
    // in this port.
    int64_t planner_image_number_ = -1;
    int lidar_angle_min_ = -1, lidar_angle_max_ = -1, lidar_stop_ = -1;
    int process_angle_min_ = -1, process_angle_max_ = -1;

    static const int GPS_POINT_NUM = 30;
    aux_point_t gps_pt_[GPS_POINT_NUM + 1];
    int gps_pt_cnt_ = 0;

    // process_x/y history, newest first: one sample every PROCESS_POINT_PERIOD ms, at most
    // PROCESS_POINT_NUM (30 s), cleared when process_ref changes (the axes change with it)
    static const int PROCESS_POINT_NUM = 30;
    static const int PROCESS_POINT_PERIOD = 1000;
    std::deque<std::pair<double, double>> process_pt_;
    double process_pt_lastt_ = -1;
    int process_ref_ = -1;

    double gps_latitude_ = 999999, gps_longitude_ = 999999;  // ANGLE_NONE (config.h) -- not included here, see .cpp

    double navp_azimuth_ = 999999, navp_latitude_ = 999999, navp_longitude_ = 999999;

    // ---- save_thread's own throttle state (istro_rt2025.cpp) ----
    // Detected once in the constructor (see detectSaveDeviceName() in
    // save_node.cpp) -- not a legacy field, replaces legacy's hardcoded
    // SAVE_DEVICE_NAME="mmcblk0p1" literal (Jetson-only, wrong on any dev VM).
    std::string save_device_name_;
    int diskstats_iop_ = -2;

    // ---- camera arrival monitoring (not legacy) ----
    // Publisher-side stalls on the camera topics were seen on the dev VM
    // (2026-09-07, doc/ai/03_progress.md): delivery pauses for ~2 s and then
    // catches up in a burst, which trips planner's VISION_LAG_PERIOD and sets
    // process_stop with nothing actually wrong. Diagnosing it needed a
    // separate probe attached to the topic; save_node already subscribes to
    // both cameras, so it can report the same numbers itself and have them in
    // the log of every drive, including on the robot where running an extra
    // probe is not practical.
    struct CameraArrivalStats
    {
        double lastt = -1;       // timeBegin() of the previous frame
        double min_ms = -1;      // over the current report window
        double max_ms = -1;
        double sum_ms = 0;
        long   count = 0;        // frames in this window
        long   stalls = 0;       // gaps over CAMERA_STALL_PERIOD in this window

        // Called from the camera callbacks; returns nothing, just accumulates.
        void arrived(double now, double stall_ms);
        void reset(void);
    };
    CameraArrivalStats camera_front_stats_, camera_rear_stats_;
    double camera_stats_to_ = -1;   // next report due, see saveTick()
    int save_camera_ = 0, save_camera2_ = 0, save_lidar_ = 0, save_wmodel_ = 0;
    double save_to_ = -1, wmgrif_to_ = -1, navmap_to_ = -1;
    long last_wmimgnum_ = -1;

    // Accumulates this tick's actually-written files before publishSaveEvent()
    // -- cleared at the start of each saveTick(), and again after
    // cb_wmgrid_snapshot() publishes its own (later-arriving) batch.
    std::vector<istrorsx_core::msg::SaveData> pending_saves_;
};
