#include "istrorsx_core/save_node.hpp"

#include <algorithm>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include "config.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "event_defs.h"
#include "lidar_defs.h"
#include "camera_defs.h"
#include "istrorsx_core/istrobtx/istro_main_save.h"
#include "istrorsx_core/istrobtx/vision.h"
#include "istrorsx_core/istrobtx/navig.h"    // navigation_point_set(), see cb_navigation_point_set()
#include "istrorsx_core/istrobtx/navmap.h"

LOG_DEFINE(loggerIstroSave2, "istroSave");

// ---------------------------------------------------------------------------
// save_thread's own file-scope constants (istro_rt2025.cpp), unchanged.
// ---------------------------------------------------------------------------

static const int    SAVE_PERIOD        =   250;  // (ms) 4x/sec save one camera+vision image, one lidar image and one wmgrid image
static const int    SAVE_PERIOD_WMGRIF = 10000;  // (ms) every 10 seconds save wmgrif (could take more time)
static const int    SAVE_PERIOD_NAVMAP =  1000;  // (ms) every 1 second write kml

static const int    SAVE_DISKSTAT_IOP_MAX = 30;  // maximum value for IOP to write new files to disk

// Not a legacy constant -- saveTick()'s own timer period, decoupled from any
// single input topic's publish rate (see save_node.hpp's class comment).
static const int    SAVE_TICK_PERIOD = 50;  // (ms)

// Camera arrival monitoring (not legacy). A gap this long between frames is
// a stall, not jitter: the real camera runs at 30 FPS and the mock at 5, so
// 1000 ms is several missed frames either way, and it fires well before
// planner's VISION_LAG_PERIOD (2400 ms) turns the same silence into a
// process_stop. Reported every CAMERA_STATS_PERIOD so a drive log carries the
// numbers without anyone having to attach test_camera_delay.
static const int    CAMERA_STALL_PERIOD = 1000;  // (ms)
static const int    CAMERA_STATS_PERIOD = 2000;  // (ms)

// Not legacy constants -- camera_front_buffer_/camera_rear_buffer_'s own
// bounds (see save_node.hpp's class comment and CameraFrameEntry). The time
// bound is the real, intentional limit (user-requested: bounds both a dead
// vision_node -- pairing can never succeed again once it's stopped
// publishing entirely -- and a merely-slow one, without waiting forever
// either way).
//
// Tuned to 600ms (down from an intermediate 3000ms) -- deliberately tighter
// than this dev VM's own CPU/ONNX visionn_server can reliably meet
// (model_predict() alone was observed taking 1199-1324ms here), so testing
// on this VM will occasionally discard a slow vision.jpg pairing -- accepted
// by design, per the user: the real target is the Jetson's own NN pipeline
// reaching ~20ms per frame, and this bound should reflect *that* real-time
// budget (with headroom for the rest of vision_node's own pipeline and
// scheduling jitter), not be loosened to paper over this VM's much slower
// stand-in hardware. A frame that misses this window on the real target
// indicates a genuine processing-time regression worth surfacing via the
// WARN below, not something to silently wait out.
static const int    CAMERA_VISION_PAIR_TIMEOUT = 600;  // (ms)

// Count bound: a pure safety net against unbounded growth if saveTick()
// somehow stalls, not an active limiter -- must still comfortably exceed
// what a real camera stream accumulates within the time bound above, or it
// would start silently competing with the (logged) time-based eviction. At
// this project's own ~30fps perception-pipeline assumption (see SAVE_PERIOD's
// comment), CAMERA_VISION_PAIR_TIMEOUT's full 600ms window is ~18 frames in
// the worst case (vision_node dead, nothing draining the buffer early via a
// match) -- sized well above that.
static const size_t CAMERA_BUFFER_MAX = 48;

static const char*  DISKSTATS_FNAME = "/proc/diskstats";
static const std::string outputDir = "out/";

// ---------------------------------------------------------------------------
// readDiskStats() -- see istro_rt2025.cpp's own comment for the
// /proc/diskstats field layout. Only called under ISTRO_SAVE_DISKSTATS_ENABLE
// (system.h), matching legacy.
//
// NOT a verbatim port of the body, unlike almost everything else in this
// file -- legacy's own fscanf("%d %d %80s %d %d %d ...", ...) reads exactly
// 14 whitespace-separated tokens per do-while iteration, assuming each
// /proc/diskstats line has exactly 11 stat fields after major/minor/name
// (the pre-Linux-4.18 format). Since fscanf()'s %d/%s conversions skip ALL
// whitespace including newlines, they don't respect line boundaries -- on
// any kernel whose lines are longer than 14 tokens (this dev VM's 7.0
// kernel has 20: 3 + 17, Linux gained discard-stats fields in 4.18 and
// flush-stats fields since, and even the real Jetson's own JetPack 6 kernel
// -- 5.15 -- already has more than 14), the first line alone leaves 6
// tokens unconsumed, desyncing every subsequent "device name" read from
// real line boundaries for the rest of the file -- silently never matching
// any real device name again (confirmed via manual testing: this exact
// failure mode, deviceName correctly detected but readDiskStats() still
// returning -2 every time). Rewritten to read one full line at a time
// (fgets()) and sscanf() only the fields actually needed from it -- immune
// to however many trailing fields a given kernel appends, since sscanf()
// simply stops once its own format string is satisfied.
// ---------------------------------------------------------------------------

static int readDiskStats(const char *deviceName, int &f1, int &f2, int &f4, int &f5, int &f6, int &f7, int &f8,
                  int &f9, int &f10, int &f11, int &f12, int &f13, int &f14)
{
    f1 = f2 = f4 = f5 = f6 = f7 = f8 = f9 = f10 = f11 = f12 = f13 = f14 = -1;

    FILE * pFile;
    pFile = fopen(DISKSTATS_FNAME, "r");
    if (pFile == NULL) return -1;

    char line[512];
    char x3[81] = "";
    int x1, x2, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14;
    bool found = false;

    while (fgets(line, sizeof(line), pFile) != NULL) {
        if (sscanf(line, "%d %d %80s %d %d %d %d %d %d %d %d %d %d %d",
                   &x1, &x2, x3, &x4, &x5, &x6, &x7, &x8, &x9, &x10, &x11, &x12, &x13, &x14) < 14) {
            continue;  // a malformed/short line -- skip it, keep scanning (unlike legacy, which gave up on the whole file)
        }
        if (strcmp(x3, deviceName) == 0) {
            found = true;
            break;
        }
    }

    fclose (pFile);

    if (!found) {
        return -2;
    }

    f1 = x1; f2 = x2; f4 = x4; f5 = x5; f6 = x6; f7 = x7; f8 = x8;
    f9 = x9; f10 = x10; f11 = x11; f12 = x12; f13 = x13; f14 = x14;

    return 0;
}

// ---------------------------------------------------------------------------
// detectSaveDeviceName() -- NOT a legacy port. Legacy hardcoded
// SAVE_DEVICE_NAME="mmcblk0p1" (istro_rt2025.cpp) since it only ever ran on
// one specific target (the Jetson's eMMC/SD card); this ROS port runs on
// dev VMs too (typically nvme0n1/sda/vda, never mmcblk0*), so a hardcoded
// name would leave readDiskStats() silently unable to find any match
// (harmless -- see saveTick()'s own diskstats_iop_ <= SAVE_DISKSTAT_IOP_MAX
// gate, -1 never blocks saving -- but noisy: an ERROR log line on every
// single tick). Resolves the real block device backing out/ instead:
// stat()'s st_dev (major:minor) -> /sys/dev/block/<major>:<minor>, a
// symlink the kernel maintains to the device's real sysfs path (e.g.
// ".../nvme0n1/nvme0n1p1") -- the basename of that target is exactly the
// same device-name string /proc/diskstats itself uses. Resolved once at
// startup (SaveNode's constructor), not per-tick -- what filesystem backs
// out/ does not change at runtime.
static std::string detectSaveDeviceName(void)
{
    struct stat st;
    if (stat(outputDir.c_str(), &st) != 0) {
        LOGM_WARN(loggerIstroSave2, "detectSaveDeviceName", "msg=\"stat() on out/ failed, falling back to legacy's own hardcoded device name\", errno=" << errno << " (" << strerror(errno) << ")");
        return "mmcblk0p1";
    }

    char sysPath[64];
    snprintf(sysPath, sizeof(sysPath), "/sys/dev/block/%u:%u", major(st.st_dev), minor(st.st_dev));

    char resolved[256];
    ssize_t len = readlink(sysPath, resolved, sizeof(resolved) - 1);
    if (len < 0) {
        LOGM_WARN(loggerIstroSave2, "detectSaveDeviceName", "msg=\"readlink() failed, falling back to legacy's own hardcoded device name\", path=\"" << sysPath << "\", errno=" << errno << " (" << strerror(errno) << ")");
        return "mmcblk0p1";
    }
    resolved[len] = 0;

    std::string path(resolved);
    size_t slash = path.find_last_of('/');
    std::string deviceName = (slash == std::string::npos) ? path : path.substr(slash + 1);

    LOGM_INFO(loggerIstroSave2, "detectSaveDeviceName", "msg=\"detected disk device backing out/\", device=\"" << deviceName << "\"");
    return deviceName;
}

// ---------------------------------------------------------------------------
// CameraFrame.msg <-> cv::Mat, VisionDebugData.msg field group <-> cv::Mat --
// same reconstruction pattern as vision_node.cpp's own cameraFrameColorToMat()/
// cameraFrameDepthToMat(), duplicated here rather than shared (each is ~6
// lines, not worth a new istrobtx home for two call sites).
// ---------------------------------------------------------------------------

static cv::Mat cameraFrameColorToMat(const istrorsx_hw::msg::CameraFrame::SharedPtr &msg)
{
    if ((msg->color_width <= 0) || (msg->color_height <= 0)) {
        return cv::Mat();
    }
    return cv::Mat(msg->color_height, msg->color_width, msg->color_cv_type,
        (void *)msg->color_data.data(), msg->color_step).clone();
}

static cv::Mat cameraFrameDepthToMat(const istrorsx_hw::msg::CameraFrame::SharedPtr &msg)
{
    if ((msg->depth_width <= 0) || (msg->depth_height <= 0)) {
        return cv::Mat();
    }
    return cv::Mat(msg->depth_height, msg->depth_width, msg->depth_cv_type,
        (void *)msg->depth_data.data(), msg->depth_step).clone();
}

static cv::Mat debugFieldToMat(int32_t width, int32_t height, int32_t cv_type, int32_t step, const std::vector<uint8_t> &data)
{
    if ((width <= 0) || (height <= 0)) {
        return cv::Mat();
    }
    return cv::Mat(height, width, cv_type, (void *)data.data(), step).clone();
}

// ---------------------------------------------------------------------------

SaveNode::SaveNode() : Node("save_node")
{
    pub_save_event_ = this->create_publisher<istrorsx_core::msg::SaveEvent>("/robot/save_event", 10);

    sub_camera_front_ = this->create_subscription<istrorsx_hw::msg::CameraFrame>(
        "/robot/camera_front_data", 10, std::bind(&SaveNode::cb_camera_front, this, std::placeholders::_1));
    sub_camera_rear_ = this->create_subscription<istrorsx_hw::msg::CameraFrame>(
        "/robot/camera_rear_data", 10, std::bind(&SaveNode::cb_camera_rear, this, std::placeholders::_1));
    sub_vision_debug_front_ = this->create_subscription<istrorsx_core::msg::VisionDebugData>(
        "/robot/vision_front_debug_data", 10, std::bind(&SaveNode::cb_vision_debug_front, this, std::placeholders::_1));
    sub_vision_debug_rear_ = this->create_subscription<istrorsx_core::msg::VisionDebugData>(
        "/robot/vision_rear_debug_data", 10, std::bind(&SaveNode::cb_vision_debug_rear, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<istrorsx_hw::msg::LidarData>(
        "/robot/lidar_data", 10, std::bind(&SaveNode::cb_lidar, this, std::placeholders::_1));
    sub_drive_command_ = this->create_subscription<istrorsx_core::msg::DriveCommand>(
        "/robot/drive_command", 10, std::bind(&SaveNode::cb_drive_command, this, std::placeholders::_1));
    sub_gps_ = this->create_subscription<istrorsx_hw::msg::GpsData>(
        "/robot/gps_data", 10, std::bind(&SaveNode::cb_gps, this, std::placeholders::_1));
    sub_navigation_ = this->create_subscription<istrorsx_core::msg::NavigationData>(
        "/robot/navigation_data", 10, std::bind(&SaveNode::cb_navigation, this, std::placeholders::_1));
    sub_navigation_point_set_ = this->create_subscription<istrorsx_core::msg::NavigationPointSet>(
        "/robot/navigation_point_set", 10, std::bind(&SaveNode::cb_navigation_point_set, this, std::placeholders::_1));
    sub_navigation_route_ = this->create_subscription<istrorsx_core::msg::NavigationRoute>(
        "/robot/navigation_route", 10, std::bind(&SaveNode::cb_navigation_route, this, std::placeholders::_1));
    sub_planner_debug_ = this->create_subscription<istrorsx_core::msg::PlannerDebugData>(
        "/robot/planner_debug_data", 10, std::bind(&SaveNode::cb_planner_debug, this, std::placeholders::_1));
    sub_planner_data_ = this->create_subscription<istrorsx_core::msg::PlannerData>(
        "/robot/planner_data", 10, std::bind(&SaveNode::cb_planner_data, this, std::placeholders::_1));

    client_wmgrid_snapshot_ = this->create_client<istrorsx_core::srv::GetWMGridSnapshot>("/robot/get_wmgrid_snapshot");

    // Legacy: "gps_pt[0].style[0] = 0;" -- marks the breadcrumb buffer empty
    // at startup (istro_rt2025.cpp's own pre-thread-start init).
    gps_pt_[0].style[0] = 0;

    save_device_name_ = detectSaveDeviceName();

    save_to_ = timeBegin();
    wmgrif_to_ = timeBegin();
    navmap_to_ = timeBegin();

    // Own timer, deliberately not driven by any single input topic's arrival
    // -- see class comment for why.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SAVE_TICK_PERIOD),
        std::bind(&SaveNode::saveTick, this));

    LOGM_INFO(loggerIstroSave2, "SaveNode", "msg=\"start\", tick_period=" << SAVE_TICK_PERIOD);
}

// ---------------------------------------------------------------------------
// Fast-path callbacks -- cache latest state, see class comment.
// ---------------------------------------------------------------------------

void SaveNode::cb_camera_front(const istrorsx_hw::msg::CameraFrame::SharedPtr msg)
{
    camera_front_stats_.arrived(timeBegin(), CAMERA_STALL_PERIOD);

    camera_front_img_ = cameraFrameColorToMat(msg);
    camera_front_depth_ = cameraFrameDepthToMat(msg);
    camera_front_image_number_ = msg->image_number;

    // Gated on conf.useCamera, same as saveCameraGroup() itself -- that's
    // the only place this buffer is ever drained (the matched-pairing
    // erase()), so pushing while it's disabled would just grow the buffer
    // forever behind nothing but the count cap below.
    if (conf.useCamera) {
        // camera_front_img_ is a shallow (refcounted) cv::Mat handle --
        // pushing it here doesn't clone the pixel data again, see class
        // comment.
        camera_front_buffer_.push_back({camera_front_image_number_, camera_front_img_, timeBegin()});
        // Should never actually trigger -- see CAMERA_BUFFER_MAX's own
        // comment -- but logged like every other discard-unpaired path
        // (CAMERA_VISION_PAIR_TIMEOUT in saveCameraGroup()) rather than left
        // silent, in case that assumption is ever wrong.
        if (camera_front_buffer_.size() > CAMERA_BUFFER_MAX) {
            LOGM_WARN(loggerIstroSave2, "cb_camera_front", "msg=\"camera buffer full, discarding oldest unpaired entry\", image_number="
                << camera_front_buffer_.front().image_number << ", buffer_max=" << CAMERA_BUFFER_MAX);
            camera_front_buffer_.pop_front();
        }
    }
}

void SaveNode::cb_camera_rear(const istrorsx_hw::msg::CameraFrame::SharedPtr msg)
{
    camera_rear_stats_.arrived(timeBegin(), CAMERA_STALL_PERIOD);

    camera_rear_img_ = cameraFrameColorToMat(msg);
    camera_rear_depth_ = cameraFrameDepthToMat(msg);
    camera_rear_image_number_ = msg->image_number;

    if (conf.useCamera) {
        camera_rear_buffer_.push_back({camera_rear_image_number_, camera_rear_img_, timeBegin()});
        if (camera_rear_buffer_.size() > CAMERA_BUFFER_MAX) {
            LOGM_WARN(loggerIstroSave2, "cb_camera_rear", "msg=\"camera2 buffer full, discarding oldest unpaired entry\", image_number="
                << camera_rear_buffer_.front().image_number << ", buffer_max=" << CAMERA_BUFFER_MAX);
            camera_rear_buffer_.pop_front();
        }
    }
}

void SaveNode::cb_vision_debug_front(const istrorsx_core::msg::VisionDebugData::SharedPtr msg)
{
    vision_front_markers_ = debugFieldToMat(msg->markers_width, msg->markers_height, msg->markers_cv_type, msg->markers_step, msg->markers_data);
    vision_front_epweight_ = debugFieldToMat(msg->epweight_width, msg->epweight_height, msg->epweight_cv_type, msg->epweight_step, msg->epweight_data);
    vision_front_elweight_ = debugFieldToMat(msg->elweight_width, msg->elweight_height, msg->elweight_cv_type, msg->elweight_step, msg->elweight_data);
    vision_front_pred_ = debugFieldToMat(msg->camera_img_pred_width, msg->camera_img_pred_height, msg->camera_img_pred_cv_type, msg->camera_img_pred_step, msg->camera_img_pred_data);
    vision_front_angle_min_ = msg->angle_min;
    vision_front_angle_max_ = msg->angle_max;
    vision_front_image_number_ = msg->image_number;
}

void SaveNode::cb_vision_debug_rear(const istrorsx_core::msg::VisionDebugData::SharedPtr msg)
{
    vision_rear_markers_ = debugFieldToMat(msg->markers_width, msg->markers_height, msg->markers_cv_type, msg->markers_step, msg->markers_data);
    vision_rear_epweight_ = debugFieldToMat(msg->epweight_width, msg->epweight_height, msg->epweight_cv_type, msg->epweight_step, msg->epweight_data);
    vision_rear_elweight_ = debugFieldToMat(msg->elweight_width, msg->elweight_height, msg->elweight_cv_type, msg->elweight_step, msg->elweight_data);
    vision_rear_pred_ = debugFieldToMat(msg->camera_img_pred_width, msg->camera_img_pred_height, msg->camera_img_pred_cv_type, msg->camera_img_pred_step, msg->camera_img_pred_data);
    vision_rear_angle_min_ = msg->angle_min;
    vision_rear_angle_max_ = msg->angle_max;
    vision_rear_image_number_ = msg->image_number;
}

void SaveNode::cb_lidar(const istrorsx_hw::msg::LidarData::SharedPtr msg)
{
    lidar_data_ = msg;
}

void SaveNode::cb_drive_command(const istrorsx_core::msg::DriveCommand::SharedPtr msg)
{
    process_angle_ = msg->angle;
}

void SaveNode::cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg)
{
    gps_latitude_ = msg->latitude;
    gps_longitude_ = msg->longitude;

    // Legacy: "store last N gps raw positions" (gps_thread's own block,
    // istro_rt2025.cpp) -- ROS's GpsData.msg only has one lat/lon pair (no
    // separate raw/filtered split, an earlier gps_node simplification), used
    // here as legacy's own "_raw" variant was.
    if (msg->fix && (msg->longitude < ANGLE_OK) && (msg->latitude < ANGLE_OK)) {
        if (gps_pt_cnt_ < GPS_POINT_NUM) {
            gps_pt_[gps_pt_cnt_].style[0] = 0;
            gps_pt_[gps_pt_cnt_ + 1].style[0] = 0;
            gps_pt_cnt_++;
        }
        for (int i = gps_pt_cnt_ - 1; i >= 1; i--) {
            gps_pt_[i] = gps_pt_[i - 1];
        }
        gps_pt_[0].name[0] = 0;
        gps_pt_[0].desc[0] = 0;
        strcpy(gps_pt_[0].style, "g");
        gps_pt_[0].longitude = msg->longitude;
        gps_pt_[0].latitude = msg->latitude;
    }
}

void SaveNode::cb_navigation(const istrorsx_core::msg::NavigationData::SharedPtr msg)
{
    navp_azimuth_ = msg->navp_azimuth;
    navp_latitude_ = msg->navp_latitude;
    navp_longitude_ = msg->navp_longitude;
}

// Keeps THIS process's own navigationPoint[]/navigationPointXY[] copy in sync
// with planner_node's QR-scan-resolved coordinates. save_node doesn't steer,
// but it *draws* that array: both navmap_draw() (the yellow navigation-point
// markers, and the fallback image midpoint when no GPS aux point exists) and
// navmap_export_kml() (its own "navigationPoint" folder) read navigationPoint[]
// directly. Without this, navmap.png/.kml would keep showing the stale
// compiled-in coordinates from navig_data.cpp after a QR scan updated the real
// ones -- silently, and only visible by eyeballing the saved map against where
// the robot actually went. Legacy had no equivalent problem: one process, one
// shared array, so save_thread saw process_thread's own navigation_point_set()
// call immediately. Same reasoning as navigation_node's own copy of this
// callback -- see doc/ai/02_specifications.md § Known interface gaps.
void SaveNode::cb_navigation_point_set(const istrorsx_core::msg::NavigationPointSet::SharedPtr msg)
{
    if (navigation_point_set(msg->point_idx, msg->point_latitude, msg->point_longitude) < 0) {
        LOGM_ERROR(loggerIstroSave2, "cb_navigation_point_set", "msg=\"navigation_point_set() failed!\", point_idx=" << msg->point_idx);
    }
}

// Keeps THIS process's own navMapSegment[] NAVMAP_FLAG_ROUTE flags in sync
// with navigation_node's live route planning -- navmap_planRouteLL()/
// navmap_plan_delete() only ever run in navigation_node's own process, so
// without this, save_node's separately-linked navMapSegment[] copy would
// never have the route flag set and navmap_draw()/navmap_export_kml()'s
// route highlighting would stay permanently empty (the bug this fixes).
// Clears every flag first (plain loop, NOT navmap_plan_delete() -- that
// function also mutates NAVMAP_FLAG_TMP_PLAN/NAVMAP_FLAG_INVISIBLE and calls
// navmap_delete(), side effects save_node's static copy must not trigger),
// then re-applies it at the indices the message carries. Indices are
// bounds-checked since they cross a process boundary -- see
// NavigationRoute.msg's own comment on why they're otherwise safe to reuse
// as-is (navMapSegment[] topology is compile-time-identical everywhere).
// Replays navigation_node's plan onto this process's own navmap copy, using the
// same primitives it used, so navmap_draw()/navmap_export_kml() see the same graph.
void SaveNode::cb_navigation_route(const istrorsx_core::msg::NavigationRoute::SharedPtr msg)
{
    // Drops the previous plan's temporary nodes/segments and clears INVISIBLE/ROUTE.
    navmap_plan_delete();

    // Temporary segments below carry node indices from the publisher's array; they
    // only line up if both sides start from the same base graph.
    if (navmap_node_cnt != msg->base_node_count) {
        LOGM_ERROR(loggerIstroSave2, "cb_navigation_route", "msg=\"base node count mismatch, plan ignored!\""
            << ", navmap_node_cnt=" << navmap_node_cnt << ", base_node_count=" << msg->base_node_count);
        return;
    }

    for (const auto &n : msg->tmp_nodes) {
        navmap_node_add(n.node_id, n.latitude, n.longitude, n.flags);
    }
    for (const auto &sg : msg->tmp_segments) {
        navmap_segment_add(sg.way_id, sg.node1_idx, sg.node2_idx, sg.flags);
    }

    for (int32_t idx : msg->route_segment_indices) {
        if ((idx >= 0) && (idx < navmap_segment_cnt)) {
            navMapSegment[idx].flags |= NAVMAP_FLAG_ROUTE;
        }
    }
    for (int32_t idx : msg->invisible_segment_indices) {
        if ((idx >= 0) && (idx < navmap_segment_cnt)) {
            navMapSegment[idx].flags |= NAVMAP_FLAG_INVISIBLE;
        }
    }
}

void SaveNode::cb_planner_debug(const istrorsx_core::msg::PlannerDebugData::SharedPtr msg)
{
    planner_image_number_ = msg->image_number;
    lidar_angle_min_ = msg->lidar_angle_min;
    lidar_angle_max_ = msg->lidar_angle_max;
    lidar_stop_ = msg->lidar_stop;
    process_angle_min_ = msg->process_angle_min;
    process_angle_max_ = msg->process_angle_max;
}

void SaveNode::cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg)
{
    if (process_ref_ != msg->process_ref) {
        process_pt_.clear();
        process_pt_lastt_ = -1;
        process_ref_ = msg->process_ref;
    }
    if ((process_pt_lastt_ >= 0) && (timeDelta(process_pt_lastt_) < PROCESS_POINT_PERIOD)) {
        return;
    }
    process_pt_lastt_ = timeBegin();
    process_pt_.push_front({msg->process_x, msg->process_y});
    while ((int)process_pt_.size() > PROCESS_POINT_NUM) {
        process_pt_.pop_back();
    }
}

// ---------------------------------------------------------------------------
// saveImage() -- 1:1 port. Returns the workspace-root-relative filename
// actually written ("" if img was empty, matching legacy's early-out), so
// callers can build a SaveData entry only when something was really saved.
// ---------------------------------------------------------------------------

std::string SaveNode::saveImage(int64_t image_number, const std::string &tag, const std::string &ext, const cv::Mat &img,
                                const std::string &aux)
{
    if (img.empty()) {
        return "";
    }

    double t = timeBegin();

    char basename[256];
    long imgnum = (image_number < 0) ? 999999 : (long)image_number;
    sprintf(basename, EVENT_TAG "_%07u_%07u_%s.%s", (unsigned int)save_rand, (unsigned int)imgnum, tag.c_str(), ext.c_str());

    std::string fullname = outputDir + basename;
    cv::imwrite(fullname, img);

    LOGM_DEBUG(loggerIstroSave2, "saveImage", tag.c_str() << "_image=\"" << basename << "\""
        << (aux.empty() ? "" : ", ") << aux);

    char tname[64];
    sprintf(tname, "istro::saveImage('%s')", tag.c_str());
    timeEnd(tname, t);

    return fullname;
}

static void pushSaved(std::vector<istrorsx_core::msg::SaveData> &pending, const std::string &tag, const std::string &filename, int64_t image_number)
{
    if (filename.empty()) {
        return;
    }
    istrorsx_core::msg::SaveData d;
    d.tag = tag;
    d.filename = filename;
    d.image_number = image_number;
    pending.push_back(d);
}

// ---------------------------------------------------------------------------
// saveCameraGroup()/saveCamera2Group() -- 1:1 port of save_thread's front-
// camera/rear-camera save blocks. vision/nnpred are only saved once a
// buffered camera frame (camera_front_buffer_/camera_rear_buffer_) with a
// matching image_number is found -- see class comment for why a single
// cached "latest" camera Mat isn't enough (no shared-struct atomicity in
// ROS, unlike legacy's single pdata, and no guarantee vision_node processes
// the exact same frame save_node would otherwise pick independently).
// ---------------------------------------------------------------------------

void SaveNode::saveCameraGroup(void)
{
    if (!conf.useCamera) {
        return;
    }

    // "camera"/"cdepth": at most once per SAVE_PERIOD window (save_camera_,
    // reset in saveTick()) AND at most once per unique image_number
    // (camera_front_saved_number_, which -- unlike save_camera_ -- does NOT
    // get reset every window). Unrelated to the vision pairing below (that's
    // what camera_front_buffer_ is for) -- camera_front_img_ itself is
    // consumed/cleared right here, same tick.
    if (!camera_front_img_.empty() && !save_camera_ && (camera_front_image_number_ != camera_front_saved_number_)) {
        save_camera_ = 1;
        camera_front_saved_number_ = camera_front_image_number_;

        pushSaved(pending_saves_, "camera", saveImage(camera_front_image_number_, "camera", "jpg", camera_front_img_), camera_front_image_number_);

        if (!camera_front_depth_.empty()) {
            cv::Mat depth_img;
            camera_draw_depth_frame(camera_front_depth_, depth_img, camera_front_image_number_, 0);
            pushSaved(pending_saves_, "cdepth", saveImage(camera_front_image_number_, "cdepth", "jpg", depth_img), camera_front_image_number_);
        }
        camera_front_img_ = cv::Mat();
        camera_front_depth_ = cv::Mat();
    }

    // Vision pairing: look up the *specific* buffered frame vision_node
    // actually processed (by image_number), not whatever's currently
    // newest -- see class comment.
    auto match = std::find_if(camera_front_buffer_.begin(), camera_front_buffer_.end(),
        [this](const CameraFrameEntry &e) { return e.image_number == vision_front_image_number_; });
    if (match != camera_front_buffer_.end()) {
        // Vision::drawOutput()'s own drawEPWeight() indexes epweight_/
        // elweight_ up to ELINES_COUNT/EPOINTS_COUNT unconditionally -- a
        // real vision_node always populates all four VisionDebugData Mats
        // together from one eval() call (never just markers), but this
        // guard protects against a malformed/partial message crashing
        // save_node outright (SIGFPE observed during testing) rather than
        // just skipping that one save, which is all legacy's own
        // same-process pipeline never had to defend against.
        if (!vision_front_markers_.empty() && !vision_front_epweight_.empty() && !vision_front_elweight_.empty()) {
            cv::Mat vision_img;
            vision.drawOutput(match->img, vision_img, vision_front_markers_, vision_front_epweight_, vision_front_elweight_,
                vision_front_angle_min_, vision_front_angle_max_);
            // discarded= rides along on saveImage()'s own line: how many
            // camera frames were dropped unpaired since the previous vision
            // save. 0 means every frame in between found its match.
            pushSaved(pending_saves_, "vision", saveImage(vision_front_image_number_, "vision", "jpg", vision_img,
                "discarded=" + std::to_string(camera_front_discarded_)), vision_front_image_number_);
            camera_front_discarded_ = 0;
        }
        if (!vision_front_pred_.empty()) {
            pushSaved(pending_saves_, "nnpred", saveImage(vision_front_image_number_, "nnpred", "png", vision_front_pred_), vision_front_image_number_);
        }

        // Pairing complete -- now truly consumed, see class comment.
        camera_front_buffer_.erase(match);
        vision_front_markers_ = cv::Mat();
        vision_front_epweight_ = cv::Mat();
        vision_front_elweight_ = cv::Mat();
        vision_front_pred_ = cv::Mat();
    }

    // Anything left at the front of the buffer older than the pairing
    // timeout is never going to match now (vision_node either skipped this
    // exact frame -- a normal consequence of running slower than camera_node
    // -- or has stopped publishing entirely) -- give up on it explicitly and
    // say so, rather than let it linger silently: user-requested visibility
    // into a failure mode SaveEvent.msg itself can't surface (nothing is
    // published for a tag that was never generated).
    while (!camera_front_buffer_.empty() && (timeDelta(camera_front_buffer_.front().cache_time) >= CAMERA_VISION_PAIR_TIMEOUT)) {
        // Counted, not logged. This used to be one WARN per dropped frame and
        // it drowned the log -- 6128 lines on the front camera alone in a
        // 4:46 run, ~21/s. The drop is normal (camera_node 30 FPS vs
        // vision_node ~12/s), so the useful figure is how many were dropped
        // between two saves, which is reported with the save itself below.
        //LOGM_WARN(loggerIstroSave2, "saveCameraGroup", "msg=\"camera image never paired with vision-debug data, discarding\", image_number="
        //    << camera_front_buffer_.front().image_number << ", timeout_ms=" << CAMERA_VISION_PAIR_TIMEOUT);
        camera_front_discarded_++;
        camera_front_buffer_.pop_front();
    }
}

void SaveNode::saveCamera2Group(void)
{
    if (!conf.useCamera) {
        return;
    }

    // See saveCameraGroup()'s identical structure/comments -- same reasoning
    // throughout, mirrored here for the rear camera.
    if (!camera_rear_img_.empty() && !save_camera2_ && (camera_rear_image_number_ != camera_rear_saved_number_)) {
        save_camera2_ = 1;
        camera_rear_saved_number_ = camera_rear_image_number_;

        pushSaved(pending_saves_, "rcamera", saveImage(camera_rear_image_number_, "rcamera", "jpg", camera_rear_img_), camera_rear_image_number_);

        if (!camera_rear_depth_.empty()) {
            cv::Mat depth_img;
            camera_draw_depth_frame(camera_rear_depth_, depth_img, camera_rear_image_number_, 1);
            pushSaved(pending_saves_, "rcdepth", saveImage(camera_rear_image_number_, "rcdepth", "jpg", depth_img), camera_rear_image_number_);
        }
        camera_rear_img_ = cv::Mat();
        camera_rear_depth_ = cv::Mat();
    }

    auto match = std::find_if(camera_rear_buffer_.begin(), camera_rear_buffer_.end(),
        [this](const CameraFrameEntry &e) { return e.image_number == vision_rear_image_number_; });
    if (match != camera_rear_buffer_.end()) {
        // See saveCameraGroup()'s identical guard for why epweight_/
        // elweight_ are also checked, not just markers_.
        if (!vision_rear_markers_.empty() && !vision_rear_epweight_.empty() && !vision_rear_elweight_.empty()) {
            cv::Mat vision_img;
            vision.drawOutput(match->img, vision_img, vision_rear_markers_, vision_rear_epweight_, vision_rear_elweight_,
                vision_rear_angle_min_, vision_rear_angle_max_);
            // See saveCameraGroup() -- dropped since the previous rvision save.
            pushSaved(pending_saves_, "rvision", saveImage(vision_rear_image_number_, "rvision", "jpg", vision_img,
                "discarded=" + std::to_string(camera_rear_discarded_)), vision_rear_image_number_);
            camera_rear_discarded_ = 0;
        }
        if (!vision_rear_pred_.empty()) {
            pushSaved(pending_saves_, "rnnpred", saveImage(vision_rear_image_number_, "rnnpred", "png", vision_rear_pred_), vision_rear_image_number_);
        }

        camera_rear_buffer_.erase(match);
        vision_rear_markers_ = cv::Mat();
        vision_rear_epweight_ = cv::Mat();
        vision_rear_elweight_ = cv::Mat();
        vision_rear_pred_ = cv::Mat();
    }

    while (!camera_rear_buffer_.empty() && (timeDelta(camera_rear_buffer_.front().cache_time) >= CAMERA_VISION_PAIR_TIMEOUT)) {
        // Counted, not logged -- see saveCameraGroup() above.
        //LOGM_WARN(loggerIstroSave2, "saveCamera2Group", "msg=\"camera2 image never paired with vision-debug data, discarding\", image_number="
        //    << camera_rear_buffer_.front().image_number << ", timeout_ms=" << CAMERA_VISION_PAIR_TIMEOUT);
        camera_rear_discarded_++;
        camera_rear_buffer_.pop_front();
    }
}

// ---------------------------------------------------------------------------
// saveLidarGroup() -- 1:1 port. lidar_angle_min/max/stop/process_angle_min/max
// come from PlannerDebugData.msg, cached by cb_planner_debug() -- correlated
// with each other by construction (planner_node publishes them together),
// but not necessarily with *this* tick's lidar_data_ (best-effort, same
// category as legacy's own pdata->process_angle, which was itself always
// "whatever the most recent value happened to be" relative to save_thread's
// own wake-up -- no per-scan correlation existed in legacy either).
// image_number, unlike those, comes from lidar_data_ itself (LidarData.msg,
// stamped by lidar_node from ImageNumber.msg) -- the same
// per-source-message pattern as camera_front_image_number_/
// vision_front_image_number_ elsewhere in this file, so the saved lidar.png
// is tagged with the number actually associated with that scan rather than
// planner_node's separately-cached value. Sets write_wmodel (saveTick()'s
// own local) when a new lidar.png was actually written this tick.
// ---------------------------------------------------------------------------

void SaveNode::saveLidarGroup(void)
{
    if (!(conf.useLidar && lidar_data_ && (lidar_data_->point_count > 0) && !save_lidar_)) {
        return;
    }
    save_lidar_ = 1;

    int64_t image_number = lidar_data_->image_number;

    int cnt = lidar_data_->point_count;
    std::vector<lidar_data_t> data(cnt);
    for (int i = 0; i < cnt; i++) {
        data[i].sync = lidar_data_->sync[i];
        data[i].angle = lidar_data_->angle[i];
        data[i].distance = lidar_data_->distance[i];
        data[i].quality = lidar_data_->quality[i];
    }

    cv::Mat lidar_img;
    lidar_draw_output(data.data(), cnt, lidar_img, lidar_stop_, lidar_angle_min_, lidar_angle_max_,
        process_angle_, process_angle_min_, process_angle_max_, image_number);
    pushSaved(pending_saves_, "lidar", saveImage(image_number, "lidar", "png", lidar_img), image_number);

    // Consumed -- see saveCameraGroup()'s identical comment on why.
    lidar_data_.reset();
}

// ---------------------------------------------------------------------------
// saveNavMap() -- 1:1 port of legacy saveNavMap(), minus the "gps_raw" auxpt
// (GpsData.msg has no separate raw/filtered lat/lon to plot a second point
// for, see cb_gps()'s own note) -- one "gps" point plus one "navp" point.
// Legacy's own version reads its image_number via save_readData() (i.e.
// pdata->image_number -- process_thread's own tick counter, unrelated to
// wmodel's separate counter) -- planner_image_number_ (cached by
// cb_planner_debug()) is the exact equivalent here. Needs nothing from
// wmodel/lidar/planner_node at all otherwise (just the GPS breadcrumb trail
// and the static route graph, see navmap_init()) -- called directly from
// saveTick() on its own SAVE_PERIOD_NAVMAP throttle, not nested inside the
// wmodel-triggered path the way legacy happens to (for scheduling
// convenience only, not a real functional dependency -- see saveTick()'s
// own comment on this call site).
// ---------------------------------------------------------------------------

void SaveNode::saveNavMap(void)
{
    aux_point_t auxpt[2 + 1];
    int np = 0;

    if ((gps_latitude_ < ANGLE_OK) && (gps_longitude_ < ANGLE_OK)) {
        strcpy(auxpt[np].name, "gps");
        snprintf(auxpt[np].desc, sizeof(auxpt[np].desc), "gps_latitude=%0.7f, gps_longitude=%0.7f", gps_latitude_, gps_longitude_);
        strcpy(auxpt[np].style, "style4");
        auxpt[np].longitude = gps_longitude_;
        auxpt[np].latitude = gps_latitude_;
        np++;
    }

    if ((navp_latitude_ < ANGLE_OK) && (navp_longitude_ < ANGLE_OK)) {
        strcpy(auxpt[np].name, "navp");
        snprintf(auxpt[np].desc, sizeof(auxpt[np].desc), "navp_latitude=%0.7f, navp_longitude=%0.7f, navp_azimuth=%0.2f",
            navp_latitude_, navp_longitude_, navp_azimuth_);
        strcpy(auxpt[np].style, "style4");
        auxpt[np].longitude = navp_longitude_;
        auxpt[np].latitude = navp_latitude_;
        np++;
    }

    auxpt[np].style[0] = 0;

    // Same EVENT_TAG_<save_rand>_<image_number>_navmap.kml naming as
    // saveImage() itself uses for the .png -- legacy's own saveNavMap()
    // builds this filename with the identical sprintf() pattern (str="navmap",
    // ext="kml") before calling navmap_export_kml(); a fixed
    // "<EVENT_TAG>_navmap.kml" name (an earlier version of this function)
    // would silently overwrite the same file every time instead of keeping
    // one KML per save, unlike every other saveImage()-based tag.
    // process_x/y history as absolute coordinates: its newest sample is put on the newest GPS fix and the
    // older ones keep their offset from it -- same x ~ east / y ~ north metres as navigation_getXY()
    std::vector<aux_point_t> process_pt;
    if ((process_ref_ > 0) && (gps_pt_cnt_ > 0) && !process_pt_.empty()) {
        double gx, gy;
        navigation_getXY(gps_pt_[0].latitude, gps_pt_[0].longitude, gx, gy);
        for (const auto &p : process_pt_) {
            aux_point_t pt;
            pt.name[0] = 0;
            pt.desc[0] = 0;
            strcpy(pt.style, "p");
            navigation_getLL(gx + p.first - process_pt_.front().first, gy + p.second - process_pt_.front().second,
                             pt.latitude, pt.longitude);
            process_pt.push_back(pt);
        }
        aux_point_t end;
        end.style[0] = 0;
        process_pt.push_back(end);
    }
    aux_point_t *paux3 = process_pt.empty() ? NULL : process_pt.data();

    char kmlBasename[256];
    long imgnum = (planner_image_number_ < 0) ? 999999 : (long)planner_image_number_;
    sprintf(kmlBasename, EVENT_TAG "_%07u_%07u_navmap.kml", (unsigned int)save_rand, (unsigned int)imgnum);
    std::string kmlname = outputDir + kmlBasename;
    navmap_export_kml(kmlname, auxpt, gps_pt_, paux3);
    pushSaved(pending_saves_, "navmap_kml", kmlname, planner_image_number_);

    cv::Mat navmap_img;
    navmap_draw(navmap_img, auxpt, gps_pt_, paux3);
    pushSaved(pending_saves_, "navmap_png", saveImage(planner_image_number_, "navmap", "png", navmap_img), planner_image_number_);
}

// ---------------------------------------------------------------------------
// cb_wmgrid_snapshot() -- GetWMGridSnapshot.srv response handler. Async by
// necessity (this project's single-threaded rclcpp::executor convention),
// see class comment. Reconstructs save_node's own wmodel from the raw
// response, then does exactly the throttle-and-draw legacy's save_thread
// does inline right after computing write_wmodel=1.
// ---------------------------------------------------------------------------

void SaveNode::cb_wmgrid_snapshot(rclcpp::Client<istrorsx_core::srv::GetWMGridSnapshot>::SharedFuture future)
{
    auto response = future.get();

    for (int gy = 0; gy < WMGRID_HEIGHT; gy++) {
        for (int gx = 0; gx < WMGRID_WIDTH; gx++) {
            wmodel.pgrid->g[gy][gx] = response->grid_data[static_cast<size_t>(gy) * WMGRID_WIDTH + gx];
        }
    }
    wmodel.pgrid->grid_x0 = response->grid_x0;
    wmodel.pgrid->grid_y0 = response->grid_y0;

    wmodel.image_number = response->image_number;
    wmodel.last_x0 = response->last_x0;
    wmodel.last_y0 = response->last_y0;
    wmodel.last_alfa = response->last_alfa;
    wmodel.last_ref = response->last_ref;
    wmodel.last_angle = response->last_angle;
    wmodel.last_angle_min = response->last_angle_min;
    wmodel.last_angle_max = response->last_angle_max;

    if ((wmodel.image_number != last_wmimgnum_) && !save_wmodel_) {
        save_wmodel_ = 1;
        last_wmimgnum_ = wmodel.image_number;

        cv::Mat wmgrid_img;
        wmodel.drawGrid(wmgrid_img);
        pushSaved(pending_saves_, "wmgrid", saveImage(last_wmimgnum_, "wmgrid", "png", wmgrid_img), last_wmimgnum_);

        if (timeDelta(wmgrif_to_) >= SAVE_PERIOD_WMGRIF) {
            cv::Mat wmgrif_img;
            wmodel.drawGridFull(wmgrif_img);
            pushSaved(pending_saves_, "wmgrif", saveImage(last_wmimgnum_, "wmgrif", "png", wmgrif_img), last_wmimgnum_);
            wmgrif_to_ = timeBegin();
        }
    }

    publishSaveEvent();
}

// ---------------------------------------------------------------------------
// CameraArrivalStats -- min/max/avg of the inter-arrival gap on a camera
// topic, accumulated per report window. Deliberately does nothing but
// arithmetic: it runs inside the camera callbacks, which must stay cheap.
// ---------------------------------------------------------------------------

void SaveNode::CameraArrivalStats::arrived(double now, double stall_ms)
{
    if (lastt >= 0) {
        double gap = timeDelta2(lastt, now);
        if (min_ms < 0 || gap < min_ms) min_ms = gap;
        if (gap > max_ms) max_ms = gap;
        sum_ms += gap;
        count++;
        if (gap > stall_ms) stalls++;
    }
    lastt = now;
}

void SaveNode::CameraArrivalStats::reset(void)
{
    // lastt is deliberately kept: the gap across a report boundary is still a
    // real gap, and zeroing it here would hide exactly the stalls this is for.
    min_ms = -1;
    max_ms = -1;
    sum_ms = 0;
    count = 0;
    stalls = 0;
}

// ---------------------------------------------------------------------------
// reportCameraStats() -- one line per CAMERA_STATS_PERIOD per camera, saying
// either that delivery stalled or what it looked like when it did not.
//
// WARN when a gap exceeded CAMERA_STALL_PERIOD, INFO otherwise, so a drive log
// can be grepped for the bad windows without losing the healthy baseline that
// makes them interpretable. count=0 is itself reported: no frames at all in
// two seconds is the most severe case and must not read as "nothing to say".
// ---------------------------------------------------------------------------

void SaveNode::reportCameraStats(void)
{
    double now = timeBegin();
    if (camera_stats_to_ < 0) {                       // first tick after start
        camera_stats_to_ = timeAdd2(now, CAMERA_STATS_PERIOD);
        return;
    }
    if (timeDelta2(now, camera_stats_to_) > 0) {      // not due yet
        return;
    }
    camera_stats_to_ = timeAdd2(now, CAMERA_STATS_PERIOD);

    struct { const char *name; CameraArrivalStats *st; } cams[] = {
        { "front", &camera_front_stats_ },
        { "rear",  &camera_rear_stats_ },
    };

    // Fixed one decimal: the default ostream format switches to scientific
    // for anything over 1e6 precision-wise and logged "max=2e+02" for a plain
    // 200 ms gap, which is unreadable in a drive log and unusable for grep.
    std::ostringstream fmt;
    fmt << std::fixed << std::setprecision(1);

    for (auto &c : cams) {
        // Report a camera only once it has delivered at least one frame ever.
        // There is no separate conf flag for the rear camera (both callbacks
        // gate on conf.useCamera), so without this a single-camera setup --
        // which is every setup so far -- would emit a "rear stalled, count=0"
        // warning every 2 s for a camera nobody is publishing. A camera that
        // delivered and then went quiet still reports, which is the case
        // worth warning about.
        if (c.st->lastt < 0) {
            continue;
        }
        CameraArrivalStats *st = c.st;
        double avg = (st->count > 0) ? (st->sum_ms / st->count) : -1;

        fmt.str("");
        fmt << ", count=" << st->count
            << ", min=" << st->min_ms << ", max=" << st->max_ms << ", avg=" << avg;

        if (st->stalls > 0 || st->count == 0) {
            LOGM_WARN(loggerIstroSave2, "reportCameraStats",
                "msg=\"camera STALL\", camera=\"" << c.name << "\""
                << ", stalls=" << st->stalls << fmt.str()
                << ", stall_period=" << CAMERA_STALL_PERIOD);
        } else {
            LOGM_INFO(loggerIstroSave2, "reportCameraStats",
                "msg=\"camera stats\", camera=\"" << c.name << "\"" << fmt.str());
        }
        st->reset();
    }
}

// ---------------------------------------------------------------------------
// saveTick() -- save_node's own timer-driven tick, see class comment for why
// this (not any single subscribed topic's arrival) replaces legacy's
// blocking threads.getData(THDATA_STATE_PROCESSED, THDATA_STATE_SAVING).
// 1:1 port of save_thread's per-iteration body up to (not including) the
// wmodel save, which only continues asynchronously in cb_wmgrid_snapshot()
// above once requested.
// ---------------------------------------------------------------------------

void SaveNode::saveTick(void)
{
    reportCameraStats();

#ifdef ISTRO_SAVE_DISKSTATS_ENABLE
    int x1, x2, x4, x5, x6, x7, x8, x9, x10, x11, x13, x14;
    int res = readDiskStats(save_device_name_.c_str(), x1, x2, x4, x5, x6, x7, x8, x9, x10, x11, diskstats_iop_, x13, x14);
    if (res < 0) {
        LOGM_ERROR(loggerIstroSave2, "saveTick", "msg=\"readDiskStats failed!\", res=" << res);
    }
    if (diskstats_iop_ > SAVE_DISKSTAT_IOP_MAX) {
        LOGM_WARN(loggerIstroSave2, "saveTick", "msg=\"readDiskStats: high number of IO in progress!\", diskstats_iop=" << diskstats_iop_);
    }
#endif

    if ((timeDelta(save_to_) >= SAVE_PERIOD) && (diskstats_iop_ <= SAVE_DISKSTAT_IOP_MAX)) {
        save_camera_ = 0;
        save_camera2_ = 0;
        save_lidar_ = 0;
        save_wmodel_ = 0;
        save_to_ = timeBegin();
    }

    pending_saves_.clear();

    if (!conf.useNosave) {
        saveCameraGroup();
        saveCamera2Group();
        saveLidarGroup();

        if (save_lidar_ && !save_wmodel_ && client_wmgrid_snapshot_->service_is_ready()) {
            auto request = std::make_shared<istrorsx_core::srv::GetWMGridSnapshot::Request>();
            client_wmgrid_snapshot_->async_send_request(request,
                std::bind(&SaveNode::cb_wmgrid_snapshot, this, std::placeholders::_1));
        }

        // Deliberately NOT nested inside the wmodel-triggered path above,
        // unlike legacy (which only checks this alongside a fresh wmgrid
        // save, purely for scheduling convenience -- saveNavMap() itself
        // needs nothing from wmodel/lidar, just the GPS breadcrumb trail and
        // the static route graph). Nesting it there would mean navmap.kml/
        // png silently never save at all whenever lidar/wmodel data isn't
        // flowing (e.g. conf.useLidar off, or planner_node down) -- the same
        // category of unwanted coupling already fixed once for saveTick()
        // itself, see class comment.
        if (timeDelta(navmap_to_) >= SAVE_PERIOD_NAVMAP) {
            saveNavMap();
            navmap_to_ = timeBegin();
        }
    }

    if (!pending_saves_.empty()) {
        publishSaveEvent();
    }

    LOGM_DEBUG(loggerIstroSave2, "saveTick", "msg=\"tick\", planner_image_number=" << planner_image_number_
        << ", save_camera=" << save_camera_ << ", save_camera2=" << save_camera2_
        << ", save_lidar=" << save_lidar_ << ", save_wmodel=" << save_wmodel_
        << ", diskstats_iop=" << diskstats_iop_);
}

// ---------------------------------------------------------------------------

void SaveNode::publishSaveEvent(void)
{
    double tpub = timeBegin();

    if (pending_saves_.empty()) {
        return;
    }
    istrorsx_core::msg::SaveEvent msg;
    msg.files = pending_saves_;
    pub_save_event_->publish(msg);
    pending_saves_.clear();
    timeEnd("istro::save_node.publishSaveEvent", tpub);
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (istro_main_save(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroSave2, "main", "msg=\"istro_main_save() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<SaveNode>();
    rclcpp::spin(node);

    istro_close_save();
    rclcpp::shutdown();
    return 0;
}
