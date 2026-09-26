#include "istrorsx_core/planner_node.hpp"

#include <cmath>

#include "config.h"
#include "ctrlboard_defs.h"
#include "threads.h"
#include "dataset.h"
#include "logger.h"
#include "mtime.h"
#include "lidar_defs.h"
#include "istrorsx_core/istrobtx/navig.h"
#include "istrorsx_core/istrobtx/istro_main_planner.h"

LOG_DEFINE(loggerIstroPlanner2, "istroPlanner");

// ---------------------------------------------------------------------------
// process_thread's own file-scope constants (istro_rt2025.cpp), unchanged.
// ---------------------------------------------------------------------------

static const int    CALIB_TIME1_SPEED_CHK =     2500;
static const int    CALIB_TIME2_ANGLE_CHK =     3500;
static const int    CALIB_TIME3_FINISH    =     7000;

static const double CALIB_GPS_SPEED_MIN_VALUE =   0.2;
static const double CALIB_GPS_COURSE_MAX_DIFF =  15.0;
static const double CALIB_IRCV500_MIN_VALUE   =   5.0;
static const double CALIB_YAW_MAX_DIFF        =   5.0;

static const double CALIB_NAV_ANGLE_MIN       = -35.0;
static const double CALIB_NAV_ANGLE_MAX       = +35.0;

static const double CALIB_NAVP_DIST1          =  20.0;  // do not perform calibration 20 metres before navigation point (navigation point will be detected in distance 10 metres)
static const double CALIB_NAVP_DIST2          =  15.0;  // do not perform calibration 15 metres after navigation point

static const int    WRONGWAY_TIME1_STOP   =  3000;
static const int    WRONGWAY_TIME2_BACK   =  3250;
static const int    WRONGWAY_TIME3_STOP   =  3500;
static const int    WRONGWAY_TIME4_BACK   =  6000;  // 8000;
static const int    WRONGWAY_TIME5_STOP   =  9000;  // 11000;

static const double WRONGWAY_ANGLE_MIN  = -60.0;    // min/max difference between target direction and our heading (if exceeded wrongway-check will start)
static const double WRONGWAY_ANGLE_MAX  = +60.0;
static const double WRONGWAY_YAW0_DIFF  =  45.0;    // target direction must not change more than 45 degrees during wrongway-check

static const double WRONGWAY_IRCV500_MIN_VALUE   =   5.0;  // najnizsia rychlost, pri ktorej uz zvacsujeme wrongway timeout (stojime)
static const int    WRONGWAY_HOLD_GRACE   =  3000;  // ms after a loadarea wait / emergency before WrongWay may be judged again

static const int    DETECTOBST_CALIB_TIMEOUT     =  10;  /* we need to see obstacle for at least 10ms (2 frames) before telling to process_thread */
static const int    DETECTOBST_NOCALIB_TIMEOUT   = 110;  /* during calibration - 3 frames (2*50ms + 10ms) */

static const int    DETECTOBST_ANGLE_MIN =  40;
static const int    DETECTOBST_ANGLE_MAX = 140;

static const int    DETECTOBST_BCK_ANGLE_MIN =  60;      /* values for rear camera obstacle detection */
static const int    DETECTOBST_BCK_ANGLE_MAX = 120;
static const int    DETECTOBST_BCK_COUNT     =   3;

static const int    LOADAREA_TIME_BALL_WAIT   = 4000;    /* cakanie pred vylozenim lopticky */
static const int    LOADAREA_TIME_BALL_DROP   = 8000;    /* cakanie po vylozeni lopticky */

static const int    LOADAREA_STATE_NONE       =    0;
static const int    LOADAREA_STATE_BALL_WAIT  =    1;
static const int    LOADAREA_STATE_BALL_DROP  =    2;

static const double UPDATEXY_DT_MIN =    0.0;
static const double UPDATEXY_DT_MAX =  500.0;     /* max time interval to calculate (in miliseconds) */
static const double UPDATEXY_DT_STEP =   5.0;     /* integration step (in miliseconds) */

static const double UPDATEXY_IRCV500_MAX = 100.0;   /* corresponds to maximum speed of 3 metres/sec */

static const double UPDATEXY_ZERO_EPS   = 0.000001;
static const double UPDATEXY_SPEED_COEF = 0.03;   /* (in metres)  one impulse from encoders = 3 centimetres */
static const double UPDATEXY_D_COEF     = 0.335;  /* distance from the front wheel to the rear axle (in metres) */

static const double UPDATEXY_RANGLE_MAX =  90.0;   /* what relative ctrlb_velocity (difference to sa_straight) */
static const double UPDATEXY_ALFA_MAX   =  18.5;   /* corresponds to what alfa*/

static const int    SPEEDCTL_MINMAX_CNT =     15;  // pri menej ako 15 preruseniach (cca 300ms) po sebe od MINMAX neresetuj timer pre SPEEDCTL_V2/V3_TIME

static const int    SPEEDCTL_V3_DIST =       300;  // (cm) v akej vzdialenosti analyzujeme
static const int    SPEEDCTL_V3_INT_LENGTH =  40;  // aky siroky interval hladame, podobne ako DMAP_MIN_INTERVAL_LENGTH
static const int    SPEEDCTL_V3_SHIFT_MAX =  -10;  // kolko moze byt maximalny odklon od stredu, aby mohol ist robot rychlostou V3
static const int    SPEEDCTL_V3_TIME =      4000;  // po akom case sa moze zvysit rychlost na V3

static const int    SPEEDCTL_V2_DIST =       200;  // (cm) v akej vzdialenosti analyzujeme
static const int    SPEEDCTL_V2_INT_LENGTH =  30;  // aky siroky interval hladame, podobne ako DMAP_MIN_INTERVAL_LENGTH
static const int    SPEEDCTL_V2_SHIFT_MAX =    0;  // kolko moze byt maximalny odklon od stredu, aby mohol ist robot rychlostou V3
static const int    SPEEDCTL_V2_TIME =      2000;  // po akom case sa moze zvysit rychlost na V2

static const int    SPEEDCTL_SET_TIME =      400;  // ako casto nastavovat velocity aj ked sa nezmeni

static const double PROCESS_DIR_MIN =  75;   // 45-135 caused a lot of errors, trying to go directly to grass
static const double PROCESS_DIR_MAX = 105;

static const int    PROCESS_PERIOD_MIN = 20;  // how often will we process new data from sensors (goal is every 20ms to have new decision)
static const int    PROCESS_LAG_PERIOD = 3*20;  // if no data are comming from control board (euler_x), report processing lag

static const int    PROCESS_NOANGLE_TO = 1200;
static const int    PROCESS_NOANGLE_MIN =  30;
static const int    PROCESS_NOANGLE_MAX = 150;

static const int    PROCESS_CHANGE_GPS = 8;
static const int    PROCESS_CHANGE_CTRLB1 = 16;
static const int    PROCESS_CHANGE_CTRLB2 = 32;

static const double PROCESS_LIDAR_SHRINK = 0.60;  // shrink factor for lidar data - process_dmap detects obstacles below 1m, we have to draw them closer into grid [150 -> 90]

static const int    VISION_LAG_PERIOD = 4*600;  // if no data are comming from vision, force process_stop

static const int    CONESEEK_STOP_INTLEN = 13;    /* pri akej velkosti kuzela (v stupnoch) zastavit a vylozit lopticku */

// plannerTick()'s own WallTimer period -- the ROS replacement for legacy's
// "msleep(2); continue;" busy-wait poll, NOT for PROCESS_PERIOD_MIN (which is
// kept as its own gate inside plannerTick(), same as legacy). See the class
// comment in planner_node.hpp.
static const int    PLANNER_TICK_PERIOD = 5;

// DegreeMap::find()'s "how wide an interval must be to count as free" --
// same value vision_node.cpp already defines locally for itself (both port
// the same legacy literal, independently, matching legacy's own per-file
// local const -- see e.g. vision.cpp's own copy of this same constant).
static const int    DMAP_MIN_INTERVAL_LENGTH = 20;

static const double CALIB_PROCESS_ANGLE_MIN   =  85.0;
static const double CALIB_PROCESS_ANGLE_MAX   =  95.0;

// angle_fixi() (the int-typed twin of angle_fixd()) is genuinely unused in
// legacy istro_rt2025.cpp itself -- zero call sites, not just unreached
// branches -- so it's dropped here rather than kept as dead unused code.
static double angle_fixd(double angle)
{
    while (angle > 180) {
        angle -= 360;
    }
    while (angle <= -180) {
        angle += 360;
    }
    return angle;
}

// ---------------------------------------------------------------------------
// DegreeMap <-> VisionData.msg / LidarData.msg reconstruction helpers
// ---------------------------------------------------------------------------

static void visionDataToDmap(const istrorsx_core::msg::VisionData &msg, DegreeMap &dmap)
{
    for (int i = 0; i < DEGREE_MAP_COUNT; i++) {
        dmap.dmap[i] = msg.dmap[i];
        dmap.dist[i] = msg.dist[i];
        dmap.maxd[i] = msg.maxd[i];
    }
}

// ---------------------------------------------------------------------------

PlannerNode::PlannerNode() : Node("planner_node")
{
    pub_drive_ = this->create_publisher<istrorsx_core::msg::DriveCommand>("/robot/drive_command", 10);
    pub_planner_data_ = this->create_publisher<istrorsx_core::msg::PlannerData>("/robot/planner_data", 10);
    pub_planner_debug_ = this->create_publisher<istrorsx_core::msg::PlannerDebugData>("/robot/planner_debug_data", 10);
    pub_vision_control_ = this->create_publisher<istrorsx_core::msg::VisionControl>("/robot/vision_control", 10);
    pub_navigation_point_set_ = this->create_publisher<istrorsx_core::msg::NavigationPointSet>("/robot/navigation_point_set", 10);
    pub_image_number_ = this->create_publisher<istrorsx_hw::msg::ImageNumber>("/robot/image_number", 10);

    sub_vision_front_ = this->create_subscription<istrorsx_core::msg::VisionData>(
        "/robot/vision_front_data", 10, std::bind(&PlannerNode::cb_vision_front, this, std::placeholders::_1));
    sub_vision_rear_ = this->create_subscription<istrorsx_core::msg::VisionData>(
        "/robot/vision_rear_data", 10, std::bind(&PlannerNode::cb_vision_rear, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<istrorsx_hw::msg::LidarData>(
        "/robot/lidar_data", 10, std::bind(&PlannerNode::cb_lidar, this, std::placeholders::_1));
    // These four are what triggers plannerTick()'s processing, so they must not
    // queue behind it. Own MutuallyExclusive group -> own thread. Safe without
    // extra locking because each one's whole body writes to the DataSet through
    // threads.getData(THDATA_STATE_SHARED_LOCK) and touches no member of this
    // class. Do not move anything else here without checking that again.
    cbg_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_opt;
    sensor_opt.callback_group = cbg_sensors_;

    sub_servo_ = this->create_subscription<istrorsx_hw::msg::ServoData>(
        "/robot/servo_data", 10, std::bind(&PlannerNode::cb_servo, this, std::placeholders::_1), sensor_opt);
    sub_imu_ = this->create_subscription<istrorsx_hw::msg::ImuData>(
        "/robot/imu_data", 10, std::bind(&PlannerNode::cb_imu, this, std::placeholders::_1), sensor_opt);
    sub_gps_ = this->create_subscription<istrorsx_hw::msg::GpsData>(
        "/robot/gps_data", 10, std::bind(&PlannerNode::cb_gps, this, std::placeholders::_1), sensor_opt);
    sub_navigation_ = this->create_subscription<istrorsx_core::msg::NavigationData>(
        "/robot/navigation_data", 10, std::bind(&PlannerNode::cb_navigation, this, std::placeholders::_1), sensor_opt);
    sub_config_ = this->create_subscription<istrorsx_core::msg::ConfigUpdate>(
        "/robot/config_update", 10, std::bind(&PlannerNode::cb_config, this, std::placeholders::_1));

    srv_wmgrid_snapshot_ = this->create_service<istrorsx_core::srv::GetWMGridSnapshot>(
        "/robot/get_wmgrid_snapshot", std::bind(&PlannerNode::handleGetWMGridSnapshot, this, std::placeholders::_1, std::placeholders::_2));

    // Legacy's local "DataSet data;" fields that DataSet::DataSet() sets to
    // ANGLE_NONE / PROCESS_STATE_NONE (the rest are member initializers, see
    // planner_node.hpp).
    process_angle_ = (int)ANGLE_NONE;
    process_state_ = PROCESS_STATE_NONE;
    process_yaw_ = ANGLE_NONE;
    qrscan_latitude_  = ANGLE_NONE;
    qrscan_longitude_ = ANGLE_NONE;

    // Legacy process_thread's own pre-while(1) local variable initialization.
    process_to_   = timeBegin();
    minmax_lastt_ = timeBegin();
    noangle_to_   = timeBegin();

    calibInit(calib_);
    calibInit(calib2_);   // ISTRO_CALIB2
    wrongwayInit(wrongway_);
    detectobstInit(detectobst_);
    loadareaInit(loadarea_);
    speedctlInit(speedctl_);

    // 5ms -- replaces legacy's "msleep(2); continue;" busy-wait poll, not its
    // PROCESS_PERIOD_MIN gate (that one lives inside plannerTick(), same as
    // legacy). See planner_node.hpp's class comment and
    // doc/ai/01_architecture.md's planner_node section.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(PLANNER_TICK_PERIOD),
        std::bind(&PlannerNode::plannerTick, this));

    // Legacy: LOGM_INFO(loggerIstro, "process_thread", "msg=\"start\"").
    // useCamera/useLidar are logged too -- they gate cb_vision_*/cb_lidar and
    // the vision-lag watchdog below, so their value explains a lot of what
    // does or doesn't show up in the rest of the log.
    LOGM_INFO(loggerIstroPlanner2, "PlannerNode", "msg=\"start\", tick_period=" << PLANNER_TICK_PERIOD
        << ", process_period_min=" << PROCESS_PERIOD_MIN
        << ", useCamera=" << conf.useCamera << ", useLidar=" << conf.useLidar);
}

// ---------------------------------------------------------------------------
// Fast-path callbacks -- fold new perception data into wmodel immediately,
// matching legacy's non-blocking checkData(THDATA_STATE_VISION_PROCESSED/
// VISION2_PROCESSED/LIDAR_CAPTURED) polls at the top of process_thread's
// while(1) body.
// ---------------------------------------------------------------------------

void PlannerNode::cb_vision_front(const istrorsx_core::msg::VisionData::SharedPtr msg)
{
    // Legacy: "if (conf.useCamera) { pdata = checkData(VISION_PROCESSED...) }".
    // The "else { vision_lastt = timeBegin(); }" half lives in plannerTick(),
    // since it has to keep running even when no message ever arrives.
    if (!conf.useCamera) {
        return;
    }

    if (vision_writeData(*msg) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_vision_front", "msg=\"vision_writeData() failed!\"");
        return;
    }

    DegreeMap dmap;
    int angle_min, angle_max;
    double qrscan_latitude, qrscan_longitude;
    if (vision_readData(dmap, angle_min, angle_max, qrscan_latitude, qrscan_longitude) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_vision_front", "msg=\"vision_readData() failed!\"");
        return;
    }

    double t2 = timeBegin();

#ifdef ISTRO_VISION_ORANGECONE
    // do not write cone positions to obstacle grid, just calculate coneseek values
    LOGM_DEBUG(loggerIstroPlanner2, "cb_vision_front", "msg=\"process_vision_coneseek\"");
    dmap.findbnd(coneseek_angle_min_, coneseek_angle_max_, coneseek_intlen_);
    coneseek_stop_ = (coneseek_intlen_ >= CONESEEK_STOP_INTLEN);
    LOGM_INFO(loggerIstroPlanner2, "cb_vision_front", "dmap_vision: image_number=" << image_number_
        << ", coneseek_angle_min=" << coneseek_angle_min_ << ", coneseek_angle_max=" << coneseek_angle_max_
        << ", coneseek_intlen=" << coneseek_intlen_ << ", coneseek_stop=" << coneseek_stop_ << ", process_dir=" << process_dir_);
#else
    LOGM_DEBUG(loggerIstroPlanner2, "cb_vision_front", "msg=\"process_vision\"");
    // Legacy also gated this on "(pdata->process_ref == data.process_ref)" --
    // the incoming pdata's process_ref/x/y/yaw came from capture_readData()'s
    // pose-at-capture-time stamping. Per 01_architecture.md decision #1
    // (simplified pose), planner_node uses its own *current* pose here
    // instead of a pose carried alongside the image, so that comparison is
    // now trivially true and collapses away -- flagged there as a
    // documented deviation from legacy, not an oversight.
    if (process_yaw_ < ANGLE_OK) {
        wmodel.updateGrid(dmap, process_x_, process_y_, process_yaw_,
            process_ref_, process_angle_, process_angle_min_, process_angle_max_,
            WMGRID_VISION_MBIT, WMGRID_VISION_VBIT, image_number_);
    }
#endif

    // copy "qrscan output" from the vision message to the local dataset
    qrscan_latitude_  = qrscan_latitude;
    qrscan_longitude_ = qrscan_longitude;

    process_change_ += 1;
    vision_lastt_ = timeBegin();

#ifdef ISTRO_VISION_ORANGECONE
    timeEnd("istro::planner_node.process_vision_coneseek", t2);
#else
    timeEnd("istro::planner_node.process_vision", t2);
#endif
}

void PlannerNode::cb_vision_rear(const istrorsx_core::msg::VisionData::SharedPtr msg)
{
    // Legacy: "if (conf.useCamera) { pdata = checkData(VISION2_PROCESSED...) }".
    if (!conf.useCamera) {
        return;
    }

    if (vision2_writeData(*msg) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_vision_rear", "msg=\"vision2_writeData() failed!\"");
        return;
    }

    DegreeMap dmap;
    int angle_min, angle_max;
    if (vision2_readData(dmap, angle_min, angle_max) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_vision_rear", "msg=\"vision2_readData() failed!\"");
        return;
    }

    LOGM_DEBUG(loggerIstroPlanner2, "cb_vision_rear", "msg=\"process_vision2\"");

    // ISTRO_CAMERA_REAR (assumed active, matching vision_node's own
    // precedent of not #ifdef-guarding this at the ROS layer). Legacy's own
    // "!pdata->camera2_img_pred.empty()" validity gate has no equivalent
    // here (that field is VisionDebugData.msg-only, not sent to
    // planner_node) -- the message simply arriving is itself the "valid
    // data" signal.
    int detectobst_bck = (angle_min < 0) || (angle_max < 0) ||
                         (angle_min > DETECTOBST_BCK_ANGLE_MIN) ||
                         (angle_max < DETECTOBST_BCK_ANGLE_MAX);
    if (detectobst_bck > 0) {
        vision_stop_bck_cnt_++;
    } else {
        vision_stop_bck_cnt_ = 0;
    }
    vision_stop_bck_ = (vision_stop_bck_cnt_ >= DETECTOBST_BCK_COUNT);
    LOGM_DEBUG(loggerIstroPlanner2, "cb_vision_rear", "msg=\"vision2_detectobst_bck\"" << ", vision_stop_bck=" << vision_stop_bck_
        << ", vision2_angle_min=" << angle_min << ", vision2_angle_max=" << angle_max
        << ", detectobst_bck=" << detectobst_bck << ", vision_stop_bck_cnt=" << vision_stop_bck_cnt_);
}

void PlannerNode::cb_lidar(const istrorsx_hw::msg::LidarData::SharedPtr msg)
{
    // Legacy: "if (conf.useLidar) { pdata = checkData(LIDAR_CAPTURED...) }".
    if (!conf.useLidar) {
        return;
    }

    if (lidar_writeData(*msg) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_lidar", "msg=\"lidar_writeData() failed!\"");
        return;
    }

    DegreeMap dmap;
    int angle_min, angle_max, stop;
    if (lidar_readData(dmap, angle_min, angle_max, stop) < 0) {
        LOGM_ERROR(loggerIstroPlanner2, "cb_lidar", "msg=\"lidar_readData() failed!\"");
        return;
    }

    double t2 = timeBegin();
    LOGM_DEBUG(loggerIstroPlanner2, "cb_lidar", "msg=\"process_lidar\"");

    // copy lidar variable to the local dataset
    lidar_stop_ = stop;
    lidar_angle_min_ = angle_min;
    lidar_angle_max_ = angle_max;

    LOGM_INFO(loggerIstroPlanner2, "cb_lidar", "dmap_lidar: lidar_angle_min=" << angle_min << ", lidar_angle_max=" << angle_max
        << ", lidar_stop=" << stop << ", process_dir=" << process_dir_);

    // See cb_vision_front()'s comment -- the "same reference" check
    // collapses away for the same reason (decision #1).
    if (process_yaw_ < ANGLE_OK) {
        dmap.shrink(PROCESS_LIDAR_SHRINK);
        wmodel.updateGrid(dmap, process_x_, process_y_, process_yaw_,
            process_ref_, process_angle_, process_angle_min_, process_angle_max_,
            WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number_);
    }

    process_change_ += 2;
    timeEnd("istro::planner_node.process_lidar", t2);
}

// ---------------------------------------------------------------------------
// Cache-path callbacks -- write into the local DataSet round trip, read back
// at the start of plannerTick() via process_readData().
// ---------------------------------------------------------------------------

void PlannerNode::cb_servo(const istrorsx_hw::msg::ServoData::SharedPtr msg)
{
    // Names the executor's second thread the first time it lands here. The name
    // is thread-local and sticks, so every later line from that thread -- tick
    // included -- carries it. "[]" in a log means it ran before this fired.
    LOG_THREAD_NAME_ONCE("exec2");
    servo_writeData(*msg);
}

void PlannerNode::cb_imu(const istrorsx_hw::msg::ImuData::SharedPtr msg)
{
    LOG_THREAD_NAME_ONCE("exec2");
    imu_writeData(*msg);
}

void PlannerNode::cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg)
{
    LOG_THREAD_NAME_ONCE("exec2");
    gps_writeData(*msg);
}

void PlannerNode::cb_navigation(const istrorsx_core::msg::NavigationData::SharedPtr msg)
{
    LOG_THREAD_NAME_ONCE("exec2");
    navigation_writeData(*msg);
}

// Applies a runtime conf change made in another process. speedctl reads
// conf.velocityFwd/velocityFwd2/velocityFwd3 directly (see speedctlProcess),
// so writing them here is all that is needed -- exactly what the legacy
// monolith got for free from having a single global conf.
//
// Logged at INFO deliberately: this changes how fast the robot drives, and
// the 2026-09-10 park session was hard to diagnose precisely because nothing
// recorded that the two processes disagreed about it.
void PlannerNode::cb_config(const istrorsx_core::msg::ConfigUpdate::SharedPtr msg)
{
    if (!msg->update_velocity) {
        return;
    }

    conf.velocityFwd  = msg->velocity_fwd;
    conf.velocityFwd2 = msg->velocity_fwd2;
    conf.velocityFwd3 = msg->velocity_fwd3;
    conf.velocityBack = msg->velocity_back;

    LOGM_INFO(loggerIstroPlanner2, "cb_config", "msg=\"velocity config updated\", velocityFwd=" << conf.velocityFwd
        << ", velocityFwd2=" << conf.velocityFwd2 << ", velocityFwd3=" << conf.velocityFwd3
        << ", velocityBack=" << conf.velocityBack);
}

// ---------------------------------------------------------------------------
// GetWMGridSnapshot.srv -- pull-based, see doc/ai/01_architecture.md
// planner_node decision #4. Never calls WMGrid::drawGrid()/drawGridFull()
// itself.
// ---------------------------------------------------------------------------

void PlannerNode::handleGetWMGridSnapshot(
    const std::shared_ptr<istrorsx_core::srv::GetWMGridSnapshot::Request> /*request*/,
    std::shared_ptr<istrorsx_core::srv::GetWMGridSnapshot::Response> response)
{
    response->grid_width  = WMGRID_WIDTH;
    response->grid_height = WMGRID_HEIGHT;
    response->grid_cv_type = 0;   // CV_8UC1
    response->grid_step   = WMGRID_WIDTH;

    response->grid_data.resize(static_cast<size_t>(WMGRID_WIDTH) * WMGRID_HEIGHT);
    for (int gy = 0; gy < WMGRID_HEIGHT; gy++) {
        for (int gx = 0; gx < WMGRID_WIDTH; gx++) {
            response->grid_data[static_cast<size_t>(gy) * WMGRID_WIDTH + gx] = wmodel.pgrid->get(gy, gx);
        }
    }

    response->grid_x0 = wmodel.pgrid->grid_x0;
    response->grid_y0 = wmodel.pgrid->grid_y0;

    response->image_number = wmodel.image_number;
    response->last_x0 = wmodel.last_x0;
    response->last_y0 = wmodel.last_y0;
    response->last_alfa = wmodel.last_alfa;
    response->last_ref = wmodel.last_ref;
    response->last_angle = wmodel.last_angle;
    response->last_angle_min = wmodel.last_angle_min;
    response->last_angle_max = wmodel.last_angle_max;

    LOGM_DEBUG(loggerIstroPlanner2, "handleGetWMGridSnapshot", "msg=\"snapshot served\", image_number=" << wmodel.image_number);
}

// ---------------------------------------------------------------------------
// Cross-process DataSet round trip.
// ---------------------------------------------------------------------------

int PlannerNode::servo_writeData(const istrorsx_hw::msg::ServoData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ctrlb_time1    = timeBegin();
    pdata->ctrlb_state    = msg.state;
    pdata->ctrlb_ircv     = msg.ircv;
    pdata->ctrlb_ircv500  = msg.ircv500;
    pdata->ctrlb_angle    = msg.angle;
    pdata->ctrlb_velocity = msg.velocity;
    pdata->ctrlb_loadd    = msg.loadd;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroPlanner2, "servo_writeData", "state=" << msg.state << ", velocity=" << msg.velocity << ", angle=" << msg.angle);

    return 0;
}

int PlannerNode::imu_writeData(const istrorsx_hw::msg::ImuData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ctrlb_time2      = timeBegin();
    pdata->ctrlb_euler_x    = msg.euler_x;
    pdata->ctrlb_euler_y    = msg.euler_y;
    pdata->ctrlb_euler_z    = msg.euler_z;
    pdata->ctrlb_calib_gyro  = msg.calib_gyro;
    pdata->ctrlb_calib_accel = msg.calib_accel;
    pdata->ctrlb_calib_mag   = msg.calib_mag;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroPlanner2, "imu_writeData", "euler_x=" << ioff(msg.euler_x, 2));

    return 0;
}

int PlannerNode::gps_writeData(const istrorsx_hw::msg::GpsData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_time   = timeBegin();
    pdata->gps_speed  = msg.speed;
    pdata->gps_course = msg.course;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroPlanner2, "gps_writeData", "speed=" << ioff(msg.speed, 3) << ", course=" << ioff(msg.course, 2));

    return 0;
}

int PlannerNode::navigation_writeData(const istrorsx_core::msg::NavigationData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_navp_idx     = msg.navp_idx;
    pdata->gps_navp_dist    = msg.navp_dist;
    pdata->gps_navp_azimuth = msg.navp_azimuth;
    pdata->gps_navp_maxdist = msg.navp_maxdist;
    pdata->gps_navp_loadarea = msg.navp_loadarea;

    pdata->gps_ref = msg.ref;
    pdata->gps_x   = msg.x;
    pdata->gps_y   = msg.y;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroPlanner2, "navigation_writeData", "navp_idx=" << msg.navp_idx
        << ", navp_dist=" << ioff(msg.navp_dist, 3) << ", navp_azimuth=" << ioff(msg.navp_azimuth, 2)
        << ", ref=" << msg.ref);

    return 0;
}

int PlannerNode::process_readData(double &gps_time, double &gps_speed, double &gps_course,
        double &gps_navp_dist, double &gps_navp_azimuth, double &gps_navp_maxdist, int &gps_navp_loadarea, int &gps_navp_idx,
        double &ctrlb_time1, double &ctrlb_ircv500, int &ctrlb_angle, int &ctrlb_velocity, int &ctrlb_loadd,
        double &ctrlb_time2, double &ctrlb_euler_x,
        int &gps_ref, double &gps_x, double &gps_y, int &ctrlb_state)
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    gps_time = pdata->gps_time;
    gps_speed = pdata->gps_speed;
    gps_course = pdata->gps_course;
    gps_navp_dist = pdata->gps_navp_dist;
    gps_navp_azimuth = pdata->gps_navp_azimuth;
    gps_navp_maxdist = pdata->gps_navp_maxdist;
    gps_navp_loadarea = pdata->gps_navp_loadarea;
    gps_navp_idx = pdata->gps_navp_idx;

    gps_ref = pdata->gps_ref;
    gps_x = pdata->gps_x;
    gps_y = pdata->gps_y;

    ctrlb_time1 = pdata->ctrlb_time1;
    ctrlb_ircv500 = pdata->ctrlb_ircv500;
    ctrlb_angle = pdata->ctrlb_angle;
    ctrlb_velocity = pdata->ctrlb_velocity;
    ctrlb_loadd = pdata->ctrlb_loadd;
    ctrlb_time2 = pdata->ctrlb_time2;
    ctrlb_euler_x = pdata->ctrlb_euler_x;
    ctrlb_state = pdata->ctrlb_state;

    // No log here on purpose -- legacy's own process_readData() has its
    // LOGM_DEBUG commented out ("logging is performed only if timestamps will
    // change") and does it in the caller instead. plannerTick() does the same.
    threads.setData(pdata, THDATA_STATE_SHARED, 1);
#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::process_readData", t);
#endif

    return 0;
}

int PlannerNode::vision_writeData(const istrorsx_core::msg::VisionData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    visionDataToDmap(msg, pdata->vision_dmap);
    pdata->vision_angle_min = msg.angle_min;
    pdata->vision_angle_max = msg.angle_max;
    pdata->qrscan_latitude  = msg.qrscan_latitude;
    pdata->qrscan_longitude = msg.qrscan_longitude;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int PlannerNode::vision_readData(DegreeMap &dmap, int &angle_min, int &angle_max, double &qrscan_latitude, double &qrscan_longitude)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    dmap.copy(pdata->vision_dmap);
    angle_min = pdata->vision_angle_min;
    angle_max = pdata->vision_angle_max;
    qrscan_latitude = pdata->qrscan_latitude;
    qrscan_longitude = pdata->qrscan_longitude;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int PlannerNode::vision2_writeData(const istrorsx_core::msg::VisionData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    visionDataToDmap(msg, pdata->vision2_dmap);
    pdata->vision2_angle_min = msg.angle_min;
    pdata->vision2_angle_max = msg.angle_max;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int PlannerNode::vision2_readData(DegreeMap &dmap, int &angle_min, int &angle_max)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    dmap.copy(pdata->vision2_dmap);
    angle_min = pdata->vision2_angle_min;
    angle_max = pdata->vision2_angle_max;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int PlannerNode::lidar_writeData(const istrorsx_hw::msg::LidarData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    int cnt = msg.point_count;
    if (cnt > LIDAR_DATA_NUM) {
        cnt = LIDAR_DATA_NUM;
    }
    pdata->lidar_data_cnt = cnt;
    for (int i = 0; i < cnt; i++) {
        pdata->lidar_data[i].sync = msg.sync[i];
        pdata->lidar_data[i].angle = msg.angle[i];
        pdata->lidar_data[i].distance = msg.distance[i];
        pdata->lidar_data[i].quality = msg.quality[i];
    }

    // Compute lidar_dmap right here, same as legacy calling Lidar::process()
    // synchronously inside process_thread (not a separate thread) -- see
    // doc/ai/01_architecture.md's vision_node section.
    lidar_process(pdata->lidar_data, pdata->lidar_data_cnt, pdata->lidar_dmap, pdata->lidar_stop);
    pdata->lidar_dmap.find(DMAP_MIN_INTERVAL_LENGTH, process_dir_, pdata->lidar_angle_min, pdata->lidar_angle_max);

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int PlannerNode::lidar_readData(DegreeMap &dmap, int &angle_min, int &angle_max, int &stop)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    dmap.copy(pdata->lidar_dmap);
    angle_min = pdata->lidar_angle_min;
    angle_max = pdata->lidar_angle_max;
    stop = pdata->lidar_stop;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

// ---------------------------------------------------------------------------
// update_angle_interval() / update_speed_interval()
// ---------------------------------------------------------------------------

void PlannerNode::updateAngleInterval(double &angle_min, double &angle_max, double angle)
{
    if ((angle_min >= ANGLE_OK) || (angle_max >= ANGLE_OK)) {
        angle_min = angle_max = angle;
        return;
    }

    if ((angle_min <= angle) && (angle <= angle_max)) {
        return;
    }

    if (angle < angle_min) {
        double angle2 = angle + 360;

        if (angle2 <= angle_max) {
            return;
        }

        double dt1 = angle_min - angle;
        double dt2 = angle2 - angle_max;
        if (dt1 < dt2) {
            angle_min = angle;
        } else {
            angle_max = angle2;
        }
        return;
    }

    if (angle > angle_max) {
        double angle2 = angle - 360;

        if (angle_min <= angle2) {
            return;
        }

        double dt1 = angle_min - angle2;
        double dt2 = angle - angle_max;
        if (dt1 < dt2) {
            angle_min = angle2;
        } else {
            angle_max = angle;
        }
        return;
    }
}

void PlannerNode::updateSpeedInterval(double &speed_min, double &speed_max, double speed)
{
    if ((speed_min < 0) || (speed_max < 0)) {
        speed_min = speed_max = speed;
        return;
    }

    if (speed_max < speed) {
        speed_max = speed;
    } else
    if (speed_min > speed) {
        speed_min = speed;
    }
}

// ---------------------------------------------------------------------------
// calib_process() (+ calib_t)
// ---------------------------------------------------------------------------

void PlannerNode::calibInit(calib_t &calib)
{
    calib.azimuth = conf.calibGpsAzimuth;
    calib.yaw     = conf.calibImuYaw;

    calib.ok = (calib.azimuth < (int)ANGLE_OK) && (calib.yaw < (int)ANGLE_OK);
    calib.first = !calib.ok;
    calib.navp_ok = -1;
    calib.nav_angle_ok = -1;

    calib.time = -1;
    calib.gps_course_min = ANGLE_NONE;
    calib.gps_course_max = ANGLE_NONE;
    calib.yaw_min = ANGLE_NONE;
    calib.yaw_max = ANGLE_NONE;
    calib.gps_speed_min = -1;
    calib.gps_speed_max = -1;
}

int PlannerNode::calibReset(calib_t &calib, int idx, const std::string &log_reason)
{
    if ((!calib.ok) && (calib.time >= 0)) {
        calib.time = -1;

        LOGM_INFO(loggerIstroPlanner2, "calibReset" + std::to_string(idx), "msg=\"calibration interrupted!\", reason=\"" << log_reason
                << "\", calib.ok=" << calib.ok);
    }

    return 0;
}

int PlannerNode::calibProcess(calib_t &calib, int idx, double nav_angle, int process_angle_min, int process_angle_max,
        double gps_course, double yaw, double gps_speed, double navp_dist, double navp_maxdist, double ctrlb_ircv500)
{
    (void)process_angle_min;
    (void)process_angle_max;

    // check calibration timeout
    if (calib.ok) {
        if (calib.time < 0) {
            calib.time = timeBegin();
        }
        // idx==1 uses CALIB_TIMEOUT1 (12000000ms), idx==2 uses CALIB_TIMEOUT2
        // (120000ms) -- kept as a literal ternary, matching legacy exactly.
        if (timeDelta(calib.time) > (idx == 1 ? 12000000 : 120000)) {
            calib.ok = 0;
            calib.time = -1;
            calib.navp_ok = -1;
            calib.nav_angle_ok = -1;
            LOGM_INFO(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration needed (timeout)!\", calib.ok=" << calib.ok);
        }
    }

    // no calibration needed
    if (calib.ok) {
        return 0;
    }

    // check "nav_angle"
    int nav_angle_ok = -1;
    if (nav_angle < ANGLE_OK) {
        nav_angle_ok = (nav_angle >= CALIB_NAV_ANGLE_MIN) && (nav_angle <= CALIB_NAV_ANGLE_MAX);
        if ((!nav_angle_ok) && (calib.first)) {
            nav_angle_ok = 1;
        }
    }
    if (nav_angle_ok != calib.nav_angle_ok) {
        LOGM_DEBUG(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration - nav_angle_ok changed...\", calib.ok=" << calib.ok
                    << ", calib.nav_angle_ok=" << nav_angle_ok << ", calib.nav_angle_ok_old=" << calib.nav_angle_ok
                    << ", calib.first=" << calib.first << ", nav_angle=" << ioff(nav_angle, 2));
        calib.nav_angle_ok = nav_angle_ok;
    }

    if (calib.nav_angle_ok == 0) {
        calib.time = -1;
        return 0;
    }

    // is distance to next navigation point ok?
    if ((navp_dist >= 0) && (navp_maxdist >= 0)) {
        int navp_ok = (navp_dist > CALIB_NAVP_DIST1) && ((navp_maxdist - navp_dist) > CALIB_NAVP_DIST2);
        if ((!navp_ok) && (calib.first)) {
            navp_ok = 1;
        }
        if (navp_ok != calib.navp_ok) {
            if ((!navp_ok) && (!calib.ok) && (calib.time >= 0)) {
                calibReset(calib, idx, "close to navigation point");
            } else {
                LOGM_DEBUG(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration - navp_ok changed...\", calib.ok=" << calib.ok
                    << ", calib.navp_ok=" << navp_ok << ", calib.navp_ok_old=" << calib.navp_ok
                    << ", calib.first=" << calib.first
                    << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_maxdist=" << ioff(navp_maxdist, 3));
            }
        }
        calib.navp_ok = navp_ok;
    } else {
        calib.navp_ok = -1;
    }

    if (calib.navp_ok == 0) {
        calibReset(calib, idx, "navp_ok");
        return 0;
    }

    // start time calculation if not initialized
    if (calib.time < 0) {
        calib.time = timeBegin();
        calib.gps_course_min = calib.gps_course_max = ANGLE_NONE;
        calib.yaw_min = calib.yaw_max = ANGLE_NONE;
        calib.gps_speed_min = calib.gps_speed_max = -1;
        LOGM_INFO(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration start!\", calib.ok=" << calib.ok
            << ", calib.navp_ok=" << calib.navp_ok << ", calib.nav_angle_ok=" << calib.nav_angle_ok);
    }

    LOGM_TRACE(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration in progress...\", calib.ok=" << calib.ok
        << ", calib.navp_ok=" << calib.navp_ok << ", calib.nav_angle_ok=" << calib.nav_angle_ok << ", calib.first=" << calib.first
        << ", gps_course=" << ioff(gps_course, 2) << ", yaw=" << ioff(yaw, 2) << ", gps_speed=" << ioff(gps_speed, 3)
        << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_maxdist=" << ioff(navp_maxdist, 3) << ", ircv500=" << ioff(ctrlb_ircv500, 2));

    // gps speed check
    if ((calib.time >= 0) && (timeDelta(calib.time) >= CALIB_TIME1_SPEED_CHK)) {
        updateSpeedInterval(calib.gps_speed_min, calib.gps_speed_max, gps_speed);
        if (gps_speed < CALIB_GPS_SPEED_MIN_VALUE) {
            calibReset(calib, idx, "low gps speed");
        }
        if (ctrlb_ircv500 < CALIB_IRCV500_MIN_VALUE) {
            calibReset(calib, idx, "low ircv500");
        }
    }

    // gps course and yaw check
    if ((calib.time >= 0) && (timeDelta(calib.time) >= CALIB_TIME2_ANGLE_CHK)) {
        updateAngleInterval(calib.gps_course_min, calib.gps_course_max, gps_course);
        updateAngleInterval(calib.yaw_min, calib.yaw_max, yaw);
        if ((calib.gps_course_max - calib.gps_course_min) > CALIB_GPS_COURSE_MAX_DIFF) {
            calibReset(calib, idx, "gps_course difference");
        }
        if ((calib.yaw_max - calib.yaw_min) > CALIB_YAW_MAX_DIFF) {
            calibReset(calib, idx, "yaw difference");
        }
    }

    // finished?
    if ((calib.time >= 0) && (timeDelta(calib.time) >= CALIB_TIME3_FINISH)) {
        calib.ok = 1;
        calib.first = 0;
        calib.azimuth = gps_course;
        calib.yaw = yaw;
        calib.time = -1;
        LOGM_INFO(loggerIstroPlanner2, "calibProcess" + std::to_string(idx), "msg=\"calibration finished!\", calib.ok=" << calib.ok
            << ", calib.yaw=" << ioff(calib.yaw, 2) << ", calib.azimuth=" << ioff(calib.azimuth, 2)
            << ", calib.delta=" << ioff(calib.azimuth - calib.yaw, 2));
    }

    return 1;
}

// ---------------------------------------------------------------------------
// wrongway_process() (+ wrongway_t)
// ---------------------------------------------------------------------------

void PlannerNode::wrongwayInit(wrongway_t &wrongway)
{
    wrongway.ok = 1;
    wrongway.time = -1;
    wrongway.yaw0 = ANGLE_NONE;
    wrongway.yaw0_min = ANGLE_NONE;
    wrongway.yaw0_max = ANGLE_NONE;
    wrongway.ircv_low_time = -1;
    wrongway.ircv_low_dt = 0;
}

int PlannerNode::wrongwayInprogress(wrongway_t &wrongway)
{
    return ((!wrongway.ok) && (wrongway.time >= 0));
}

int PlannerNode::wrongwayReset(wrongway_t &wrongway, const std::string &log_reason)
{
    if ((wrongway.ok) && (wrongway.time >= 0)) {
        wrongway.time = -1;

        LOGM_INFO(loggerIstroPlanner2, "wrongwayReset", "msg=\"wrongway-check interrupted!\", reason=\"" << log_reason
                << "\", wrongway.ok=" << wrongway.ok);
    }
    return 0;
}

int PlannerNode::wrongwayCheck(wrongway_t &wrongway, double yaw, double angle, double navp_dist, double navp_maxdist, double ctrlb_ircv500)
// (angle > -180) && (angle <= 180)
{
    if (!wrongway.ok) {
        return 0;
    }

    if ((angle >= WRONGWAY_ANGLE_MIN) && (angle <= WRONGWAY_ANGLE_MAX)) {
        if (wrongway.time >= 0) {
            LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway-check interrupted (angle is OK)!\", wrongway.ok=" << wrongway.ok
                << ", angle=" << ioff(angle, 2));
        }
        wrongway.time = -1;
        return 0;
    }

    // is distance to next navigation point ok?
    if ((navp_dist >= 0) && (navp_maxdist >= 0)) {
        // fixme: podmienku CALIB_NAVP_DIST2 je potrebne ignorovat, lebo ak sa robot po dosiahnuti navigacneho bodu zacne hned vzdalovat,
        //   tak (navp_maxdist - navp_dist) je vzdy nula a "wrongway-check" sa nikdy nespusti
        int navp_ok = (navp_dist > CALIB_NAVP_DIST1);
        if (!navp_ok) {
            if (wrongway.time >= 0) {
                LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway-check interrupted (navp_ok is FALSE)!\", wrongway.ok=" << wrongway.ok
                    << ", navp_ok=" << navp_ok);
            }
            wrongway.time = -1;
            return 0;
        }
    }

    if (wrongway.time < 0) {
        wrongway.time = timeBegin();
        wrongway.yaw0_min = wrongway.yaw0_max = ANGLE_NONE;
        wrongway.ircv_low_time = -1;
        wrongway.ircv_low_dt = 0;
        LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway-check start!\", wrongway.ok=" << wrongway.ok);
    }

    // nie je rychlost prilis nizka?
    if (ctrlb_ircv500 < WRONGWAY_IRCV500_MIN_VALUE) {
        if (wrongway.ircv_low_time >= 0) {
            wrongway.ircv_low_dt += timeDelta(wrongway.ircv_low_time);
        }
        wrongway.ircv_low_time = timeBegin();
        LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"ircv speed LOW!\", wrongway.ircv_low_dt=" << ioff(wrongway.ircv_low_dt, 3));
    } else {
        if (wrongway.ircv_low_time >= 0) {
            wrongway.ircv_low_time = -1;
            LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"ircv speed OK!\", wrongway.ircv_low_dt=" << ioff(wrongway.ircv_low_dt, 3));
        }
    }

    double yaw0 = yaw + angle;
    while (yaw0 >= 360) {
        yaw0 -= 360;
    }
    while (yaw0 < 0) {
        yaw0 += 360;
    }
    updateAngleInterval(wrongway.yaw0_min, wrongway.yaw0_max, yaw0);

    if ((wrongway.yaw0_max - wrongway.yaw0_min) > WRONGWAY_YAW0_DIFF) {
        wrongway.time = -1;
        LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway-check interrupted (yaw0_diff exceeded)!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0_min=" << ioff(wrongway.yaw0_min, 2) << ", wrongway.yaw0_max=" << ioff(wrongway.yaw0_max, 2));
        return 0;
    }

    if (timeDelta(wrongway.time) >= 20000 + wrongway.ircv_low_dt) {
        wrongway.ok = 0;
        wrongway.time = -1;
        wrongway.yaw0 = yaw0;

        LOGM_INFO(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway detected!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << wrongway.yaw0 << ", yaw=" << ioff(yaw, 2) << ", angle=" << angle);
        return 1;
    }

    LOGM_TRACE(loggerIstroPlanner2, "wrongwayCheck", "msg=\"wrongway-check in progress...\", wrongway.ok=" << wrongway.ok
        << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2) << ", yaw0=" << ioff(yaw0, 2)
        << ", ircv500=" << ioff(ctrlb_ircv500, 2) << ", wrongway.ircv_low_dt=" << ioff(wrongway.ircv_low_dt, 3)
        << ", wrongway.yaw0_min=" << ioff(wrongway.yaw0_min, 2) << ", wrongway.yaw0_max=" << ioff(wrongway.yaw0_max, 2));

    return 0;
}

int PlannerNode::wrongwayForce(wrongway_t &wrongway, double yaw, double angle)
// (angle > -180) && (angle <= 180)
{
    if (!wrongway.ok) {
        return 0;
    }

    double yaw0 = yaw + angle;
    while (yaw0 >= 360) {
        yaw0 -= 360;
    }
    while (yaw0 < 0) {
        yaw0 += 360;
    }

    wrongway.ok = 0;
    wrongway.time = -1;
    wrongway.yaw0 = yaw0;

    LOGM_INFO(loggerIstroPlanner2, "wrongwayForce", "msg=\"wrongway forced!\", wrongway.ok=" << wrongway.ok
        << ", wrongway.yaw0=" << wrongway.yaw0 << ", yaw=" << ioff(yaw, 2) << ", angle=" << angle);

    return 1;
}

int PlannerNode::wrongwayProcess(wrongway_t &wrongway, double yaw, int &process_angle, int &process_velocity, int &process_state, int process_stop)
{
    if (wrongway.ok) {
        process_rear_enabled_ = false;
        publishVisionControl(qrscan_enabled_, process_rear_enabled_);
        return 0;
    }

    if (wrongway.time < 0) {
        wrongway.time = timeBegin();
        LOGM_INFO(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway start!\", wrongway.ok=" << wrongway.ok);
        process_rear_enabled_ = true;
        publishVisionControl(qrscan_enabled_, process_rear_enabled_);
    }
    if (timeDelta(wrongway.time) < WRONGWAY_TIME1_STOP) {
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway in progress (STOP1)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME2_BACK) {
        double angle = wrongway.yaw0 - yaw;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        angle = -angle;
        process_angle = 90 - trunc(angle);
        if (process_angle < NAVIGATION_ANGLE_MIN) {
            process_angle = NAVIGATION_ANGLE_MIN;
        }
        if (process_angle > NAVIGATION_ANGLE_MAX) {
            process_angle = NAVIGATION_ANGLE_MAX;
        }
        process_velocity = VEL_ZERO + conf.velocityBack;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway in progress (BACK2)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME3_STOP) {
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway in progress (STOP3)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME4_BACK) {
        double angle = wrongway.yaw0 - yaw;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        angle = -angle;
        process_angle = 90 - trunc(angle);
        if (process_angle < NAVIGATION_ANGLE_MIN) {
            process_angle = NAVIGATION_ANGLE_MIN;
        }
        if (process_angle > NAVIGATION_ANGLE_MAX) {
            process_angle = NAVIGATION_ANGLE_MAX;
        }
        process_velocity = VEL_ZERO + conf.velocityBack;
        process_state = PROCESS_STATE_WRONGWAY;
        LOGM_TRACE(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway in progress (BACK4)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2) << ", angle=" << ioff(angle, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
    } else
    if (timeDelta(wrongway.time) < WRONGWAY_TIME5_STOP) {
        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_WRONGWAY;
        process_rear_enabled_ = false;
        publishVisionControl(qrscan_enabled_, process_rear_enabled_);
        LOGM_TRACE(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway in progress (STOP5)...!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
    } else {
        wrongway.ok = 1;
        wrongway.time = -1;
        process_rear_enabled_ = false;
        publishVisionControl(qrscan_enabled_, process_rear_enabled_);
        LOGM_INFO(loggerIstroPlanner2, "wrongwayProcess", "msg=\"wrongway finished!\", wrongway.ok=" << wrongway.ok
            << ", wrongway.yaw0=" << ioff(wrongway.yaw0, 2) << ", yaw=" << ioff(yaw, 2)
            << ", process_angle=" << process_angle << ", process_velocity=" << process_velocity
            << ", process_stop=" << process_stop);
        return 0;
    }

    return 1;
}

void PlannerNode::wrongwayHold(const char *reason)
{
    if (ww_hold_reason_ != reason) {
        if (ww_hold_reason_.empty()) {
            ww_hold_start_ = timeBegin();
        }
        LOGM_INFO(loggerIstroPlanner2, "wrongwayHold", "msg=\"wrongway hold start!\", reason=\"" << reason
            << "\", previous_reason=\"" << ww_hold_reason_ << "\", wrongway.ok=" << wrongway_.ok);
        ww_hold_reason_ = reason;
    }
    if (!wrongway_.ok) {
        LOGM_INFO(loggerIstroPlanner2, "wrongwayHold", "msg=\"wrongway cancelled!\", reason=\"" << reason
            << "\", wrongway_dt=" << ((wrongway_.time >= 0) ? (int)timeDelta(wrongway_.time) : -1));
        wrongwayInit(wrongway_);
        process_rear_enabled_ = false;
        publishVisionControl(qrscan_enabled_, process_rear_enabled_);
    }
    wrongwayReset(wrongway_, reason);
    noangle_to_ = timeBegin();
    ww_hold_end_ = timeBegin();
}

void PlannerNode::wrongwayHoldEnd()
{
    if (!ww_hold_reason_.empty()) {
        LOGM_INFO(loggerIstroPlanner2, "wrongwayHold", "msg=\"wrongway hold end!\", reason=\"" << ww_hold_reason_
            << "\", hold_dt=" << (int)timeDelta(ww_hold_start_) << ", grace=" << WRONGWAY_HOLD_GRACE);
        ww_hold_reason_.clear();
    }
}

// ---------------------------------------------------------------------------
// detectobst_process() (+ detectobst_t)
// ---------------------------------------------------------------------------

void PlannerNode::detectobstInit(detectobst_t &detectobst)
{
    detectobst.noobstacle = -1;
    detectobst.time = -1;
}

int PlannerNode::detectobstProcess(detectobst_t &detectobst, int calib_ok, int process_angle_min, int process_angle_max)
{
    int noobst = (process_angle_min <= DETECTOBST_ANGLE_MIN) && (process_angle_max >= DETECTOBST_ANGLE_MAX) &&
                 (process_angle_min >= 0) && (process_angle_max >= 0);

    if (noobst) {
        if ((detectobst.noobstacle > 0) && (detectobst.time >= 0)) {
            LOGM_TRACE(loggerIstroPlanner2, "detectobstProcess", "msg=\"obstacle ignored!\", noobstacle=" << detectobst.noobstacle
                << ", dt=" << ioff(timeDelta(detectobst.time), 2) << ", process_angle_min=" << process_angle_min << ", process_angle_max=" << process_angle_max);
        }
        detectobst.noobstacle = 1;
        detectobst.time = -1;
    } else {
        if (detectobst.time < 0) {
            detectobst.noobstacle = 1;
            detectobst.time = timeBegin();
        }
        if (calib_ok) {
            if (timeDelta(detectobst.time) >= DETECTOBST_CALIB_TIMEOUT) {
                detectobst.noobstacle = 0;
            }
        } else {
            if (timeDelta(detectobst.time) >= DETECTOBST_NOCALIB_TIMEOUT) {
                detectobst.noobstacle = 0;
            }
        }
    }

    if (detectobst.noobstacle <= 0) {
        LOGM_TRACE(loggerIstroPlanner2, "detectobstProcess", "msg=\"obstacle detected!\", noobstacle=" << detectobst.noobstacle
            << ", dt=" << ioff(timeDelta(detectobst.time), 2) << ", process_angle_min=" << process_angle_min << ", process_angle_max=" << process_angle_max);
    }

    return detectobst.noobstacle > 0;
}

// ---------------------------------------------------------------------------
// loadarea_process() (+ loadarea_t)
// ---------------------------------------------------------------------------

void PlannerNode::loadareaInit(loadarea_t &loadarea)
{
    loadarea.state = LOADAREA_STATE_NONE;
    loadarea.time = -1;
    loadarea.cnt = 0;
    loadarea.navp_idx = -1;
}

int PlannerNode::loadareaProcess(loadarea_t &loadarea, int gps_navp_loadarea, int ctrlb_loadd, int &process_angle, int &process_velocity, int &process_state,
        int gps_navp_idx, double gps_navp_azimuth, double qrscan_latitude, double qrscan_longitude)
{
    if ((gps_navp_loadarea == NAVIGATION_AREA_LOADING) || (gps_navp_loadarea == NAVIGATION_AREA_UNLOADING)) {

        if ((gps_navp_loadarea == NAVIGATION_AREA_LOADING) && (ctrlb_loadd != 1)) {
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_LOADING;
            LOGM_TRACE(loggerIstroPlanner2, "loadareaProcess", "msg=\"loading area - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        } else
        if ((gps_navp_loadarea == NAVIGATION_AREA_UNLOADING) && (ctrlb_loadd != 0)) {
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_UNLOADING;
            LOGM_TRACE(loggerIstroPlanner2, "loadareaProcess", "msg=\"unloading area - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        }
    }

    // sme v novom navigacnom bode, ktory ma priznak BALLDROP?
    if ((gps_navp_loadarea == NAVIGATION_AREA_BALLDROP) && (loadarea.navp_idx != gps_navp_idx)) {
        std::string state_str = "ball drop";
        if (loadarea.state == LOADAREA_STATE_BALL_WAIT) {
            state_str = "ball drop - waiting!";
            if (timeDelta(loadarea.time) >= LOADAREA_TIME_BALL_WAIT) {
                loadarea.state = LOADAREA_STATE_BALL_DROP;
                loadarea.cnt++;
            }
        } else
        if (loadarea.state == LOADAREA_STATE_BALL_DROP) {
            state_str = "ball drop - dropping!";
            if (timeDelta(loadarea.time) >= LOADAREA_TIME_BALL_DROP) {
                state_str = "ball drop - finished!";
                loadarea.state = LOADAREA_STATE_NONE;
                loadarea.navp_idx = gps_navp_idx;
            }
        } else {
            state_str = "ball drop - init!";
            loadarea.state = LOADAREA_STATE_BALL_WAIT;
            loadarea.time = timeBegin();
        }

        process_angle = 90;
        process_velocity = VEL_ZERO;
        process_state = PROCESS_STATE_BALLDROP + loadarea.cnt;
        LOGM_TRACE(loggerIstroPlanner2, "loadareaProcess", "msg=\"" << state_str << "\", gps_navp_loadarea=" << gps_navp_loadarea
            << ", process_state=" << process_state << ", gps_navp_idx=" << gps_navp_idx
            << ", loadarea_state=" << loadarea.state << ", loadarea_dt=" << ioff(timeDelta(loadarea.time), 2)
            << ", loadarea_cnt=" << loadarea.cnt << ", loadarea_navp_idx=" << loadarea.navp_idx);
        return 1;
    }

    // wait for scanning qr-coordinates?
    if (gps_navp_idx >= 0) {
        if (gps_navp_azimuth >= ANGLE_OK) {
            qrscan_enabled_ = true;
            publishVisionControl(qrscan_enabled_, process_rear_enabled_);
            if ((qrscan_latitude < ANGLE_OK) && (qrscan_longitude < ANGLE_OK)) {
                publishNavigationPointSet(gps_navp_idx, qrscan_latitude, qrscan_longitude);
            }
            process_angle = 90;
            process_velocity = VEL_ZERO;
            process_state = PROCESS_STATE_QRSCAN_COORD;
            LOGM_TRACE(loggerIstroPlanner2, "loadareaProcess", "msg=\"qrscan coordinates - waiting!\", gps_navp_loadarea=" << gps_navp_loadarea
                << ", ctrlb_loadd=" << ctrlb_loadd << ", process_state=" << process_state
                << ", gps_navp_idx=" << gps_navp_idx << ", gps_navp_azimuth=" << ioff(gps_navp_azimuth, 2));
            return 1;
        } else {
            qrscan_enabled_ = false;
            publishVisionControl(qrscan_enabled_, process_rear_enabled_);
            return 0;
        }
    }

    return 0;
}

// ---------------------------------------------------------------------------
// update_xy() -- odometry integration
// ---------------------------------------------------------------------------

int PlannerNode::updateXY(double &process_x, double &process_y, double &process_yaw, double &process_time,
        double ctrlb_ircv500, int ctrlb_angle, int ctrlb_velocity)
{
    double t = timeBegin();
    double dt = timeDelta2(process_time, t);

    process_time = t;
    if ((process_yaw >= ANGLE_OK) || (ctrlb_angle < 0) || (ctrlb_ircv500 < 0) || (ctrlb_ircv500 > UPDATEXY_IRCV500_MAX)) {
        LOGM_ERROR(loggerIstroPlanner2, "updateXY", "msg=\"error: ctrlb values not initialised!\"");
        return -1;
    }
    if ((dt < UPDATEXY_DT_MIN) || (dt > UPDATEXY_DT_MAX)) {
        LOGM_ERROR(loggerIstroPlanner2, "updateXY", "msg=\"error: max time exceeded!\", dt=" << ioff(dt, 2));
        return -2;
    }

    double process_x_old = process_x;
    double process_y_old = process_y;
    double process_yaw_old = process_yaw;
    double dt_old = dt;

    double speed = ctrlb_ircv500 * UPDATEXY_SPEED_COEF;
    if (ctrlb_velocity < VEL_ZERO) {
        speed = -speed;
    }

    double da = (ctrlb_angle - SA_STRAIGHT);
    if (da > UPDATEXY_RANGLE_MAX) {
        da = UPDATEXY_RANGLE_MAX;
    }
    if (da < -UPDATEXY_RANGLE_MAX) {
        da = -UPDATEXY_RANGLE_MAX;
    }
    double alfa = UPDATEXY_ALFA_MAX * da / UPDATEXY_RANGLE_MAX;

    while (dt > UPDATEXY_ZERO_EPS) {
        double dt0 = dt;
        if (dt0 > UPDATEXY_DT_STEP) {
            dt0 = UPDATEXY_DT_STEP;
        }
        process_x += (dt0 / 1000.0) * speed * cos(alfa * M_PI / 180.0) * sin(process_yaw * M_PI / 180.0);
        process_y += (dt0 / 1000.0) * speed * cos(alfa * M_PI / 180.0) * cos(process_yaw * M_PI / 180.0);
        process_yaw += ((dt0 / 1000.0) * speed * sin(alfa * M_PI / 180.0) / UPDATEXY_D_COEF) * (180.0 / M_PI);
        dt -= dt0;
    }

    LOGM_DEBUG(loggerIstroPlanner2, "updateXY",
        "process_x=" << ioff(process_x, 3) << ", process_y=" << ioff(process_y, 3)
        << ", process_yaw=" << ioff(process_yaw, 3) << ", ctrlb_ircv500=" << ioff(ctrlb_ircv500, 2)
        << ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity
        << ", speed=" << ioff(speed, 2) << ", alfa=" << ioff(alfa, 2) << ", dt=" << ioff(dt_old, 2)
        << ", process_x_old=" << ioff(process_x_old, 3) << ", process_y_old=" << ioff(process_y_old, 3)
        << ", process_yaw_old=" << ioff(process_yaw_old, 3));

    return 0;
}

// ---------------------------------------------------------------------------
// speedctl_process() (+ speedctl_t)
// ---------------------------------------------------------------------------

void PlannerNode::speedctlInit(speedctl_t &speedctl)
{
    speedctl.minmax_cnt = 0;
    speedctl.time = -1;
    speedctl.time2 = -1;
    speedctl.time3 = -1;
    speedctl.timet = -1;
    speedctl.velocity_last = -1;
    speedctl.state_last = -1;
}

void PlannerNode::speedctlTimeReset(speedctl_t &speedctl)
{
    speedctl.minmax_cnt = 0;
    speedctl.time = -1;
    speedctl.timet = -1;
}

int PlannerNode::speedctlGetshift(int dir, int angle1, int angle2)
{
    if ((angle1 >= 0) && (angle2 >= 0)) {
        int sh1 = angle1 - dir;
        if (sh1 < 0) sh1 = -sh1;
        int sh2 = angle2 - dir;
        if (sh2 < 0) sh2 = -sh2;
        if ((angle1 <= dir) && (angle2 >= dir)) {
            return (sh1 < sh2) ? -sh1 : -sh2;
        }
        return (sh1 < sh2) ? sh1 : sh2;
    }
    return 180;
}

int PlannerNode::speedctlProcess(speedctl_t &speedctl, const DegreeMap &dmap, int process_dir, int &process_velocity, int &process_state, int &process_stop)
{
    if ((process_state != PROCESS_STATE_NAV_ANGLE) &&
        (process_state != PROCESS_STATE_MIN_MAX) &&
        (process_state != PROCESS_STATE_CALIBRATION)) {
        speedctlTimeReset(speedctl);
        speedctl.state_last = process_state;
        return 0;
    }
    if ((process_stop) || (process_velocity >= 0)) {
        speedctlTimeReset(speedctl);
        speedctl.state_last = process_state;
        return 0;
    }

    int velocity = -1;
    int velocity_old = process_velocity;

    if (process_state == PROCESS_STATE_CALIBRATION) {
        velocity = VEL_ZERO + conf.velocityFwd;

        speedctlTimeReset(speedctl);
        LOGM_DEBUG(loggerIstroPlanner2, "speedctlProcess", "msg=\"CALIBRATION\""
            << ", velocity=" << velocity << ", process_state=" << process_state
            << ", process_dir=" << process_dir
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    } else
    if (process_state == PROCESS_STATE_MIN_MAX) {
        velocity = VEL_ZERO + conf.velocityFwd;

        if (speedctl.time >= 0) {
            speedctl.minmax_cnt++;
            if (speedctl.minmax_cnt > SPEEDCTL_MINMAX_CNT) {
                speedctlTimeReset(speedctl);
            }
        }
        LOGM_DEBUG(loggerIstroPlanner2, "speedctlProcess", "msg=\"MIN_MAX\""
            << ", velocity=" << velocity << ", process_state=" << process_state
            << ", process_dir=" << process_dir << ", minmax_cnt=" << speedctl.minmax_cnt
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    } else
    if (process_state == PROCESS_STATE_NAV_ANGLE) {
        speedctl.minmax_cnt = 0;
        if (speedctl.time < 0) {
            speedctl.time = speedctl.time2 = speedctl.time3 = timeBegin();
        }

        int angle_min1, angle_max1, sh1;
        int angle_min2, angle_max2, sh2;
        int angle_min3, angle_max3, sh3;

        DegreeMap dmap2;
        dmap2.init();
        dmap2.copy(dmap);

        dmap2.find(DMAP_MIN_INTERVAL_LENGTH, process_dir, angle_min1, angle_max1);
        sh1 = speedctlGetshift(process_dir, angle_min1, angle_max1);
        dmap2.remap(SPEEDCTL_V2_DIST);
        dmap2.find(SPEEDCTL_V2_INT_LENGTH, process_dir, angle_min2, angle_max2);
        sh2 = speedctlGetshift(process_dir, angle_min2, angle_max2);
        dmap2.remap(SPEEDCTL_V3_DIST);
        dmap2.find(SPEEDCTL_V3_INT_LENGTH, process_dir, angle_min3, angle_max3);
        sh3 = speedctlGetshift(process_dir, angle_min3, angle_max3);

        int sf = 0;
        if (sh2 > SPEEDCTL_V2_SHIFT_MAX) {
            sf += 1;
            speedctl.time2 = timeBegin();
        }
        if (sh3 > SPEEDCTL_V3_SHIFT_MAX) {
            sf += 2;
            speedctl.time3 = timeBegin();
        }

        velocity = VEL_ZERO + conf.velocityFwd;
        if (timeDelta(speedctl.time3) >= SPEEDCTL_V3_TIME) {
            sf += 8;
            velocity = VEL_ZERO + conf.velocityFwd3;
        } else
        if (timeDelta(speedctl.time2) >= SPEEDCTL_V2_TIME) {
            sf += 4;
            velocity = VEL_ZERO + conf.velocityFwd2;
        }

        LOGM_DEBUG(loggerIstroPlanner2, "speedctlProcess", "msg=\"NAV_ANGLE\""
            << ", velocity=" << velocity << ", process_state=" << process_state
            << ", process_dir=" << process_dir
            << ", sf=" << sf << ", sh1=" << sh1 << ", sh2=" << sh2 << ", sh3=" << sh3
            << ", angle_min1=" << angle_min1 << ", angle_max1=" << angle_max1
            << ", angle_min2=" << angle_min2 << ", angle_max2=" << angle_max2
            << ", angle_min3=" << angle_min3 << ", angle_max3=" << angle_max3
            << ", velocity_old=" << velocity_old << ", process_stop=" << process_stop);
    }

    if (velocity > 0) {
        int timeto = 0;
        if (speedctl.timet < 0) {
            speedctl.timet = timeBegin();
            timeto = 1;
        } else {
            timeto = timeDelta(speedctl.timet) >= SPEEDCTL_SET_TIME;
        }
        if ((speedctl.state_last != process_state) || (speedctl.velocity_last != velocity) || (timeto)) {
            process_velocity = velocity;
            LOGM_DEBUG(loggerIstroPlanner2, "speedctlProcess", "msg=\"set process_velocity!\""
                << ", process_state=" << process_state << ", state_last=" << speedctl.state_last
                << ", velocity=" << velocity << ", velocity_last=" << speedctl.velocity_last
                << ", timeto=" << timeto);
            speedctl.timet = timeBegin();
        }
        speedctl.velocity_last = velocity;
    }

    speedctl.state_last = process_state;
    return 1;
}

// ---------------------------------------------------------------------------
// plannerTick() -- 5ms tick, port of ONE process_thread while() iteration:
// the "wait" poll, the PROCESS_PERIOD_MIN-gated process_readData(), and (only
// when do_process) the whole decision chain through process_writeData().
// ---------------------------------------------------------------------------

void PlannerNode::plannerTick()
{
    int do_process = 0;

    // Legacy's own while() condition: "while (!thread_testcancel() &&
    // (conf.useCamera || conf.useLidar))" -- with neither sensor configured
    // process_thread's body never ran at all, so neither does this tick.
    if (!conf.useCamera && !conf.useLidar) {
        return;
    }

    if (tw_ < 0) {
        tw_ = timeBegin();
    }

    // Legacy's "} else { vision_lastt = timeBegin(); }" half of the
    // "if (conf.useCamera)" vision block -- to avoid VISION_LAG_PERIOD firing
    // when the camera is not used at all. Has to live here rather than in
    // cb_vision_front(), since with useCamera=0 that callback never runs.
    if (!conf.useCamera) {
        vision_lastt_ = timeBegin();
    }

    // Initialised to DataSet::DataSet()'s own defaults -- legacy read these
    // into its persistent local "DataSet data;", so they were never
    // uninitialised even before the first successful process_readData().
    double gps_time = -1, gps_speed = -1, gps_course = ANGLE_NONE;
    double gps_navp_dist = -1, gps_navp_azimuth = ANGLE_NONE, gps_navp_maxdist = -1;
    int gps_navp_loadarea = NAVIGATION_AREA_NONE, gps_navp_idx = -1;
    double ctrlb_time1 = -1, ctrlb_ircv500 = -1;
    int ctrlb_angle = -1, ctrlb_velocity = -1, ctrlb_loadd = -1;
    double ctrlb_time2 = -1, ctrlb_euler_x = ANGLE_NONE;
    int gps_ref = 0;
    double gps_x = 0.0, gps_y = 0.0;
    int ctrlb_state = -1;

    // every 20ms we should get new data from control_board -> this event will trigger the processing
    if (timeDelta(process_to_) >= PROCESS_PERIOD_MIN) {
        /* read data from GPS and control board (shared dataset) */
        if (process_readData(gps_time, gps_speed, gps_course,
                gps_navp_dist, gps_navp_azimuth, gps_navp_maxdist, gps_navp_loadarea, gps_navp_idx,
                ctrlb_time1, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd,
                ctrlb_time2, ctrlb_euler_x,
                gps_ref, gps_x, gps_y, ctrlb_state) < 0) {
            LOGM_ERROR(loggerIstroPlanner2, "plannerTick", "msg=\"process_readData() failed!\"");
            return;
        }

        // dont process data if ctrlb_euler_x is not known!!
        if (ctrlb_euler_x < ANGLE_OK) {
        if ((last_gps_time_ != gps_time) || (last_ctrlb_time1_ != ctrlb_time1) || (last_ctrlb_time2_ != ctrlb_time2)) {
            do_process = 1;
            process_change_ += 4;
            if (last_gps_time_ != gps_time) {
                process_change_ += PROCESS_CHANGE_GPS;
            }
            if (last_ctrlb_time1_ != ctrlb_time1) {
                process_change_ += PROCESS_CHANGE_CTRLB1;
            }
            if (last_ctrlb_time2_ != ctrlb_time2) {
                process_change_ += PROCESS_CHANGE_CTRLB2;
            }
            // Legacy logs this under the "process_readData" module name even
            // though the call site is in process_thread -- kept as-is, so
            // grepping legacy and ported logs side by side still lines up.
            // "lastp_dist" is the one field legacy logged here that has no
            // equivalent in the port: gps_lastp_dist is gps_thread-internal
            // state, not carried by GpsData.msg/NavigationData.msg.
            LOGM_DEBUG(loggerIstroPlanner2, "process_readData",
                "process_change=" << process_change_ << ", gps_speed=" << ioff(gps_speed, 3) << ", gps_course=" << ioff(gps_course, 2) <<
                ", ctrlb_ircv500=" << ioff(ctrlb_ircv500, 2) <<
                ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity << ", ctrlb_loadd=" << ctrlb_loadd <<
                ", euler_x=" << ioff(ctrlb_euler_x, 2) <<
                ", navp_dist=" << ioff(gps_navp_dist, 3) << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2) << ", navp_maxdist=" << ioff(gps_navp_maxdist, 3) <<
                ", navp_loadarea=" << gps_navp_loadarea << ", navp_idx=" << gps_navp_idx <<
                ", gps_ref=" << gps_ref << ", gps_x=" << ioff(gps_x, 2) << ", gps_y=" << ioff(gps_y, 2));
            last_gps_time_ = gps_time;
            last_ctrlb_time1_ = ctrlb_time1;
            last_ctrlb_time2_ = ctrlb_time2;
        }
        }
    }

    if (!do_process) {
        // no new data -> wait for the next tick (legacy: "msleep(2); continue;")
        return;
    }
    timeEnd("istro::planner_node.wait", tw_);
    tw_ = -1;
    double t = timeBegin();

    /* detect processing lag */
    int process_dt = timeDelta(process_to_);
    if (process_dt > PROCESS_LAG_PERIOD) {
        LOGM_DEBUG(loggerIstroPlanner2, "plannerTick", "msg=\"processing_lag detected!\", image_number=" << image_number_ << ", process_dt=" << process_dt);
        /* processing_lag should not cause wrongway */
        if (process_dt > (int)(PROCESS_NOANGLE_TO / 3)) {
            noangle_to_ = timeBegin();
        }
    }
    process_to_ = timeBegin();

    // check if we are continuously receiving images from vision
    vision_stop_ = (vision_lastt_ < 0) || (timeDelta(vision_lastt_) >= VISION_LAG_PERIOD);

    process_backward_ = wrongwayInprogress(wrongway_);

    int process_stop_old = process_stop_;
    if (!process_backward_) {
        process_stop_ = (vision_stop_ > 0) || (lidar_stop_ > 0);
    } else {
        process_stop_ = (vision_stop_bck_ > 0);
    }
    if (process_stop_ != process_stop_old) {
        LOGM_INFO(loggerIstroPlanner2, "plannerTick", "msg=\"process_stop changed!\", process_stop=" << process_stop_
            << ", vision_stop=" << vision_stop_ << ", lidar_stop=" << lidar_stop_ << ", vision_stop_bck=" << vision_stop_bck_
            << ", process_backward=" << process_backward_ << ", process_stop_old=" << process_stop_old);
    }

    /* processing start */
    LOGM_DEBUG(loggerIstroPlanner2, "plannerTick", "msg=\"do_process\", process_dt=" << process_dt <<
        ", process_change1=" << (process_change_ & 7) << ", process_change2=" << ((process_change_ >> 3) & 7));

    // update "process_ref"
    int process_ref_old = process_ref_;
    if (!process_ref_ && (gps_ref > 0) && (calib_.azimuth < ANGLE_OK) && (calib_.yaw < ANGLE_OK)) {
        process_ref_ = 1;
        // clear grid - because the coordinate system will be changed
        wmodel.init();
        LOGM_INFO(loggerIstroPlanner2, "plannerTick", "msg=\"process_ref set...\", process_ref=" << process_ref_
              << ", calib.azimuth=" << ioff(calib_.azimuth, 2) << ", calib.yaw=" << ioff(calib_.yaw, 2));
    }

    // calculate "yaw"
    int yaw_src;
    double yaws = ANGLE_NONE;  // raw yaw value from sensor - 0 is not north
    double yawc = ANGLE_NONE;  // calibrated yaw value (azimuth) - 0 means north
    yaw_src = 0;
    if (process_ref_) {
        if (process_ref_old) {
            yaws = angle_fixd(process_yaw_ - calib_.azimuth + calib_.yaw);
            yawc = process_yaw_;
        } else {
            yaws = process_yaw_;
            yawc = angle_fixd(process_yaw_ - calib_.yaw + calib_.azimuth);
        }
    } else {
        yaws = process_yaw_;
        yawc = ANGLE_NONE;
    }

    // use "ctrlb_euler_x" - overwrite calculated yaw with new euler_x value from compass (only if new value was received)
    if ((process_change_ & PROCESS_CHANGE_CTRLB2) > 0) {
        yaw_src = 1;
        if (process_ref_) {
            yaws = ctrlb_euler_x;
            yawc = angle_fixd(ctrlb_euler_x - calib_.yaw + calib_.azimuth);
        } else {
            yaws = ctrlb_euler_x;
            yawc = ANGLE_NONE;
        }
    }

    // NOT_YET_MIGRATED: legacy's ahrs_yaw/gps_lastp_azimuth yaw fallback
    // block was already dead/commented-out code in istro_rt2025.cpp itself
    // (see doc/ai/01_architecture.md planner_node analysis) -- not ported.

    if (process_ref_) {
        process_yaw_ = yawc;
    } else {
        process_yaw_ = yaws;
    }

    // calculate "nav_angle"
    double yawn = ANGLE_NONE;
    double nav_angle = ANGLE_NONE;
    if ((conf.navigationImuYaw < (int)ANGLE_OK) && (yaws < ANGLE_OK)) {
        nav_angle = angle_fixd(conf.navigationImuYaw - yaws);
        yawn = yaws;
        LOGM_TRACE(loggerIstroPlanner2, "plannerTick", "calc_nav_angle(\"YAW\"): image_number=" << image_number_ << ", nav_angle=" << ioff(nav_angle, 2)
            << ", nav_yaw=" << conf.navigationImuYaw << ", process_yaw=" << ioff(process_yaw_, 2)
            << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yaw_src=" << yaw_src);
    } else
    if ((gps_navp_azimuth < ANGLE_OK) && (yawc < ANGLE_OK)) {
        nav_angle = angle_fixd(gps_navp_azimuth - yawc);
        yawn = yawc;
        LOGM_TRACE(loggerIstroPlanner2, "plannerTick", "calc_nav_angle(\"AZIMUTH_YAW\"): image_number=" << image_number_ << ", nav_angle=" << ioff(nav_angle, 2)
            << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2) << ", process_yaw=" << ioff(process_yaw_, 2)
            << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yaw_src=" << yaw_src
            << ", calib.yaw=" << ioff(calib_.yaw, 2) << ", calib.azimuth=" << ioff(calib_.azimuth, 2));
    }

    // wrongway_check (not while held or within WRONGWAY_HOLD_GRACE after, see wrongwayHold())
    int ww_grace = (ww_hold_end_ >= 0) && (timeDelta(ww_hold_end_) < WRONGWAY_HOLD_GRACE);
    if ((yawn < ANGLE_OK) && (nav_angle < ANGLE_OK) && !ww_grace) {
        wrongwayCheck(wrongway_, yawn, nav_angle, gps_navp_dist, gps_navp_maxdist, ctrlb_ircv500);
    }

    // calculate "obstacle distance map" -> "process_angle_min/max"
    process_dir_ = 90;
    if (nav_angle < ANGLE_OK) {
        process_dir_ = 90 - trunc(nav_angle);
        if (process_dir_ < PROCESS_DIR_MIN) {
            process_dir_ = PROCESS_DIR_MIN;
        }
        if (process_dir_ > PROCESS_DIR_MAX) {
            process_dir_ = PROCESS_DIR_MAX;
        }
    }
    DegreeMap process_dmap;
    int eval_res = wmodel.evalGrid(process_x_, process_y_, process_yaw_, process_dmap);
    process_dmap.find(DMAP_MIN_INTERVAL_LENGTH, process_dir_, process_angle_min_, process_angle_max_);
    LOGM_TRACE(loggerIstroPlanner2, "plannerTick", "msg=\"process_dmap\", image_number=" << image_number_
        << ", process_ref=" << process_ref_ << ", process_x=" << ioff(process_x_, 2) << ", process_y=" << ioff(process_y_, 2)
        << ", process_yaw=" << ioff(process_yaw_, 2) << ", nav_angle=" << ioff(nav_angle, 2)
        << ", process_angle_min=" << process_angle_min_ << ", process_angle_max=" << process_angle_max_);

    if (eval_res < 0) {
        LOGM_TRACE(loggerIstroPlanner2, "plannerTick", "msg=\"fix process_angle_min/max\", image_number=" << image_number_
            << ", eval_res=" << eval_res
            << ", process_angle_min=" << process_angle_min_ << ", process_angle_max=" << process_angle_max_
            << ", last_process_angle_min=" << last_process_angle_min_ << ", last_process_angle_max=" << last_process_angle_max_);
        process_angle_min_ = last_process_angle_min_;
        process_angle_max_ = last_process_angle_max_;
    } else {
        last_process_angle_min_ = process_angle_min_;
        last_process_angle_max_ = process_angle_max_;
    }

    // calculate "process_angle"
    process_angle_ = (int)ANGLE_NONE;
    process_velocity_ = -1;
    process_state_ = PROCESS_STATE_NONE;

    // LOAD/UNLOAD AREA - wait for un/loading + scanning geo-coordinates
    loadareaProcess(loadarea_, gps_navp_loadarea, ctrlb_loadd, process_angle_, process_velocity_, process_state_,
        gps_navp_idx, gps_navp_azimuth, qrscan_latitude_, qrscan_longitude_);

    // DETECT OBSTACLES
    detectobstProcess(detectobst_, calib_.ok, process_angle_min_, process_angle_max_);

    // calculate calibration
    int calib_res = 0;
    if (detectobst_.noobstacle <= 0) {
        calib_res = calibReset(calib_, 1, "obstacle");
        calibReset(calib2_, 2, "obstacle");   // ISTRO_CALIB2
    } else
    if (process_angle_ < (int)ANGLE_OK) {
        calib_res = calibReset(calib_, 1, "loadarea");
        calibReset(calib2_, 2, "loadarea");   // ISTRO_CALIB2
    } else {
        calib_res = calibProcess(calib_, 1, nav_angle, process_angle_min_, process_angle_max_,
                        gps_course, yaws, gps_speed, gps_navp_dist, gps_navp_maxdist, ctrlb_ircv500);
        calibProcess(calib2_, 2, nav_angle, process_angle_min_, process_angle_max_,   // ISTRO_CALIB2 -- logging/monitoring only, result unused
                        gps_course, yaws, gps_speed, gps_navp_dist, gps_navp_maxdist, ctrlb_ircv500);
    }

    // WRONG_WAY -- held while a loadarea wait set process_angle_ or the emergency button is pressed
    if (process_angle_ < (int)ANGLE_OK) {
        wrongwayHold("loadarea");
    } else
    if (ctrlb_state == CTRLB_STATE_EBTN) {
        wrongwayHold("emergency");
    } else {
        wrongwayHoldEnd();
        if (wrongwayProcess(wrongway_, yawn, process_angle_, process_velocity_, process_state_, process_stop_) > 0) {
            LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"WRONG_WAY\"): image_number=" << image_number_ << ", process_angle=" << process_angle_
                << ", process_velocity=" << process_velocity_ << ", process_stop=" << process_stop_);
        }
    }
    ww_grace = (ww_hold_end_ >= 0) && (timeDelta(ww_hold_end_) < WRONGWAY_HOLD_GRACE);
    if (ww_grace) {
        noangle_to_ = timeBegin();
    }

    // NO OBSTACLE - perform calibration or go to cone / navigation angle
    if ((process_angle_ >= (int)ANGLE_OK) && (detectobst_.noobstacle > 0)) {
        if (calib_res > 0) {
            process_angle_ = 90;
            process_state_ = PROCESS_STATE_CALIBRATION;
            LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"CALIBRATION\"): image_number=" << image_number_ << ", process_angle=" << process_angle_);
        } else
        if ((coneseek_angle_min_ >= 0) && (coneseek_angle_max_ >= 0)) {
            process_angle_ = (coneseek_angle_min_ + coneseek_angle_max_) / 2;
            process_state_ = PROCESS_STATE_CONE_SEEK;
            LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"CONE_SEEK\"): image_number=" << image_number_
                << ", coneseek_angle_min=" << coneseek_angle_min_ << ", coneseek_angle_max=" << coneseek_angle_max_
                << ", coneseek_intlen=" << coneseek_intlen_
                << ", process_angle=" << process_angle_ << ", process_dir=" << process_dir_
                << ", process_yaw=" << ioff(process_yaw_, 2) << ", yaw_src=" << yaw_src);
        } else
        if (nav_angle < ANGLE_OK) {
            // NOT_YET_MIGRATED: legacy's "nava_delay == 12345" NAV_ANGLE_DELAY
            // branch is provably unreachable dead code in istro_rt2025.cpp
            // itself (nava_delay is only ever 0 or 1) -- ported faithfully
            // as dead code below rather than silently dropped, matching
            // this project's "port 1:1 including its own warts" convention.
            int nava_delay = (timeDelta(minmax_lastt_) < 2000) && (nav_angle < CALIB_NAV_ANGLE_MIN) && (nav_angle > CALIB_NAV_ANGLE_MAX);
            int nava_noobst = -1;
            if (nava_delay) {
                double nx = process_x_;
                double ny = process_y_;
                double nyaw = process_yaw_;
                double nt = timeAdd2(process_time_, -2000);
                double ux = nx, uy = ny, uyaw = nyaw, ut = nt;
                updateXY(ux, uy, uyaw, ut, 15, SA_STRAIGHT, VEL_BACK);

                DegreeMap ndmap;
                int namin = -1;
                int namax = -1;
                wmodel.evalGrid(ux, uy, uyaw, ndmap);
                ndmap.find(DMAP_MIN_INTERVAL_LENGTH, 90, namin, namax);
                nava_noobst = (namin <= DETECTOBST_ANGLE_MIN) && (namax >= DETECTOBST_ANGLE_MAX);
                if (nava_noobst) {
                    nava_delay = 0;
                }
            }
            if (nava_delay == 12345) {
                process_angle_ = 90;
                process_state_ = PROCESS_STATE_NAV_ANGLE;
                LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"NAV_ANGLE_DELAY\"): image_number=" << image_number_ << ", nav_angle=" << ioff(nav_angle, 2)
                    << ", process_angle=" << process_angle_ << ", nava_delay=" << nava_delay << ", nava_noobst=" << nava_noobst
                    << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yawn=" << ioff(yawn, 2) << ", yaw_src=" << yaw_src);
            } else {
                process_angle_ = 90 - trunc(nav_angle);
                if (process_angle_ < NAVIGATION_ANGLE_MIN) {
                    process_angle_ = NAVIGATION_ANGLE_MIN;
                }
                if (process_angle_ > NAVIGATION_ANGLE_MAX) {
                    process_angle_ = NAVIGATION_ANGLE_MAX;
                }
                process_state_ = PROCESS_STATE_NAV_ANGLE;
                LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"NAV_ANGLE\"): image_number=" << image_number_ << ", nav_angle=" << ioff(nav_angle, 2)
                    << ", process_angle=" << process_angle_ << ", nava_delay=" << nava_delay << ", nava_noobst=" << nava_noobst
                    << ", yaws=" << ioff(yaws, 2) << ", yawc=" << ioff(yawc, 2) << ", yawn=" << ioff(yawn, 2) << ", yaw_src=" << yaw_src);
            }
        }
    }
    // obstacle detected -> MIN_MAX
    if ((process_angle_ >= (int)ANGLE_OK) && (process_angle_min_ >= 0) && (process_angle_max_ >= 0)) {
        process_angle_ = (process_angle_min_ + process_angle_max_) / 2;
        process_state_ = PROCESS_STATE_MIN_MAX;
        minmax_lastt_ = timeBegin();
        LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"MIN_MAX\"): image_number=" << image_number_ << ", navp_azimuth=" << ioff(gps_navp_azimuth, 2)
            << ", process_yaw=" << ioff(process_yaw_, 2)
            << ", process_angle_min=" << process_angle_min_ << ", process_angle_max=" << process_angle_max_
            << ", process_angle=" << process_angle_ << ", process_dir=" << process_dir_ << ", yaw_src=" << yaw_src);
    }
    // no route found? force wrongway after some time
    if ((process_angle_ >= (int)ANGLE_OK) && (yawn < ANGLE_OK) && (timeDelta(noangle_to_) >= PROCESS_NOANGLE_TO)) {
        double wwf_angle = nav_angle;
        if (nav_angle >= ANGLE_OK) {
            wwf_angle = 0;
        }
        wrongwayForce(wrongway_, yawn, wwf_angle);
        if (wrongwayProcess(wrongway_, yawn, process_angle_, process_velocity_, process_state_, process_stop_) > 0) {
            LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"WRONG_WAY_FORCE\"): image_number=" << image_number_ << ", process_angle=" << process_angle_
                << ", process_velocity=" << process_velocity_ << ", noangle_dt=" << ((int)timeDelta(noangle_to_)));
        }
    }
    if (process_angle_ >= (int)ANGLE_OK) {
        LOGM_INFO(loggerIstroPlanner2, "plannerTick", "process_angle(\"NO_ANGLE\"): image_number=" << image_number_ << ", process_angle=" << process_angle_ << ", process_velocity=" << process_velocity_
            << ", noangle_dt=" << ((int)timeDelta(noangle_to_)));
    } else {
        if ((process_angle_ < PROCESS_NOANGLE_MIN) || (process_angle_ > PROCESS_NOANGLE_MAX)) {
            LOGM_WARN(loggerIstroPlanner2, "plannerTick", "msg=\"process_noangle_min/max!\", image_number=" << image_number_
                << ", process_angle=" << process_angle_ << ", process_velocity=" << process_velocity_
                << ", noangle_dt=" << ((int)timeDelta(noangle_to_)));
        }
        noangle_to_ = timeBegin();
    }

    // NOT_YET_MIGRATED: legacy's process_wrongway_force (GUI debug-key
    // trigger, "volatile int process_wrongway_force") has no ROS equivalent
    // input source yet -- no keyboard/GUI debug publisher exists for
    // planner_node.

#ifdef ISTRO_CALIB2
    if (process_angle_ >= (int)ANGLE_OK) {
        calibReset(calib2_, 2, "process_noangle");
    } else {
        if ((process_angle_ < CALIB_PROCESS_ANGLE_MIN) || (process_angle_ > CALIB_PROCESS_ANGLE_MAX)) {
            calibReset(calib2_, 2, "process_angle_check");
        }
    }
#endif

    // determine driving speed/velocity
    speedctlProcess(speedctl_, process_dmap, process_dir_, process_velocity_, process_state_, process_stop_);

    // update robot position based on last position (from process_time time)
    // NOT_YET_MIGRATED: legacy's commented-out GPS-based process_x/y
    // correction ("skusime zakomentovat...") was already dead code in
    // istro_rt2025.cpp itself -- not ported, matches project convention.
    updateXY(process_x_, process_y_, process_yaw_, process_time_,
        ctrlb_ircv500, ctrlb_angle, ctrlb_velocity);

    // legacy: "pdata->image_number = image_number++;" -- this frame gets the
    // pre-increment value, so image_number_ - 1 is what was just published.
    int64_t image_number = image_number_++;

    // Legacy's process_writeData() log: the single most important record of
    // what this decision cycle actually produced. Kept under legacy's own
    // "process_writeData" module name, and placed exactly where legacy called
    // process_writeData() -- the publishXxx() helpers stay pure (no DataSet
    // access, no logging), matching every other node in this workspace.
    LOGM_DEBUG(loggerIstroPlanner2, "process_writeData", "image_number=" << image_number
        << ", process_angle=" << process_angle_ << ", process_velocity=" << process_velocity_
        << ", process_state=" << process_state_ << ", process_stop=" << process_stop_
        << ", process_backward=" << process_backward_ << ", process_dir=" << process_dir_
        << ", process_ref=" << process_ref_ << ", process_x=" << ioff(process_x_, 2) << ", process_y=" << ioff(process_y_, 2)
        << ", process_yaw=" << ioff(process_yaw_, 2) << ", coneseek_stop=" << coneseek_stop_);

    publishDriveCommand(process_angle_, process_velocity_, process_state_, process_stop_, process_yaw_);
    publishPlannerData(image_number, process_dir_, process_ref_, process_x_, process_y_,
        process_backward_, coneseek_stop_, coneseek_intlen_);
    publishPlannerDebugData(image_number, lidar_angle_min_, lidar_angle_max_, lidar_stop_,
        process_angle_min_, process_angle_max_);
    publishImageNumber(image_number);

    process_change_ = 0;
    timeEnd("istro::planner_node.process", t);

    LOGM_INFO(loggerIstroPlanner2, "plannerTick", "msg=\"data processed\", image=" << image_number_ - 1);
}

// ---------------------------------------------------------------------------
// Publish helpers -- pure parameters, no DataSet access (no legacy
// counterpart to mirror).
// ---------------------------------------------------------------------------

void PlannerNode::publishDriveCommand(int angle, int velocity, int state, int stop, double yaw)
{
    double tpub = timeBegin();

    istrorsx_core::msg::DriveCommand msg;
    msg.angle = angle;
    msg.velocity = velocity;
    msg.state = state;
    msg.stop = stop;
    msg.yaw = yaw;
    pub_drive_->publish(msg);
    timeEnd("istro::planner_node.publishDriveCommand", tpub);
}

void PlannerNode::publishPlannerData(int64_t image_number, int process_dir, int process_ref, double process_x, double process_y,
        int process_backward, int coneseek_stop, int coneseek_intlen)
{
    double tpub = timeBegin();

    istrorsx_core::msg::PlannerData msg;
    msg.image_number = image_number;
    msg.process_dir = process_dir;
    msg.process_ref = process_ref;
    msg.process_x = process_x;
    msg.process_y = process_y;
    msg.process_backward = process_backward;
    msg.coneseek_stop = coneseek_stop;
    msg.coneseek_intlen = coneseek_intlen;
    pub_planner_data_->publish(msg);
    timeEnd("istro::planner_node.publishPlannerData", tpub);
}

void PlannerNode::publishVisionControl(bool qrscan_enabled, bool process_rear_enabled)
{
    double tpub = timeBegin();

    istrorsx_core::msg::VisionControl msg;
    msg.qrscan_enabled = qrscan_enabled;
    msg.process_rear_enabled = process_rear_enabled;
    pub_vision_control_->publish(msg);
    timeEnd("istro::planner_node.publishVisionControl", tpub);
}

void PlannerNode::publishNavigationPointSet(int point_idx, double point_latitude, double point_longitude)
{
    double tpub = timeBegin();

    istrorsx_core::msg::NavigationPointSet msg;
    msg.point_idx = point_idx;
    msg.point_latitude = point_latitude;
    msg.point_longitude = point_longitude;
    pub_navigation_point_set_->publish(msg);
    timeEnd("istro::planner_node.publishNavigationPointSet", tpub);
}

void PlannerNode::publishImageNumber(int64_t image_number)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::ImageNumber msg;
    msg.image_number = image_number;
    pub_image_number_->publish(msg);
    timeEnd("istro::planner_node.publishImageNumber", tpub);
}

void PlannerNode::publishPlannerDebugData(int64_t image_number, int lidar_angle_min, int lidar_angle_max, int lidar_stop,
    int process_angle_min, int process_angle_max)
{
    double tpub = timeBegin();

    istrorsx_core::msg::PlannerDebugData msg;
    msg.image_number = image_number;
    msg.lidar_angle_min = lidar_angle_min;
    msg.lidar_angle_max = lidar_angle_max;
    msg.lidar_stop = lidar_stop;
    msg.process_angle_min = process_angle_min;
    msg.process_angle_max = process_angle_max;
    pub_planner_debug_->publish(msg);
    timeEnd("istro::planner_node.publishPlannerDebugData", tpub);
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (istro_main_planner(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroPlanner2, "main", "msg=\"istro_main_planner() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<PlannerNode>();

    // Two threads, two MutuallyExclusive groups: the sensor subscriptions that
    // trigger processing on one, everything else (5 ms timer, vision, lidar,
    // config, the WMGrid service) on the default group as before. More threads
    // would idle -- two exclusive groups can never run three callbacks at once.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    istro_close_planner();
    rclcpp::shutdown();
    return 0;
}
