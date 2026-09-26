#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dmap.h"
#include "wmodel.h"

#include "istrorsx_hw/msg/servo_data.hpp"
#include "istrorsx_hw/msg/imu_data.hpp"
#include "istrorsx_hw/msg/gps_data.hpp"
#include "istrorsx_hw/msg/lidar_data.hpp"
#include "istrorsx_hw/msg/image_number.hpp"
#include "istrorsx_core/msg/navigation_data.hpp"
#include "istrorsx_core/msg/vision_data.hpp"
#include "istrorsx_core/msg/drive_command.hpp"
#include "istrorsx_core/msg/planner_data.hpp"
#include "istrorsx_core/msg/planner_debug_data.hpp"
#include "istrorsx_core/msg/vision_control.hpp"
#include "istrorsx_core/msg/navigation_point_set.hpp"
#include "istrorsx_core/msg/config_update.hpp"
#include "istrorsx_core/srv/get_wm_grid_snapshot.hpp"

// process_thread's own collaborator structs (istro_rt2025.cpp), unchanged --
// see planner_node.cpp for calibInit()/wrongwayInit()/etc.
typedef struct {
    int    ok;
    double azimuth;
    double yaw;

    int    first;           // first calibration needs to be performed?
    int    navp_ok;         // distance to navigation point is OK -> calibration could be performed at current position
    int    nav_angle_ok;    // course towards next goal is OK

    double time;
    double gps_course_min;
    double gps_course_max;
    double yaw_min;
    double yaw_max;
    double gps_speed_min;
    double gps_speed_max;
} calib_t;

typedef struct {
    int    ok;       // 1 = do check, 0 = go backward!

    double time;
    double yaw0;     // what direction should I turn to (0..360)
    double yaw0_min;
    double yaw0_max;

    double ircv_low_time;  // cas kedy naposledy bola rychlost moc nizka (stojime)
    double ircv_low_dt;    // celkovy cas ako dlho sme stali (od spustenia wrongway_check) - o tolko dlhsie musime cakat na vyhodnotenie wrongway_check
} wrongway_t;

typedef struct {
    int    noobstacle;     // 1 = no obstacle, 0 = obstacle detected!
    double time;
} detectobst_t;

typedef struct {
    int    state;     // 0 = no state, 1 = BALLUNLOAD
    double time;
    int    cnt;       // kolka lopticka ma byt vylozena, budeme to kodovat ako (PROCESS_STATE_BALLUNLOAD + cnt)
    int    navp_idx;  // v ktorom navigacnom bode sme naposledy robili "ball drop"
} loadarea_t;

typedef struct {
    int minmax_cnt;    // kolko krat bol minmax
    int velocity_last;
    int state_last;
    double time;
    double time2;
    double time3;
    double timet;
} speedctl_t;

// Planner/fusion node: 1:1 port of istro_rt2025.cpp's process_thread and its
// seven collaborator functions (update_angle_interval/calib_process/
// wrongway_process/detectobst_process/loadarea_process/update_xy/
// speedctl_process), plus the legacy global WorldModel wmodel (istro_main_planner.h/.cpp).
// Combines navigation_node's route/position output with vision_node's/
// lidar_node's perception output into the final steering/speed/state
// decision (DriveCommand.msg), consumed by drive_node.
//
// process_thread's own busy-poll structure (checkData() on the shared
// dataset pool for vision/lidar, a 20ms-gated recheck of GPS/controlboard
// state) becomes 4 ROS entry points instead: 3 event-driven "fast path"
// callbacks (cb_vision_front/cb_vision_rear/cb_lidar) that fold new
// perception data into wmodel immediately on message arrival -- matching
// legacy's own non-blocking checkData() polls, now event-driven -- plus one
// rclcpp::WallTimer (plannerTick(), mirroring drive_node's own controlTick()
// precedent) that runs the actual decision chain using the latest cached
// GPS/ServoData/ImuData/NavigationData state.
//
// The timer fires every PLANNER_TICK_PERIOD (5ms) -- it replaces legacy's
// busy-wait "msleep(2); continue;" poll, NOT its 20ms processing gate. Both
// levels are kept 1:1: plannerTick() re-checks for new controlboard/GPS data
// often (5ms), but only runs the decision chain once PROCESS_PERIOD_MIN
// (20ms) has elapsed since the last one AND a timestamp actually changed.
// Ticking at 20ms instead would alias against the controlboard's own 20ms
// cadence and silently halve the decision rate. See
// doc/ai/01_architecture.md's planner_node section for the full design
// writeup and the decisions behind it.
class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    // ---- ROS callbacks: fast path -- fold new perception data into wmodel
    // immediately, matching legacy's checkData(THDATA_STATE_VISION_PROCESSED/
    // VISION2_PROCESSED/LIDAR_CAPTURED) polls ----
    void cb_vision_front(const istrorsx_core::msg::VisionData::SharedPtr msg);
    void cb_vision_rear(const istrorsx_core::msg::VisionData::SharedPtr msg);
    void cb_lidar(const istrorsx_hw::msg::LidarData::SharedPtr msg);

    // ---- ROS callbacks: cache latest state for plannerTick(), following the
    // cross-process DataSet round-trip pattern (05_migration_guide.md) ----
    void cb_servo(const istrorsx_hw::msg::ServoData::SharedPtr msg);
    void cb_imu(const istrorsx_hw::msg::ImuData::SharedPtr msg);
    void cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg);
    void cb_navigation(const istrorsx_core::msg::NavigationData::SharedPtr msg);
    // Runtime conf change from drive_node (operator speed keys). Legacy needed
    // nothing here: one process, one conf. See ConfigUpdate.msg.
    void cb_config(const istrorsx_core::msg::ConfigUpdate::SharedPtr msg);

    // ---- GetWMGridSnapshot.srv server -- pull-based, called by the future
    // save_node only when it decides to render+save (01_architecture.md
    // planner_node decision #4). Never calls WMGrid::drawGrid()/drawGridFull()
    // itself -- those stay exclusive to save_node. ----
    void handleGetWMGridSnapshot(
        const std::shared_ptr<istrorsx_core::srv::GetWMGridSnapshot::Request> request,
        std::shared_ptr<istrorsx_core::srv::GetWMGridSnapshot::Response> response);

    // ---- Cross-process DataSet round-trip: *_writeData() mirror the legacy
    // writers (ctrlBoard_writeData()'s data1/data2 split -> servo_writeData()/
    // imu_writeData(), matching drive_node's own naming; gps_writeData() for
    // the raw fix; navigation_writeData() for navigation_node's routing
    // output); process_readData() is process_thread's own literal function
    // name, read back at the start of plannerTick(). ----
    int servo_writeData(const istrorsx_hw::msg::ServoData &msg);
    int imu_writeData(const istrorsx_hw::msg::ImuData &msg);
    int gps_writeData(const istrorsx_hw::msg::GpsData &msg);
    int navigation_writeData(const istrorsx_core::msg::NavigationData &msg);
    int process_readData(double &gps_time, double &gps_speed, double &gps_course,
        double &gps_navp_dist, double &gps_navp_azimuth, double &gps_navp_maxdist, int &gps_navp_loadarea, int &gps_navp_idx,
        double &ctrlb_time1, double &ctrlb_ircv500, int &ctrlb_angle, int &ctrlb_velocity, int &ctrlb_loadd,
        double &ctrlb_time2, double &ctrlb_euler_x,
        int &gps_ref, double &gps_x, double &gps_y, int &ctrlb_state);

    // vision/lidar fast-path DataSet round trip -- mirrors the shape above,
    // but write+read happen back-to-back inside the same callback (no
    // separate deferred tick), since that's what legacy's own checkData()
    // poll did too.
    int vision_writeData(const istrorsx_core::msg::VisionData &msg);
    int vision_readData(DegreeMap &dmap, int &angle_min, int &angle_max, double &qrscan_latitude, double &qrscan_longitude);
    int vision2_writeData(const istrorsx_core::msg::VisionData &msg);
    int vision2_readData(DegreeMap &dmap, int &angle_min, int &angle_max);
    int lidar_writeData(const istrorsx_hw::msg::LidarData &msg);
    int lidar_readData(DegreeMap &dmap, int &angle_min, int &angle_max, int &stop);

    // ---- process_thread's own collaborator functions, ported as private
    // methods (matching drive_node's own precedent of turning loop()'s
    // file-scope helper functions into methods) -- wrongwayProcess()/
    // loadareaProcess() need this-> to publish VisionControl.msg/
    // NavigationPointSet.msg, which legacy did via direct function calls
    // into vision_thread/navig.cpp instead. ----
    void updateAngleInterval(double &angle_min, double &angle_max, double angle);
    void updateSpeedInterval(double &speed_min, double &speed_max, double speed);

    void calibInit(calib_t &calib);
    int  calibReset(calib_t &calib, int idx, const std::string &log_reason);
    int  calibProcess(calib_t &calib, int idx, double nav_angle, int process_angle_min, int process_angle_max,
        double gps_course, double yaw, double gps_speed, double navp_dist, double navp_maxdist, double ctrlb_ircv500);

    void wrongwayInit(wrongway_t &wrongway);
    int  wrongwayInprogress(wrongway_t &wrongway);
    int  wrongwayReset(wrongway_t &wrongway, const std::string &log_reason);
    int  wrongwayCheck(wrongway_t &wrongway, double yaw, double angle, double navp_dist, double navp_maxdist, double ctrlb_ircv500);
    int  wrongwayForce(wrongway_t &wrongway, double yaw, double angle);
    int  wrongwayProcess(wrongway_t &wrongway, double yaw, int &process_angle, int &process_velocity, int &process_state, int process_stop);
    // Not in legacy: keeps WrongWay off while waiting (loadarea) or in emergency, see 01_architecture.md § WRONG_WAY.
    void wrongwayHold(const char *reason);
    void wrongwayHoldEnd();

    void detectobstInit(detectobst_t &detectobst);
    int  detectobstProcess(detectobst_t &detectobst, int calib_ok, int process_angle_min, int process_angle_max);

    void loadareaInit(loadarea_t &loadarea);
    int  loadareaProcess(loadarea_t &loadarea, int gps_navp_loadarea, int ctrlb_loadd, int &process_angle, int &process_velocity, int &process_state,
        int gps_navp_idx, double gps_navp_azimuth, double qrscan_latitude, double qrscan_longitude);

    int updateXY(double &process_x, double &process_y, double &process_yaw, double &process_time,
        double ctrlb_ircv500, int ctrlb_angle, int ctrlb_velocity);

    void speedctlInit(speedctl_t &speedctl);
    void speedctlTimeReset(speedctl_t &speedctl);
    int  speedctlGetshift(int dir, int angle1, int angle2);
    int  speedctlProcess(speedctl_t &speedctl, const DegreeMap &dmap, int process_dir, int &process_velocity, int &process_state, int &process_stop);

    // ---- plannerTick(): 5ms rclcpp::WallTimer, 1:1 port of one process_thread
    // while() iteration -- the "wait" poll, the PROCESS_PERIOD_MIN gate around
    // process_readData(), and (only when do_process) the whole decision chain
    // from "timeEnd(...wait)" through process_writeData(). See class comment. ----
    void plannerTick();

    // ---- Publish helpers -- pure parameters, no DataSet access (no legacy
    // counterpart to mirror), same pattern as every other node's
    // publishXxx() methods. ----
    void publishDriveCommand(int angle, int velocity, int state, int stop, double yaw);
    void publishPlannerData(int64_t image_number, int process_dir, int process_ref, double process_x, double process_y,
        int process_backward, int coneseek_stop, int coneseek_intlen);
    void publishVisionControl(bool qrscan_enabled, bool process_rear_enabled);
    void publishNavigationPointSet(int point_idx, double point_latitude, double point_longitude);
    // ImageNumber.msg -- published once per plannerTick() so camera_node
    // (istrorsx_hw, can't depend on istrorsx_core's own PlannerData.msg) can
    // stamp it onto CameraFrame.msg. See ImageNumber.msg's own comment.
    void publishImageNumber(int64_t image_number);
    // PlannerDebugData.msg -- the future save_node's input for
    // Lidar::drawOutput()'s overlay; see that message's own comment.
    void publishPlannerDebugData(int64_t image_number, int lidar_angle_min, int lidar_angle_max, int lidar_stop,
        int process_angle_min, int process_angle_max);

    // ---- Publishers ----
    rclcpp::Publisher<istrorsx_core::msg::DriveCommand>::SharedPtr pub_drive_;
    rclcpp::Publisher<istrorsx_core::msg::PlannerData>::SharedPtr pub_planner_data_;
    rclcpp::Publisher<istrorsx_core::msg::PlannerDebugData>::SharedPtr pub_planner_debug_;
    rclcpp::Publisher<istrorsx_core::msg::VisionControl>::SharedPtr pub_vision_control_;
    rclcpp::Publisher<istrorsx_core::msg::NavigationPointSet>::SharedPtr pub_navigation_point_set_;
    rclcpp::Publisher<istrorsx_hw::msg::ImageNumber>::SharedPtr pub_image_number_;

    // ---- Subscribers ----
    rclcpp::Subscription<istrorsx_core::msg::VisionData>::SharedPtr sub_vision_front_;
    rclcpp::Subscription<istrorsx_core::msg::VisionData>::SharedPtr sub_vision_rear_;
    rclcpp::Subscription<istrorsx_hw::msg::LidarData>::SharedPtr sub_lidar_;
    // Sensor callbacks run on their own thread (MultiThreadedExecutor, see main()).
    // Only subscriptions whose whole body writes to the lock-protected DataSet may
    // join it -- see the constructor.
    rclcpp::CallbackGroup::SharedPtr cbg_sensors_;

    rclcpp::Subscription<istrorsx_hw::msg::ServoData>::SharedPtr sub_servo_;
    rclcpp::Subscription<istrorsx_hw::msg::ImuData>::SharedPtr sub_imu_;
    rclcpp::Subscription<istrorsx_hw::msg::GpsData>::SharedPtr sub_gps_;
    rclcpp::Subscription<istrorsx_core::msg::NavigationData>::SharedPtr sub_navigation_;
    rclcpp::Subscription<istrorsx_core::msg::ConfigUpdate>::SharedPtr sub_config_;

    // ---- GetWMGridSnapshot.srv server ----
    rclcpp::Service<istrorsx_core::srv::GetWMGridSnapshot>::SharedPtr srv_wmgrid_snapshot_;

    // ---- Timer ----
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- process_thread's own per-iteration locals -> members ----
    double last_gps_time_ = -1;
    double last_ctrlb_time1_ = -1;
    double last_ctrlb_time2_ = -1;

    int last_process_angle_min_ = -1;
    int last_process_angle_max_ = -1;

    int64_t image_number_ = 0;
    double minmax_lastt_;
    double noangle_to_;

    // wrongwayHold() state: current hold reason (empty = not held), start and last held tick
    std::string ww_hold_reason_;
    double ww_hold_start_ = -1;
    double ww_hold_end_ = -1;

    // process_thread's own loop-timing locals: process_to_ gates the decision
    // chain to PROCESS_PERIOD_MIN (and feeds the processing-lag detection),
    // tw_ measures how long we waited for new data ("istro::planner_node.wait").
    double process_to_;
    double tw_ = -1;

    // Accumulated across ticks until a decision cycle actually runs, then
    // reset to 0 -- exactly like legacy's own "int process_change" local:
    // +1 = new front-vision data folded in (cb_vision_front), +2 = new lidar
    // data (cb_lidar), +4 = new controlboard/GPS timestamps (plannerTick),
    // plus the PROCESS_CHANGE_GPS/CTRLB1/CTRLB2 bits. The CTRLB2 bit also
    // selects the yaw source in plannerTick(), so this cannot be a per-tick
    // local -- the vision/lidar bits are contributed from other callbacks.
    int process_change_ = 0;

    int process_backward_ = -1;
    int process_stop_ = -1;
    int lidar_stop_ = -1;
    // DegreeMap::find()'s output on the raw lidar dmap (cb_lidar()) -- only
    // ever used for logging/wmodel update in legacy process_thread itself,
    // but kept here (not just locals) so PlannerDebugData.msg can publish
    // them for the future save_node's Lidar::drawOutput() call.
    int lidar_angle_min_ = -1;
    int lidar_angle_max_ = -1;
    int vision_stop_ = -1;
    int vision_stop_bck_ = -1;
    int vision_stop_bck_cnt_ = 0;
    double vision_lastt_ = -1;  // last time when vision correctly processed an image

    // process_thread's own local "DataSet data;" accumulator -> plain
    // members (NOT a DataSet round trip -- process_thread computed/owned
    // this itself, no other legacy thread ever wrote it; matches vision_node's
    // own OUTPUT-side precedent of skipping DataSet when there's no other
    // in-process consumer). Initial values mirror DataSet::DataSet()
    // (dataset.cpp) field for field, since that is what legacy's own local
    // "DataSet data;" started from -- the ANGLE_NONE/PROCESS_STATE_* ones are
    // set in the constructor instead (config.h/dataset.h not included here).
    int process_ref_ = 0;
    int process_angle_;       // (int)ANGLE_NONE
    int process_angle_min_ = -1;
    int process_angle_max_ = -1;
    int process_velocity_ = -1;
    int process_state_;       // PROCESS_STATE_NONE
    int process_dir_ = 90;
    double process_x_ = 0.0;
    double process_y_ = 0.0;
    double process_yaw_;      // ANGLE_NONE
    double process_time_ = -1;
    double qrscan_latitude_;  // ANGLE_NONE
    double qrscan_longitude_; // ANGLE_NONE
    int coneseek_angle_min_ = -1;
    int coneseek_angle_max_ = -1;
    int coneseek_intlen_ = -1;
    int coneseek_stop_ = -1;

    // Current commanded VisionControl.msg state -- legacy's
    // vision_thread_qrscan_enable()/_disable()/processRear_enable()/_disable()
    // toggled two independent process-global flags directly; here both are
    // tracked so publishVisionControl() can always send the combined,
    // current state whenever either one changes (see wrongwayProcess()/
    // loadareaProcess()).
    bool qrscan_enabled_ = false;
    bool process_rear_enabled_ = false;

    // ---- collaborator struct instances (process_thread's own locals) ----
    calib_t calib_;
    calib_t calib2_;   // ISTRO_CALIB2 -- logging/monitoring only, never affects driving (see 01_architecture.md decision #5)
    wrongway_t wrongway_;
    detectobst_t detectobst_;
    loadarea_t loadarea_;
    speedctl_t speedctl_;
};
