#ifndef __LIDAR_DEFS_H__
#define __LIDAR_DEFS_H__

#include <opencv2/opencv.hpp>

#include "dmap.h"
#include "system.h"    // ISTRO_LIDAR_FILTERSUN

// Pure data types + geometry for a lidar scan, split out of lidar.h so
// DataSet, istrorsx_core's drive_node, and (the actual reason lidar_process()
// lives here) istrorsx_core's planner_node can all use them without pulling
// in the RPLIDAR SDK. lidar.h's Lidar class (the actual hardware driver --
// connect/init/close/getData()/drawOutput()) still lives in istrorsx_hw and
// includes this header too, for drawOutput()'s own use of the calibration
// constants below.

const int LIDAR_DATA_NUM = 360 * 2;

typedef struct {
    int   sync;
    float angle;
    float distance;
    int   quality;
} lidar_data_t;

// Calibration constants needed by both lidar_process() (below) and
// istrorsx_hw's Lidar::drawOutput() -- header-only (like config.h's
// ANGLE_NONE/ANGLE_OK) so both translation units get the same values without
// either one owning the other.
static const float LIDAR_DISTANCE_MIN   =   1;    // minimum distance that could be measured by lidar (in centimeters)
static const float LIDAR_DISTANCE_DMAP  = 150;    // obstacle avoidance distance (in centimeters)
static const float LIDAR_DISTANCE_STOP  =  50;    // minimum distance that will stop the robot (in centimeters)
static const float LIDAR_DISTANCE_MAX   = 300;    // maximum distance where obstacles could be detected (required also for DegreeMap)

static const int   LIDAR_QUALITY_MIN    =   1;    // ignore lidar data if quality is low (<1)
#ifndef ISTRO_LIDAR_FILTERSUN
static const int   LIDAR_QUALITY_DMAP   =   2;    // ignore obstacles (to filter sun reflection) if quality is below (2=ignore, no filter)
static const int   LIDAR_QUALITY_STOP   =   3;    // ignore stop condition (to filter sun reflection) if quality is below (3=ignore, no filter)
#else
static const int   LIDAR_QUALITY_DMAP   =  13;    // ignore obstacles (to filter sun reflection) if quality is below (<13)
static const int   LIDAR_QUALITY_STOP   =  20;    // ignore stop condition (to filter sun reflection) if quality is below (<20)
#endif
static const int   LIDAR_QUALITY_MAX    =  45;    // ignore lidar data if quality is too high - car reflector, direct sun (>=45)

static const float LIDAR_STOP_ANGLE_MIN =  45;    // minimum angle where distance is checked
static const float LIDAR_STOP_ANGLE_MAX = 135;    // maximum angle where distance is checked
static const int   LIDAR_STOP_COUNT     =   3;    // number of angles where the distance must be exceeded

// Drawing-only calibration bounds, needed by lidar_draw_output() below
// (previously file-scope consts in istrorsx_hw's lidar.cpp, moved here for
// the same reason as everything else in this file -- see that function's
// own port note).
static const float LIDAR_DISTANCE_MAXD  = 300;    // maximum distance - only for drawing purposes
static const int   LIDAR_QUALITY_MAXD   =  60;    // maximum quality - only for drawing purposes

// Pure geometry, no Lidar-class/hardware-SDK dependency -- 1:1 port of
// legacy Lidar::process() (istro_rt2025.cpp: lidar.process(...), called
// from process_thread), converted from a class method to a free function
// since it never touched Lidar's own instance state anyway. Was dead code
// in istrorsx_hw (no caller there -- lidar_node only captures/publishes raw
// scans; process_thread's dmap computation is planner_node's job, a
// different process, hence the move here).
int lidar_process(const lidar_data_t *data, const int& data_cnt, DegreeMap& dmap, int &stop);

// 1:1 port of legacy Lidar::drawOutput() (istro_rt2025.cpp/lidar.cpp,
// called from save_thread only), converted from a class method to a free
// function for the same reason as lidar_process() above -- it never touched
// Lidar's own instance state (no this->drv/channel access), it was only
// ever trapped inside the SDK-coupled Lidar class by C++ file organization,
// not by any real functional need. Moved here so the future save_node can
// render the "lidar.png" debug overlay without linking the RPLIDAR SDK.
// istrorsx_hw's Lidar class loses this method (see lidar.h) -- lidar_node
// never called it either (drawOutput() is ISTRO_GUI-only there, and
// lidar_node has no GUI).
int lidar_draw_output(const lidar_data_t *data, const int& data_cnt, cv::Mat& img, int stop, int angle_min, int angle_max,
    int process_angle, int process_angle_min, int process_angle_max, long image_number);

#endif
