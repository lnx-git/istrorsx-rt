#ifndef __DATASET_H__
#define __DATASET_H__

#include <opencv2/opencv.hpp>   // cv::Mat -- needed by both branches below (camera_img/camera_depth)
#include "system.h"
#include "mtime.h"
#include "dmap.h"          // DegreeMap -- needed by both branches below (vision_dmap/lidar_dmap etc.)
#include "lidar_defs.h"    // lidar_data_t, LIDAR_DATA_NUM -- needed by both branches below (not the full lidar.h Lidar class, which needs the RPLIDAR SDK and stays in istrorsx_hw)

// PROCESS_STATE_* is needed by both branches below (drive_node, ISTRO_ROS2
// branch, reads/writes process_state; the legacy #ifndef branch keeps it too
// for diff-compatibility with the source project) -- moved out of the
// per-branch block it originally lived in.
const int PROCESS_STATE_NONE         =  0;
const int PROCESS_STATE_CALIBRATION  =  1;
const int PROCESS_STATE_WRONGWAY     =  2;
const int PROCESS_STATE_LOADING      =  3;
const int PROCESS_STATE_UNLOADING    =  4;
const int PROCESS_STATE_NAV_ANGLE    =  5;
const int PROCESS_STATE_MIN_MAX      =  6;
const int PROCESS_STATE_QRSCAN_COORD =  7;
const int PROCESS_STATE_CONE_SEEK    =  8;
const int PROCESS_STATE_BALLDROP     = 10;  // hodnoty 10 az 19 su rezervovane na vykladanie lopticiek
const int PROCESS_STATE_BALLDROP_9   = 19;

#ifndef ISTRO_ROS2

#include "dmap.h"

class DataSet {
public:
    /* front image */
    Mat camera_img;
    Mat camera_img_pred;

    Mat camera_depth;
    Mat camera_depth_pred;

public:
    /* rear image */
    Mat camera2_img;
    Mat camera2_img_pred;

    Mat camera2_depth;
    Mat camera2_depth_pred;

public:
    /* front image - vision data */
    Mat vision_markers;
    Mat vision_markersIM;

    Mat vision_epweight;
    Mat vision_elweight;

    DegreeMap vision_dmap;
    int vision_angle_min;
    int vision_angle_max;

public:
    /* rear image - vision data */
    Mat vision2_markers;
    Mat vision2_markersIM;

    Mat vision2_epweight;
    Mat vision2_elweight;

    DegreeMap vision2_dmap;
    int vision2_angle_min;
    int vision2_angle_max;

public:
    double qrscan_latitude;
    double qrscan_longitude;

public:
    int lidar_data_cnt;
    lidar_data_t lidar_data[LIDAR_DATA_NUM];

    DegreeMap lidar_dmap;
    int lidar_angle_min; 
    int lidar_angle_max;
    int lidar_stop;

public:
    int process_dir;  // heading (45..135) - what direction should we go (45 = right, 90 = forward, 135 = left)
    DegreeMap process_dmap;
    int process_angle_min;
    int process_angle_max;

    int coneseek_angle_min;    /* min. angle to detected cone */
    int coneseek_angle_max;    /* max. angle to detected cone */
    // shared
    int coneseek_intlen;       /* max. angle interval length */
    int coneseek_stop;

    // shared    
    int process_angle;
    int process_velocity;
    int process_stop;
    int process_state;
    int process_backward;       // process_backward=1 means that robot is going backward

    int    process_ref;         // process_ref=1 means that process_x and process_y are valid
    double process_x;           // calculated x position
    double process_y;           // calculated y position
    double process_yaw;         // robot heading
    double process_time;        // time when we calculated the last position
    
public:
    // shared
    double gps_time;            // when the data were received from GPS device

    int    gps_fix;
    double gps_latitude;
    double gps_longitude;
    double gps_latitude_raw;
    double gps_longitude_raw;
    double gps_speed;
    double gps_course;

    double gps_lastp_dist;      // distance to the previous gps fix position
    double gps_lastp_azimuth;   // azimuth to the previous gps fix position

    int    gps_navp_idx;        // index of the next navigation point (-1 = no point found)
    double gps_navp_dist;       // distance to the next navigation point
    double gps_navp_dist_raw;
    double gps_navp_azimuth;    // azimuth to the next navigation point
    double gps_navp_azimuth_raw;
    double gps_navp_maxdist;    // maximum distance to the next navigation point (needed for calibration)
    int    gps_navp_loadarea;
    double gps_navp_latitude;
    double gps_navp_longitude;
    double gps_navp_latitude_raw;
    double gps_navp_longitude_raw;

    int    gps_ref;
    double gps_x; 
    double gps_y;

public:
    // shared
    double ahrs_roll; 
    double ahrs_pitch; 
    double ahrs_yaw;

public:
    // shared
    double ctrlb_time1;         // when the data were received from controlboard
    int ctrlb_state;
    int ctrlb_ircv;
    double ctrlb_ircv500;
    int ctrlb_angle; 
    int ctrlb_velocity;
    int ctrlb_loadd;            // detekcia nakladu (load detection), 0 nebol detekovany sudok, 1 bol detekovany sudok

    double ctrlb_time2;         // when the data were received from controlboard
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int ctrlb_calib_gyro;
    int ctrlb_calib_accel;
    int ctrlb_calib_mag;

public:
    // shared
    long image_number;

public:
    DataSet();
};

#else /* #ifdef ISTRO_ROS2 */

class DataSet {
public:
    // shared -- one camera_node process handles exactly one physical camera
    // (front or rear, per -camdev), so unlike the legacy single-process app
    // there's no camera2_img/camera2_depth pair here.
    cv::Mat camera_img;      // color frame, BGR8 (CV_8UC3)
    cv::Mat camera_depth;    // depth frame, millimeters (CV_16UC1); empty if depth capture failed

public:
    // shared -- written by istrorsx_core's vision_node (its own camera2_writeData(),
    // mirroring camera_writeData() above) and read back by vision_node's own
    // vision2_readData(). Unlike every other node so far, vision_node has two
    // input streams (front+rear CameraFrame messages) sharing one process --
    // camera_img/camera_depth above serve the front tick, these serve the
    // rear tick, so the two ticks don't stomp on each other's DataSet fields.
    cv::Mat camera2_img;
    cv::Mat camera2_depth;

public:
    // shared
    double gps_time;            // when the data were received from GPS device

    int    gps_fix;
    double gps_latitude;
    double gps_longitude;
    double gps_latitude_raw;
    double gps_longitude_raw;
    double gps_speed;
    double gps_course;

    double gps_lastp_dist;      // distance to the previous gps fix position
    double gps_lastp_azimuth;   // azimuth to the previous gps fix position

public:
    // shared -- written by istrorsx_core's navigation_node (port of legacy
    // gps_thread's navigation-point/route-planning extension, navig.cpp/
    // navmap.cpp) and read back by navigation_node's own loopReadData() port,
    // exactly mirroring legacy gps_writeData()'s navigation-specific subset.
    int    gps_navp_idx;        // index of the next navigation point (-1 = no point found)
    double gps_navp_dist;       // distance to the next navigation point (route-corrected)
    double gps_navp_dist_raw;   // distance to the next navigation point (straight-line GPS)
    double gps_navp_azimuth;    // azimuth to the next navigation point (route-corrected)
    double gps_navp_azimuth_raw;// azimuth to the next navigation point (straight-line GPS)
    double gps_navp_maxdist;    // maximum distance seen to the current navigation point (for calibration)
    int    gps_navp_loadarea;   // NAVIGATION_AREA_* (navig.h)
    double gps_navp_latitude;
    double gps_navp_longitude;
    double gps_navp_latitude_raw;
    double gps_navp_longitude_raw;

    int    gps_ref;             // 1 = reference point initialized
    double gps_x;                // robot position, local XY metres, relative to the reference point
    double gps_y;                // robot position, local XY metres, relative to the reference point

public:
    // shared -- written by istrorsx_core's planner_node (its own vision_writeData()/
    // vision2_writeData(), mirroring the legacy shared pool's checkData(THDATA_STATE_VISION_PROCESSED/
    // VISION2_PROCESSED) poll) and read back immediately within the same callback via
    // vision_readData()/vision2_readData() -- vision_node's own VisionData.msg is the
    // message-based equivalent of what vision_thread wrote into this pool in legacy.
    DegreeMap vision_dmap;
    int vision_angle_min;
    int vision_angle_max;

    DegreeMap vision2_dmap;
    int vision2_angle_min;
    int vision2_angle_max;

    double qrscan_latitude;
    double qrscan_longitude;

public:
    // shared
    int lidar_data_cnt;
    lidar_data_t lidar_data[LIDAR_DATA_NUM];

    // Fills the gap the comment here used to flag as NOT_YET_MIGRATED --
    // written by istrorsx_core's planner_node (its own lidar_writeData(),
    // mirroring the legacy shared pool's checkData(THDATA_STATE_LIDAR_CAPTURED)
    // poll) and read back immediately via lidar_readData(). planner_node calls
    // the newly-extracted lidar_process() (istrobtx's lidar_defs.h) on the raw
    // scan itself, matching legacy computing lidar_dmap synchronously inside
    // process_thread (not a separate thread).
    DegreeMap lidar_dmap;
    int lidar_angle_min;
    int lidar_angle_max;
    int lidar_stop;

public:
    // shared -- written by istrorsx_core's drive_node (from the DriveCommand
    // message, itself the future planner_node's output -- port of legacy
    // process_thread) and read back by drive_node's own loop_readData() port,
    // exactly mirroring legacy loop_readData()'s process_* subset.
    // NOT_YET_MIGRATED: process_dir/process_dmap/process_angle_min/max,
    // process_backward, process_ref/process_x/process_y/process_time omitted --
    // not part of loop_readData()'s signature (only reachable via the
    // capture_readData() path, already NOT_YET_MIGRATED in lidar_node/camera_node).
    int process_angle;
    int process_velocity;
    int process_stop;
    int process_state;
    double process_yaw;         // robot heading

public:
    // shared
    double ctrlb_time1;         // when the data were received from controlboard
    int ctrlb_state;
    int ctrlb_ircv;
    double ctrlb_ircv500;
    int ctrlb_angle;
    int ctrlb_velocity;
    int ctrlb_loadd;            // detekcia nakladu (load detection), 0 nebol detekovany sudok, 1 bol detekovany sudok

    double ctrlb_time2;         // when the data were received from controlboard
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int ctrlb_calib_gyro;
    int ctrlb_calib_accel;
    int ctrlb_calib_mag;

public:
    // shared
    long image_number;

public:
    DataSet();
};

#endif

#endif
