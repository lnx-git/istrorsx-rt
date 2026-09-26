#include "istrorsx_hw/gps_node.hpp"

#include <cstring>

#include "threads.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "config.h"
#include "istrorsx_hw/istrobtx/gpsdev.h"
#include "geocalc.h"    // moved to the shared istrobtx package -- also used by istrorsx_core's navigation_node
#include "istrorsx_hw/istrobtx/istro_main_gps.h"

LOG_DEFINE(loggerIstroGps2, "istro");

GpsNode::GpsNode() : Node("gps_node")
{
    // Setup publisher for GPS telemetry
    pub_gps_ = this->create_publisher<istrorsx_hw::msg::GpsData>("/robot/gps_data", 10);

    // Setup a periodic timer for reading GPS status. GPS fixes update at
    // roughly 1 Hz, so this is far slower than ctrlboard_node's 5 ms timer.
    timer_gps_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&GpsNode::gps_read_cb, this)
    );
}

// NOTE: reduced parameter list versus the original gps_writeData() -- fields
// depending on navig.cpp/navmap.cpp (gps_navp_*, gps_ref, gps_x, gps_y) are
// computed by istrorsx_core's navigation_node instead (a separate process,
// fed by this node's own /robot/gps_data publish), not by this node. See
// doc/ai/01_architecture.md § navigation_node and DataSet in dataset.h.
int gps_writeData(double gps_time, int gps_fix, double gps_latitude, double gps_longitude, double gps_latitude_raw, double gps_longitude_raw,
        double gps_speed, double gps_course,
        double gps_lastp_dist, double gps_lastp_azimuth,
        double total_dist)
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_time = gps_time;
    pdata->gps_fix = gps_fix;
    pdata->gps_latitude = gps_latitude;
    pdata->gps_longitude = gps_longitude;
    pdata->gps_latitude_raw = gps_latitude_raw;
    pdata->gps_longitude_raw = gps_longitude_raw;
    pdata->gps_speed = gps_speed;
    pdata->gps_course = gps_course;

    pdata->gps_lastp_dist = gps_lastp_dist;
    pdata->gps_lastp_azimuth = gps_lastp_azimuth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroGps2, "gps_writeData", "fix=" << gps_fix
        << ", latitude=" << ioff(gps_latitude, 6) << ", longitude=" << ioff(gps_longitude, 6)
        << ", latitude_raw=" << ioff(gps_latitude_raw, 6) << ", longitude_raw=" << ioff(gps_longitude_raw, 6)
        << ", speed=" << ioff(gps_speed, 3) << ", course=" << ioff(gps_course, 2)
        << ", lastp_dist=" << ioff(gps_lastp_dist, 3) << ", lastp_azimuth=" << ioff(gps_lastp_azimuth, 2)
        << ", total_dist=" << ioff(total_dist, 3));

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::gps_writeData", t);
#endif

    return 0;
}

void GpsNode::publishGpsData(int gps_fix, double gps_latitude_raw, double gps_longitude_raw, double gps_speed, double gps_course)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::GpsData gps_msg;
    gps_msg.fix = (gps_fix > 0);
    gps_msg.latitude = gps_latitude_raw;
    gps_msg.longitude = gps_longitude_raw;
    gps_msg.speed = gps_speed;
    gps_msg.course = gps_course;
    pub_gps_->publish(gps_msg);
    timeEnd("istro::gps_node.publishGpsData", tpub);
}

// Periodically fetch GPS fix data from gpsd
// void *gps_thread(void *parg)
void GpsNode::gps_read_cb()
{
    double gps_time;
    int    gps_fix;
    double gps_latitude = ANGLE_NONE;
    double gps_longitude = ANGLE_NONE;
    double gps_latitude_raw = ANGLE_NONE;
    double gps_longitude_raw = ANGLE_NONE;
    double gps_speed;
    double gps_course;

#if 0  // MIGRATED_TO_NAVIGATION_NODE: reference point / gps_x,gps_y -- now computed by istrorsx_core's navigation_node (navig.cpp's navigation_ref_get/navigation_getXY), fed by this node's own /robot/gps_data publish. See 01_architecture.md.
    int    gps_ref = 0;
    double gps_ref_latitude = ANGLE_NONE;
    double gps_ref_longitude = ANGLE_NONE;

    double gps_x = 0;
    double gps_y = 0;
#endif  // MIGRATED_TO_NAVIGATION_NODE

    // previous gps fix position
    double lastp_latitude = ANGLE_NONE;
    double lastp_longitude = ANGLE_NONE;
    double gps_lastp_dist = -1;
    double gps_lastp_azimuth = ANGLE_NONE;

    double total_dist = 0;
    double gps_latitude_old = ANGLE_NONE;
    double gps_longitude_old = ANGLE_NONE;

#if 0  // MIGRATED_TO_NAVIGATION_NODE: next navigation point + route (re)planning -- now done by istrorsx_core's navigation_node (navig.cpp/navmap.cpp). See 01_architecture.md.
    // next navigation point
    int    navp_idx = -1;
    double navp_latitude = ANGLE_NONE;
    double navp_longitude = ANGLE_NONE;
    double navp_latitude_raw = ANGLE_NONE;
    double navp_longitude_raw = ANGLE_NONE;
    int    navp_loadarea = -1;        // NAVIGATION_AREA_NONE, from navig.h (istrorsx_core/istrobtx)
    int    navp_loadarea_next = 0;    // NAVIGATION_AREA_NONE, from navig.h (istrorsx_core/istrobtx)
    double gps_navp_dist = -1;
    double gps_navp_dist_raw = -1;
    double gps_navp_azimuth = ANGLE_NONE;
    double gps_navp_azimuth_raw = ANGLE_NONE;
    double gps_navp_maxdist = -1;
    double gps_navp_mindist = -1;

#ifndef ISTRO_NAVMAP_NO_ROUTING
    // navigation map variables
    int    nmap_np_idx = -1;    // last navigation point used for route planning
#endif

    // NOT_YET_MIGRATED: coneseek_stop/coneseek_intlen still need the vision/cone-detection subsystem, not ported anywhere yet -- see below.
    int    coneseek_stop = -1;
    int    coneseek_intlen = -1;
#endif  // MIGRATED_TO_NAVIGATION_NODE

    double t;
    int result = 0;

    //LOG_THREAD_NAME("gps");
    LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"start\"");

#if 0  // MIGRATED_TO_NAVIGATION_NODE: navigation_node's constructor does this priming step itself now.
    if (conf.useNavigation) {
        navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude_raw, navp_longitude_raw, navp_loadarea_next, navp_idx);
        navp_latitude = navp_latitude_raw;
        navp_longitude = navp_longitude_raw;
    }
#endif  // MIGRATED_TO_NAVIGATION_NODE

    do {
        t = timeBegin();

        if ((gps_fix = gps.getData(gps_latitude_raw, gps_longitude_raw, gps_speed, gps_course)) < 0) {
            result = -2;
            break;
        }

        if ((gps_fix > 0) && (gps_latitude_raw < ANGLE_OK) && (gps_longitude_raw < ANGLE_OK)) {
            if ((gps_latitude_old < ANGLE_OK) && (gps_longitude_old < ANGLE_OK)) {
                double dist = -1;
                double azimuth = ANGLE_NONE;
                geoCalc.getDist(gps_latitude_old, gps_longitude_old, gps_latitude_raw, gps_longitude_raw, dist, azimuth);
                if (dist > 0) total_dist += dist;
            }
            gps_latitude_old = gps_latitude_raw;
            gps_longitude_old = gps_longitude_raw;
        }

        gps_time = timeBegin();

#if 0  // NOT_YET_MIGRATED: reads coneseek_stop/coneseek_intlen written by the vision/cone-detection subsystem (not ported)
        if (gps_readData(coneseek_stop, coneseek_intlen) < 0) {
            result = -3;
            break;
        }
#endif  // NOT_YET_MIGRATED

#if 0  // MIGRATED_TO_NAVIGATION_NODE: navigation_node's navigationTick() does this on every /robot/gps_data message it receives.
        gps_ref = navigation_ref_get(gps_ref_latitude, gps_ref_longitude);

        // reference point, gps_x/y
        if (gps_fix > 0) {
            if (gps_ref <= 0) {
                LOGM_ERROR(loggerIstroGps2, "gps_thread", "msg=\"error: reference point not initialized!\"");
                result = -1;
                break;
            }
            navigation_getXY(gps_latitude_raw, gps_longitude_raw, gps_x, gps_y);
        } else {
            gps_latitude_raw = ANGLE_NONE;
            gps_longitude_raw = ANGLE_NONE;
        }
#endif  // MIGRATED_TO_NAVIGATION_NODE
        gps_latitude = gps_latitude_raw;
        gps_longitude = gps_longitude_raw;

        // calculate distance and azimuth to last known GPS position
        if (gps_fix > 0) {
            if ((lastp_latitude < ANGLE_OK) && (lastp_longitude < ANGLE_OK)) {
                geoCalc.getDist(lastp_latitude, lastp_longitude, gps_latitude, gps_longitude, gps_lastp_dist, gps_lastp_azimuth);
            } else {
                gps_lastp_dist = -1;
                gps_lastp_azimuth = ANGLE_NONE;
            }
            lastp_latitude = gps_latitude;
            lastp_longitude = gps_longitude;
        } else {
            LOGM_WARN(loggerIstroGps2, "gps_thread", "no gps-fix!");
        }

        // store last N gps raw positions
        if ((gps_fix > 0) && (gps_longitude_raw < ANGLE_OK) && (gps_latitude_raw < ANGLE_OK)) {
            if (gps_pt_cnt < GPS_POINT_NUM) {
                gps_pt[gps_pt_cnt].style[0] = 0;        // initialize the new point
                gps_pt[gps_pt_cnt + 1].style[0] = 0;    // mark the stopper
                gps_pt_cnt++;
            }

            for(int i = gps_pt_cnt - 1; i >= 1; i--) {
                gps_pt[i] = gps_pt[i - 1];
            }
            gps_pt[0].name[0] = 0;
            gps_pt[0].desc[0] = 0;
            strcpy(gps_pt[0].style, "g");
            gps_pt[0].longitude = gps_longitude_raw;
            gps_pt[0].latitude = gps_latitude_raw;
        }

#if 0  // MIGRATED_TO_NAVIGATION_NODE: distance/azimuth to next navigation point + route (re)planning -- now done by istrorsx_core's navigation_node. See 01_architecture.md § navigation_node.
        if (gps_fix > 0) {
            // calculate distance and azimuth to next navigation point, check if it was passed
            if (conf.useNavigation) {
                // wait for unknown navigation coordinates - have to be scanned from a QR-code
                if ((navp_idx >= 0) && ((navp_latitude_raw >= ANGLE_OK) || (navp_longitude_raw >= ANGLE_OK))) {
                    navigation_point_get(navp_idx, navp_latitude_raw, navp_longitude_raw);  // if > 0) { ... }
                    navp_latitude = navp_latitude_raw;
                    navp_longitude = navp_longitude_raw;
                }
                if ((navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                    geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw, navp_longitude_raw, gps_navp_dist_raw, gps_navp_azimuth_raw);
                    gps_navp_dist = gps_navp_dist_raw;
                    gps_navp_azimuth = gps_navp_azimuth_raw;
                    int point_passed = gps_navp_dist_raw < NAVIGATION_DISTANCE_THRESHOLD;
                    // do we need to approach this point very closely (un/loading area)?
                    int navp_approach = navigation_approach(conf.navigationPath, navigationPathPos-2);
                    if (point_passed && navp_approach) {
                        // ak sme vo vzdialenosti do 10m a vidime kuzel, tak ignorujeme GPS vzdialenost
                        // a cakame na priblizenie dostatocne blizko ku kuzelu
                        if (coneseek_intlen < 0) {
                        // wait for passing the second threshold or detecting that the distance is greater than the previous one
                            point_passed = (gps_navp_dist_raw < NAVIGATION_DISTANCE_THRESHOLD2) ||
                                           (gps_navp_dist_raw >= gps_navp_mindist + 1.5);
                        } else {
                            point_passed = (coneseek_stop > 0);
                        }
                    }
                    if (point_passed) {
                        LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"navigation point passed!\", pos=" << navigationPathPos
                            << ", name=\"" << conf.navigationPath[navigationPathPos-2] << conf.navigationPath[navigationPathPos-1] << "\""
                            << ", navp_dist=" << ioff(gps_navp_dist_raw, 3)
                            << ", navp_mindist=" << ioff(gps_navp_mindist, 3)
                            << ", navp_approach=" << navp_approach << ", coneseek_stop=" << coneseek_stop);
                        navp_loadarea = navp_loadarea_next;
                        navigation_next_point(conf.navigationPath, navigationPathPos, navp_latitude_raw, navp_longitude_raw, navp_loadarea_next, navp_idx);
                        navp_latitude = navp_latitude_raw;
                        navp_longitude = navp_longitude_raw;
                        gps_navp_maxdist = -1;
                        gps_navp_mindist = -1;
                        if ((navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                            geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw, navp_longitude_raw, gps_navp_dist_raw, gps_navp_azimuth_raw);
                            gps_navp_dist = gps_navp_dist_raw;
                            gps_navp_azimuth = gps_navp_azimuth_raw;
                        } else {
                            gps_navp_dist = gps_navp_dist_raw = -1;
                            gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
                        }
                    }
                } else {
                    gps_navp_dist = gps_navp_dist_raw = -1;
                    gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
                }
            } else {
                gps_navp_dist = gps_navp_dist_raw = -1;
                gps_navp_azimuth_raw = gps_navp_azimuth = ANGLE_NONE;
            }
        }

        // calculate maxdist
        if (gps_navp_dist_raw >= 0) {
            if ((gps_navp_maxdist < 0) || (gps_navp_maxdist < gps_navp_dist_raw)) {
                gps_navp_maxdist = gps_navp_dist_raw;
            }
            if ((gps_navp_mindist < 0) || (gps_navp_mindist > gps_navp_dist_raw)) {
                gps_navp_mindist = gps_navp_dist_raw;
            }
        }

        // navigation point changed? recalculate route
        int nmap_flags = 0;
#ifndef ISTRO_NAVMAP_NO_ROUTING
        if (nmap_np_idx != navp_idx) {
            if ((navp_idx >= 0) && (gps_fix > 0) && (navp_latitude_raw < ANGLE_OK) && (navp_longitude_raw < ANGLE_OK)) {
                nmap_flags += 20;
                nmap_np_idx = navp_idx;
                // plan route
                LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"nmap: planning new route...\", nmap_np_idx=" << nmap_np_idx << ", fix=" << gps_fix
                    << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                    << ", navp_latitude_raw=" << ioff(navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw, 6));
                navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw, navp_longitude_raw);
            } else {    // if (navp_idx < 0)
                if (nmap_np_idx >= 0) {
                    nmap_flags += 40;
                    nmap_np_idx = -1;
                    LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"nmap: deleting route...\", nmap_np_idx=" << nmap_np_idx);
                    navmap_plan_delete();
                }
            }
        }
        // route matching: replace gps and navigation point coordinates with points on route
        if ((nmap_np_idx >= 0) && (gps_fix > 0)) {
            int sidx = -1;
            double llat, llon;

            nmap_flags += 1;

            double dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);

            if (dd2 > NAVMAP_REROUTE_DIST * NAVMAP_REROUTE_DIST) {
                nmap_flags += 10;
                LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"nmap: rerouting...\", dd2=" << ioff(dd2, 3)
                    << ", nmap_np_idx=" << nmap_np_idx << ", fix=" << gps_fix
                    << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                    << ", navp_latitude_raw=" << ioff(navp_latitude_raw, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw, 6));
                navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw, navp_longitude_raw);

                dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);
            }

            if ((sidx >= 0) && (llat < ANGLE_OK) && (llon < ANGLE_OK)) {
                nmap_flags += 2;
                gps_latitude = llat;
                gps_longitude = llon;
                navigation_getXY(gps_latitude, gps_longitude, gps_x, gps_y);
            }

            double nlat, nlon;

            int res = navmap_routeFwd(gps_latitude, gps_longitude, NAVMAP_ROUTEFWD_DIST, nlat, nlon);

            if ((res >= 0) && (nlat < ANGLE_OK) && (nlon < ANGLE_OK)) {
                nmap_flags += 4;
                double ndist, nazimuth;
                navp_latitude = nlat;
                navp_longitude = nlon;
                geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude, navp_longitude, ndist, nazimuth);
                // distance to navigation point is real (could be replaced with route length), azimuth is changed
                gps_navp_azimuth = nazimuth;
            }
        }
#endif
#endif  // MIGRATED_TO_NAVIGATION_NODE

        timeEnd("istro::gps_thread.capture", t);

        if (gps_writeData(gps_time, gps_fix, gps_latitude, gps_longitude, gps_latitude_raw, gps_longitude_raw,
                gps_speed, gps_course, gps_lastp_dist, gps_lastp_azimuth, total_dist) < 0) {
            result = -4;
            break;
        }

        publishGpsData(gps_fix, gps_latitude_raw, gps_longitude_raw, gps_speed, gps_course);

        LOGM_DEBUG(loggerIstroGps2, "gps_thread", "msg=\"data updated\"");
    } while(0);

    if (result >= 0) {
        LOGM_INFO(loggerIstroGps2, "gps_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstroGps2, "gps_thread", "msg=\"exit(" << result << ")\"");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_gps(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroGps2, "main", "msg=\"istro_main_gps() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<GpsNode>();
    rclcpp::spin(node);

    istro_close_gps();

    rclcpp::shutdown();
    return 0;
}
