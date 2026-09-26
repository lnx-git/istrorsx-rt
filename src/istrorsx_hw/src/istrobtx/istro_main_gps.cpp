#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "threads.h"
#include "config.h"
#include "system.h"
#include "logger.h"
#include "gpsdev.h"
#include "geocalc.h"
#include "mtime.h"
#include "dataset.h"
#include "istro_main_gps.h"

using namespace std;

LOG_DEFINE(loggerIstroGps1, "istroGps");

Config conf;
GpsDevice gps;
GeoCalc geoCalc;
Threads threads;

int         gps_pt_cnt = 0;
aux_point_t gps_pt[GPS_POINT_NUM + 1];

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

int initDevices(void)
{
    if (conf.useGPSDevice) {
        // fixme
        gps_pt[0].style[0] = 0;
        if (gps.init() < 0)
            return -7;
        if (geoCalc.init() < 0)
            return -8;
    }

    if (threads.init(&dataset[0], &dataset[1], &dataset[2], &dataset[3], &dataset[4], &dataset[5], &dataset[6],
            &dataset[7], &dataset[8], &dataset[9], &dataset[10], &dataset[11], &dataset[12], &dataset[13], &dataset[14]) < 0)
        return -12;

    // one dataset will be permanently set to state "THDATA_STATE_SHARED"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_SHARED);

    return 0;
}

void closeDevices()
{
    LOGM_INFO(loggerIstroGps1, "closeDevices", "msg=\"start\"");

    threads.close();

    if (conf.useGPSDevice) {
       geoCalc.close();
       gps.close();
    }

    LOGM_INFO(loggerIstroGps1, "closeDevices", "msg=\"finish\"");
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstroGps1, "waitForStart", "msg=\"start sleep...\"");

        sleep(conf.waitDelay);
        LOGM_INFO(loggerIstroGps1, "waitForStart", "msg=\"wake up!\"");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstroGps1, "waitForStart", "msg=\"start time sleep...\"");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstroGps1, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstroGps1, "waitForStart", "msg=\"wake up!\"");
    }
}

int loop(void)
{
    LOGM_INFO(loggerIstroGps1, "loop", "msg=\"start\"");

    /* ... */

    LOGM_INFO(loggerIstroGps1, "loop", "msg=\"exit(0)\"");

    return 0;
}

int istro_main_gps(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstroGps1, "-----------------------------");
    LOGM_INFO(loggerIstroGps1, "main", "msg=\"application start\"");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstroGps1, "main", "msg=\"conf.parseArguments() failed!\"");
        return -1;
    }
    conf.printArguments();

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstroGps1, "main", "msg=\"initDevices() failed!\"");
        return -1;
    }

    waitForStart();

    loop();

    return 0;
}

// Everything that must run at application shutdown -- after rclcpp::spin()
// returns, not before it. See istro_main_ctrlboard.cpp / 01_architecture.md
// § Init / Shutdown Split for why: closing here (instead of at the end of
// istro_main_gps(), like the legacy single-process app did) keeps gps open
// for the entire lifetime of the ROS node.
void istro_close_gps(void)
{
    closeDevices();

    LOGM_INFO(loggerIstroGps1, "main", "msg=\"application exit\"");
    LOG_DESTROY();
}
