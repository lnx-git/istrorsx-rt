/***********************************************************************************************************************
<program name> -cb <control_board_device> -nogps -lidar <lidar_device> -ahrs <arhs_device> -nosave -nowait 
               -h <start_hour> -m <start_min> -cg <gps_azimuth> -ca <ahrs_yaw> -navy <yaw> -path <PxPyPz>
example:
istro_rt2025 -cb /dev/ttyUSB0 -gps /dev/ttyUSB1 -lidar /dev/ttyUSB2 -ahrs /dev/ttyACM0 -nowait
***********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
//#include <time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
//#include <exception>
//#include <string.h>
#include "threads.h"
#include "config.h"
#include "system.h"
#include "logger.h"
#include "ctrlboard.h"
#include "mtime.h"
#include "dataset.h"

using namespace cv;
using namespace std;

LOG_DEFINE(loggerIstro1, "istroCBoard");

Config conf;
ControlBoard ctrlBoard;
Threads threads;

long save_rand = 0;

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

int initDevices(void)
{
    srand (time(NULL));
    save_rand = rand() % 1000 + (rand() % 1000) * 1000 + (1 + rand() % 9)*1000000;

    if (conf.useControlBoard) {
        if (conf.useControlBoard2) {
            if (ctrlBoard.init(conf.ControlBoardPortName, conf.ControlBoard2PortName) < 0)
                return -2;
        } else {
            if (ctrlBoard.init(conf.ControlBoardPortName, NULL) < 0)
                return -3;
        }
    }

    /* ... */

    if (threads.init(&dataset[0], &dataset[1], &dataset[2], &dataset[3], &dataset[4], &dataset[5], &dataset[6], 
            &dataset[7], &dataset[8], &dataset[9], &dataset[10], &dataset[11], &dataset[12], &dataset[13], &dataset[14]) < 0) 
        return -12;

    // one dataset will be permanently set to state "THDATA_STATE_SHARED"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_SHARED);

    // another one dataset will be permanently set to state "THDATA_STATE_CTRLBOARD"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_CTRLBOARD);

    return 0;
}

void closeDevices()
{
    LOGM_INFO(loggerIstro1, "closeDevices", "msg=\"start\"");

    threads.close();

    /* ... */

    if (conf.useControlBoard) {
       ctrlBoard.close();
    }

    LOGM_INFO(loggerIstro1, "closeDevices", "msg=\"finish\"");
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstro1, "waitForStart", "msg=\"start sleep...\"");

        sleep(conf.waitDelay);
        LOGM_INFO(loggerIstro1, "waitForStart", "msg=\"wake up!\"");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstro1, "waitForStart", "msg=\"start time sleep...\"");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstro1, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstro1, "waitForStart", "msg=\"wake up!\"");
    }    
}

int loop(void)
{
    LOGM_INFO(loggerIstro1, "loop", "msg=\"start\"");

    // Ported from legacy's loop() opening (istro_rt2025.cpp:4276). Both halves are
    // cosmetic, not protocol: sleep(3) is "don't lurch at startup" (and only delays
    // this node now, not the robot), -i "DIstrobotics" just writes to the display.
    if (conf.useControlBoard) {
        sleep(3);
        if (conf.ControlBoardInitStr) {
            ctrlBoard.setLedProgram(1);
            ctrlBoard.writeString(conf.ControlBoardInitStr);
            LOGM_INFO(loggerIstro1, "loop", "msg=\"control board init sent\", init_str=\""
                << conf.ControlBoardInitStr << "\"");
        }
    }

    LOGM_INFO(loggerIstro1, "loop", "msg=\"exit(0)\"");

    return 0;
}

int istro_main_ctrlboard(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstro1, "-----------------------------");
    LOGM_INFO(loggerIstro1, "main", "msg=\"application start\"");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstro1, "main", "msg=\"conf.parseArguments() failed!\"");
        return -1;
    }
    conf.printArguments();

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstro1, "main", "msg=\"initDevices() failed!\"");
        return -1;
    }
    
    waitForStart();

    loop();

    return 0;
}

// Everything that must run at application shutdown -- after rclcpp::spin()
// returns, not before it. Calling closeDevices() here (instead of at the end
// of istro_main_ctrlboard(), like the original single-process legacy app did)
// keeps ctrlBoard open for the entire lifetime of the ROS node.
void istro_close_ctrlboard(void)
{
    // Ported from the tail of legacy's loop() (istro_rt2025.cpp:4564). Belongs here
    // and not in drive_node: Ctrl+C hits every process at once, so a SpeedCommand
    // would be a race. sleep(1) lets the board act before close(). See 01_architecture.md.
    if (conf.useControlBoard) {
        ctrlBoard.setSpeed(VEL_ZERO);
        ctrlBoard.setSteeringAngle(SA_STRAIGHT);
        ctrlBoard.displayText("bye...");
        LOGM_INFO(loggerIstro1, "istro_close_ctrlboard", "msg=\"stop sent to control board\", speed="
            << VEL_ZERO << ", angle=" << SA_STRAIGHT);
        sleep(1);
    }

    closeDevices();

    LOGM_INFO(loggerIstro1, "main", "msg=\"application exit\"");
    LOG_DESTROY();
}
