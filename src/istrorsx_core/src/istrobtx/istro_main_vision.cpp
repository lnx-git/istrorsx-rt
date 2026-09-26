#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "threads.h"
#include "config.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "istrorsx_core/istrobtx/istro_main_vision.h"
#include "istrorsx_core/istrobtx/vision.h"
#include "istrorsx_core/istrobtx/qrscan.h"
#ifdef ISTRO_VISIONN
#include "istrorsx_core/istrobtx/visionn.h"
#endif
#ifdef ISTRO_VISION_DEPTH
#include "istrorsx_core/istrobtx/vision_depth.h"
#endif

using namespace std;

LOG_DEFINE(loggerIstroVision1, "istroVision");

Config conf;
Threads threads;

Vision vision;
#ifdef ISTRO_VISIONN
VisioNN visionn;
#endif
#ifdef ISTRO_VISION_DEPTH
Vision visiond_vsn(VISIOND_EP_SIZE, VISIOND_EP_THRESHOLDM);  // specialna instancia vision s inymi parametrami size a threshold
VisionDepth visiond;
#endif
QRScanner qrscan;

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

#ifdef ISTRO_VISION_ORANGECONE

int loadSampleOnRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(313, 341)) < 0) return -1;
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(249, 328)) < 0) return -1;
    if (sample.addSample("sample/2009628_kuzele.jpg", Point(378, 323)) < 0) return -1;
    if (sample.addSample("sample/2009591_kuzele.jpg", Point(214, 78)) < 0) return -1;
    if (sample.addSample("sample/2009591_kuzele.jpg", Point(330, 213)) < 0) return -1;
    if (sample.addSample("sample/2007147_kuzele.jpg", Point(409, 220)) < 0) return -1;
    if (sample.addSample("sample/2007147_kuzele.jpg", Point(470, 190)) < 0) return -1;
    if (sample.addSample("sample/2010533_kuzele.jpg", Point(428, 217)) < 0) return -1;
    if (sample.addSample("sample/2021474_kuzele.jpg", Point(142, 354)) < 0) return -1;
    if (sample.addSample("sample/2010509_kuzele.jpg", Point(455, 219)) < 0) return -1;
    if (sample.addSample("sample/2013222_kuzele.jpg", Point(314, 163)) < 0) return -1;
    if (sample.addSample("sample/2008241_kuzele.jpg", Point(566, 303)) < 0) return -1;
    return 0;
}

int loadSampleOffRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/2004792_kuzele.jpg", Point(472, 403)) < 0) return -1;
    if (sample.addSample("sample/2004731_kuzele.jpg", Point(388, 245)) < 0) return -1;
    if (sample.addSample("sample/2013376_kuzele2.jpg", Point(201, 137)) < 0) return -1;
    if (sample.addSample("sample/2002698_kuzele.jpg", Point(284, 80)) < 0) return -1;
    if (sample.addSample("sample/2006395_kuzele.jpg", Point(320, 87)) < 0) return -1;
    if (sample.addSample("sample/2014689_kuzele.jpg", Point(148, 267)) < 0) return -1;
    return 0;
}

#else

int loadSampleOnRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(320, 370)) < 0) return -1;
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(35, 363)) < 0) return -1;
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(172, 443)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(266, 268)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(155, 349)) < 0) return -1;
    if (sample.addSample("sample/0001179_cesta_vlavo.jpg", Point(531, 334)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(366, 359)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(250, 400)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(297, 356)) < 0) return -1;
    if (sample.addSample("sample/0000021_cesta_proti_slnku.jpg", Point(361, 340)) < 0) return -1;
    if (sample.addSample("sample/0000340_cesta.jpg", Point(262, 357)) < 0) return -1;
    if (sample.addSample("sample/0104747.jpg", Point(347, 236)) < 0) return -1;
    if (sample.addSample("sample/0104747.jpg", Point(549, 397)) < 0) return -1;
    if (sample.addSample("sample/0106832.jpg", Point(256, 211)) < 0) return -1;
    if (sample.addSample("sample/0107408.jpg", Point(280, 251)) < 0) return -1;
    if (sample.addSample("sample/0107630.jpg", Point(171, 366)) < 0) return -1;
    if (sample.addSample("sample/0108350.jpg", Point(163, 236)) < 0) return -1;
    if (sample.addSample("sample/0107775.jpg", Point(559, 373)) < 0) return -1;

    if (sample.addSample("sample/0201759_rr2017.jpg", Point(376, 393)) < 0) return -1;
//  if (sample.addSample("sample/0200409_rr2017.jpg", Point(365, 407)) < 0) return -1;
    if (sample.addSample("sample/0200412_rr2017.jpg", Point(309, 404)) < 0) return -1;
    if (sample.addSample("sample/0200418_rr2017.jpg", Point(351, 361)) < 0) return -1;
    if (sample.addSample("sample/0200086_rr2017.jpg", Point(536, 438)) < 0) return -1;

    if (sample.addSample("sample/0320078_rt2018.jpg", Point(25, 442)) < 0) return -1;
    if (sample.addSample("sample/0405484_rr2019.jpg", Point(238, 109)) < 0) return -1;
    if (sample.addSample("sample/0405484_rr2019.jpg", Point(203, 93)) < 0) return -1;
    if (sample.addSample("sample/0400864_rr2019.jpg", Point(21, 401)) < 0) return -1;

    return 0;
}

int loadSampleOffRoad(SamplePixels &sample)
{
    if (sample.addSample("sample/0001032_cesta_oblacno.jpg", Point(20, 200)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(235, 110)) < 0) return -1;
    if (sample.addSample("sample/0001126_cesta_vpravo.jpg", Point(103, 176)) < 0) return -1;
    if (sample.addSample("sample/0001179_cesta_vlavo.jpg", Point(4, 105)) < 0) return -1;

/*
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(342, 120)) < 0) return -1;
    if (sample.addSample("sample/0001210_trava_zelena.jpg", Point(468, 73)) < 0) return -1;
//  if (sample.addSample("sample/0001227_trava_modra.jpg", Point(459, 246)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(499, 100)) < 0) return -1;
    if (sample.addSample("sample/0000013_preexponovane.jpg", Point(491, 70)) < 0) return -1;
    if (sample.addSample("sample/0000130_trava.jpg", Point(145, 101)) < 0) return -1;
    if (sample.addSample("sample/0000189_cesta.jpg", Point(605, 234)) < 0) return -1;

    if (sample.addSample("sample/0409562_rr2019.jpg", Point(454, 207)) < 0) return -1;
    if (sample.addSample("sample/0411049_rr2019.jpg", Point(539, 147)) < 0) return -1;
*/

#ifdef ISTRO_VISION_YELLOWLANE
    if (sample.addSample("sample/1013001_zlta.jpg", Point(77, 348)) < 0) return -1;
    if (sample.addSample("sample/1013133_zlta.jpg", Point(145, 428)) < 0) return -1;
    if (sample.addSample("sample/1013324_zlta.jpg", Point(390, 356)) < 0) return -1;
    if (sample.addSample("sample/1014326_zlta.jpg", Point(191, 326)) < 0) return -1;
    if (sample.addSample("sample/1014779_zlta.jpg", Point(208, 306)) < 0) return -1;
    if (sample.addSample("sample/1014888_zlta.jpg", Point(235, 420)) < 0) return -1;
    if (sample.addSample("sample/1004748_zlta.jpg", Point(14, 358)) < 0) return -1;
#endif

    return 0;
}

#endif

int initVision(void)
{
    SamplePixels sampleOnRoad;
    SamplePixels sampleOffRoad;

    if (loadSampleOnRoad(sampleOnRoad) < 0) {
        LOGM_ERROR(loggerIstroVision1, "initVision", "error loading sample1!");
        return -1;
    }
    if (loadSampleOffRoad(sampleOffRoad) < 0) {
        LOGM_ERROR(loggerIstroVision1, "initVision", "error loading sample2!");
        return -1;
    }

    LOGM_INFO(loggerIstroVision1, "initVision", "msg=\"vision.init()\"");
    vision.init(sampleOnRoad, sampleOffRoad);

#ifdef ISTRO_VISION_DEPTH
    LOGM_INFO(loggerIstroVision1, "initVision", "msg=\"visiond_vsn.init()\"");
    visiond_vsn.init(sampleOnRoad, sampleOffRoad);
#endif

    return 0;
}

#ifdef ISTRO_VISIONN
int initVisioNN(void)
{
    if (visionn.init() < 0) {
        LOGM_ERROR(loggerIstroVision1, "initVisioNN", "error: -1!");
        return -1;
    }

    if (visionn.send_hello() < 0) {
        LOGM_ERROR(loggerIstroVision1, "initVisioNN", "error: -2!");
        return -2;
    }

    if (visionn.recv_hello() < 0) {
        LOGM_ERROR(loggerIstroVision1, "initVisioNN", "error: -3!");
        return -3;
    }

    return 0;
}
#endif

// initVision()/initVisioNN()/visiond.init() run unconditionally at startup,
// matching istro_rt2025.cpp's main() -- not gated behind conf.useCamera
// (that flag meant "this process owns a physical camera", which is now
// camera_node's concern, not vision_node's; see istro_main_navigation.cpp's
// analogous conf.useGPSDevice/conf.useNavigation note). conf.useQRScan is
// kept, unlike conf.useCamera -- it's not a hardware-ownership flag, it's
// the same runtime QR-scan on/off switch vision_thread itself checked.
int initDevices(void)
{
    if (initVision() < 0)
        return -6;

#ifdef ISTRO_VISIONN
    if (initVisioNN() < 0)
        return -13;
#endif

#ifdef ISTRO_VISION_DEPTH
    if (visiond.init() < 0)
        return -14;
#endif

    if (conf.useQRScan) {
        qrscan.init();
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
    LOGM_INFO(loggerIstroVision1, "closeDevices", "msg=\"start\"");

    threads.close();

#ifdef ISTRO_VISION_DEPTH
    visiond.close();
#endif
#ifdef ISTRO_VISIONN
    visionn.close();
#endif
    // vision/visiond_vsn/qrscan have no close() (matches legacy -- "vision.close();" is commented out there too)

    LOGM_INFO(loggerIstroVision1, "closeDevices", "msg=\"finish\"");
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstroVision1, "waitForStart", "msg=\"start sleep...\"");

        sleep(conf.waitDelay);
        LOGM_INFO(loggerIstroVision1, "waitForStart", "msg=\"wake up!\"");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstroVision1, "waitForStart", "msg=\"start time sleep...\"");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstroVision1, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstroVision1, "waitForStart", "msg=\"wake up!\"");
    }
}

int istro_main_vision(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstroVision1, "-----------------------------");
    LOGM_INFO(loggerIstroVision1, "main", "msg=\"application start\"");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstroVision1, "main", "msg=\"conf.parseArguments() failed!\"");
        return -1;
    }
    conf.printArguments();

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstroVision1, "main", "msg=\"initDevices() failed!\"");
        return -1;
    }

    waitForStart();

    return 0;
}

// Everything that must run at application shutdown -- after rclcpp::spin()
// returns, not before it. See istro_main_ctrlboard.cpp / 01_architecture.md
// § Init / Shutdown Split.
void istro_close_vision(void)
{
    closeDevices();

    LOGM_INFO(loggerIstroVision1, "main", "msg=\"application exit\"");
    LOG_DESTROY();
}
