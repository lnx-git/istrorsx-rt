#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "threads.h"
#include "config.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "geocalc.h"
#include "wmodel.h"
#include "istrorsx_core/istrobtx/istro_main_save.h"
#include "istrorsx_core/istrobtx/navig.h"
#include "istrorsx_core/istrobtx/vision.h"
#include "istrorsx_core/istrobtx/navmap.h"

using namespace std;

LOG_DEFINE(loggerIstroSave1, "istroSave");

Config conf;
GeoCalc geoCalc;
Threads threads;

// WorldModel's own constructor already allocates+initializes its grid
// (WorldModel::WorldModel() -> pgrid = new WMGrid(); init();) -- no explicit
// init() call needed here, same as istro_main_planner.cpp's own wmodel.
WorldModel wmodel;

Vision vision;

long save_rand = 0;

const int DATASET_NUM = THDATA_NUM;
DataSet dataset[DATASET_NUM];

// Same fixed calibration samples as istro_main_vision.cpp's own
// loadSampleOnRoad()/loadSampleOffRoad() -- duplicated here (not shared)
// since each process needs its own Vision instance populated identically;
// see that file's own copy for why these exact sample points, unchanged.
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
        LOGM_ERROR(loggerIstroSave1, "initVision", "error loading sample1!");
        return -1;
    }
    if (loadSampleOffRoad(sampleOffRoad) < 0) {
        LOGM_ERROR(loggerIstroSave1, "initVision", "error loading sample2!");
        return -1;
    }

    LOGM_INFO(loggerIstroSave1, "initVision", "msg=\"vision.init()\"");
    vision.init(sampleOnRoad, sampleOffRoad);

    return 0;
}

// initVision() runs unconditionally at startup, matching every other node's
// own initVision()/initDevices() -- not gated behind conf.useCamera (that
// flag means "this process owns a physical camera", not applicable to
// save_node either).
int initDevices(void)
{
    if (initVision() < 0)
        return -6;

    // geoCalc.init() + navigation_init(&geoCalc) MUST run before navmap_init()
    // -- found missing via a real, silent bug: navmap_init()'s own
    // navmap_calcXY() (and later, every navmap_draw() call's own midpoint
    // conversion) call navigation_getXY(), which returns x=y=0 for *every*
    // input until navig_ref_latitude/navig_ref_longitude have been set by
    // navigation_init() -- with every node and the midpoint alike collapsing
    // to (0,0), every navMapSegment line drawn in navmap.png degenerates to a
    // single point at the image's exact center, invisible against the
    // background. navmap_export_kml() never hit this (it writes raw lat/lon
    // straight from navMapNode[]/navigationPoint[], no XY conversion at all),
    // which is exactly why the KML export looked correct while the PNG was
    // silently empty -- an earlier version of this function had reasoned
    // save_node only needs navmap_init()'s compile-time route graph, not
    // navigation_node's own live route-matching state, and skipped
    // navigation_init() entirely on that basis; true for the route-matching
    // part, but missed that navmap_calcXY()/navmap_draw() both depend on
    // navigation_init() regardless. Matches istro_main_navigation.cpp's own
    // ordering exactly (geoCalc.init() -> navigation_init(&geoCalc) ->
    // navmap_init()).
    if (geoCalc.init() < 0)
        return -9;

    if (navigation_init(&geoCalc) < 0)
        return -10;

    if (navmap_init() < 0)
        return -11;

    if (threads.init(&dataset[0], &dataset[1], &dataset[2], &dataset[3], &dataset[4], &dataset[5], &dataset[6],
            &dataset[7], &dataset[8], &dataset[9], &dataset[10], &dataset[11], &dataset[12], &dataset[13], &dataset[14]) < 0)
        return -12;

    // one dataset will be permanently set to state "THDATA_STATE_SHARED"
    threads.getData(THDATA_STATE_NEW, THDATA_STATE_SHARED);

    return 0;
}

void closeDevices()
{
    LOGM_INFO(loggerIstroSave1, "closeDevices", "msg=\"start\"");

    threads.close();

    // vision has no close() (matches every other node's own Vision instance)

    LOGM_INFO(loggerIstroSave1, "closeDevices", "msg=\"finish\"");
}

void waitForStart(void)
{
    if(!conf.useNoWait) {
        LOGM_INFO(loggerIstroSave1, "waitForStart", "msg=\"start sleep...\"");

        sleep(conf.waitDelay);
        LOGM_INFO(loggerIstroSave1, "waitForStart", "msg=\"wake up!\"");
    }

    if(conf.useStartTime) {
        time_t rawtime;
        struct tm * ptm;
        LOGM_INFO(loggerIstroSave1, "waitForStart", "msg=\"start time sleep...\"");
        while(1) {
            time ( &rawtime );
            ptm = localtime ( &rawtime );
            LOGM_INFO(loggerIstroSave1, "waitForStart", "time: " << iozf((ptm->tm_hour)%24, 2) << ":" << iozf(ptm->tm_min, 2) << ":" << iozf(ptm->tm_sec, 2));
            if(((ptm->tm_hour)%24 == conf.startHour) && (ptm->tm_min == conf.startMinute)) {
                break;
            }
            sleep(15);
        }
        LOGM_INFO(loggerIstroSave1, "waitForStart", "msg=\"wake up!\"");
    }
}

int istro_main_save(int argc, char** argv)
{
    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstroSave1, "-----------------------------");
    LOGM_INFO(loggerIstroSave1, "main", "msg=\"application start\"");

    if (conf.parseArguments(argc, argv) < 0) {
        LOGM_ERROR(loggerIstroSave1, "main", "msg=\"conf.parseArguments() failed!\"");
        return -1;
    }
    conf.printArguments();

    // Legacy main(): "srand(time(NULL)); save_rand = rand() % 1000 + ...".
    srand(time(NULL));
    save_rand = rand() % 1000 + (rand() % 1000) * 1000 + (1 + rand() % 9) * 1000000;

    if (initDevices() < 0) {
        LOGM_ERROR(loggerIstroSave1, "main", "msg=\"initDevices() failed!\"");
        return -1;
    }

    waitForStart();

    return 0;
}

// Everything that must run at application shutdown -- after rclcpp::spin()
// returns, not before it. See istro_main_ctrlboard.cpp / 01_architecture.md
// § Init / Shutdown Split.
void istro_close_save(void)
{
    closeDevices();

    LOGM_INFO(loggerIstroSave1, "main", "msg=\"application exit\"");
    LOG_DESTROY();
}
