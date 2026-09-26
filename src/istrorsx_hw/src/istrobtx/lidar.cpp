#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "lidar.h"
#include "ctrlboard.h"
#include "system.h"
#include "mtime.h"
#include "dmap.h"
#include "logger.h"

// LIDAR_DISTANCE_MIN/DMAP/STOP/MAX, LIDAR_QUALITY_MIN/DMAP/STOP/MAX (incl.
// the ISTRO_LIDAR_FILTERSUN variant), LIDAR_STOP_ANGLE_MIN/MAX, and
// LIDAR_STOP_COUNT moved to istrobtx's lidar_defs.h (header-only constants,
// like config.h's ANGLE_NONE/ANGLE_OK) -- shared with lidar_defs.cpp's
// lidar_process(). LIDAR_DISTANCE_MAXD/LIDAR_QUALITY_MAXD/outputFName (all
// drawing-only) moved there too, along with drawOutput() itself -- see
// lidar.h's own note on why.

LOG_DEFINE(loggerLidar, "Lidar");

#ifdef ISTRO_RPLIDAR

// ---------------------------------------------------------------------------
// Ported to Slamtec SDK v2.x (see script/install/rplidar_install.sh /
// doc/ai/04_installation.md). API changes versus the original SDK v1.6.1 port:
//   - RPlidarDriver::CreateDriver()/DisposeDriver()   -> sl::createLidarDriver(), delete drv
//   - drv->connect(port, baudrate) directly            -> sl::createSerialPortChannel() + drv->connect(channel)
//   - IS_OK()/IS_FAIL()                                -> SL_IS_OK()/SL_IS_FAIL()
//   - u_result, rplidar_response_device_health_t       -> sl_result, sl_lidar_response_device_health_t
//   - RPLIDAR_STATUS_ERROR                             -> SL_LIDAR_STATUS_ERROR
//   - drv->startMotor()                                -> drv->setMotorSpeed() (default arg starts it)
//   - drv->grabScanData() + rplidar_response_measurement_node_t (6-bit quality
//     packed with the sync bit) -> drv->grabScanDataHq() + sl_lidar_response_measurement_node_hq_t
//     (separate `flag`/`quality` fields, quality now 0-255). getData() below
//     converts the hq fields back to the same value ranges the original code
//     produced (angle in degrees, distance in mm, quality 0-63), verified
//     against the SDK's own internal sl::convert() helper (sl_lidar_driver.cpp)
//     so the LIDAR_QUALITY_*/LIDAR_DISTANCE_* thresholds below stay valid unchanged.
//   - startScan(0,1) / startScan() (old V106 vs. pre-V106 split) -> startScan(false, false)
//     (single unified API in the new SDK, no more ISTRO_RPLIDAR_V106 branch
//     needed -- useTypicalScan MUST be false on this hardware, see the
//     comment at the startScan() call site in init() below for why)
// (lidar_rplidar.cpp/.h, an unused duplicate of this file/lidar.h in the
// legacy source dump, was not carried over into this package -- only
// lidar.cpp/lidar.h were actually compiled by the legacy CMakeLists.txt.)
// ---------------------------------------------------------------------------

int Lidar::init(const char *portName)
{
    LOGM_INFO(loggerLidar, "init", "Lidar::init");

    const char * opt_com_path = portName;
    sl_u32       opt_com_baudrate = 115200;

    auto driverResult = createLidarDriver();
    if (!driverResult) {
        LOGM_ERROR(loggerLidar, "init", "insufficent memory!");
        return -1;
    }
    drv = *driverResult;

    // make connection...
    auto channelResult = createSerialPortChannel(opt_com_path, opt_com_baudrate);
    if (!channelResult) {
        LOGM_ERROR(loggerLidar, "init", "cannot create serial channel for \"" << opt_com_path << "\"");
        delete drv;
        drv = NULL;
        return -2;
    }
    channel = *channelResult;

    if (SL_IS_FAIL(drv->connect(channel))) {
        LOGM_ERROR(loggerLidar, "init", "cannot bind to serial port \"" << opt_com_path << "\"");
        delete drv;
        drv = NULL;
        delete channel;
        channel = NULL;
        return -2;
    }

    // check health...
    if (!checkHealth()) {
        LOGM_ERROR(loggerLidar, "init", "checkHealth failed!");
        delete drv;
        drv = NULL;
        delete channel;
        channel = NULL;
        return -3;
    }

    drv->setMotorSpeed();  // ported from startMotor() -- default arg (DEFAULT_MOTOR_SPEED) starts the motor at the standard speed, same as the old startMotor()

    // start scan... (force=false, useTypicalScan=false -- MUST be false on this
    // hardware: this RPLIDAR A1 runs firmware 1.15, older than express-scan
    // support (added in fw 1.17). SDK v2.x's getTypicalScanMode() hardcodes
    // EXPRESS_SCAN for any firmware without the GET_LIDAR_CONF protocol
    // (fw < 1.24) regardless of what the device actually supports -- so
    // useTypicalScan=true sends an EXPRESS_SCAN command (0x82) this lidar
    // silently doesn't answer (grabScanDataHq() then spins with no data,
    // even though connect()/getHealth() succeed, since those use older
    // commands). useTypicalScan=false forces the legacy plain SCAN command
    // (0x20), which this firmware does support. SDK 1.12.0 used to check
    // this via checkExpressScanSupported() and fall back automatically;
    // that fallback was removed in SDK v2.x.
    sl_result op_result = drv->startScan(false, false);
    if (SL_IS_FAIL(op_result)) {
        LOGM_ERROR(loggerLidar, "init", "start scan failed!, res=" << (int)op_result);
        delete drv;
        drv = NULL;
        delete channel;
        channel = NULL;
        return -4;
    }

#ifdef ISTRO_GUI
    namedWindow("Lidar", 0 );
    resizeWindow("Lidar", 640, 480);

    /* fixme: not calling imshow in Lidar::init() sometimes caused segmentation fault in Lidar::drawOutput() */
    Mat img;
    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    imshow("Lidar", img);
#endif

    return 0;
}

bool Lidar::checkHealth(void)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;
//int fixme_checkhealth;
//return true;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro SL_IS_OK is the preperred way to judge whether the operation is succeed.
        LOGM_TRACE(loggerLidar, "checkHealth", "status=" << (int)healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            LOGM_ERROR(loggerLidar, "checkHealth", "internal error detected, please reboot the device!");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        LOGM_ERROR(loggerLidar, "checkHealth", "cannot retrieve health code!, res=" << (int)op_result);
        return false;
    }
}

void Lidar::close(void)
{
    int fixme_stopmotor;

    //drv->stop();
    //drv->setMotorSpeed(0);  // ported from stopMotor()

    delete drv;
    drv = NULL;
    delete channel;
    channel = NULL;
}

int Lidar::getData(lidar_data_t *data, int& data_cnt)
{
    sl_result op_result;
    size_t count = 8192;  // LIDAR_DATA_NUM;
    sl_lidar_response_measurement_node_hq_t nodes[8192];  // LIDAR_DATA_NUM];
    int fixme_getdata_8192;

    data_cnt = -1;

    // Not part of the original SDK v1.6.1 code -- added because init() can now
    // leave drv == NULL after a failure (createLidarDriver()/connect()/
    // checkHealth()/startScan() all delete+null it on error), and lidar_node
    // keeps calling getData() on its timer regardless of whether init()
    // succeeded (same "runs regardless of init success" pattern gps_node/
    // ctrlboard_node already have). Without this guard, that would dereference
    // a NULL drv and crash instead of failing cleanly like GpsDevice::getData()
    // does when gpsd isn't running.
    if (!drv) {
        LOGM_ERROR(loggerLidar, "getData", "drv is NULL (Lidar not initialized)!");
        return -1;
    }

    op_result = drv->grabScanDataHq(nodes, count);
    if (!SL_IS_OK(op_result)) {
        LOGM_ERROR(loggerLidar, "getData", "grabScanDataHq failed!, res=" << (int)op_result);
        return -1;
    }

    drv->ascendScanData(nodes, count);

    if (count > LIDAR_DATA_NUM) {
        LOGM_WARN(loggerLidar, "getData", "count larger than 720!, count=" << (int)count);
        count = LIDAR_DATA_NUM;
    }

    for (int pos = 0; pos < (int)count; ++pos) {
        data->sync = nodes[pos].flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT;
        data->angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
        data->distance = nodes[pos].dist_mm_q2 / 4.0f;
        data->quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  // hq quality is 0-255 (pre-shifted); shift back down to the original 0-63 range
        data++;
    }

    data_cnt = (int)count;
    return 0;
}

#else

int Lidar::init(const char *portName) 
{
    LOGM_WARN(loggerLidar, "init", "MOCK LIDAR implementation, DEBUG ONLY!!");
    
#ifdef ISTRO_GUI    
    namedWindow("Lidar", 0 );
    resizeWindow("Lidar", 640, 480);

    /* fixme: not calling imshow in Lidar::init() sometimes caused segmentation fault in Lidar::drawOutput() */
    Mat img;
    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    imshow("Lidar", img);
#endif

    return 0; 
}

bool Lidar::checkHealth(void) 
{ 
    return true; 
}

void Lidar::close(void) 
{ 
}

int Lidar::getData(lidar_data_t *data, int& data_cnt)
{
    int i = 0;
    float f = 0;  // fix: 1000 = no obstacle

    data[i].sync = 0;  data[i].angle =   0.23;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   1.39;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   2.56;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   3.72;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   4.89;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 1;  data[i].angle =   6.06;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   7.22;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   8.39;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =   9.55;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  10.72;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  11.88;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  13.05;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  14.20;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  15.38;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  16.55;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  17.70;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  18.88;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  20.03;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  21.20;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  22.36;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  23.53;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  24.70;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  25.86;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  27.16;  data[i].distance = 00137.50+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  28.31;  data[i].distance = 00147.50+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  29.36;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  30.14;  data[i].distance = 00173.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  31.31;  data[i].distance = 00187.75+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  32.84;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  33.27;  data[i].distance = 00303.25+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  33.94;  data[i].distance = 00440.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  34.02;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  34.34;  data[i].distance = 00556.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  35.19;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  37.52;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle =  42.38;  data[i].distance = 00134.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  42.44;  data[i].distance = 00126.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  44.41;  data[i].distance = 00144.25+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  44.58;  data[i].distance = 00156.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  45.25;  data[i].distance = 00168.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  46.41;  data[i].distance = 00186.00+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  47.17;  data[i].distance = 00228.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  47.56;  data[i].distance = 00205.00+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  47.88;  data[i].distance = 00297.00+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  48.41;  data[i].distance = 00345.75+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  48.86;  data[i].distance = 00257.50+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  48.89;  data[i].distance = 00416.50+f;  data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  49.36;  data[i].distance = 00526.50;    data[i++].quality =  9;
    data[i].sync = 0;  data[i].angle =  56.16;  data[i].distance = 00129.75+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  58.48;  data[i].distance = 00149.00+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  58.72;  data[i].distance = 00138.75+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  59.64;  data[i].distance = 00161.50+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  60.30;  data[i].distance = 00177.00+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  61.47;  data[i].distance = 00194.25+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  61.80;  data[i].distance = 00214.00+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  62.80;  data[i].distance = 00273.25+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.20;  data[i].distance = 00318.75+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  63.36;  data[i].distance = 00374.25+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.36;  data[i].distance = 00241.25+f;  data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  63.69;  data[i].distance = 00465.75+f;  data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  64.09;  data[i].distance = 00600.00;    data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  64.58;  data[i].distance = 00888.50;    data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  65.06;  data[i].distance = 01683.00;    data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  72.67;  data[i].distance = 04191.75;    data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  73.62;  data[i].distance = 00000.00;    data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  73.86;  data[i].distance = 03831.25;    data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  74.80;  data[i].distance = 00000.00;    data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  75.06;  data[i].distance = 03455.25;    data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  75.95;  data[i].distance = 00000.00;    data[i++].quality = 11;
    data[i].sync = 0;  data[i].angle =  76.23;  data[i].distance = 03181.75;    data[i++].quality = 12;
    data[i].sync = 0;  data[i].angle =  77.12;  data[i].distance = 00000.00;    data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  77.45;  data[i].distance = 02916.00;    data[i++].quality = 14;
    data[i].sync = 0;  data[i].angle =  78.28;  data[i].distance = 00000.00;    data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  78.62;  data[i].distance = 02689.75;    data[i++].quality = 13;
    data[i].sync = 0;  data[i].angle =  79.45;  data[i].distance = 00000.00;    data[i++].quality = 10;
    data[i].sync = 0;  data[i].angle =  79.86;  data[i].distance = 02500.50;    data[i++].quality = 17;
    data[i].sync = 0;  data[i].angle =  81.06;  data[i].distance = 02351.25;    data[i++].quality = 16;
    data[i].sync = 0;  data[i].angle =  82.25;  data[i].distance = 02222.00;    data[i++].quality = 17;
    data[i].sync = 0;  data[i].angle =  83.47;  data[i].distance = 02093.75;    data[i++].quality = 18;
    data[i].sync = 0;  data[i].angle =  84.64;  data[i].distance = 01990.50;    data[i++].quality = 21;
    data[i].sync = 0;  data[i].angle =  85.83;  data[i].distance = 01893.00;    data[i++].quality = 18;
    data[i].sync = 0;  data[i].angle =  87.02;  data[i].distance = 01807.50;    data[i++].quality = 22;
    data[i].sync = 0;  data[i].angle =  88.27;  data[i].distance = 01731.00;    data[i++].quality = 21;
    data[i].sync = 0;  data[i].angle =  89.41;  data[i].distance = 01665.75;    data[i++].quality = 23;
    data[i].sync = 0;  data[i].angle =  90.61;  data[i].distance = 01591.75;    data[i++].quality = 27;
    data[i].sync = 0;  data[i].angle =  91.84;  data[i].distance = 01531.50;    data[i++].quality = 22;
    data[i].sync = 0;  data[i].angle =  93.02;  data[i].distance = 01472.00;    data[i++].quality = 25;
    data[i].sync = 0;  data[i].angle =  94.19;  data[i].distance = 01426.25;    data[i++].quality = 24;
    data[i].sync = 0;  data[i].angle =  95.39;  data[i].distance = 01383.75;    data[i++].quality = 27;
    data[i].sync = 0;  data[i].angle =  96.59;  data[i].distance = 01339.75;    data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle =  97.81;  data[i].distance = 01297.50;    data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle =  98.94;  data[i].distance = 01260.50;    data[i++].quality = 26;
    data[i].sync = 0;  data[i].angle = 100.14;  data[i].distance = 01226.75;    data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 101.39;  data[i].distance = 01196.75;    data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 102.59;  data[i].distance = 01168.50;    data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 103.77;  data[i].distance = 01139.25;    data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 104.91;  data[i].distance = 01111.75;    data[i++].quality = 28;
    data[i].sync = 0;  data[i].angle = 106.06;  data[i].distance = 01087.75;    data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 107.33;  data[i].distance = 01062.25;    data[i++].quality = 29;
    data[i].sync = 0;  data[i].angle = 108.52;  data[i].distance = 01042.75;    data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 109.69;  data[i].distance = 01022.75;    data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 110.84;  data[i].distance = 01004.25;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 112.11;  data[i].distance = 00984.75;    data[i++].quality = 30;
    data[i].sync = 0;  data[i].angle = 113.23;  data[i].distance = 00967.00;    data[i++].quality = 33;
    data[i].sync = 0;  data[i].angle = 114.50;  data[i].distance = 00951.75;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 115.69;  data[i].distance = 00935.50;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 116.81;  data[i].distance = 00921.00;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 117.98;  data[i].distance = 00907.75;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 119.14;  data[i].distance = 00894.75;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 120.45;  data[i].distance = 00882.25;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 121.56;  data[i].distance = 00873.25;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 122.70;  data[i].distance = 00861.25;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 124.02;  data[i].distance = 00850.00;    data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 125.14;  data[i].distance = 00840.75;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 126.27;  data[i].distance = 00833.50;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 127.44;  data[i].distance = 00824.50;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 128.66;  data[i].distance = 00816.25;    data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 129.78;  data[i].distance = 00809.25;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 131.03;  data[i].distance = 00803.00;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 132.16;  data[i].distance = 00795.50;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 133.39;  data[i].distance = 00789.25;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 134.50;  data[i].distance = 00785.00;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 135.67;  data[i].distance = 00778.00;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 136.86;  data[i].distance = 00774.00;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 138.17;  data[i].distance = 00769.25;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 139.28;  data[i].distance = 00766.00;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 140.47;  data[i].distance = 00761.00;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 141.77;  data[i].distance = 00757.50;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 142.78;  data[i].distance = 00756.75;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 144.12;  data[i].distance = 00752.25;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 145.17;  data[i].distance = 00748.75;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 146.33;  data[i].distance = 00748.25;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 147.64;  data[i].distance = 00745.75;    data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 148.80;  data[i].distance = 00744.50;    data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 149.92;  data[i].distance = 00744.25;    data[i++].quality = 43;
    data[i].sync = 0;  data[i].angle = 151.14;  data[i].distance = 00744.25;    data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 152.31;  data[i].distance = 00743.25;    data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 153.42;  data[i].distance = 00744.50;    data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 154.47;  data[i].distance = 00744.50;    data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 155.70;  data[i].distance = 00747.00;    data[i++].quality = 41;
    data[i].sync = 0;  data[i].angle = 156.88;  data[i].distance = 00746.50;    data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 158.02;  data[i].distance = 00748.50;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 159.28;  data[i].distance = 00749.50;    data[i++].quality = 39;
    data[i].sync = 0;  data[i].angle = 160.39;  data[i].distance = 00752.50;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 161.58;  data[i].distance = 00756.00;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 162.67;  data[i].distance = 00758.25;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 163.72;  data[i].distance = 00760.50;    data[i++].quality = 33;
    data[i].sync = 0;  data[i].angle = 164.92;  data[i].distance = 00764.00;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 166.06;  data[i].distance = 00766.50;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 167.28;  data[i].distance = 00771.50;    data[i++].quality = 40;
    data[i].sync = 0;  data[i].angle = 168.44;  data[i].distance = 00776.25;    data[i++].quality = 38;
    data[i].sync = 0;  data[i].angle = 169.55;  data[i].distance = 00781.50;    data[i++].quality = 36;
    data[i].sync = 0;  data[i].angle = 170.69;  data[i].distance = 00788.75;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 171.81;  data[i].distance = 00793.75;    data[i++].quality = 38+20;
    data[i].sync = 0;  data[i].angle = 173.08;  data[i].distance = 00799.50;    data[i++].quality = 39+20;
    data[i].sync = 0;  data[i].angle = 174.19;  data[i].distance = 00805.75;    data[i++].quality = 39+20;
    data[i].sync = 0;  data[i].angle = 175.36;  data[i].distance = 00812.75;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 176.48;  data[i].distance = 00819.00;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 177.53;  data[i].distance = 00828.25;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 178.67;  data[i].distance = 00836.00;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 179.81;  data[i].distance = 00846.50;    data[i++].quality = 37;
    data[i].sync = 0;  data[i].angle = 181.03;  data[i].distance = 00856.00;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 182.23;  data[i].distance = 00865.25;    data[i++].quality = 31;
    data[i].sync = 0;  data[i].angle = 183.28;  data[i].distance = 00875.50;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 184.53;  data[i].distance = 00886.00;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 185.77;  data[i].distance = 00895.50;    data[i++].quality = 35;
    data[i].sync = 0;  data[i].angle = 186.94;  data[i].distance = 00909.75;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 188.08;  data[i].distance = 00921.00;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 189.16;  data[i].distance = 00936.50;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 190.31;  data[i].distance = 00950.50;    data[i++].quality = 34;
    data[i].sync = 0;  data[i].angle = 191.53;  data[i].distance = 00964.00;    data[i++].quality = 32;
    data[i].sync = 0;  data[i].angle = 192.62;  data[i].distance = 00982.50;    data[i++].quality = 29;
    data[i].sync = 0;  data[i].angle = 193.75;  data[i].distance = 01026.00;    data[i++].quality = 14;
    data[i].sync = 0;  data[i].angle = 201.78;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 202.95;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 204.11;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 205.28;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 206.44;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 207.61;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 208.77;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 209.94;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 211.11;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 212.27;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 213.44;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 214.59;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 215.77;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 216.92;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 218.09;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 219.27;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 220.42;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 221.59;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 222.75;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 223.92;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 225.08;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 226.25;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 227.41;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 228.58;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 229.75;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 230.91;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 232.08;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 233.23;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 234.41;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 235.56;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 236.73;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 237.91;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 239.06;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 240.23;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 241.39;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 242.56;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 243.72;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 244.89;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 246.06;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 247.22;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 248.39;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 249.55;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 250.72;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 251.88;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 253.05;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 254.20;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 255.38;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 256.55;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 257.70;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 258.88;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 260.03;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 261.20;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 262.36;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 263.53;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 264.70;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 265.86;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 267.03;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 268.19;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 269.36;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 270.52;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 271.69;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 272.84;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 274.02;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 275.19;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 276.34;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 277.52;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 278.67;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 279.84;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 281.00;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 282.17;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 283.34;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 284.50;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 285.67;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 286.83;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 288.00;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 289.16;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 290.33;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 291.48;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 292.66;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 293.83;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 294.98;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 296.16;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 297.31;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 298.48;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 299.64;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 300.81;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 301.98;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 303.14;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 304.31;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 305.47;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 306.64;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 307.80;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 308.97;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 310.12;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 311.30;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 312.47;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 313.62;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 314.80;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 315.95;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 317.12;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 318.28;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 319.45;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 320.62;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 321.78;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 322.95;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 324.11;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 325.28;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 326.44;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 327.61;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 328.77;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 329.94;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 331.11;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 332.27;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 333.44;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 334.59;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 335.77;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 336.92;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 338.09;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 339.27;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 340.42;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 341.59;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 342.75;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 343.92;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 345.08;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 346.25;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 347.41;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 348.58;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 349.75;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 350.91;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 352.08;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 353.23;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 354.41;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 355.56;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 356.73;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 357.91;  data[i].distance = 00000.00;    data[i++].quality =  0;
    data[i].sync = 0;  data[i].angle = 359.06;  data[i].distance = 00000.00;    data[i++].quality =  0;
    
    data_cnt = i;

    // no obstacle
    for (i = 0; i < data_cnt; i++) {
        data[i].quality = 0;
        data[i].distance = 0;
        //data[i].quality = 30;
        //data[i].distance = 150;
    }

/*
    // decrease the distance of all values to be below LIDAR_DISTANCE_THRESHOLD
    for (i = 0; i < data_cnt; i++) {
        data[i].distance = data[i].distance * 0.4;
    }
*/    
/*    
    lidar_data_t data2[LIDAR_DATA_NUM];
    int i2 = 30;    // rotate 45 "degrees"
    for(i = 0; i < LIDAR_DATA_NUM; i++) {
        data2[i].sync = data[i].sync;  
        data2[i].angle = data[i].angle;  
        data2[i].distance = data[i2].distance;  
        data2[i].quality = data[i2].quality;
        if (++i2 >= LIDAR_DATA_NUM) {
            i2 = 0;
        }
    }
    memcpy(data, data2, LIDAR_DATA_NUM * sizeof(lidar_data_t));    
*/

    msleep(140);    // 7 samples per second
    
    return 0;
}

#endif

// Lidar::process() moved to istrobtx's lidar_defs.h/.cpp as the free
// function lidar_process() -- see lidar.h's declaration comment.

/*
{"LIDAR_DATA_FILE":[
{"image_number":"0001527","lidar_data":[
[11.09,2600.50,13],
[12.00,3054.00,10]]},

{"image_number":"0001527","lidar_data":[
[11.09,2600.50,13],
[12.00,3054.00,10]]}
]}
*/

