#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <opencv2/opencv.hpp>
#include "dmap.h"
#include "system.h"

using namespace cv;
using namespace std;

#ifdef ISTRO_RPLIDAR

// Ported from SDK v1.6.1 (rplidar.h, rp::standalone::rplidar namespace) to the
// current Slamtec SDK v2.x (sl_lidar.h, sl:: namespace) -- see
// doc/ai/05_migration_guide.md / doc/ai/03_progress.md for the API mapping.
#include "sl_lidar.h"    // Slamtec LIDAR SDK, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;

#endif
    
using namespace cv;
using namespace std;

// LIDAR_DATA_NUM/lidar_data_t/lidar_process()/lidar_draw_output() all live
// in the istrobtx package now (pure data types + geometry, no SDK
// dependency) so DataSet and istrorsx_core (drive_node, planner_node, and
// the future save_node for lidar_draw_output() specifically) can carry/
// process/render a lidar scan without pulling in the RPLIDAR SDK.
#include "lidar_defs.h"

class Lidar {
public:
#ifdef ISTRO_RPLIDAR
    ILidarDriver * drv;
    // New in the SDK v2.x port: the driver connects via a separate channel
    // object (instead of init() taking a port/baudrate pair directly), and
    // its lifetime must outlive the driver -- kept as a member so close() can
    // free both.
    IChannel     * channel;
#endif

public:  // output
//  int lidar_data_cnt;
//  lidar_data_t lidar_data[LIDAR_DATA_NUM];

//  int lidar_sint; 
float lidar_lobst_fi; 
float lidar_robst_fi;
float lidar_stear_vect;

//  Mat lidar_img;

public:
    int init(const char *portName);
    void close(void);
    
    bool checkHealth(void);

    int getData(lidar_data_t *data, int& data_cnt);
    // process()/drawOutput() moved to istrobtx's lidar_defs.h/.cpp as the
    // free functions lidar_process()/lidar_draw_output() -- neither touched
    // Lidar's own instance state, and istrorsx_core's planner_node/future
    // save_node need to call them without the RPLIDAR SDK this class pulls
    // in. See doc/ai/05_migration_guide.md.
};

#endif
