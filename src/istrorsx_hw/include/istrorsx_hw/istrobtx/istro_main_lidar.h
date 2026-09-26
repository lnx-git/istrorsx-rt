#ifndef __ISTRO_MAIN_LIDAR_H__
#define __ISTRO_MAIN_LIDAR_H__

#include "lidar.h"
#include "threads.h"

extern Lidar lidar;
extern Threads threads;

// NOTE for whoever ports capture_lidar_thread() into lidar_node.cpp next:
// unlike gps_thread/ctrlBoard_thread (which just read/write a single,
// always-"THDATA_STATE_SHARED" DataSet slot -- see gps_writeData() /
// ctrlBoard_writeData()), capture_lidar_thread() in istro_rt2025.cpp is a
// stage in the camera-capture pipeline: it claims a fresh slot via
// threads.getData(THDATA_STATE_NEW, THDATA_STATE_LIDAR_CAPTURING), writes
// lidar_data/lidar_data_cnt into it, and hands it off via
// threads.setData(pdata, THDATA_STATE_LIDAR_CAPTURED, 1) for a later
// pipeline stage (vision_thread/process_thread) to consume -- driven by
// capture_camera_thread producing THDATA_STATE_NEW slots, not by its own
// timer. It also reads image_number/process_dir/process_ref/process_x/
// process_y/process_yaw via capture_readData() (vision/navigation pose
// state, not lidar data) before capturing. None of capture_camera_thread,
// vision_thread, process_thread, navig.cpp, or navmap.cpp are ported, so
// this producer/consumer chain can't be reproduced as-is. This scaffold
// only covers Lidar init/shutdown (mirroring istro_main_gps.h/.cpp) --
// lidar_node.cpp's timer callback will need its own design decision here
// (most likely: capture+publish on its own timer like gps_node does,
// instead of waiting on the legacy camera pipeline), not a line-by-line
// port of capture_lidar_thread(). See doc/ai/05_migration_guide.md.

int istro_main_lidar(int argc, char** argv);
void istro_close_lidar(void);

#endif
