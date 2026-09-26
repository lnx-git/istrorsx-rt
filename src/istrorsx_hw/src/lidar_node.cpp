#include "istrorsx_hw/lidar_node.hpp"

#include <cstring>

#include "threads.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "config.h"
#include "istrorsx_hw/istrobtx/lidar.h"
#include "istrorsx_hw/istrobtx/istro_main_lidar.h"

LOG_DEFINE(loggerIstroLidar2, "istroLidar");

LidarNode::LidarNode() : Node("lidar_node")
{
    // Setup publisher for lidar scan data
    pub_lidar_ = this->create_publisher<istrorsx_hw::msg::LidarData>("/robot/lidar_data", 10);

    // Setup a periodic timer for capturing a full lidar scan. lidar.getData()
    // (grabScanDataHq() underneath) blocks until a complete 360-degree scan is
    // ready, which itself takes roughly 150-200 ms with this hardware's legacy
    // scan mode (see the startScan() NOTE in lidar.cpp) -- the timer period
    // here is deliberately shorter than that, so capture is effectively
    // back-to-back and paced by the blocking call itself, the same way the
    // original capture_lidar_thread()'s while-loop had no artificial sleep.
    timer_lidar_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LidarNode::lidar_read_cb, this)
    );

    // See ImageNumber.msg's own comment for why this can't just be a
    // PlannerData.msg subscription (istrorsx_hw cannot depend on
    // istrorsx_core, which depends on istrorsx_hw).
    sub_image_number_ = this->create_subscription<istrorsx_hw::msg::ImageNumber>(
        "/robot/image_number", 10,
        [this](const istrorsx_hw::msg::ImageNumber::SharedPtr msg) { cb_image_number(msg); });
}

void LidarNode::cb_image_number(const istrorsx_hw::msg::ImageNumber::SharedPtr msg)
{
    last_image_number_ = msg->image_number;
}

// NOTE: unlike the original capture_lidar_thread() (which claimed a pipeline
// dataset slot via THDATA_STATE_NEW/THDATA_STATE_LIDAR_CAPTURING/
// THDATA_STATE_LIDAR_CAPTURED for vision_thread/process_thread to consume --
// see doc/ai/05_migration_guide.md and the NOTE in istro_main_lidar.h), this
// writes into the same always-available THDATA_STATE_SHARED slot
// gps_writeData()/ctrlBoard_writeData() use, since the camera-capture
// pipeline that originally produced/consumed those pipeline states isn't
// ported.
int lidar_writeData(int lidar_data_cnt, const lidar_data_t *lidar_data)
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->lidar_data_cnt = lidar_data_cnt;
    if (lidar_data_cnt > 0) {
        memcpy(pdata->lidar_data, lidar_data, lidar_data_cnt * sizeof(lidar_data_t));
    }

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroLidar2, "lidar_writeData", "lidar_data_cnt=" << lidar_data_cnt);

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::lidar_writeData", t);
#endif

    return 0;
}

void LidarNode::publishLidarData(int lidar_data_cnt, const lidar_data_t *lidar_data)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::LidarData lidar_msg;
    lidar_msg.image_number = last_image_number_;
    lidar_msg.point_count = lidar_data_cnt;
    lidar_msg.sync.resize(lidar_data_cnt);
    lidar_msg.angle.resize(lidar_data_cnt);
    lidar_msg.distance.resize(lidar_data_cnt);
    lidar_msg.quality.resize(lidar_data_cnt);
    for (int i = 0; i < lidar_data_cnt; i++) {
        lidar_msg.sync[i] = lidar_data[i].sync;
        lidar_msg.angle[i] = lidar_data[i].angle;
        lidar_msg.distance[i] = lidar_data[i].distance;
        lidar_msg.quality[i] = lidar_data[i].quality;
    }
    pub_lidar_->publish(lidar_msg);
    timeEnd("istro::lidar_node.publishLidarData", tpub);
}

// Periodically capture a full lidar scan
// void *capture_lidar_thread(void *parg)
void LidarNode::lidar_read_cb()
{
    int          lidar_data_cnt = -1;
    lidar_data_t lidar_data[LIDAR_DATA_NUM];

    double t;
    int result = 0;

    //LOG_THREAD_NAME("capture_lidar");
    LOGM_INFO(loggerIstroLidar2, "capture_lidar_thread", "msg=\"start\"");

    do {
        t = timeBegin();

#if 0  // NOT_YET_MIGRATED: reads process_dir/process_ref/process_x/process_y/process_yaw
       // (vision/navigation pose state, written by capture_camera_thread) via capture_readData().
       // planner_node (istro_rt2025.cpp's process_thread) now exists, but deliberately doesn't
       // publish this pose for lidar_node to stamp onto captured scans -- see
       // doc/ai/01_architecture.md's planner_node decision #1 (simplified pose: planner_node uses
       // its own *current* pose when folding LidarData into wmodel, not a pose captured alongside
       // the scan). Not a missing dependency, a deliberate simplification -- revisit only if
       // real-world testing shows it causes meaningful obstacle misplacement.
       // (image_number itself is NOT part of this gap -- it's handled separately, see
       // cb_image_number()/ImageNumber.msg above and publishLidarData()'s use of
       // last_image_number_.)
        int    process_dir;
        int    process_ref;
        double process_x;
        double process_y;
        double process_yaw;

        if (capture_readData(process_dir, process_ref, process_x, process_y, process_yaw) < 0) {
            result = -2;
            break;
        }
#endif  // NOT_YET_MIGRATED

        if (lidar.getData(lidar_data, lidar_data_cnt) < 0) {
            LOGM_INFO(loggerIstroLidar2, "capture_lidar_thread", "msg=\"lidar.getData() failed!\"");
            result = -3;
            break;
        }

        timeEnd("istro::capture_lidar_thread.capture", t);

        if (lidar_writeData(lidar_data_cnt, lidar_data) < 0) {
            result = -4;
            break;
        }

        publishLidarData(lidar_data_cnt, lidar_data);

        LOGM_DEBUG(loggerIstroLidar2, "capture_lidar_thread", "msg=\"data captured\"");
    } while(0);

    if (result >= 0) {
        LOGM_INFO(loggerIstroLidar2, "capture_lidar_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstroLidar2, "capture_lidar_thread", "msg=\"exit(" << result << ")\"");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_lidar(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroLidar2, "main", "msg=\"istro_main_lidar() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);

    istro_close_lidar();

    rclcpp::shutdown();
    return 0;
}
