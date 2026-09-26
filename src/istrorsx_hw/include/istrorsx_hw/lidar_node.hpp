#pragma once

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/lidar_data.hpp"
#include "istrorsx_hw/msg/image_number.hpp"
#include "lidar_defs.h"

class LidarNode : public rclcpp::Node {
public:
    LidarNode();

private:
    // Timer callback for periodic lidar scan capture
    void lidar_read_cb();

    // Caches planner_node's latest virtual-clock value (see ImageNumber.msg)
    // for publishLidarData() to stamp onto the next published LidarData.
    void cb_image_number(const istrorsx_hw::msg::ImageNumber::SharedPtr msg);

    // Builds and publishes LidarData from a plain scan buffer.
    void publishLidarData(int lidar_data_cnt, const lidar_data_t *lidar_data);

    // Timers
    rclcpp::TimerBase::SharedPtr timer_lidar_;

    // ROS2 Publishers
    rclcpp::Publisher<istrorsx_hw::msg::LidarData>::SharedPtr pub_lidar_;

    // ROS2 Subscribers
    rclcpp::Subscription<istrorsx_hw::msg::ImageNumber>::SharedPtr sub_image_number_;

    // 0 until the first ImageNumber.msg arrives -- matches camera_node's
    // last_image_number_ starting state.
    int64_t last_image_number_ = 0;
};
