#pragma once

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/ups_data.hpp"
#include "istrorsx_hw/istrobtx/ina219.h"

// Waveshare UPS Power Module (C) battery telemetry -- no legacy
// (istro_rt2025.cpp) equivalent, this hardware isn't part of the original
// robot. Unlike ctrlboard_node/gps_node/lidar_node/camera_node, there is no
// Config/Threads/DataSet/istro_main_X scaffold here: this follows
// keyboard_node's (istrorsx_core) precedent for a "new, no legacy
// counterpart" node -- a plain rclcpp::Node with its own minimal main().
// One deliberate difference from keyboard_node: this still loads
// conf/log4cxx.xml (see ups_node.cpp's main()), so periodic readings land in
// the same shared logout/istro_<EVENT_TAG>.log every other node writes to --
// useful for correlating a future low-battery brownout against other
// subsystems' logs, which keyboard_node had no analogous reason to do.
class UpsNode : public rclcpp::Node {
public:
    UpsNode();

private:
    // Timer callback: reads all four INA219 registers, derives percentage,
    // and publishes UpsData.
    void upsReadCb();

    Ina219 ina219_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_ups_;

    // ROS2 Publisher
    rclcpp::Publisher<istrorsx_hw::msg::UpsData>::SharedPtr pub_ups_;
};
