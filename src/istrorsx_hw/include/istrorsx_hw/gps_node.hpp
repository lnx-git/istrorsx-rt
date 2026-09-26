#pragma once

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/gps_data.hpp"

class GpsNode : public rclcpp::Node {
public:
    GpsNode();

private:
    // Timer callback for periodic GPS polling
    void gps_read_cb();

    // Builds and publishes GpsData from plain parameters.
    void publishGpsData(int gps_fix, double gps_latitude_raw, double gps_longitude_raw, double gps_speed, double gps_course);

    // Timers
    rclcpp::TimerBase::SharedPtr timer_gps_;

    // ROS2 Publishers
    rclcpp::Publisher<istrorsx_hw::msg::GpsData>::SharedPtr pub_gps_;
};
