#pragma once

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/speed_command.hpp"
#include "istrorsx_hw/msg/led_config.hpp"
#include "istrorsx_hw/msg/device_action.hpp"
#include "istrorsx_hw/msg/imu_data.hpp"
#include "istrorsx_hw/msg/servo_data.hpp"

class CtrlBoardNode : public rclcpp::Node {
public:
    CtrlBoardNode();

private:
    // Message callback declarations
    // Waits out CTRLBOARD_WRITE_GAP_MS since the previous board write, if needed.
    // Call before every ctrlBoard write -- see the KNOWN ISSUE in ctrlboard_node.cpp.
    void paceWrite();
    double last_write_t_ = -1;

    void cb_speed(const istrorsx_hw::msg::SpeedCommand::SharedPtr msg);
    void cb_led(const istrorsx_hw::msg::LedConfig::SharedPtr msg);
    void cb_action(const istrorsx_hw::msg::DeviceAction::SharedPtr msg);

    // Timer callback for periodic serial communication reading
    void serial_read_cb();

    // ---- Publish helpers -- each builds and publishes one message from
    // plain parameters, called from serial_read_cb() once it has a fresh
    // reading. ----
    void publishServoData(int state, double heading, int ircv, double ircv500, int angle, int velocity, int loadd, int cbtime,
        int ulsd1, int ulsd2, int ulsd3, int ulsd4, int ulsd5);
    void publishImuData(double euler_x, double euler_y, double euler_z, int calib_gyro, int calib_accel, int calib_mag);

    // Timers
    rclcpp::TimerBase::SharedPtr timer_serial_;

    // ROS2 Subscriptions
    rclcpp::Subscription<istrorsx_hw::msg::SpeedCommand>::SharedPtr sub_speed_;
    rclcpp::Subscription<istrorsx_hw::msg::LedConfig>::SharedPtr sub_led_;
    rclcpp::Subscription<istrorsx_hw::msg::DeviceAction>::SharedPtr sub_action_;

    // ROS2 Publishers
    rclcpp::Publisher<istrorsx_hw::msg::ImuData>::SharedPtr pub_imu_;
    rclcpp::Publisher<istrorsx_hw::msg::ServoData>::SharedPtr pub_servo_;
};

