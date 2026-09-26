#include "istrorsx_hw/ups_node.hpp"

#include "logger.h"

LOG_DEFINE(loggerIstroUps, "istroUps");

UpsNode::UpsNode() : Node("ups_node")
{
    this->declare_parameter<int>("i2c_bus", 7);
    this->declare_parameter<int>("i2c_addr", 0x41);
    this->declare_parameter<int>("poll_period_ms", 2000);

    int i2c_bus = this->get_parameter("i2c_bus").as_int();
    int i2c_addr = this->get_parameter("i2c_addr").as_int();
    int poll_period_ms = this->get_parameter("poll_period_ms").as_int();

    if (ina219_.init(i2c_bus, i2c_addr) < 0) {
        LOGM_ERROR(loggerIstroUps, "UpsNode", "msg=\"Ina219::init() failed!\", i2c_bus=" << i2c_bus
            << ", i2c_addr=" << i2c_addr);
    }

    pub_ups_ = this->create_publisher<istrorsx_hw::msg::UpsData>("/robot/ups_data", 10);

    timer_ups_ = this->create_wall_timer(
        std::chrono::milliseconds(poll_period_ms),
        std::bind(&UpsNode::upsReadCb, this));
}

void UpsNode::upsReadCb()
{
    double bus_voltage = ina219_.getBusVoltage_V();
    double shunt_voltage = ina219_.getShuntVoltage_mV() / 1000;
    double current = ina219_.getCurrent_mA();
    double power = ina219_.getPower_W();

    // Same estimate/clamp as ina219.py's own __main__: 9V=empty, 12.6V=full
    // for this pack.
    double percentage = (bus_voltage - 9) / 3.6 * 100;
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    istrorsx_hw::msg::UpsData msg;
    msg.bus_voltage = bus_voltage;
    msg.shunt_voltage = shunt_voltage;
    msg.current = current / 1000;
    msg.power = power;
    msg.percentage = percentage;
    pub_ups_->publish(msg);

    LOGM_DEBUG(loggerIstroUps, "upsReadCb", "bus_voltage=" << ioff(bus_voltage, 3)
        << ", shunt_voltage=" << ioff(shunt_voltage, 6) << ", current=" << ioff(current / 1000, 6)
        << ", power=" << ioff(power, 6) << ", percentage=" << ioff(percentage, 2));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    LOG_CONFIG_LOAD("conf/log4cxx.xml");
    LOG_THREAD_NAME("main");
    LOG_INFO(loggerIstroUps, "-----------------------------");
    LOGM_INFO(loggerIstroUps, "main", "msg=\"application start\"");

    auto node = std::make_shared<UpsNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
