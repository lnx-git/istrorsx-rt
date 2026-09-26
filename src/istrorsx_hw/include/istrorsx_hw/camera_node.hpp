#pragma once

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/camera_frame.hpp"
#include "istrorsx_hw/msg/image_number.hpp"

class CameraNode : public rclcpp::Node {
public:
    CameraNode();

private:
    // Timer callback for periodic camera frame capture
    void camera_read_cb();

    // Caches planner_node's latest virtual-clock value (see ImageNumber.msg)
    // for camera_read_cb() to stamp onto the next captured CameraFrame.
    void cb_image_number(const istrorsx_hw::msg::ImageNumber::SharedPtr msg);

    // Builds and publishes CameraFrame from plain color/depth frames.
    void publishCameraFrame(const cv::Mat &camera_img, const cv::Mat &camera_depth);

    // Timers
    rclcpp::TimerBase::SharedPtr timer_camera_;

    // ROS2 Publishers
    rclcpp::Publisher<istrorsx_hw::msg::CameraFrame>::SharedPtr pub_camera_;

    // ROS2 Subscribers
    rclcpp::Subscription<istrorsx_hw::msg::ImageNumber>::SharedPtr sub_image_number_;

    // 0 until the first ImageNumber.msg arrives -- matches legacy's own
    // pre-process_ref image_number=0 starting state.
    int64_t last_image_number_ = 0;
};
