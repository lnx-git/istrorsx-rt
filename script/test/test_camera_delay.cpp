// Standalone delivery-jitter probe: subscribes to CameraFrame exactly the way
// vision_node does and reports the inter-arrival gap every 10 messages.
//
// Built OUTSIDE the colcon workspace on purpose (see
// test_camera_delay_build.sh) so it can be compiled and run while a full test
// is in progress, without rebuilding or disturbing anything.
//
// WHY: on 2026-09-07 planner_node's cb_vision_front() stopped firing for up
// to 2.5 s at a time while its own 5 ms timer kept ticking, and per-minute
// counts showed no message loss at all (281 published -> 282 received). So
// delivery is not dropping messages, it is arriving in bursts -- long silence
// then a rapid catch-up. A Python probe saw the same thing, which is why this
// one is C++: an interpreted subscriber is one more variable, and the point
// here is to find out whether a plain rclcpp node with NO timer and NO work
// of its own also sees the gaps. If it does, the cause is upstream of any one
// node's executor; if it does not, the executor is where to look.
//
// Deliberately does nothing per message except take a timestamp: no copying
// the image out, no processing. Anything it measures is therefore delivery,
// not this program's own slowness.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "istrorsx_hw/msg/camera_frame.hpp"

using istrorsx_hw::msg::CameraFrame;
using Clock = std::chrono::steady_clock;

// Report every this many messages. 10 at 5 FPS is a line every ~2 s, which is
// often enough to see a stall as it happens.
static const int REPORT_EVERY = 10;

class DelayProbe : public rclcpp::Node
{
public:
    explicit DelayProbe(const std::string &topic, int report_every)
        : Node("test_camera_delay"), report_every_(report_every)
    {
        // Depth 10 and default (reliable) QoS -- the same as camera_node's
        // publisher and vision_node's subscription, so this measures what
        // they would see rather than some other configuration.
        sub_ = this->create_subscription<CameraFrame>(
            topic, 10,
            [this](const CameraFrame::SharedPtr msg) { this->cb(msg); });

        printf("subscribed to %s, reporting every %d messages\n",
               topic.c_str(), report_every_);
        printf("%-12s %8s %8s %8s %8s   %s\n",
               "time", "n", "min_ms", "max_ms", "avg_ms", "image_number");
        fflush(stdout);
    }

private:
    void cb(const CameraFrame::SharedPtr msg)
    {
        const auto now = Clock::now();

        if (have_prev_) {
            const double dt_ms =
                std::chrono::duration<double, std::milli>(now - prev_).count();
            gaps_.push_back(dt_ms);
            worst_ = std::max(worst_, dt_ms);
        }
        prev_ = now;
        have_prev_ = true;
        total_++;

        if ((int)gaps_.size() >= report_every_) {
            report(msg->image_number);
            gaps_.clear();
        }
    }

    void report(int64_t image_number)
    {
        double lo = *std::min_element(gaps_.begin(), gaps_.end());
        double hi = *std::max_element(gaps_.begin(), gaps_.end());
        double sum = 0.0;
        for (double g : gaps_) sum += g;

        // Wall-clock time so a line can be lined up against the shared log.
        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm tmv;
        localtime_r(&t, &tmv);
        char ts[16];
        strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);

        printf("%-12s %8d %8.1f %8.1f %8.1f   %ld%s\n",
               ts, total_, lo, hi, sum / gaps_.size(), (long)image_number,
               hi >= 1000.0 ? "   <-- STALL" : "");
        fflush(stdout);
    }

public:
    void summary() const
    {
        printf("\n%d messages received, worst single gap %.1f ms\n", total_, worst_);
    }

private:
    rclcpp::Subscription<CameraFrame>::SharedPtr sub_;
    int report_every_;
    std::vector<double> gaps_;
    Clock::time_point prev_;
    bool have_prev_ = false;
    int total_ = 0;
    double worst_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Plain positional args rather than ROS parameters: this is a throwaway
    // probe, and "--ros-args -p" would be more typing than the whole program.
    std::string topic = "/robot/camera_front_data";
    int report_every = REPORT_EVERY;
    if (argc > 1) topic = argv[1];
    if (argc > 2) report_every = std::max(1, atoi(argv[2]));

    auto node = std::make_shared<DelayProbe>(topic, report_every);
    rclcpp::spin(node);
    node->summary();

    rclcpp::shutdown();
    return 0;
}
