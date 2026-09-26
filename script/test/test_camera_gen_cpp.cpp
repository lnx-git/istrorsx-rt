// C++ twin of test_camera_gen.py -- same message, same rate, same per-tick
// timing split, so the two can be compared directly.
//
// WHY: the ~2 s publisher stalls on /robot/camera_front_data have survived
// every explanation tried so far (CPU/memory/disk, the 512 KB shared-memory
// segment, one slow reader, BEST_EFFORT QoS -- see doc/ai/03_progress.md).
// The one variable never removed is that the publisher is Python while the
// subscribers are C++. This removes it: if a C++ publisher sending the
// identical 1.74 MB CameraFrame at the identical rate also stalls, the
// interpreter is not involved and the cause is in rclcpp/rmw or below. If it
// runs clean, the Python side is where to look.
//
// Built OUTSIDE colcon (test_camera_gen_cpp_build.sh), like test_camera_delay,
// so it can be compiled and run while a full test is in progress.
//
// Kept deliberately identical to the Python version in the things that could
// matter: same source image from sample/, same depth profile parsed out of
// vision_depth.cpp, payload precomputed once so each tick is only field
// assignment plus publish(), same QoS (depth 10, RELIABLE default).

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "istrorsx_hw/msg/camera_frame.hpp"
#include "istrorsx_hw/msg/image_number.hpp"

using istrorsx_hw::msg::CameraFrame;
using istrorsx_hw::msg::ImageNumber;
using Clock = std::chrono::steady_clock;

// Literal cv::Mat::type() values, as CameraFrame.msg documents them.
static const int CV_TYPE_8UC3 = 16;
static const int CV_TYPE_16UC1 = 2;

static const int DEPTH_WIDTH = 848;    // CAMERA_DEPTH_FRAME_WIDTH
static const int DEPTH_HEIGHT = 480;   // CAMERA_DEPTH_FRAME_HEIGHT

// Same two default scenes as the Python version, pinned to exact filenames.
static std::string resolveImage(const std::string &ws, const std::string &name)
{
    if (name == "trava") return ws + "/sample/0000130_trava.jpg";
    if (name == "cesta") return ws + "/sample/0000189_cesta.jpg";
    if (name.find('/') != std::string::npos) return name;
    return ws + "/sample/" + name;
}

// Parsed out of vision_depth.cpp rather than copied, exactly as the Python
// version does it, so the mock can never drift from the reference the
// obstacle detector itself uses.
static std::vector<uint16_t> loadDepthProfile(const std::string &ws, const std::string &name)
{
    std::string path = ws + "/src/istrorsx_core/src/istrobtx/vision_depth.cpp";
    std::ifstream f(path);
    if (!f) { fprintf(stderr, "cannot open %s\n", path.c_str()); exit(1); }
    std::string src((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    std::regex re(name + R"(\s*\[\s*CAMERA_DEPTH_FRAME_HEIGHT\s*\]\s*=\s*\{([^}]*)\})");
    std::smatch m;
    if (!std::regex_search(src, m, re)) {
        fprintf(stderr, "depth_profile '%s' not found in %s\n", name.c_str(), path.c_str());
        exit(1);
    }

    std::vector<uint16_t> out;
    std::string body = m[1].str();
    std::regex num(R"(\d+)");
    for (auto it = std::sregex_iterator(body.begin(), body.end(), num);
         it != std::sregex_iterator(); ++it) {
        out.push_back(static_cast<uint16_t>(std::stoi(it->str())));
    }
    if (static_cast<int>(out.size()) != DEPTH_HEIGHT) {
        fprintf(stderr, "depth_profile '%s' has %zu entries, expected %d\n",
                name.c_str(), out.size(), DEPTH_HEIGHT);
        exit(1);
    }
    return out;
}

class CameraGenCpp : public rclcpp::Node
{
public:
    CameraGenCpp(const std::string &ws, const std::string &image,
                 const std::string &camera, double fps, bool stats)
        : Node("test_camera_gen_cpp"), stats_(stats), period_nominal_ms_(1000.0 / fps)
    {
        cv::Mat color = cv::imread(resolveImage(ws, image), cv::IMREAD_COLOR);
        if (color.empty()) { fprintf(stderr, "cannot read image\n"); exit(1); }

        color_w_ = color.cols;
        color_h_ = color.rows;
        color_step_ = static_cast<int>(color.step);
        color_bytes_.assign(color.data, color.data + color.total() * color.elemSize());

        // One depth row per profile entry, constant across all columns --
        // same construction as VisionDepth::getTestData().
        std::vector<uint16_t> prof = loadDepthProfile(ws, "cdepth_col_ref1");
        depth_bytes_.resize(static_cast<size_t>(DEPTH_WIDTH) * DEPTH_HEIGHT * 2);
        uint16_t *p = reinterpret_cast<uint16_t *>(depth_bytes_.data());
        for (int r = 0; r < DEPTH_HEIGHT; ++r)
            for (int c = 0; c < DEPTH_WIDTH; ++c)
                p[static_cast<size_t>(r) * DEPTH_WIDTH + c] = prof[r];

        std::string topic = "/robot/camera_" + camera + "_data";
        pub_ = this->create_publisher<CameraFrame>(topic, 10);

        // Same as camera_node (and as the Python original): the virtual clock
        // is stamped onto each frame, not generated here. save_node pairs
        // camera frames with planner output ON THIS NUMBER, so a generator
        // that leaves it at 0 publishes frames save_node can never match --
        // camera.jpg/cdepth.jpg then silently stop being written while
        // everything else still works. That is exactly what the first version
        // of this file did.
        sub_image_number_ = this->create_subscription<ImageNumber>(
            "/robot/image_number", 10,
            [this](const ImageNumber::SharedPtr m) { image_number_ = m->image_number; });

        printf("publishing %s: color %dx%d, depth %dx%d, %.1f FPS  (C++)\n",
               topic.c_str(), color_w_, color_h_, DEPTH_WIDTH, DEPTH_HEIGHT, fps);
        printf("message: %zu B\n\n",
               color_bytes_.size() + depth_bytes_.size());

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / fps),
            [this]() { this->tick(); });
    }

private:
    void tick()
    {
        // Same three numbers as the Python version, for the same reason: a
        // stall can hide in any of them and only the split says which.
        //   period : start-to-start, should equal 1/fps
        //   prep   : building the message (payload is precomputed, so this is
        //            only field assignment plus the vector copies)
        //   pub    : publish() itself, i.e. what rclcpp/rmw costs
        auto t_start = Clock::now();
        double gap_ms = prev_end_.time_since_epoch().count()
            ? std::chrono::duration<double, std::milli>(t_start - prev_end_).count() : 0.0;
        double period_ms = prev_start_.time_since_epoch().count()
            ? std::chrono::duration<double, std::milli>(t_start - prev_start_).count() : 0.0;

        CameraFrame msg;
        msg.image_number = image_number_;
        msg.color_width = color_w_;
        msg.color_height = color_h_;
        msg.color_cv_type = CV_TYPE_8UC3;
        msg.color_step = color_step_;
        msg.color_data = color_bytes_;

        msg.depth_width = DEPTH_WIDTH;
        msg.depth_height = DEPTH_HEIGHT;
        msg.depth_cv_type = CV_TYPE_16UC1;
        msg.depth_step = DEPTH_WIDTH * 2;
        msg.depth_data = depth_bytes_;

        auto t_prep = Clock::now();
        pub_->publish(msg);
        auto t_end = Clock::now();

        double prep_ms = std::chrono::duration<double, std::milli>(t_prep - t_start).count();
        double pub_ms = std::chrono::duration<double, std::milli>(t_end - t_prep).count();

        ++frames_;
        prev_start_ = t_start;
        prev_end_ = t_end;

        if (frames_ > 1) { mm(lo_period_, hi_period_, period_ms); }
        mm(lo_prep_, hi_prep_, prep_ms);
        mm(lo_pub_, hi_pub_, pub_ms);

        // The first tick has no previous one, so period/idle are undefined.
        if (stats_ && frames_ > 1) {
            bool late = period_ms > 2.0 * period_nominal_ms_;
            printf("n=%-6ld period=%8.1f [%6.1f/%7.1f]  idle=%8.1f  "
                   "prep=%5.2f [%5.2f]  pub=%6.2f [%6.2f]%s\n",
                   frames_, period_ms, lo_period_, hi_period_, gap_ms,
                   prep_ms, hi_prep_, pub_ms, hi_pub_,
                   late ? "   <-- LATE" : "");
            fflush(stdout);
        }
    }

    static void mm(double &lo, double &hi, double v)
    {
        if (lo == 0.0 || v < lo) lo = v;
        if (v > hi) hi = v;
    }

    rclcpp::Publisher<CameraFrame>::SharedPtr pub_;
    rclcpp::Subscription<ImageNumber>::SharedPtr sub_image_number_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint8_t> color_bytes_, depth_bytes_;
    int color_w_ = 0, color_h_ = 0, color_step_ = 0;
    int64_t image_number_ = 0;
    long frames_ = 0;

    bool stats_;
    double period_nominal_ms_;
    Clock::time_point prev_start_{}, prev_end_{};
    double lo_period_ = 0, hi_period_ = 0;
    double lo_prep_ = 0, hi_prep_ = 0;
    double lo_pub_ = 0, hi_pub_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string ws = argc > 4 ? argv[4] : ".";
    std::string image = argc > 1 ? argv[1] : "cesta";
    std::string camera = argc > 2 ? argv[2] : "front";
    double fps = argc > 3 ? atof(argv[3]) : 5.0;

    rclcpp::spin(std::make_shared<CameraGenCpp>(ws, image, camera, fps, true));
    rclcpp::shutdown();
    return 0;
}
