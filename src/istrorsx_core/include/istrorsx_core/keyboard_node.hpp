#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Keyboard-input publisher: replaces legacy loop_waitKey()'s cv::waitKey()-
// based key detection (which needed an X11 OpenCV window, and therefore a
// VNC connection on a headless robot) with a plain terminal-raw-mode reader,
// running in whatever terminal this node was launched from -- no GUI/VNC
// needed. Publishes sensor_msgs/msg/Joy on the same "/joy" topic a real
// gamepad's standard "joy" package would (see xbox_controller_run.sh),
// using drive_node's own JOY_AXIS_*/JOY_BUTTON_* wire contract
// (drive_node.hpp) -- drive_node doesn't need to know which one is talking.
//
// A plain terminal has no keyup event (unlike X11/evdev) -- holding a key
// down only produces repeated keydown bytes at the OS's own auto-repeat
// cadence (a real initial delay, e.g. ~300-600ms, before a faster repeat
// rate kicks in). To present this as a steady, continuously-held axis/
// button the way a real joystick would, a background thread records the
// last-seen time of each mapped key (mapKeyTimes_), and a fixed-rate
// publish timer (PUBLISH_PERIOD_MS) treats a key as "still held" if it was
// seen within the last KEY_HOLD_TIMEOUT_MS -- long enough to bridge the
// OS's own initial-repeat-delay gap, short enough that release is still
// detected promptly. This decouples the /joy publish rate entirely from
// the terminal's own irregular repeat timing.
//
// ESC/'x'/'X' are handled locally (exit this node) -- not forwarded as a
// Joy message; asking a remote node to shut itself down via a stray
// keypress would be an odd, risky side channel.
class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode();
    ~KeyboardNode() override;

private:
    void publishTick();
    void readerThreadMain();

    void enableRawMode();
    void disableRawMode();

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread reader_thread_;
    std::atomic<bool> running_{true};

    struct termios orig_termios_{};
    bool raw_mode_enabled_ = false;

    // Last-seen time per mapped (lowercased) key, guarded by mutex_ --
    // written by readerThreadMain(), read by publishTick(). A small, fixed
    // key set and a 10Hz read rate make a plain mutex simpler (and just as
    // fast in practice) than per-key atomics.
    std::mutex mutex_;
    std::unordered_map<char, std::chrono::steady_clock::time_point> last_seen_;
    // Which keys were "held" as of the previous publishTick() call -- used
    // to print a message only on the press *edge*, not every tick.
    std::unordered_map<char, bool> was_held_;
};
