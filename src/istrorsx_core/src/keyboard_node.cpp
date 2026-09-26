#include "istrorsx_core/keyboard_node.hpp"

#include "mtime.h"
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "istrorsx_core/drive_node.hpp"

// How long a key stays considered "held" after its last-seen byte, before
// publishTick() treats it as released. Must comfortably bridge a terminal's
// own initial key-repeat delay (commonly a few hundred ms) -- see
// keyboard_node.hpp's class comment.
static const int KEY_HOLD_TIMEOUT_MS = 500;

// Fixed /joy publish rate -- deliberately decoupled from the terminal's own
// (irregular, OS-dependent) key-repeat cadence.
static const int PUBLISH_PERIOD_MS = 100;

static char toMappedKey(char c)
{
    // Normalize to the lowercase legacy key char this project's convention
    // already uses (drive_node.cpp's loopWaitKey() treats upper/lower the
    // same throughout).
    if ((c >= 'A') && (c <= 'Z')) {
        return c - 'A' + 'a';
    }
    return c;
}

static bool isMappedKey(char c)
{
    switch (c) {
    case 'q': case 'e': case 'a': case 'd': case 'w': case 's':
    case ' ': case 'p': case 'l': case 'z': case 'u': case 'm': case 'g':
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        return true;
    default:
        return false;
    }
}

static const char *keyDescription(char c)
{
    switch (c) {
    case 'q': return "steer full left";
    case 'e': return "steer full right";
    case 'a': return "steer nudge left";
    case 'd': return "steer nudge right";
    case 'w': return "throttle forward (velocity++)";
    case 's': return "throttle backward (velocity--)";
    case ' ': return "STOP";
    case 'p': return "start BACKFWD test program";
    case 'l': return "cycle LED program";
    case 'z': return "force wrongway-check";
    case 'u': return "mode: AUTONOMOUS";
    case 'm': return "mode: MANUAL";
    case 'g': return "GPS point debug print";
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        return "absolute velocity preset";
    default: return "?";
    }
}

KeyboardNode::KeyboardNode() : Node("keyboard_node")
{
    if (!isatty(STDIN_FILENO)) {
        throw std::runtime_error("keyboard_node needs an interactive terminal on stdin (not a pipe/redirect)");
    }

    pub_joy_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    enableRawMode();
    reader_thread_ = std::thread(&KeyboardNode::readerThreadMain, this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(PUBLISH_PERIOD_MS),
        std::bind(&KeyboardNode::publishTick, this));

    std::cout <<
        "keyboard_node: controls (hold a key to keep its axis/button active)\n"
        "  q/e = steer full left/right   a/d = steer nudge left/right\n"
        "  w/s = throttle forward/back   SPACE = stop\n"
        "  u = AUTONOMOUS   m = MANUAL   p = program   l = LED   z = wrongway-force\n"
        "  g = GPS debug print   0-9 = absolute velocity preset\n"
        "  x / ESC = quit keyboard_node\n";
}

KeyboardNode::~KeyboardNode()
{
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    disableRawMode();
}

void KeyboardNode::enableRawMode()
{
    if (tcgetattr(STDIN_FILENO, &orig_termios_) != 0) {
        throw std::runtime_error("keyboard_node: tcgetattr() failed");
    }

    struct termios raw = orig_termios_;
    // Disable canonical (line-buffered) mode and local echo; keep ISIG so
    // Ctrl+C still generates SIGINT and shuts the node down the normal way.
    raw.c_lflag &= ~(ICANON | ECHO);
    // VMIN=0, VTIME=1 (tenths of a second) -- read() returns after ~100ms
    // even with no input, so readerThreadMain()'s loop can notice running_
    // went false and exit promptly on shutdown, instead of blocking forever
    // waiting for one more keypress.
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        throw std::runtime_error("keyboard_node: tcsetattr() failed");
    }
    raw_mode_enabled_ = true;
}

void KeyboardNode::disableRawMode()
{
    if (raw_mode_enabled_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        raw_mode_enabled_ = false;
    }
}

void KeyboardNode::readerThreadMain()
{
    while (running_) {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) {
            continue;   // VTIME timeout, no byte available -- just re-check running_
        }

        if ((c == 27) || (c == 'x') || (c == 'X')) {   // ESC / x / X -- local quit, not sent as a Joy message
            running_ = false;
            rclcpp::shutdown();
            break;
        }

        char key = toMappedKey(c);
        if (!isMappedKey(key)) {
            continue;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        last_seen_[key] = std::chrono::steady_clock::now();
    }
}

void KeyboardNode::publishTick()
{
    double tpub = timeBegin();

    auto now = std::chrono::steady_clock::now();
    std::unordered_map<char, bool> held;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &kv : last_seen_) {
            auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - kv.second).count();
            held[kv.first] = age_ms < KEY_HOLD_TIMEOUT_MS;
        }
    }
    auto isHeld = [&](char c) {
        auto it = held.find(c);
        return (it != held.end()) && it->second;
    };

    // Print only on the press edge (wasn't held last tick, is held now) --
    // "if it's actually being sent as a message", not every repeat.
    for (const auto &kv : held) {
        bool now_held = kv.second;
        bool before_held = was_held_.count(kv.first) ? was_held_[kv.first] : false;
        if (now_held && !before_held) {
            std::cout << "keyboard_node: '" << kv.first << "' -> " << keyDescription(kv.first) << "\n";
        }
        was_held_[kv.first] = now_held;
    }

    sensor_msgs::msg::Joy msg;
    msg.axes.resize(2, 0.0f);
    msg.buttons.resize(24, 0);

    // Steering nudge axis -- 'q'/'e' (full lock) are buttons instead, see
    // JOY_BUTTON_STEER_FULL_LEFT/RIGHT below: they set steering_angle once
    // per press in legacy (SA_MAX/SA_MIN), not a continuous value, so a
    // discrete button is a more direct match than an extreme axis reading.
    if (isHeld('a')) {
        msg.axes[DriveNode::JOY_AXIS_STEERING] = 0.5f;
    } else if (isHeld('d')) {
        msg.axes[DriveNode::JOY_AXIS_STEERING] = -0.5f;
    }

    if (isHeld('w')) {
        msg.axes[DriveNode::JOY_AXIS_THROTTLE] = 1.0f;
    } else if (isHeld('s')) {
        msg.axes[DriveNode::JOY_AXIS_THROTTLE] = -1.0f;
    }

    msg.buttons[DriveNode::JOY_BUTTON_STOP]            = isHeld(' ') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_PROGRAM]         = isHeld('p') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_LED]             = isHeld('l') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_WRONGWAY]        = isHeld('z') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_MODE_AUTONOMOUS] = isHeld('u') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_MODE_MANUAL]     = isHeld('m') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_GPS_DEBUG]       = isHeld('g') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_STEER_FULL_LEFT]  = isHeld('q') ? 1 : 0;
    msg.buttons[DriveNode::JOY_BUTTON_STEER_FULL_RIGHT] = isHeld('e') ? 1 : 0;
    // Every message from this node is keyboard-sourced -- lets drive_node's
    // pollKey()/applyJoystickSteering() route steering through the nudge
    // model below instead of a real gamepad's proportional mapping (same
    // axis/topic, no other way to tell them apart). See this flag's own
    // comment in drive_node.hpp.
    msg.buttons[DriveNode::JOY_BUTTON_SOURCE_KEYBOARD] = 1;
    for (int i = 0; i < 10; i++) {
        msg.buttons[DriveNode::JOY_BUTTON_DIGIT_BASE + i] = isHeld(static_cast<char>('0' + i)) ? 1 : 0;
    }

    pub_joy_->publish(msg);
    timeEnd("istro::keyboard_node.publishTick", tpub);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<KeyboardNode>();
        rclcpp::spin(node);
    } catch (const std::exception &ex) {
        std::cerr << "keyboard_node: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
