#pragma once

#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "istrorsx_hw/msg/speed_command.hpp"
#include "istrorsx_hw/msg/led_config.hpp"
#include "istrorsx_hw/msg/device_action.hpp"
#include "istrorsx_hw/msg/servo_data.hpp"
#include "istrorsx_hw/msg/gps_data.hpp"
#include "istrorsx_core/msg/drive_command.hpp"
#include "istrorsx_core/msg/config_update.hpp"

// Control node: a 1:1 port of istro_rt2025.cpp's prog_start()/prog_stop()/
// prog_loop(), loop_waitKey(), loop_readData(), and loop() (see
// doc/ai/01_architecture.md Section "Planned Package Split"). Ties together
// the future planner_node's output (DriveCommand, port of legacy
// process_thread), manual input (sensor_msgs/msg/Joy, keyboard now / Xbox
// controller later), and feedback from ctrlboard_node (ServoData), and
// decides what to publish to ctrlboard_node (SpeedCommand/LedConfig/
// DeviceAction).
//
// Per project decision, this reuses the legacy Threads/DataSet shared-state
// pattern exactly as istro_rt2025.cpp did: incoming ROS messages are first
// written into this process's own DataSet (via *_writeData(), mirroring
// ctrlBoard_writeData()/gps_writeData() in the other nodes), and loopReadData()
// reads them back out of THDATA_STATE_SHARED, the same way the legacy
// single-process app's loop_readData() did -- not read directly out of the
// last-received message struct.
//
// loop()'s blocking for(;;) becomes a ROS wall timer (controlTick(), one
// call = one legacy loop iteration); legacy loop_waitKey()'s cv::waitKey()
// becomes pollKey(), deriving a legacy-style key char from the latest Joy
// message instead (buttons edge-triggered, axes level-triggered -- see
// pollKey()'s own comment in drive_node.cpp for the exact mapping). Every
// legacy ctrlBoard.setXxx()/displayText()/setBallDrop() call site becomes a
// publishXxx() helper that publishes the equivalent message, in the same
// place in the control flow -- ctrlboard_node applies it to the real board.
class DriveNode : public rclcpp::Node {
public:
    DriveNode();

    // Legacy loop()'s post-for(;;) shutdown tail (stop the robot, "bye"
    // display text) -- called from main() after rclcpp::spin() returns, not
    // from the destructor, so publishing still works (matches every other
    // node's istro_close_<subsystem>() being called after spin() too).
    void sendFinalStop();

    // Joy axes/buttons mapping -- this node's own wire contract for manual
    // input: both keyboard_node and a real gamepad (via the standard "joy"
    // package) must agree with these indices. Axes and the first 8 buttons
    // follow a standard Xbox controller physical layout; buttons[10..] are
    // synthetic -- no physical gamepad has that many buttons, only
    // keyboard_node populates them (10-19 = one per legacy '0'-'9'
    // absolute-velocity-preset key, 20 = legacy 'g', 21-22 = legacy 'q'/'e').
    // Public (not this node's own private implementation detail) since
    // keyboard_node needs to read these same indices to publish a
    // compatible Joy message.
    static constexpr size_t JOY_AXIS_STEERING = 0;            // left stick X: +1=left nudge (SA_MAX direction), -1=right nudge (SA_MIN direction) -- see JOY_BUTTON_STEER_FULL_LEFT/RIGHT for the full-lock legacy 'q'/'e' keys
    static constexpr size_t JOY_AXIS_THROTTLE = 1;            // left stick Y: +1=forward (w), -1=backward (s)
    static constexpr size_t JOY_BUTTON_STOP            = 0;   // A -- immediate stop (legacy ' ')
    static constexpr size_t JOY_BUTTON_PROGRAM         = 1;   // B -- start BACKFWD test program (legacy 'p')
    static constexpr size_t JOY_BUTTON_LED             = 2;   // X -- cycle LED test program (legacy 'l')
    static constexpr size_t JOY_BUTTON_WRONGWAY        = 3;   // Y -- force-wrongway debug toggle (legacy 'z')
    static constexpr size_t JOY_BUTTON_MODE_AUTONOMOUS = 6;   // Back/Select (legacy 'u')
    static constexpr size_t JOY_BUTTON_MODE_MANUAL     = 7;   // Start (legacy 'm')
    static constexpr size_t JOY_BUTTON_DIGIT_BASE      = 10;  // buttons[10+i] = digit '0'+i, i in 0..9
    static constexpr size_t JOY_BUTTON_GPS_DEBUG       = 20;  // keyboard_node only (legacy 'g' -- GPS-point debug print)
    // legacy 'q'/'e' set steering_angle to SA_MAX/SA_MIN ONCE per press (not
    // a continuous increment like 'w'/'s') -- a discrete, one-shot legacy
    // action, so it's a button (edge-triggered via cb_joy(), like every
    // other one-shot key here) rather than an extreme axis deflection
    // value. A real gamepad's own full stick deflection reaches the same
    // SA_MAX/SA_MIN result through applyJoystickSteering()'s proportional
    // mapping instead (see JOY_BUTTON_SOURCE_KEYBOARD below) -- pollKey()'s
    // own axis-threshold check now only fires for keyboard_node's synthetic
    // Joy messages.
    static constexpr size_t JOY_BUTTON_STEER_FULL_LEFT  = 21;  // keyboard_node only (legacy 'q')
    static constexpr size_t JOY_BUTTON_STEER_FULL_RIGHT = 22;  // keyboard_node only (legacy 'e')

    // keyboard_node always sets this =1 on every Joy message it publishes;
    // no real gamepad's button count reaches this index. Lets pollKey()/
    // applyJoystickSteering() tell a real controller's continuous stick
    // position apart from keyboard_node's synthetic 0.5/-0.5 nudge values on
    // the same axis/topic -- see applyJoystickSteering()'s own comment.
    // Fixes a real reported issue: releasing a real joystick to center left
    // the wheels turned (no key fires within pollKey()'s 0.3 deadzone), only
    // counter-nudging or full-stop could recenter them.
    static constexpr size_t JOY_BUTTON_SOURCE_KEYBOARD = 23;

private:
    // ---- ROS callbacks ----
    void cb_servo(const istrorsx_hw::msg::ServoData::SharedPtr msg);
    void cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg);
    void cb_drive(const istrorsx_core::msg::DriveCommand::SharedPtr msg);
    void cb_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

    // ---- 1:1 port of istro_rt2025.cpp's prog_start()/prog_stop()/prog_loop() ----
    void progStart(int type, int dir);
    void progStop();
    int progLoop(int ctrlb_state, int &angle_change, int &angle, int &velocity_change, int &velocity);

    // True when the latest Joy message is keyboard_node's synthetic one
    // (JOY_BUTTON_SOURCE_KEYBOARD set), false for a real gamepad -- shared by
    // pollKey()/applyJoystickSteering()/loopWaitKey()'s 'w'/'s' cases, all of
    // which need to treat the two sources differently on the same axes.
    bool isKeyboardJoy() const;

    // Derives a legacy-style key char from the latest Joy message (replaces
    // cv::waitKey()) -- see this method's definition in drive_node.cpp for
    // the exact axes/buttons mapping.
    char pollKey();

    // Not a legacy port -- proportional steering for a real gamepad's
    // continuous stick position (see JOY_BUTTON_SOURCE_KEYBOARD's own
    // comment for why this needs to be a separate path from pollKey()'s
    // char-based nudge model). Called once per controlTick(), alongside
    // loopWaitKey().
    void applyJoystickSteering();

    // Not a legacy port -- proportional throttle for a real gamepad's
    // continuous stick position, mirroring applyJoystickSteering() above for
    // the exact same reason: pollKey()'s 'w'/'s' nudge never fires inside
    // its deadzone, so releasing the stick left the robot at its last
    // commanded speed instead of stopping.
    void applyJoystickThrottle();

    // ---- 1:1 port of istro_rt2025.cpp's loop_waitKey() (minus the delayms
    // parameter, which only existed to pass into cv::waitKey()) ----
    int loopWaitKey(int &steering_change, int &steering_type, int &steering_angle,
        int &velocity_change, int &velocity, int &velocity_stop, int process_velocity, double process_yaw,
        double gps_latitude, double gps_longitude);

    // ---- 1:1 port of istro_rt2025.cpp's loop_readData() ----
    int loopReadData(int &process_angle, int &process_velocity, int &process_state, int &process_stop, double &process_yaw,
        int &ctrlb_state, int &ctrlb_velocity, int &gps_fix, double &gps_latitude, double &gps_longitude, double &gps_course);

    // ---- Port of istro_rt2025.cpp's loop()'s per-iteration body, driven by
    // a ROS wall timer instead of a blocking for(;;) ----
    void controlTick();

    // Diagnostic only: logs a WRONGWAY reversal that met every condition but the robot never moved.
    // See doc/ai/01_architecture.md, WRONG_WAY "Backward check".
    void wrongwayBackCheck(int process_state, int process_stop, int ctrlb_state, int ctrlb_velocity);

    // ---- Publish helpers -- each mirrors one legacy ctrlBoard.XXX() call site ----
    void publishSpeed(int speed);
    void publishSteering(int angle);
    void publishLedProgram(int prg);
    void publishLedMask(int mask, int r, int g, int b, int blink);
    void publishDisplayText(const std::string &text);
    void publishBallDrop(int cnt);
    // Broadcasts a conf.velocityFwd*/velocityBack change so the other nodes'
    // own Config copies follow. In the legacy monolith there was one global
    // conf and this needed no message at all -- see ConfigUpdate.msg.
    void publishConfigVelocity();

    // ---- Publishers ----
    rclcpp::Publisher<istrorsx_hw::msg::SpeedCommand>::SharedPtr pub_speed_;
    rclcpp::Publisher<istrorsx_hw::msg::LedConfig>::SharedPtr pub_led_;
    rclcpp::Publisher<istrorsx_hw::msg::DeviceAction>::SharedPtr pub_action_;
    rclcpp::Publisher<istrorsx_core::msg::ConfigUpdate>::SharedPtr pub_config_;

    // ---- Subscribers ----
    rclcpp::Subscription<istrorsx_hw::msg::ServoData>::SharedPtr sub_servo_;
    rclcpp::Subscription<istrorsx_hw::msg::GpsData>::SharedPtr sub_gps_;
    rclcpp::Subscription<istrorsx_core::msg::DriveCommand>::SharedPtr sub_drive_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

    // ---- Timer ----
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- Joy input state ----
    bool have_joy_ = false;
    sensor_msgs::msg::Joy last_joy_;
    char pending_key_ = 0;   // one-shot button-edge event, set in cb_joy(), consumed by pollKey()

    // ROS params (declared in the constructor) -- a real gamepad has no
    // speed feedback loop to the physical ESC, so a small accidental
    // throttle-axis deflection (e.g. the diagonal component of an
    // intentionally lateral steering motion, both axes being on the same
    // physical stick) is felt as a real, immediate launch. These cap how far
    // loopWaitKey()'s 'w'/'s' cases can push velocity, and how much the
    // throttle axis must deflect before they fire at all -- joystick input
    // only, see isKeyboardJoy(). Defaults match the values the user
    // confirmed live on hardware.
    int   joystick_velocity_max_fwd_  = 12;    // offset above VEL_ZERO
    int   joystick_velocity_max_back_ = -17;   // offset below VEL_ZERO (negative)
    float joystick_throttle_deadzone_ = 0.5f;

    // ---- prog_*() state (legacy free-standing globals -> members) ----
    int prog_type_      = 0;    // 0 = PROG_TYPE_NONE, see drive_node.cpp
    int prog_dir_        = 1;
    int prog_step_       = 0;
    int prog_step_old_   = -1;
    int prog_wait_        = 0;
    int prog_cnt_         = 0;
    double prog_stept_   = 0;
    double prog_stept2_  = 0;
    double prog_waitt_   = 0;

    // ---- loop_waitKey() state (legacy globals -> members) ----
    int ppoint_ = 0;
    int ledprg_ = 0;

    // ---- loop()'s per-iteration state, persisted across ticks -- real
    // values (SA_STRAIGHT/VEL_ZERO/conf.velocityFwd-derived) set in the
    // constructor body, see drive_node.cpp. ----
    int steering_change_   = 0;
    int steering_type_;
    int steering_angle_;
    int steering_type_old_;

    int velocity_change_ = 1;
    int velocity_;
    int velocity_stop_ = 0;
    double velocity_lastt_ = 0;

    int process_velocity_last_ = -1;

    double display_lastt_ = 0;
    double led_to_ = 0;
    double balldrop_lastt_ = -1;

    // Legacy loop()-local `int prog_stop = 0;` -- renamed to avoid shadowing
    // the progStop() method above (legal in the original free-function C++,
    // not in a class where both would be members).
    int prog_stop_flag_ = -1;
    double prog_stopt_ = -1;

    const char *pstate_str_ = nullptr;

    // ---- wrongwayBackCheck() state ----
    double wwback_t_ = -1;
    int    wwback_velocity_ = -1;
    int    wwback_accepted_ = 0;
    int    wwback_stale_ = 0;
    double wwback_ircv500_max_ = 0;
    int    wwback_ctrlb_state_ = -1;
    int    wwback_ctrlb_velocity_ = -1;
};
