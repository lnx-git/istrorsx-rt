#include "istrorsx_core/drive_node.hpp"

#include <cstdio>

#include "ctrlboard_defs.h"
#include "config.h"
#include "threads.h"
#include "dataset.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "istrorsx_core/istrobtx/istro_main_drive.h"

LOG_DEFINE(loggerIstroDrive2, "istroDrive");

// ---- Legacy istro_rt2025.cpp constants, right above the ported functions ----
static const int PROG_TYPE_NONE    = 0;
static const int PROG_TYPE_SNAKE   = 1;
static const int PROG_TYPE_BACKFWD = 2;

static const int PROG_REPEATCMD_PERIOD =   500;  // ms -- resend speed/steering this often even if unchanged

static const int PROG1_STRAIGHT_PERIOD =  6000;  // ms -- straight stretch before/after the SNAKE turns
static const int PROG1_TURN_PERIOD     = 20000;  // ms -- duration of one SNAKE turn
static const int PROG1_TURN_COUNT      =     2;  // number of SNAKE turns performed

static const int PROG2_FWDBACK_PERIOD  =  200;
static const int PROG2_WAIT_PERIOD     = 1000;
static const int PROG2_RUN_COUNT       =   10;

static const int AUTONOMOUS = 1;
static const int MANUAL     = 2;

static const int VELOCITY_SET_PERIOD =  600;  // ms -- resend speed this often even if unchanged; 0 = never
static const int DISPLAY_PERIOD      =  500;  // ms -- how often to refresh the LCD display text; 0 = never
static const int PROG_STOP_TIME      = 1000;  // ms -- how long the robot must be idle before logging "prog_stop start!"

static const int    WRONGWAY_BACKCHECK_MIN_TIME = 1500;  // ms -- shorter reversals (BACK2 is 250 ms) are not judged
static const double WRONGWAY_BACKCHECK_IRCV500  =  5.0;  // encoder speed that counts as moving = planner's WRONGWAY_IRCV500_MIN_VALUE
static const int    WRONGWAY_BACKCHECK_STALE    =  500;  // ms -- ServoData older than this cannot confirm anything

// ---- *_writeData(): write an incoming ROS message into this process's own
// DataSet (THDATA_STATE_SHARED), mirroring ctrlBoard_writeData()/gps_writeData()
// in the other nodes -- see drive_node.hpp's class comment for why. ----

static int servo_writeData(const istrorsx_hw::msg::ServoData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->ctrlb_time1    = timeBegin();
    pdata->ctrlb_state    = msg.state;
    pdata->ctrlb_ircv     = msg.ircv;
    pdata->ctrlb_ircv500  = msg.ircv500;
    pdata->ctrlb_angle    = msg.angle;
    pdata->ctrlb_velocity = msg.velocity;
    pdata->ctrlb_loadd    = msg.loadd;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroDrive2, "servo_writeData", "state=" << msg.state << ", velocity=" << msg.velocity << ", angle=" << msg.angle);

    return 0;
}

static int gps_writeData(const istrorsx_hw::msg::GpsData &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_time      = timeBegin();
    pdata->gps_fix       = msg.fix ? 1 : 0;
    pdata->gps_latitude  = msg.latitude;
    pdata->gps_longitude = msg.longitude;
    pdata->gps_speed     = msg.speed;
    pdata->gps_course    = msg.course;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroDrive2, "gps_writeData", "fix=" << msg.fix << ", course=" << ioff(msg.course, 2));

    return 0;
}

static int drive_writeData(const istrorsx_core::msg::DriveCommand &msg)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->process_angle    = msg.angle;
    pdata->process_velocity = msg.velocity;
    pdata->process_state    = msg.state;
    pdata->process_stop     = msg.stop;
    pdata->process_yaw      = msg.yaw;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroDrive2, "drive_writeData", "angle=" << msg.angle << ", velocity=" << msg.velocity
        << ", state=" << msg.state << ", stop=" << msg.stop);

    return 0;
}

// ---------------------------------------------------------------------------

DriveNode::DriveNode() : Node("drive_node")
{
    pub_speed_  = this->create_publisher<istrorsx_hw::msg::SpeedCommand>("/robot/speed_command", 10);
    pub_led_    = this->create_publisher<istrorsx_hw::msg::LedConfig>("/robot/led_config", 10);
    pub_action_ = this->create_publisher<istrorsx_hw::msg::DeviceAction>("/robot/device_action", 10);
    // Runtime conf.velocityFwd*/velocityBack changes -- see ConfigUpdate.msg for why
    // this needs a message at all (the legacy monolith had one shared conf object).
    pub_config_ = this->create_publisher<istrorsx_core::msg::ConfigUpdate>("/robot/config_update", 10);

    sub_servo_ = this->create_subscription<istrorsx_hw::msg::ServoData>(
        "/robot/servo_data", 10, std::bind(&DriveNode::cb_servo, this, std::placeholders::_1));

    sub_gps_ = this->create_subscription<istrorsx_hw::msg::GpsData>(
        "/robot/gps_data", 10, std::bind(&DriveNode::cb_gps, this, std::placeholders::_1));

    sub_drive_ = this->create_subscription<istrorsx_core::msg::DriveCommand>(
        "/robot/drive_command", 10, std::bind(&DriveNode::cb_drive, this, std::placeholders::_1));

    // Plain "/joy" topic (not "/robot/joy") -- matches the standard ROS2
    // "joy" package's default topic for a physical gamepad, so a real
    // controller works against drive_node without any renaming, and a
    // future keyboard-input publisher just targets the same topic.
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&DriveNode::cb_joy, this, std::placeholders::_1));

    // Real-joystick-only safety caps -- see these members' own comment in
    // drive_node.hpp for why. Overridable at launch/CLI (--ros-args -p ...),
    // e.g. to raise/lower the cap without a rebuild.
    this->declare_parameter<int>("joystick_velocity_max_fwd", joystick_velocity_max_fwd_);
    this->declare_parameter<int>("joystick_velocity_max_back", joystick_velocity_max_back_);
    this->declare_parameter<double>("joystick_throttle_deadzone", joystick_throttle_deadzone_);
    joystick_velocity_max_fwd_  = this->get_parameter("joystick_velocity_max_fwd").as_int();
    joystick_velocity_max_back_ = this->get_parameter("joystick_velocity_max_back").as_int();
    joystick_throttle_deadzone_ = (float)this->get_parameter("joystick_throttle_deadzone").as_double();

    // Legacy loop()'s pre-for(;;) local variable initialization -- ported
    // here since it needs SA_STRAIGHT/VEL_ZERO (ctrlboard_defs.h) and
    // conf.velocityFwd (only valid once istro_main_drive() has parsed CLI
    // args, which main() guarantees runs before this constructor).
    steering_type_     = AUTONOMOUS;
    steering_angle_    = SA_STRAIGHT;
    steering_type_old_ = steering_type_;

    velocity_      = VEL_ZERO + conf.velocityFwd;   // initial speed
    velocity_lastt_ = timeBegin();

    display_lastt_ = timeBegin();
    led_to_        = timeBegin();

    // 20 Hz -- fast enough for smooth control, matches the doc/260823_homologizacia
    // demo's control_tick() rate. Legacy loop() paced itself at ~10ms via
    // cv::waitKey(10)'s own blocking delay; this timer replaces that.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DriveNode::controlTick, this));
}

void DriveNode::cb_servo(const istrorsx_hw::msg::ServoData::SharedPtr msg)
{
    servo_writeData(*msg);
}

void DriveNode::cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg)
{
    gps_writeData(*msg);
}

void DriveNode::cb_drive(const istrorsx_core::msg::DriveCommand::SharedPtr msg)
{
    drive_writeData(*msg);
}

void DriveNode::cb_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto button_edge = [&](size_t idx) {
        bool now    = (msg->buttons.size() > idx) && msg->buttons[idx];
        bool before = (last_joy_.buttons.size() > idx) && last_joy_.buttons[idx];
        return now && !before;
    };

    if (button_edge(JOY_BUTTON_MODE_MANUAL)) {
        pending_key_ = 'm';
    } else if (button_edge(JOY_BUTTON_MODE_AUTONOMOUS)) {
        pending_key_ = 'u';
    } else if (button_edge(JOY_BUTTON_STOP)) {
        pending_key_ = ' ';
    } else if (button_edge(JOY_BUTTON_PROGRAM)) {
        pending_key_ = 'p';
    } else if (button_edge(JOY_BUTTON_LED)) {
        pending_key_ = 'l';
    } else if (button_edge(JOY_BUTTON_WRONGWAY)) {
        pending_key_ = 'z';
    } else if (button_edge(JOY_BUTTON_GPS_DEBUG)) {
        pending_key_ = 'g';
    } else if (button_edge(JOY_BUTTON_STEER_FULL_LEFT)) {
        pending_key_ = 'q';
    } else if (button_edge(JOY_BUTTON_STEER_FULL_RIGHT)) {
        pending_key_ = 'e';
    } else {
        for (int i = 0; i < 10; i++) {
            if (button_edge(JOY_BUTTON_DIGIT_BASE + i)) {
                pending_key_ = static_cast<char>('0' + i);
                break;
            }
        }
    }

    last_joy_ = *msg;
    have_joy_ = true;
}

// ---- 1:1 port of prog_start()/prog_stop() ----

void DriveNode::progStart(int type, int dir)
{
    prog_type_ = type;
    prog_dir_ = dir;
    prog_step_ = 0;
    prog_step_old_ = -1;
    prog_stept_ = timeBegin();
    prog_wait_ = 0;
    prog_cnt_ = 0;
}

void DriveNode::progStop()
{
    prog_type_ = PROG_TYPE_NONE;
}

// ---- 1:1 port of prog_loop() ----

int DriveNode::progLoop(int ctrlb_state, int &angle_change, int &angle, int &velocity_change, int &velocity)
{
    int change = 0;
    int step_old = prog_step_;

    // wait: program execution is interrupted?
    if (CTRLB_STATE_OBSTACLE(ctrlb_state)) {
        if (!prog_wait_) {
            LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"wait: program was interrupted!\"");
            prog_wait_ = 1;
            prog_waitt_ = timeBegin();
            change = 1;
        }
    } else {
        if (prog_wait_) {
            LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"wait: program will continue...\"");
            prog_wait_ = 0;
            prog_stept_ += timeBegin() - prog_waitt_;
            change = 1;
        }
    }

    if (prog_step_old_ != prog_step_) {
        change = 1;
    }

    /* SNAKE */
    if ((prog_type_ == PROG_TYPE_SNAKE) && (!prog_wait_)) {
        switch (prog_step_) {
        case 0:  // go-straight
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #0 - go-straight\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept_) >= PROG1_STRAIGHT_PERIOD) {
                prog_step_ = 1;
                prog_stept_ = timeBegin();
            }
            break;
        case 1:  // turn-left
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #1 - turn-left\"");
            }
            angle = SA_MAX;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept_) >= PROG1_TURN_PERIOD) {
                if ((++prog_cnt_) < PROG1_TURN_COUNT) {
                    prog_step_ = 2;
                } else {
                    prog_step_ = 8;
                }
                prog_stept_ = timeBegin();
            }
            break;
        case 2:  // turn-right
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #2 - turn-right\"");
            }
            angle = SA_MIN;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept_) >= PROG1_TURN_PERIOD) {
                if ((++prog_cnt_) < PROG1_TURN_COUNT) {
                    prog_step_ = 1;
                } else {
                    prog_step_ = 8;
                }
                prog_stept_ = timeBegin();
            }
            break;
        case 8:  // go-straight
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #8 - go-straight\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept_) >= PROG1_STRAIGHT_PERIOD) {
                prog_step_ = 9;
                prog_stept_ = timeBegin();
            }
            break;
        case 9:
            LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #9: finished!\"");
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            progStop();
            break;
        }
    }

    /* BACKFWD */
    if ((prog_type_ == PROG_TYPE_BACKFWD) && (!prog_wait_)) {
        switch (prog_step_) {
        case 0:  // go-back
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #0 - back\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityBack;
            if (timeDelta(prog_stept_) >= PROG2_FWDBACK_PERIOD) {
                prog_step_ = 1;
                prog_stept_ = timeBegin();
            }
            break;
        case 1:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #1 - wait\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept_) >= PROG2_WAIT_PERIOD) {
                prog_step_ = 6;
                prog_stept_ = timeBegin();
            }
            break;
        // legacy steps 2-5 (wait2/wait3/wait4/wait5) were commented out in
        // the original source (istro_rt2025.cpp) -- kept out here too.
        case 6:  // go-forward
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #6 - forward\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO + conf.velocityFwd;
            if (timeDelta(prog_stept_) >= PROG2_FWDBACK_PERIOD) {
                prog_step_ = 7;
                prog_stept_ = timeBegin();
            }
            break;
        case 7:  // stop & wait
            if (change) {
                LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #7 - wait\"");
            }
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            if (timeDelta(prog_stept_) >= PROG2_WAIT_PERIOD) {
                if ((++prog_cnt_) < PROG2_RUN_COUNT) {
                    prog_step_ = 0;
                } else {
                    prog_step_ = 9;
                }
                prog_stept_ = timeBegin();
            }
            break;
        case 9:
            LOGM_INFO(loggerIstroDrive2, "progLoop", "msg=\"step #9: finished!\"");
            angle = SA_STRAIGHT;
            velocity = VEL_ZERO;
            progStop();
            break;
        }
    }

    if (change) {
        prog_stept2_ = prog_stept_;
    }
    if ((timeDelta(prog_stept2_) >= PROG_REPEATCMD_PERIOD) && (!prog_wait_)) {
        change = 2;
        prog_stept2_ = timeBegin();
        LOGM_DEBUG(loggerIstroDrive2, "progLoop", "change=" << change);
    }

    velocity_change = change;
    angle_change = change;

    if (change == 1) {
        LOGM_INFO(loggerIstroDrive2, "progLoop", "prog_type=" << prog_type_ << ", prog_step_old=" << prog_step_old_
            << ", prog_step=" << prog_step_ << ", prog_cnt=" << prog_cnt_ << ", prog_wait=" << prog_wait_
            << ", angle=" << angle << ", velocity=" << velocity << ", change=" << change);
    }

    prog_step_old_ = step_old;

    return change;
}

bool DriveNode::isKeyboardJoy() const
{
    return (last_joy_.buttons.size() > JOY_BUTTON_SOURCE_KEYBOARD)
        && (last_joy_.buttons[JOY_BUTTON_SOURCE_KEYBOARD] != 0);
}

// ---- Joy -> legacy key char (replaces cv::waitKey()) ----

char DriveNode::pollKey()
{
    // Buttons are edge-triggered in cb_joy() (pending_key_ set once per
    // physical press, consumed here) -- matches a real keypress's one-shot
    // nature. Axes are read fresh every call (level-triggered): as long as
    // the stick stays deflected, the corresponding legacy key keeps firing
    // every tick, which is the natural way to read a continuous joystick
    // axis (unlike a one-shot keyboard event).
    if (pending_key_ != 0) {
        char c = pending_key_;
        pending_key_ = 0;
        return c;
    }

    if (!have_joy_) {
        return 0;
    }

    // Steering axis: only keyboard_node's synthetic Joy (JOY_BUTTON_SOURCE_
    // KEYBOARD set) goes through this discrete nudge/full-lock mapping. A
    // real gamepad's steering is handled proportionally by
    // applyJoystickSteering() instead -- see that flag's own comment.
    if (isKeyboardJoy() && (last_joy_.axes.size() > JOY_AXIS_STEERING)) {
        float v = last_joy_.axes[JOY_AXIS_STEERING];
        if (v > 0.9f) return 'q';        // full left
        if (v < -0.9f) return 'e';       // full right
        if (v > 0.3f) return 'a';        // nudge left
        if (v < -0.3f) return 'd';       // nudge right
    }

    // Throttle axis: same isKeyboardJoy() split as steering above -- a real
    // gamepad's throttle is handled proportionally by
    // applyJoystickThrottle() instead, so this nudge model (and
    // joystick_throttle_deadzone_) now only ever applies to keyboard_node's
    // synthetic +-1.0 axis value.
    if (isKeyboardJoy() && (last_joy_.axes.size() > JOY_AXIS_THROTTLE)) {
        float v = last_joy_.axes[JOY_AXIS_THROTTLE];
        if (v > joystick_throttle_deadzone_) return 'w';
        if (v < -joystick_throttle_deadzone_) return 's';
    }

    return 0;
}

// Not a legacy port -- proportional steering for a real gamepad's continuous
// stick position. Fixes a reported real-hardware issue: pollKey()'s own
// nudge model (steering_angle += / -= 10 per tick while past its 0.3
// threshold) never fires while the stick sits in its deadzone, so releasing
// the stick to center left the wheels turned -- only counter-nudging or the
// full-stop key could recenter them. This maps the axis directly onto
// [SA_MIN, SA_MAX] instead, so center == SA_STRAIGHT.
void DriveNode::applyJoystickSteering()
{
    if (isKeyboardJoy() || !have_joy_ || (last_joy_.axes.size() <= JOY_AXIS_STEERING)) {
        return;
    }

    float v = last_joy_.axes[JOY_AXIS_STEERING];

    // Only take over from AUTONOMOUS once the stick is genuinely deflected
    // (matches 'a'/'d'/'q'/'e's own manual-override semantics) -- but once
    // we're already tracking it (steering_type_ == MANUAL), keep tracking
    // all the way back to center too, so easing off doesn't stop just short
    // of SA_STRAIGHT. 0.05 matches joy_node's own configured deadzone
    // (xbox_controller_run.sh), not a second, inconsistent threshold.
    if ((v > -0.05f) && (v < 0.05f) && (steering_type_ != MANUAL)) {
        return;
    }

    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;

    // SA_MIN/SA_MAX aren't symmetric around SA_STRAIGHT (215/334/455) --
    // scale each side by its own span. Matches JOY_AXIS_STEERING's existing
    // documented convention: +1 -> SA_MAX direction, -1 -> SA_MIN direction.
    int target = (v >= 0)
        ? SA_STRAIGHT + (int)(v * (SA_MAX - SA_STRAIGHT))
        : SA_STRAIGHT + (int)(v * (SA_STRAIGHT - SA_MIN));

    if (target != steering_angle_) {
        steering_type_ = MANUAL;
        steering_angle_ = target;
        steering_change_ = 1;
        progStop();
    }
}

// Not a legacy port -- proportional throttle for a real gamepad's continuous
// stick position, mirroring applyJoystickSteering() above for the exact same
// reported issue: pollKey()'s 'w'/'s' nudge never fires inside its deadzone,
// so releasing the stick left the robot going at its last commanded speed
// instead of stopping.
void DriveNode::applyJoystickThrottle()
{
    if (isKeyboardJoy() || !have_joy_ || (last_joy_.axes.size() <= JOY_AXIS_THROTTLE)) {
        return;
    }

    float v = last_joy_.axes[JOY_AXIS_THROTTLE];

    // Same take-over/keep-tracking rule as applyJoystickSteering(): only
    // grab control once genuinely deflected past the deadzone, but once
    // already MANUAL, keep tracking all the way back to VEL_ZERO too -- the
    // fix for this bug.
    if ((v > -joystick_throttle_deadzone_) && (v < joystick_throttle_deadzone_) && (steering_type_ != MANUAL)) {
        return;
    }

    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;

    // joystick_velocity_max_fwd_/_back_ double as the proportional range's
    // endpoints here -- v=+1 -> VEL_ZERO+max_fwd, v=-1 -> VEL_ZERO+max_back
    // (already negative) -- so the safety cap is a by-construction property
    // of this mapping, not a separate clamp.
    int target = VEL_ZERO + (int)(v * (v >= 0 ? joystick_velocity_max_fwd_ : -joystick_velocity_max_back_));

    if (target != velocity_) {
        steering_type_ = MANUAL;
        velocity_ = target;
        velocity_change_ = 1;
        velocity_stop_ = 0;
        progStop();
    }
}

// ---- 1:1 port of loop_waitKey() ----

int DriveNode::loopWaitKey(int &steering_change, int &steering_type, int &steering_angle,
        int &velocity_change, int &velocity, int &velocity_stop, int process_velocity, double /*process_yaw*/,
        double gps_latitude, double gps_longitude)
{
    int velocity_change_old = velocity_change;

    char c = pollKey();

    // Keypresses were never logged -- only visible through whatever each case
    // happened to log. Axis keys repeat every tick while held, buttons fire once.
    if (c != 0) {
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"key pressed\", key=\""
            << (((c >= 32) && (c < 127)) ? c : '?') << "\", code=" << (int)c
            << ", src=\"" << (isKeyboardJoy() ? "keyboard" : "joystick") << "\"");
    }

    if ((c == 27) || (c == 'x') || (c == 'X')) {
        return 1;
    }

    switch (c) {
    case 'l':
    case 'L':
        steering_type = MANUAL;
        if (++ledprg_ > 4) ledprg_ = 0;
        publishLedProgram(ledprg_);
        break;
    case 'g':
    case 'G':
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "ppoint=" << ppoint_
            << ", gps_latitude=" << ioff(gps_latitude, 7) << ", gps_longitude=" << ioff(gps_longitude, 7));
        ppoint_++;
        break;
    case 'q':
    case 'Q':
        steering_type = MANUAL;
        steering_angle = SA_MAX;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'e':
    case 'E':
        steering_type = MANUAL;
        steering_angle = SA_MIN;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'a':
    case 'A':
        steering_type = MANUAL;
        steering_angle += 10;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'd':
    case 'D':
        steering_type = MANUAL;
        steering_angle -= 10;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_angle=" << steering_angle);
        steering_change = 1;
        break;
    case 'w':
    case 'W':
        velocity++;
        // "|| (process_velocity >= 0)" disabled: planner_node re-sends velocity every
        // SPEEDCTL_SET_TIME, so the switch to MANUAL was random (~1 press in 3). Its job
        // -- making a manual speed stick -- is ConfigUpdate's now. See 03_progress.md.
        if ((velocity_stop > 0) /* || (process_velocity >= 0) */) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"velocity++\", velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    case 's':
    case 'S':
        velocity--;
        // "|| (process_velocity >= 0)" commented out -- see the 'w' case above.
        if ((velocity_stop > 0) /* || (process_velocity >= 0) */) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"velocity--\", velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    case ' ':
        steering_type = MANUAL;
        steering_angle = SA_STRAIGHT;
        steering_change = 1;
        velocity = VEL_ZERO;
        velocity_change = 1;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"stop!\"");
        break;
    case 'm':
    case 'M':
        steering_type = MANUAL;
        progStop();
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_type=\"MANUAL\"");
        break;
    case 'u':
    case 'U':
        steering_type = AUTONOMOUS;
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_type=\"AUTONOMOUS\"");
        break;
    case 'p':
    case 'P':
        steering_type = MANUAL;
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"start program!\", steering_type=\"MANUAL\"");
        progStart(PROG_TYPE_BACKFWD, +1);
        break;
    case 'z':
    case 'Z':
        // NOT_YET_MIGRATED: process_wrongway_force -- vision/navigation debug
        // flag, no consumer ported yet. See doc/ai/03_progress.md.
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"wrongway_force!\"");
        break;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        velocity = VEL_ZERO + (c - '0') * 3;
        // "|| (process_velocity >= 0)" commented out -- see the 'w' case above.
        if ((velocity_stop > 0) /* || (process_velocity >= 0) */) {
            velocity_stop = 0;
            steering_type = MANUAL;
            LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "steering_type=\"MANUAL\"");
        }
        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "velocity=" << velocity << ", process_velocity=" << process_velocity);
        velocity_change = 1;
        break;
    default:
        ;
    }

    /* manually changing velocity causes configuration changes */
    if ((velocity_change_old != velocity_change) && (velocity_change > 0) && (velocity > VEL_ZERO) && (velocity != VEL_ZERO + conf.velocityFwd)) {
        int velocityFwd_old = conf.velocityFwd;
        int velocityFwd2_old = conf.velocityFwd2;
        int velocityFwd3_old = conf.velocityFwd3;

        conf.velocityFwd = velocity - VEL_ZERO;

        if (conf.velocityFwd2 >= 0) {
            conf.velocityFwd2 = conf.velocityFwd + (velocityFwd2_old - velocityFwd_old);
        }
        if (conf.velocityFwd3 >= 0) {
            conf.velocityFwd3 = conf.velocityFwd + (velocityFwd3_old - velocityFwd_old);
        }

        LOGM_INFO(loggerIstroDrive2, "loopWaitKey", "msg=\"velocity change!\", velocityFwd=" << conf.velocityFwd
            << ", velocityFwd2=" << conf.velocityFwd2 << ", velocityFwd3=" << conf.velocityFwd3
            << ", velocityFwd_old=" << velocityFwd_old << ", velocityFwd2_old=" << velocityFwd2_old << ", velocityFwd3_old=" << velocityFwd3_old);

        // planner_node has its own conf copy and would overwrite this. See ConfigUpdate.msg.
        publishConfigVelocity();
    }

    return 0;
}

// ---- 1:1 port of loop_readData() ----

int DriveNode::loopReadData(int &process_angle, int &process_velocity, int &process_state, int &process_stop, double &process_yaw,
        int &ctrlb_state, int &ctrlb_velocity, int &gps_fix, double &gps_latitude, double &gps_longitude, double &gps_course)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    process_angle = pdata->process_angle;
    process_velocity = pdata->process_velocity;
    process_state = pdata->process_state;
    process_stop = pdata->process_stop;
    process_yaw = pdata->process_yaw;
    ctrlb_state = pdata->ctrlb_state;
    ctrlb_velocity = pdata->ctrlb_velocity;
    gps_fix = pdata->gps_fix;
    gps_latitude = pdata->gps_latitude;
    gps_longitude = pdata->gps_longitude;
    gps_course = pdata->gps_course;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroDrive2, "loopReadData", "process_angle=" << process_angle
        << ", process_velocity=" << process_velocity << ", process_state=" << process_state << ", process_stop=" << process_stop
        << ", ctrlb_state=" << ctrlb_state << ", ctrlb_velocity=" << ctrlb_velocity
        << ", gps_fix=" << gps_fix << ", gps_course=" << ioff(gps_course, 2));

    return 0;
}

// ---- Not in legacy: WRONGWAY reversal diagnostics ----

void DriveNode::wrongwayBackCheck(int process_state, int process_stop, int ctrlb_state, int ctrlb_velocity)
{
    DataSet *pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return;
    }
    double ctrlb_ircv500 = pdata->ctrlb_ircv500;
    double ctrlb_time1 = pdata->ctrlb_time1;
    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    // every condition under which the robot is supposed to be reversing right now
    int autonomous = (steering_type_ == AUTONOMOUS);
    int wrongway = (process_state == PROCESS_STATE_WRONGWAY);
    int reverse = (process_velocity_last_ >= 0) && (process_velocity_last_ < VEL_ZERO);
    int ebtn = (ctrlb_state == CTRLB_STATE_EBTN);
    int obst = ctrlb_obstState(ctrlb_state, process_velocity_last_);

    if (autonomous && wrongway && reverse && (process_stop <= 0) && !ebtn && !obst) {
        if (wwback_t_ < 0) {
            wwback_t_ = timeBegin();
            wwback_accepted_ = 0;
            wwback_stale_ = 0;
            wwback_ircv500_max_ = 0;
        }
        if ((ctrlb_time1 < 0) || (timeDelta(ctrlb_time1) > WRONGWAY_BACKCHECK_STALE)) {
            wwback_stale_ = 1;
        }
        if ((ctrlb_velocity >= 0) && (ctrlb_velocity < VEL_ZERO)) {
            wwback_accepted_ = 1;
        }
        if (wwback_accepted_ && (ctrlb_ircv500 > wwback_ircv500_max_)) {
            wwback_ircv500_max_ = ctrlb_ircv500;
        }
        wwback_velocity_ = process_velocity_last_;
        wwback_ctrlb_state_ = ctrlb_state;
        wwback_ctrlb_velocity_ = ctrlb_velocity;
        return;
    }
    if (wwback_t_ < 0) {
        return;
    }

    int back_dt = (int)timeDelta(wwback_t_);
    wwback_t_ = -1;
    if (back_dt < WRONGWAY_BACKCHECK_MIN_TIME) {
        return;
    }
    const char *end = !autonomous ? "manual" : (ebtn ? "emergency" : ((process_stop > 0) ? "process_stop" :
                      (obst ? "board_obstacle" : (!wrongway ? "wrongway_end" : "phase_end"))));
    const char *reason = wwback_stale_ ? "no_telemetry" : (!wwback_accepted_ ? "board_ignored" :
                         ((wwback_ircv500_max_ < WRONGWAY_BACKCHECK_IRCV500) ? "no_wheel_motion" : nullptr));
    if (reason != nullptr) {
        LOGM_WARN(loggerIstroDrive2, "wrongwayBackCheck", "msg=\"wrongway backward FAILED!\", reason=\"" << reason
            << "\", back_dt=" << back_dt << ", velocity=" << wwback_velocity_ << ", ctrlb_state=" << wwback_ctrlb_state_
            << ", ctrlb_velocity=" << wwback_ctrlb_velocity_ << ", ircv500_max=" << ioff(wwback_ircv500_max_, 2)
            << ", end=\"" << end << "\"");
    } else {
        LOGM_INFO(loggerIstroDrive2, "wrongwayBackCheck", "msg=\"wrongway backward OK\", back_dt=" << back_dt
            << ", velocity=" << wwback_velocity_ << ", ctrlb_state=" << wwback_ctrlb_state_
            << ", ctrlb_velocity=" << wwback_ctrlb_velocity_ << ", ircv500_max=" << ioff(wwback_ircv500_max_, 2)
            << ", end=\"" << end << "\"");
    }
}

// ---- Publish helpers -- each mirrors one legacy ctrlBoard.XXX() call site ----

void DriveNode::publishSpeed(int speed)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::SpeedCommand msg;
    msg.set_speed = true;
    msg.speed = speed;
    pub_speed_->publish(msg);
    timeEnd("istro::drive_node.publishSpeed", tpub);
}

void DriveNode::publishSteering(int angle)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::SpeedCommand msg;
    msg.set_steering_angle = true;
    msg.steering_angle = angle;
    pub_speed_->publish(msg);
    timeEnd("istro::drive_node.publishSteering", tpub);
}

void DriveNode::publishLedProgram(int prg)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::LedConfig msg;
    msg.set_led_program = true;
    msg.led_program = prg;
    pub_led_->publish(msg);
    timeEnd("istro::drive_node.publishLedProgram", tpub);
}

void DriveNode::publishLedMask(int mask, int r, int g, int b, int blink)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::LedConfig msg;
    msg.set_led_mask = true;
    msg.led_mask = mask;
    msg.led_mask_r = r;
    msg.led_mask_g = g;
    msg.led_mask_b = b;
    msg.led_mask_blink = blink;
    pub_led_->publish(msg);
    timeEnd("istro::drive_node.publishLedMask", tpub);
}

void DriveNode::publishDisplayText(const std::string &text)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::DeviceAction msg;
    msg.display_text = true;
    msg.display_text_data = text;
    pub_action_->publish(msg);
    timeEnd("istro::drive_node.publishDisplayText", tpub);
}

void DriveNode::publishBallDrop(int cnt)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::DeviceAction msg;
    msg.set_ball_drop = true;
    msg.ball_drop_cnt = cnt;
    pub_action_->publish(msg);
    timeEnd("istro::drive_node.publishBallDrop", tpub);
}

void DriveNode::publishConfigVelocity()
{
    double tpub = timeBegin();

    istrorsx_core::msg::ConfigUpdate msg;
    msg.update_velocity = true;
    msg.velocity_fwd  = conf.velocityFwd;
    msg.velocity_fwd2 = conf.velocityFwd2;
    msg.velocity_fwd3 = conf.velocityFwd3;
    msg.velocity_back = conf.velocityBack;
    pub_config_->publish(msg);
    timeEnd("istro::drive_node.publishConfigVelocity", tpub);
}

// ---- Port of loop()'s per-iteration body ----

void DriveNode::controlTick()
{
    steering_change_ = 0;

    int process_stop = -1;
    int process_state = -1;
    int process_angle = (int)ANGLE_NONE;
    int process_velocity = -1;
    int ctrlb_state = 0;
    int ctrlb_velocity = -1;
    int gps_fix = -1;
    double gps_latitude = 0;
    double gps_longitude = 0;
    double gps_course = 0;
    double process_yaw = ANGLE_NONE;

    if (loopReadData(process_angle, process_velocity, process_state, process_stop, process_yaw,
            ctrlb_state, ctrlb_velocity, gps_fix, gps_latitude, gps_longitude, gps_course) < 0) {
        // Legacy loop() breaks out of for(;;) entirely on this failure (only
        // reachable once threads.close() has run) -- here it just means this
        // tick has nothing to act on; try again next tick.
        return;
    }

    if (steering_type_ == AUTONOMOUS) {
        /* keep track of the last process_velocity we saw */
        /* because process_thread often does not set any velocity (process_velocity = -1) */
        /* this complicates velocity control here, but also allows manually increasing robot speed */
        /* process_stop must always be set (>=0) though */
        if (process_stop > 0) {
            process_velocity_last_ = VEL_ZERO;
        } else if (process_velocity >= 0) {
            process_velocity_last_ = process_velocity;
        }

        int velocity_set = 0;    // send velocity even if its value doesn't change?
        if ((VELOCITY_SET_PERIOD > 0) && (timeDelta(velocity_lastt_) >= VELOCITY_SET_PERIOD)) {
            velocity_set = 1;
        }
        /* if the current robot speed is different compared to what we sent last time, then send the speed again! */
        if ((ctrlb_velocity >= 0) && (process_velocity_last_ >= 0) && (ctrlb_velocity != process_velocity_last_)) {
            if ((process_velocity < 0) || ((process_stop > 0) && (velocity_stop_ != 0))) {
                velocity_set = 1;
                LOGM_DEBUG(loggerIstroDrive2, "controlTick", "msg=\"ctrlb/process_velocity difference!\", ctrlb_velocity=" << ctrlb_velocity
                    << ", process_velocity_last=" << process_velocity_last_);
            }
        }

        if ((process_velocity_last_ == VEL_ZERO) &&
            (process_state == PROCESS_STATE_WRONGWAY || process_state == PROCESS_STATE_MIN_MAX ||
             process_state == PROCESS_STATE_NAV_ANGLE || process_state == PROCESS_STATE_NONE)) {
            if (timeDelta(prog_stopt_) >= PROG_STOP_TIME) {
                if (prog_stop_flag_ < 0) {
                    LOGM_INFO(loggerIstroDrive2, "controlTick", "msg=\"prog_stop start!\", process_state=" << process_state
                        << ", process_velocity_last=" << process_velocity_last_);
                }
                prog_stop_flag_ = 1;
            }
        } else {
            prog_stop_flag_ = 0;
            prog_stopt_ = timeBegin();
        }

        /* setSpeed */
        /* set velocity calculated by process_thread */
        if ((process_stop <= 0) && (process_velocity >= 0)) {
            // send new velocity only if there is no obstacle
            if (!ctrlb_obstState(ctrlb_state, process_velocity) && (ctrlb_state != CTRLB_STATE_EBTN)) {
                velocity_stop_ = 0;
                velocity_lastt_ = timeBegin();
                publishSpeed(process_velocity);
            }
        } else
        /* perform stop (if process_stop is forced) or resend last velocity */
        if ((process_stop >= 0) && (ctrlb_state != CTRLB_STATE_EBTN)) {
            if ((process_stop > 0) && ((velocity_stop_ == 0) || velocity_set)) {
                velocity_stop_ = 1;
                velocity_lastt_ = timeBegin();
                publishSpeed(VEL_ZERO);
                publishLedProgram(CTRLB_LED_PROGRAM_RED);    // red: wrongway / process_stop
            } else
            if (!ctrlb_obstState(ctrlb_state, process_velocity_last_) && (process_velocity_last_ >= 0)) {
                if ((process_stop == 0) && ((velocity_stop_ > 0) || velocity_set)) {
                    velocity_stop_ = 0;
                    velocity_lastt_ = timeBegin();
                    publishSpeed(process_velocity_last_);
                }
            }
        }

        /* setSteeringAngle - must have lower priority compared to speed (stopping the robot is more important than turning!) */
        if (process_angle < (int)ANGLE_OK) {
            steering_angle_ = (int)((process_angle - 90) * SA_MULT) + SA_STRAIGHT;
            publishSteering(steering_angle_);
        }

        /* BALL_DROP - every few seconds send how many balls should already be dropped */
        if ((process_state >= PROCESS_STATE_BALLDROP) && (process_state <= PROCESS_STATE_BALLDROP_9)) {
            if ((balldrop_lastt_ < 0) || (timeDelta(balldrop_lastt_) > 350)) {
                balldrop_lastt_ = timeBegin();
                publishBallDrop(process_state - PROCESS_STATE_BALLDROP);
            }
        } else {
            balldrop_lastt_ = -1;
        }

        /* update LED state */
        if ((timeDelta(led_to_) > 200) && (process_stop <= 0)) {
            led_to_ = timeBegin();
            if (process_state == PROCESS_STATE_NONE) {
                pstate_str_ = nullptr;
                publishLedProgram(CTRLB_LED_PROGRAM_WHITE);
            } else
            if (process_state == PROCESS_STATE_CALIBRATION) {
                pstate_str_ = "CALB";
                publishLedMask(255, 255, 255, 0, 0);            // yellow
            } else
            if (process_state == PROCESS_STATE_WRONGWAY) {
                pstate_str_ = "WROW";
                publishLedProgram(CTRLB_LED_PROGRAM_RED);       // red: wrongway / process_stop
            } else
            if (process_state == PROCESS_STATE_LOADING) {
                pstate_str_ = "LOAD";
                publishLedMask(255, 255, 0, 255, 0);            // magenta
            } else
            if (process_state == PROCESS_STATE_UNLOADING) {
                pstate_str_ = "UNLD";
                publishLedMask(255, 0, 255, 255, 0);            // cyan
            } else
            if (process_state == PROCESS_STATE_CONE_SEEK) {
                pstate_str_ = "CONE";
                publishLedMask(255, 255, 0, 255, 0);            // magenta
            } else
            if ((process_state >= PROCESS_STATE_BALLDROP) && (process_state <= PROCESS_STATE_BALLDROP_9)) {
                pstate_str_ = "BALL";
                publishLedMask(255, 0, 255, 255, 0);            // cyan
            } else
            if (process_state == PROCESS_STATE_NAV_ANGLE) {
                pstate_str_ = "NAVA";
                publishLedProgram(CTRLB_LED_PROGRAM_GREEN);
            } else
            if (process_state == PROCESS_STATE_MIN_MAX) {
                pstate_str_ = "MMAX";
                publishLedProgram(CTRLB_LED_PROGRAM_BLUE);
            } else
            if (process_state == PROCESS_STATE_QRSCAN_COORD) {
                pstate_str_ = "QRSC";
                publishLedMask(255, 255, 0, 255, 1);            // magenta+blink
            }
        }
    }

    wrongwayBackCheck(process_state, process_stop, ctrlb_state, ctrlb_velocity);

    int velocity_old = velocity_;

    /* handle key/joy input */
    if (loopWaitKey(steering_change_, steering_type_, steering_angle_,
            velocity_change_, velocity_, velocity_stop_, process_velocity, process_yaw, gps_latitude, gps_longitude) > 0) {
        LOGM_INFO(loggerIstroDrive2, "controlTick", "msg=\"quit requested\"");
        rclcpp::shutdown();
        return;
    }

    applyJoystickSteering();
    applyJoystickThrottle();

    if ((steering_type_ == MANUAL) && (steering_type_old_ != steering_type_)) {
        publishLedProgram(CTRLB_LED_PROGRAM_OFF);
    }
    steering_type_old_ = steering_type_;

    /* run the automatic steering/vehicle program */
    if ((steering_type_ == MANUAL) && (prog_type_ != PROG_TYPE_NONE)) {
        progLoop(ctrlb_state, steering_change_, steering_angle_, velocity_change_, velocity_);
    }

    if (steering_change_) {
        steering_change_ = 0;
        publishSteering(steering_angle_);
    }
    if (velocity_change_) {
        velocity_change_ = 0;
        publishSpeed(velocity_);
        /* update process_velocity_last */
        if ((velocity_old != velocity_) && (process_velocity_last_ == velocity_old)) {
            process_velocity_last_ = velocity_;
        }
    }

    if ((DISPLAY_PERIOD > 0) && (timeDelta(display_lastt_) >= DISPLAY_PERIOD)) {
        char ss[30];
        display_lastt_ = timeBegin();
        /* display format: "Af: S0 V098 A092 G180" */
        int vv = ((steering_type_ == AUTONOMOUS) && (process_velocity >= 0)) ? process_velocity : velocity_;
        snprintf(ss, sizeof(ss), "%c%c:S%dv%03d A%03d %s",
            (steering_type_ == AUTONOMOUS) ? ('A') : ((prog_type_ != PROG_TYPE_NONE) ? ('P') : ('M')),
            (ctrlb_state == CTRLB_STATE_EBTN) ? ('e') : ((ctrlb_obstState(ctrlb_state, vv)) ? 'o' : 'f'),
            velocity_stop_,
            vv,
            steering_angle_,
            ((process_stop > 0) ? ("STOP") : ((pstate_str_ == nullptr) ? ("-") : (pstate_str_))));
        publishDisplayText(ss);
    }
}

// Legacy loop()'s post-for(;;) shutdown tail.
void DriveNode::sendFinalStop()
{
    publishSpeed(VEL_ZERO);
    publishSteering(SA_STRAIGHT);
    publishDisplayText("bye...");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_drive(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroDrive2, "main", "msg=\"istro_main_drive() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<DriveNode>();
    rclcpp::spin(node);

    node->sendFinalStop();
    istro_close_drive();

    rclcpp::shutdown();
    return 0;
}
