#include "istrorsx_hw/ctrlboard_node.hpp"

#include "threads.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "dataset.h"
#include "istrorsx_hw/istrobtx/ctrlboard.h"
#include "istrorsx_hw/istrobtx/istro_main_ctrlboard.h"

LOG_DEFINE(loggerIstro2, "istro");

// KNOWN ISSUE (control board firmware/serial, found 2026-09-12): the board drops
// a command that arrives too soon after the previous one. Measured in the Sad JK
// logs: 8 of 13 stop commands were ignored, and every one of those was issued
// while other writes had gone out in the preceding 200 ms. Legacy paced its board
// writes with msleep(5) after each one (istro_rt2025.cpp:4524); that pacing was
// lost in the port, which sent steering and speed in the same millisecond.
//
// This restores the gap. It sleeps only when writes actually come back to back,
// so a lone command costs nothing. Note it blocks serial_read_cb() for the
// duration -- acceptable, the board buffers telemetry and legacy blocked the same
// way. See doc/ai/03_progress.md for the full analysis, including the second half
// of the problem (a dropped command is never re-sent in MANUAL) which is NOT
// fixed here.
static const int CTRLBOARD_WRITE_GAP_MS = 10;

void CtrlBoardNode::paceWrite()
{
    if (last_write_t_ >= 0) {
        double dt = timeDelta(last_write_t_);
        if (dt < CTRLBOARD_WRITE_GAP_MS) {
            msleep((long)(CTRLBOARD_WRITE_GAP_MS - dt));
        }
    }
    last_write_t_ = timeBegin();
}

CtrlBoardNode::CtrlBoardNode() : Node("ctrlboard_node")
{
    // Setup subscribers for custom hardware control messages
    sub_speed_ = this->create_subscription<istrorsx_hw::msg::SpeedCommand>(
        "/robot/speed_command", 10, std::bind(&CtrlBoardNode::cb_speed, this, std::placeholders::_1));

    sub_led_ = this->create_subscription<istrorsx_hw::msg::LedConfig>(
        "/robot/led_config", 10, std::bind(&CtrlBoardNode::cb_led, this, std::placeholders::_1));

    sub_action_ = this->create_subscription<istrorsx_hw::msg::DeviceAction>(
        "/robot/device_action", 10, std::bind(&CtrlBoardNode::cb_action, this, std::placeholders::_1));

    // Setup publishers for telemetry read back from the control board
    pub_imu_ = this->create_publisher<istrorsx_hw::msg::ImuData>("/robot/imu_data", 10);
    pub_servo_ = this->create_publisher<istrorsx_hw::msg::ServoData>("/robot/servo_data", 10);

    // Setup a 5 ms periodic timer for reading hardware status and sensors
    timer_serial_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&CtrlBoardNode::serial_read_cb, this)
    );
}

// Process incoming speed commands and motion states
void CtrlBoardNode::cb_speed(const istrorsx_hw::msg::SpeedCommand::SharedPtr msg) 
{
    if (msg->trigger_start) {
        paceWrite();
        ctrlBoard.start();
    }
    if (msg->trigger_stop) {
        paceWrite();
        ctrlBoard.stop();
    }
    if (msg->set_steering_angle) {
        paceWrite();
        ctrlBoard.setSteeringAngle(msg->steering_angle);
    }
    if (msg->set_speed) {
        paceWrite();
        ctrlBoard.setSpeed(msg->speed);
    }
}

// Process incoming LED color messages
void CtrlBoardNode::cb_led(const istrorsx_hw::msg::LedConfig::SharedPtr msg)
{
    if (msg->set_led_index) {
        paceWrite();
        ctrlBoard.setLedIndex(msg->led_index, msg->led_index_r, msg->led_index_g, msg->led_index_b, msg->led_index_blink);
    }
    if (msg->set_led_mask) {
        paceWrite();
        ctrlBoard.setLedMask(msg->led_mask, msg->led_mask_r, msg->led_mask_g, msg->led_mask_b, msg->led_mask_blink);
    }
    if (msg->set_led_program) {
        paceWrite();
        ctrlBoard.setLedProgram(msg->led_program);
    }
}

// Process hardware device actions, strings, and auxiliary triggers
void CtrlBoardNode::cb_action(const istrorsx_hw::msg::DeviceAction::SharedPtr msg)
{
    if (msg->display_text) {
        paceWrite();
        ctrlBoard.displayText(msg->display_text_data.c_str());
    }
    if (msg->write_string) {
        paceWrite();
        ctrlBoard.writeString(msg->write_string_data.c_str());
    }
    if (msg->set_xx4) {
        paceWrite();
        ctrlBoard.setXX4();
    }
    if (msg->set_xx5) {
        paceWrite();
        ctrlBoard.setXX5();
    }
    if (msg->set_ball_drop) {
        paceWrite();
        ctrlBoard.setBallDrop(msg->ball_drop_cnt);
    }
}

int ctrlBoard_writeData(double ctrlb_time1, int ctrlb_state, int ctrlb_ircv, double ctrlb_ircv500, int ctrlb_angle, int ctrlb_velocity, int ctrlb_loadd,
        double ctrlb_time2, double ctrlb_euler_x, double ctrlb_euler_y, double ctrlb_euler_z, 
        int ctrlb_calib_gyro, int ctrlb_calib_accel, int ctrlb_calib_mag, 
        int data1, int data2) 
{
    DataSet *pdata;

#ifdef THDATA_LOG_TRACE0
    double t = timeBegin();
#endif
    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    if (data1) {
        pdata->ctrlb_time1 = ctrlb_time1;
        pdata->ctrlb_state = ctrlb_state;
        pdata->ctrlb_ircv = ctrlb_ircv;
        pdata->ctrlb_ircv500 = ctrlb_ircv500;
        pdata->ctrlb_angle = ctrlb_angle; 
        pdata->ctrlb_velocity = ctrlb_velocity;
        pdata->ctrlb_loadd = ctrlb_loadd;
    }
    if (data2) {
        pdata->ctrlb_time2 = ctrlb_time2;
        pdata->ctrlb_euler_x = ctrlb_euler_x;
        pdata->ctrlb_euler_y = ctrlb_euler_y;
        pdata->ctrlb_euler_z = ctrlb_euler_z;
        pdata->ctrlb_calib_gyro = ctrlb_calib_gyro; 
        pdata->ctrlb_calib_accel = ctrlb_calib_accel;
        pdata->ctrlb_calib_mag = ctrlb_calib_mag;
    }

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    if (data1 && data2) {
        LOGM_DEBUG(loggerIstro2, "ctrlBoard_writeData", "state=" << ctrlb_state << 
            ", ctrlb_ircv=" << ctrlb_ircv << ", ircv500=" << ioff(ctrlb_ircv500, 2) << 
            ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity << ", ctrlb_loadd=" << ctrlb_loadd <<
            ", euler_x=" << ioff(ctrlb_euler_x, 2) << ", euler_y=" << ioff(ctrlb_euler_y, 2) << ", euler_z=" << ioff(ctrlb_euler_z, 2) << 
            ", calib_gyro=" << ctrlb_calib_gyro << ", calib_accel=" << ctrlb_calib_accel << ", calib_mag=" << ctrlb_calib_mag);
    } else
    if (data1 && !data2) {
        LOGM_DEBUG(loggerIstro2, "ctrlBoard_writeData", "state=" << ctrlb_state <<
            ", ctrlb_ircv=" << ctrlb_ircv << ", ircv500=" << ioff(ctrlb_ircv500, 2) << 
            ", ctrlb_angle=" << ctrlb_angle << ", ctrlb_velocity=" << ctrlb_velocity << ", ctrlb_loadd=" << ctrlb_loadd);
    } else
    if (!data1 && data2) {
        LOGM_DEBUG(loggerIstro2, "ctrlBoard_writeData", "euler_x=" << ioff(ctrlb_euler_x, 2) <<
            ", euler_y=" << ioff(ctrlb_euler_y, 2) << ", euler_z=" << ioff(ctrlb_euler_z, 2) <<
            ", calib_gyro=" << ctrlb_calib_gyro << ", calib_accel=" << ctrlb_calib_accel <<
            ", calib_mag=" << ctrlb_calib_mag);
    }

#ifdef THDATA_LOG_TRACE0
    timeEnd("istro::ctrlBoard_writeData", t);
#endif

    return 0;
}

void CtrlBoardNode::publishServoData(int state, double heading, int ircv, double ircv500, int angle, int velocity, int loadd, int cbtime,
    int ulsd1, int ulsd2, int ulsd3, int ulsd4, int ulsd5)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::ServoData servo_msg;
    servo_msg.state = state;
    servo_msg.heading = heading;
    servo_msg.ircv = ircv;
    servo_msg.ircv500 = ircv500;
    servo_msg.angle = angle;
    servo_msg.velocity = velocity;
    servo_msg.loadd = loadd;
    servo_msg.cbtime = cbtime;
    servo_msg.ulsd1 = ulsd1;
    servo_msg.ulsd2 = ulsd2;
    servo_msg.ulsd3 = ulsd3;
    servo_msg.ulsd4 = ulsd4;
    servo_msg.ulsd5 = ulsd5;
    pub_servo_->publish(servo_msg);
    timeEnd("istro::ctrlboard_node.publishServoData", tpub);
}

void CtrlBoardNode::publishImuData(double euler_x, double euler_y, double euler_z, int calib_gyro, int calib_accel, int calib_mag)
{
    double tpub = timeBegin();

    istrorsx_hw::msg::ImuData imu_msg;
    imu_msg.euler_x = euler_x;
    imu_msg.euler_y = euler_y;
    imu_msg.euler_z = euler_z;
    imu_msg.calib_gyro = calib_gyro;
    imu_msg.calib_accel = calib_accel;
    imu_msg.calib_mag = calib_mag;
    pub_imu_->publish(imu_msg);
    timeEnd("istro::ctrlboard_node.publishImuData", tpub);
}

// Periodically fetch IMU and servo feedback data from the serial port
// void *ctrlBoard_thread(void *parg)
void CtrlBoardNode::serial_read_cb()
{
    double ctrlb_time1;
    int    ctrlb_state;       // stav robota 1-STOP, 2-FWD, 5-OBST
    double ctrlb_heading;     // heading z kompasu (nepouziva sa)
    int    ctrlb_ircv;        // inkrement z IRC (Incremental rotary encoders) v casovom intervale, moze poslat 1 a viacej impulzov
    double ctrlb_ircv500;     // suma ircv za poslednych 500ms
    int    ctrlb_angle;       // aktualne nastavenie serva riadenia
    int    ctrlb_velocity;    // aktualne nastavenie motora
    int    ctrlb_loadd;       // detekcia nakladu (load detection)
    int    ctrlb_cbtime;      // cas v milisekundach: millis() % 10000
    int    ctrlb_ulsd1;       // lavy ultrazvuk v cm
    int    ctrlb_ulsd2;       // stredny/predny ultrazvuk v cm
    int    ctrlb_ulsd3;       // pravy ultrazvuk v cm
    int    ctrlb_ulsd4;       // zadny ultrazvuk v cm
    int    ctrlb_ulsd5;       // zadny ultrazvuk v cm

    double ctrlb_time2;
    double ctrlb_euler_x;
    double ctrlb_euler_y;
    double ctrlb_euler_z;
    int    ctrlb_calib_gyro;
    int    ctrlb_calib_accel;
    int    ctrlb_calib_mag;

    int res, res2;
    int result = 0;

    //LOG_THREAD_NAME("ctrlBoard");
    LOGM_INFO(loggerIstro2, "ctrlBoard_thread", "msg=\"start\"");

    do {
#ifdef CTRLBOARD_LOG_TRACE0
        double t = timeBegin();
#endif

        ctrlb_time1 = timeBegin();
        if ((res = ctrlBoard.getServoData(ctrlb_state, ctrlb_heading, ctrlb_ircv, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd, ctrlb_cbtime, 
                      ctrlb_ulsd1, ctrlb_ulsd2, ctrlb_ulsd3, ctrlb_ulsd4, ctrlb_ulsd5)) < 0) {
            result = -1;
            break;
        }

        ctrlb_time2 = timeBegin();
        if ((res2 = ctrlBoard.getImuData(ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag)) < 0) {
            result = -2;
            break;
        }

#ifdef CTRLBOARD_LOG_TRACE0
        timeEnd("istro::ctrlBoard_thread.capture", t);
#endif
        if ((res > 0) || (res2 > 0)) {
            if (ctrlBoard_writeData(ctrlb_time1, ctrlb_state, ctrlb_ircv, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd,
                    ctrlb_time2, ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag,
                    res > 0, res2 > 0) < 0) {
                result = -3;
                break;
            }

            if (res > 0) {
                publishServoData(ctrlb_state, ctrlb_heading, ctrlb_ircv, ctrlb_ircv500, ctrlb_angle, ctrlb_velocity, ctrlb_loadd, ctrlb_cbtime,
                    ctrlb_ulsd1, ctrlb_ulsd2, ctrlb_ulsd3, ctrlb_ulsd4, ctrlb_ulsd5);
            }

            if (res2 > 0) {
                publishImuData(ctrlb_euler_x, ctrlb_euler_y, ctrlb_euler_z, ctrlb_calib_gyro, ctrlb_calib_accel, ctrlb_calib_mag);
            }
#ifdef CTRLBOARD_LOG_TRACE0
            LOGM_DEBUG(loggerIstro2, "ctrlBoard_thread", "msg=\"data updated\"");
#endif
        }
    } while(0);

    if (result >= 0) {
        LOGM_INFO(loggerIstro2, "ctrlBoard_thread", "msg=\"exit(" << result << ")\"");
    } else {
        LOGM_ERROR(loggerIstro2, "ctrlBoard_thread", "msg=\"exit(" << result << ")\"");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_ctrlboard(argc, argv) != 0) {
        LOGM_ERROR(loggerIstro2, "main", "msg=\"istro_main_ctrlboard() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<CtrlBoardNode>();
    rclcpp::spin(node);

    istro_close_ctrlboard();

    rclcpp::shutdown();
    return 0;
}
