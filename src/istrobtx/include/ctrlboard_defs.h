#ifndef __CTRLBOARD_DEFS_H__
#define __CTRLBOARD_DEFS_H__

// Control-board calibration constants and state codes, split out of
// ctrlboard.h so code that only needs these numbers (Config::Config(),
// istrorsx_core's drive_node) doesn't have to pull in the ControlBoard
// class/serial I/O itself. ctrlboard.h (istrorsx_hw) still includes this and
// keeps the rest.
//
// The same values are also declared as message constants in
// istrorsx_hw/msg/SpeedCommand.msg (SA_STRAIGHT/SA_MIN/SA_MAX/VEL_ZERO) and
// istrorsx_hw/msg/ServoData.msg (STATE_*), for ROS-message-only consumers
// that don't link this header -- keep both in sync if these ever change.

// Steering angle limits
#define SA_STRAIGHT    334    //340    // upravene 13.4.2019 na 340, lebo pri 334 zatacal moc vlavo
#define SA_MIN         215    //SA_STRAIGHT-50
#define SA_MAX         455    //SA_STRAIGHT+50
#define SA_FIX         (SA_STRAIGHT - 90)
#define SA_MULT        (120.0 / 90)

// Velocity limits
#define VEL_ZERO    335
#define VEL_MAX     VEL_ZERO + 445
#define VEL_MIN     VEL_ZERO - 222
#define VEL_OPT     VEL_ZERO + 8     // lipol: (8 + 7)
#define VEL_BACK    VEL_ZERO - 11    // lipol: (-10 - 5)
//#define VEL_HIGH    VEL_ZERO + 16

#define CTRLB_STATE_START  0   // nic nerobi
#define CTRLB_STATE_STOP   1   // zastavi robota, ale ak je predchadzajuci stav bol stop tak nic nerobi
#define CTRLB_STATE_FWD    2   // robot ide dopredu, pozera ci nie je prekazka pred robotom, prepina do stavu OBSTF
#define CTRLB_STATE_BCK    3   // robot ide dozadu, pozera ci nie je prekazka za robotom robotom, prepina do stavu OBSTB
#define CTRLB_STATE_RECO   4   // recovery - nepouziva sa
#define CTRLB_STATE_OBSTF  5   // prekazka vpredu; nedovoli ist robotovi dopredu, ak predchadzajuci stav bol fwd tak zastavi robota
#define CTRLB_STATE_OBSTB  6   // prekazka vzadu; nedovoli ist robotovi dozadu, ak predchadzajuci stab bol bck tak zastavi robota
#define CTRLB_STATE_OBSTA  7   // prekazka vpredu aj vzadu; v podstate ako stop a nedovoli ist robotovi ani dopredu ani dozadu
#define CTRLB_STATE_EBTN   8   // emergency button; bolo stlacene emergency tlacitko, zastavi robota ak ebtn bol predchadzajuci stav iny ako ebtn inac nic nerobi

#define CTRLB_STATE_OBSTACLE(s) ((s == CTRLB_STATE_OBSTF) || (s == CTRLB_STATE_OBSTB) || (s == CTRLB_STATE_OBSTA) || (s == CTRLB_STATE_EBTN))

#define CTRLB_LED_PROGRAM_OFF    0
#define CTRLB_LED_PROGRAM_WHITE  1
#define CTRLB_LED_PROGRAM_RED    2
#define CTRLB_LED_PROGRAM_GREEN  3
#define CTRLB_LED_PROGRAM_BLUE   4

// Pure decision logic on ctrlb_state/ctrlb_velocity ints, no ControlBoard/
// serial-I/O dependency -- split out of ctrlboard.cpp (istrorsx_hw) so
// istrorsx_core's drive_node can reuse the exact same obstacle-priority
// logic ctrlboard.cpp itself no longer needs to duplicate. Was ctrlboard_obst.h/.cpp,
// merged in here (2026-08-27) so this project has one consistent
// "xxx_defs.h/.cpp" name for a subsystem's shared-but-hardware-independent
// pieces, rather than a mix of "_defs"/"_types"/"_obst"-style names.
int ctrlb_obstState(int ctrlb_state, int ctrlb_velocity);
/* returns 1 if obstacle is detected in that direction in which the robot is moving */

#endif
