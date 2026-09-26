# Constants shared by every package that builds a SpeedCommand.
#
# MUST MATCH src/istrobtx/include/ctrlboard_defs.h -- nothing enforces it, and
# SA_STRAIGHT was wrong here (335 vs 334) until 2026-09-11. These are raw board
# units: no documented m/s mapping, VEL_ZERO is the only guaranteed meaning (stop).
int32 SA_STRAIGHT=334
int32 SA_MIN=215
int32 SA_MAX=455
int32 VEL_ZERO=335

bool trigger_start

bool trigger_stop

bool set_steering_angle
int32 steering_angle

bool set_speed
int32 speed
