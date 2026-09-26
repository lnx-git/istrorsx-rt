# state values, ported from the legacy CTRLB_STATE_* #defines (ctrlboard.h).
# Declared here so any package reading `state` (not only istrorsx_hw itself)
# shares one canonical source instead of re-declaring the same numbers.
int32 STATE_START=0   # does nothing
int32 STATE_STOP=1    # robot stopped; no-op if the previous state was already STOP
int32 STATE_FWD=2     # robot driving forward, watching for a forward obstacle -> switches to STATE_OBSTF
int32 STATE_BCK=3     # robot driving backward, watching for a rear obstacle -> switches to STATE_OBSTB
int32 STATE_RECO=4    # recovery -- unused
int32 STATE_OBSTF=5   # obstacle in front; blocks forward motion, stops the robot if it was previously STATE_FWD
int32 STATE_OBSTB=6   # obstacle behind; blocks backward motion, stops the robot if it was previously STATE_BCK
int32 STATE_OBSTA=7   # obstacle both front and back; effectively a stop, blocks motion in either direction
int32 STATE_EBTN=8    # emergency button pressed; hardware itself stops the robot, no-op if already STATE_EBTN

int32 state       # robot state, one of the STATE_* constants above
float64 heading   # compass heading (not used)
int32 ircv        # increment from IRC (Incremental rotary encoders) in time interval, can send 1 or more pulses
float64 ircv500   # sum of ircv over the last 500ms

int32 angle       # current steering servo setting
int32 velocity    # current motor setting
int32 loadd       # load detection
int32 cbtime      # time in milliseconds: millis() % 10000

int32 ulsd1       # left ultrasonic sensor in cm
int32 ulsd2       # middle/front ultrasonic sensor in cm
int32 ulsd3       # right ultrasonic sensor in cm
int32 ulsd4       # rear ultrasonic sensor in cm
int32 ulsd5       # rear ultrasonic sensor in cm
