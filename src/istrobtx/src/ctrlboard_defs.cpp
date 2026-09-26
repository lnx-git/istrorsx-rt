#include "ctrlboard_defs.h"

#include "mtime.h"
#include "logger.h"

LOG_DEFINE(loggerCtrlboardObst, "ctrlboardObst");

double obstf_to = -1;    // when was the forward obstacle last seen?

const int CTRLB_OBSTF_TIMEOUT = 1000;  // v milisekundach

int ctrlb_obstState(int ctrlb_state, int ctrlb_velocity)
/* returns 1 if obstacle is detected in that direction in which the robot is moving */
{
    if ((ctrlb_velocity < 0) || (ctrlb_velocity == VEL_ZERO)) {
        return (ctrlb_state == CTRLB_STATE_OBSTA);
    }

    if (ctrlb_velocity >= VEL_ZERO) {
        int obstf = (ctrlb_state == CTRLB_STATE_OBSTF) || (ctrlb_state == CTRLB_STATE_OBSTA);
        // keep reporting obstacle forward (=1) for addidional CTRLB_OBSTF_TIMEOUT miliseconds
        if (obstf) {
            obstf_to = timeBegin();
        } else {
            if (obstf_to >= 0) {
                if (timeDelta(obstf_to) < CTRLB_OBSTF_TIMEOUT) {
                    obstf = 1;
                    LOGM_TRACE(loggerCtrlboardObst, "ctrlb_obstState", "msg=\"obstacle forward (time)!\", obstf=" << obstf
                        << ", dt=" << ioff(timeDelta(obstf_to), 2));
                } else {
                    LOGM_TRACE(loggerCtrlboardObst, "ctrlb_obstState", "msg=\"obstacle forward (timeout)!\", obstf=" << obstf
                        << ", dt=" << ioff(timeDelta(obstf_to), 2));
                    obstf_to = -1;
                }
            }
        }
        return obstf;
    } else {
        return (ctrlb_state == CTRLB_STATE_OBSTB) || (ctrlb_state == CTRLB_STATE_OBSTA);
    }
}
