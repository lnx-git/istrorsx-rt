#ifndef __ISTRO_MAIN_SAVE_H__
#define __ISTRO_MAIN_SAVE_H__

#include "config.h"
#include "geocalc.h"
#include "threads.h"
#include "wmodel.h"
#include "istrorsx_core/istrobtx/vision.h"

extern Config conf;
extern Threads threads;

// Needed for navigation_init(&geoCalc) -- navmap_init()'s own navmap_calcXY()
// (and later, navmap_draw()'s per-tick midpoint conversion) both call
// navigation_getXY(), which silently returns x=y=0 until navigation_init()
// has set the reference point this instance provides -- see initDevices()'s
// own comment.
extern GeoCalc geoCalc;

// save_node's own WorldModel/WMGrid instance -- NOT shared with planner_node
// (which lives in a different process). Populated fresh from each
// GetWMGridSnapshot.srv response; the only place in this whole port that
// ever calls WMGrid::drawGrid()/drawGridFull() (see 01_architecture.md's
// planner_node decision #4).
extern WorldModel wmodel;

// save_node's own Vision instance -- needed for Vision::drawOutput()'s
// "vision"/"rvision" overlay image, which reads this->epoints/epdist/elines/
// epmask (populated by init() below, same calibration samples vision_node's
// own instance uses). Only drawOutput() is ever called on it here -- no
// eval()/QR-scan/NN round trip, this instance never processes a live frame,
// only re-draws markers/weights vision_node already computed and published
// via VisionDebugData.msg.
extern Vision vision;

// Legacy's own per-run random session id (istro_rt2025.cpp's main(),
// "srand(time(NULL)); save_rand = rand() % 1000 + ...") -- disambiguates
// output filenames across restarts within the same shared out/ directory.
// Generated once in istro_main_save(), matching legacy's own placement
// (before threads.start(), i.e. before the node that uses it is even
// constructed).
extern long save_rand;

int istro_main_save(int argc, char** argv);
void istro_close_save(void);

#endif
