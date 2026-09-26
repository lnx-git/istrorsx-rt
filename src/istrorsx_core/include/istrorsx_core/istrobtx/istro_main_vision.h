#ifndef __ISTRO_MAIN_VISION_H__
#define __ISTRO_MAIN_VISION_H__

#include "config.h"
#include "threads.h"
#include "istrorsx_core/istrobtx/vision.h"
#include "istrorsx_core/istrobtx/qrscan.h"
#ifdef ISTRO_VISIONN
#include "istrorsx_core/istrobtx/visionn.h"
#endif
#ifdef ISTRO_VISION_DEPTH
#include "istrorsx_core/istrobtx/vision_depth.h"
#endif

extern Config conf;
extern Threads threads;

extern Vision vision;
#ifdef ISTRO_VISIONN
extern VisioNN visionn;
#endif
#ifdef ISTRO_VISION_DEPTH
extern Vision visiond_vsn;      // special Vision instance, own eps/ept, converts VisionDepth's obstacle mask into a DegreeMap the same way the color path does
extern VisionDepth visiond;
#endif
extern QRScanner qrscan;

int istro_main_vision(int argc, char** argv);
void istro_close_vision(void);

#endif
