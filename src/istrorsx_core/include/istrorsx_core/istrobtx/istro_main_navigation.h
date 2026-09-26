#ifndef __ISTRO_MAIN_NAVIGATION_H__
#define __ISTRO_MAIN_NAVIGATION_H__

#include "config.h"
#include "geocalc.h"
#include "threads.h"

extern Config conf;
extern GeoCalc geoCalc;
extern Threads threads;

int istro_main_navigation(int argc, char** argv);
void istro_close_navigation(void);

#endif
