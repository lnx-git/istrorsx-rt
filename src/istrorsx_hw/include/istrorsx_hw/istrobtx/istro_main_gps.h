#ifndef __ISTRO_MAIN_GPS_H__
#define __ISTRO_MAIN_GPS_H__

#include "gpsdev.h"
#include "geocalc.h"
#include "threads.h"

extern GpsDevice gps;
extern GeoCalc geoCalc;
extern Threads threads;

// Normally defined in navmap.h (not ported -- see 05_migration_guide.md), but
// this struct itself has no other navmap dependency, so it's declared here
// directly for gps_pt[]'s sake (last N raw GPS positions, for KML export/display).
typedef struct {
    char   name[20];
    char   desc[200];
    char   style[20];
    double longitude;
    double latitude;
} aux_point_t;

const int GPS_POINT_NUM = 30;

extern int         gps_pt_cnt;
extern aux_point_t gps_pt[];

int istro_main_gps(int argc, char** argv);
void istro_close_gps(void);

#endif
