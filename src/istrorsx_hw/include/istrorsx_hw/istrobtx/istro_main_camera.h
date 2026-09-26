#ifndef __ISTRO_MAIN_CAMERA_H__
#define __ISTRO_MAIN_CAMERA_H__

#include "camera.h"
#include "config.h"
#include "threads.h"

extern Config conf;
extern Camera camera;
extern Threads threads;

int istro_main_camera(int argc, char** argv);
void istro_close_camera(void);

#endif
