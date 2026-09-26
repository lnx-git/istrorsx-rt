#ifndef __ISTRO_MAIN_DRIVE_H__
#define __ISTRO_MAIN_DRIVE_H__

#include "config.h"
#include "threads.h"
#include "dataset.h"

extern Config conf;
extern Threads threads;

int istro_main_drive(int argc, char** argv);
void istro_close_drive(void);

#endif
