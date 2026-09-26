#ifndef __ISTRO_MAIN_CTRLBOARD_H__
#define __ISTRO_MAIN_CTRLBOARD_H__

#include "ctrlboard.h"
#include "threads.h"

extern ControlBoard ctrlBoard;
extern Threads threads;

int istro_main_ctrlboard(int argc, char** argv);
void istro_close_ctrlboard(void);

#endif
