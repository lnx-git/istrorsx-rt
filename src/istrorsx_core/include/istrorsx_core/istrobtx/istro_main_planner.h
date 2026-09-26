#ifndef __ISTRO_MAIN_PLANNER_H__
#define __ISTRO_MAIN_PLANNER_H__

#include "config.h"
#include "threads.h"
#include "wmodel.h"

extern Config conf;
extern Threads threads;

// The legacy process_thread's global WorldModel instance (istro_rt2025.cpp:
// "WorldModel wmodel;") -- planner_node's own grid, updated via
// updateGrid()/evalGrid() only. Never drawn here: WMGrid::drawGrid()/
// drawGridFull() stay exclusive to the future save_node (see
// doc/ai/01_architecture.md's planner_node decision #4), which gets its own
// separate WorldModel instance fed by planner_node's GetWMGridSnapshot.srv
// service, not this one.
extern WorldModel wmodel;

int istro_main_planner(int argc, char** argv);
void istro_close_planner(void);

#endif
