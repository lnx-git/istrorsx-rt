# Output of the (not yet ported) planner_node -- consumed by control_node as
# the AUTONOMOUS-mode driving decision. Mirrors legacy loop_readData()'s
# process_angle/process_velocity/process_state/process_stop/process_yaw
# fields (istro_rt2025.cpp), which process_thread wrote into the shared
# DataSet and the main loop() read back every tick. process_thread combined
# navigation_node's route/position output with vision_node's perception
# output into this one final steering/speed decision -- see
# doc/ai/01_architecture.md § Planned Package Split.

int32 ANGLE_OK=999998   # legacy sentinel (config.h): angle field below is only acted on if angle < ANGLE_OK

int32 angle      # target steering angle; only acted on if angle < ANGLE_OK -- see control_node
int32 velocity   # target velocity (istrorsx_hw/msg/SpeedCommand's VEL_ZERO=335 is "stopped"); -1 = "not set this tick" (legacy semantics)
int32 state      # PROCESS_STATE_* (dataset.h): 0=NONE,1=CALIBRATION,2=WRONGWAY,3=LOADING,4=UNLOADING,5=NAV_ANGLE,6=MIN_MAX,7=QRSCAN_COORD,8=CONE_SEEK,10-19=BALLDROP
int32 stop       # <0 = not set, 0 = no stop requested, >0 = stop requested (overrides velocity; still lower priority than a hardware obstacle)
float64 yaw      # robot heading, legacy process_yaw
