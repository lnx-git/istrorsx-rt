# Output of navigation_node -- port of legacy gps_thread's navigation-point/
# route-planning extension (istro_rt2025.cpp, navig.cpp/navmap.cpp), which
# gps_thread wrote into the shared DataSet via gps_writeData() and other
# threads (process_thread) read back. Intended for the future planner_node's
# loadarea_process()/steering decisions -- see doc/ai/01_architecture.md.
# Mirrors DataSet's gps_navp_ / gps_ref / gps_x / gps_y fields exactly
# (dataset.h, istrobtx), with the "gps_" / "gps_navp_" prefix dropped, same
# convention as DriveCommand.msg dropping "process_".

int32   navp_idx             # index into navigationPoint[] of the next point on the route, -1 = none
float64 navp_dist            # distance to the next navigation point, metres (route-corrected)
float64 navp_dist_raw        # distance to the next navigation point, metres (straight-line GPS)
float64 navp_azimuth         # azimuth to the next navigation point, degrees (route-corrected)
float64 navp_azimuth_raw     # azimuth to the next navigation point, degrees (straight-line GPS)
float64 navp_maxdist         # maximum distance seen to the current navigation point (needed for calibration)
int32   navp_loadarea        # NAVIGATION_AREA_* (istrorsx_core/istrobtx/navig.h): NONE=-1, LOADING=0, UNLOADING=1, BALLDROP=10
float64 navp_latitude        # next navigation point's latitude (route-corrected)
float64 navp_longitude       # next navigation point's longitude (route-corrected)
float64 navp_latitude_raw    # next navigation point's latitude, from navigationPoint[] (raw, pre-route-matching)
float64 navp_longitude_raw   # next navigation point's longitude, from navigationPoint[] (raw, pre-route-matching)

int32   ref     # 1 = reference point initialized, 0 = not yet (navigation_getXY()/x/y below are meaningless until this is 1)
float64 x       # robot position, local XY metres, relative to the reference point (navigation_getXY())
float64 y       # robot position, local XY metres, relative to the reference point (navigation_getXY())
