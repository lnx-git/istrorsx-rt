#pragma once

#include "rclcpp/rclcpp.hpp"

#include "istrorsx_hw/msg/gps_data.hpp"
#include "istrorsx_core/msg/navigation_data.hpp"
#include "istrorsx_core/msg/navigation_route.hpp"
#include "istrorsx_core/msg/navigation_point_set.hpp"
#include "istrorsx_core/msg/planner_data.hpp"
#include "istrorsx_core/srv/nav_map_export.hpp"
#include "istrorsx_core/istrobtx/navig.h"

// Navigation node: a 1:1 port of istro_rt2025.cpp's gps_thread()'s
// navigation-point/route-planning extension (navig.cpp/navmap.cpp), the part
// of gps_thread NOT already ported into istrorsx_hw's gps_node (fix
// acquisition, last-fix distance/azimuth, gps_pt[] history -- all still done
// there). See doc/ai/01_architecture.md § navigation_node.
//
// gps_thread used to run this in the SAME loop iteration as the physical GPS
// read; now that gps_node (device polling) and navigation_node (this) are
// separate processes, navigation_node instead reacts to gps_node's published
// /robot/gps_data, once per message.
//
// Per project decision, reuses the legacy Threads/DataSet pattern exactly:
// every legacy thread's loop processed the shared DataSet (data written
// there by other threads, or -- here -- by an incoming ROS message), and
// wrote its own results back into it at the end. So cb_gps() writes the
// incoming fix into this process's own DataSet (gps_writeData()); then
// navigationTick() -- called from cb_gps(), one call = one legacy while(1)
// iteration -- opens by reading it straight back out into its own locals via
// navigation_readData() (NOT loopReadData() -- that name belongs to loop()'s
// own port in drive_node, a different legacy function; this is gps_thread's
// port, so it gets its own read function, matching legacy's per-thread
// readData/writeData naming), runs the ported computation, writes the
// results back in with navigation_writeData(), and publishes NavigationData.
// See doc/ai/05_migration_guide.md.
class NavigationNode : public rclcpp::Node {
public:
    NavigationNode();

private:
    // ---- ROS callback -- one call = one legacy gps_thread iteration's
    // navigation extension (the fix-acquisition part already happened in
    // gps_node, one process over) ----
    void cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg);
    // Caches planner_node's coneseek_stop/coneseek_intlen (ISTRO_VISION_ORANGECONE
    // mode only) -- resolves the gap that was hardcoded to -1/-1 in
    // navigationTick() before planner_node existed.
    void cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg);
    // Applies planner_node's QR-scan-resolved navigation point to THIS
    // process's own navigationPoint[]/navigationPointXY[] copy -- see the
    // .cpp's own comment on why a message is needed where legacy just made a
    // direct navigation_point_set() call.
    void cb_navigation_point_set(const istrorsx_core::msg::NavigationPointSet::SharedPtr msg);
    // NavMapExport.srv -- port of legacy navmap_test(): conf/<map>.osm -> out/<map>.kml + out/<map>.cpp.
    // Restores the live map afterwards, navmap_load() overwrites navMapNode[]/navMapSegment[].
    void handleNavMapExport(const std::shared_ptr<istrorsx_core::srv::NavMapExport::Request> request,
        std::shared_ptr<istrorsx_core::srv::NavMapExport::Response> response);

    // ---- DataSet round-trip (see class comment). All three follow legacy's
    // readData/writeData convention: return 0 on success, <0 on error -- a
    // caller that gets <0 must not go on to use the (unset) out-params, same
    // as gps_thread's own `if (gps_writeData(...) < 0) { result = -4; break; }`. ----
    int gps_writeData(int gps_fix, double gps_latitude_raw, double gps_longitude_raw);
    // Out-params, like drive_node's loopReadData() -- everything read here
    // is either used only within the current navigationTick() call, or (for
    // the navigation-point/route state below) already kept as members in its
    // own right, so there's no need to also stash gps_fix/gps_latitude_raw/
    // gps_longitude_raw as members.
    int navigation_readData(int &gps_fix, double &gps_latitude_raw, double &gps_longitude_raw);
    int navigation_writeData(int navp_idx, double navp_dist, double navp_dist_raw, double navp_azimuth, double navp_azimuth_raw,
        double navp_maxdist, int navp_loadarea, double navp_latitude, double navp_longitude, double navp_latitude_raw, double navp_longitude_raw,
        int gps_ref, double gps_x, double gps_y);

    // ---- Port of gps_thread's navigation-point/route-planning body ----
    void navigationTick();

    // Not a legacy port -- a plain publish helper, so unlike the *Data()
    // methods above it takes its values as parameters instead of going
    // through DataSet itself.
    void publishNavigationData(int navp_idx, double navp_dist, double navp_dist_raw, double navp_azimuth, double navp_azimuth_raw,
        double navp_maxdist, int navp_loadarea, double navp_latitude, double navp_longitude, double navp_latitude_raw, double navp_longitude_raw,
        int gps_ref, double gps_x, double gps_y);

    // Not a legacy port either -- publishes the current NAVMAP_FLAG_ROUTE-
    // flagged navMapSegment[] indices (excluding NAVMAP_FLAG_TMP_PLAN ones)
    // so save_node can apply the same flags onto its own separately-linked
    // navMapSegment[] copy before drawing navmap.png/.kml. See
    // NavigationRoute.msg's own comment for why.
    void publishNavigationRoute();

    // ---- Publisher ----
    rclcpp::Publisher<istrorsx_core::msg::NavigationData>::SharedPtr pub_navigation_;
    rclcpp::Publisher<istrorsx_core::msg::NavigationRoute>::SharedPtr pub_navigation_route_;

    // ---- Subscriber ----
    rclcpp::Subscription<istrorsx_hw::msg::GpsData>::SharedPtr sub_gps_;
    rclcpp::Subscription<istrorsx_core::msg::PlannerData>::SharedPtr sub_planner_data_;
    rclcpp::Subscription<istrorsx_core::msg::NavigationPointSet>::SharedPtr sub_navigation_point_set_;

    // ---- Service ----
    rclcpp::Service<istrorsx_core::srv::NavMapExport>::SharedPtr srv_navmap_export_;

    // ---- gps_thread's loop-persisted locals (declared once, before its
    // while(1)) -> members: only the ones actually read back across ticks
    // (e.g. when a tick has no gps fix and must fall back on the previous
    // tick's value). gps_fix/gps_latitude_raw/gps_longitude_raw/gps_latitude/
    // gps_longitude/gps_ref are fully reassigned at the top of every
    // navigationTick() call regardless, so they stay locals there instead --
    // see navigation_node.cpp. Fix/lastp_dist/azimuth/total_dist locals stay
    // in gps_node (not this node).
    int    navigationPathPos_ = 0;   // legacy file-scope global `int navigationPathPos`

    int    navp_idx_ = -1;
    double navp_latitude_ = 0;
    double navp_longitude_ = 0;
    double navp_latitude_raw_ = 0;
    double navp_longitude_raw_ = 0;
    int    navp_loadarea_ = NAVIGATION_AREA_NONE;
    int    navp_loadarea_next_ = NAVIGATION_AREA_NONE;

    double gps_navp_dist_ = -1;
    double gps_navp_dist_raw_ = -1;
    double gps_navp_azimuth_ = 0;
    double gps_navp_azimuth_raw_ = 0;
    double gps_navp_maxdist_ = -1;
    double gps_navp_mindist_ = -1;

    int    nmap_np_idx_ = -1;    // last navigation point used for route planning (#ifndef ISTRO_NAVMAP_NO_ROUTING)

    // gps_x/gps_y are only reassigned when a tick has a gps fix (see
    // navigationTick()), so -- unlike gps_ref, recomputed unconditionally
    // every tick -- they do need to persist across ticks.
    double gps_x_ = 0;
    double gps_y_ = 0;

    // planner_node's coneseek_stop/coneseek_intlen (ISTRO_VISION_ORANGECONE
    // mode only, istro_rt2025.cpp's process_thread), cached from
    // PlannerData.msg via cb_planner_data(). Unlike the gps_* locals above,
    // these DO need to persist across ticks -- gps_navp_dist/etc. only
    // update navigationTick()'s own state when a fresh GPS fix arrives, but
    // planner_node publishes PlannerData.msg on its own independent 20ms
    // schedule, so the latest value must be cached rather than read fresh
    // each navigationTick() call.
    int    coneseek_stop_ = -1;
    int    coneseek_intlen_ = -1;
};
