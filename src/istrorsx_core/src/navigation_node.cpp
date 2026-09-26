#include "istrorsx_core/navigation_node.hpp"

#include "config.h"
#include "threads.h"
#include "dataset.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "geocalc.h"
#include "istrorsx_core/istrobtx/istro_main_navigation.h"
#include "istrorsx_core/istrobtx/navig.h"
#include "istrorsx_core/istrobtx/navmap.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <unistd.h>
#include <vector>

LOG_DEFINE(loggerIstroNavigation2, "istroNavigation");

// Legacy file-scope consts, declared right before gps_thread() in
// istro_rt2025.cpp -- see navmap.h for NAVMAP_FLAG_ROUTE.
static const double NAVMAP_ROUTEFWD_DIST = 10.0;   // set navigation point 10 metres ahead on route
static const double NAVMAP_REROUTE_DIST  = 10.0;

NavigationNode::NavigationNode() : Node("navigation_node")
{
    pub_navigation_ = this->create_publisher<istrorsx_core::msg::NavigationData>("/robot/navigation_data", 10);
    pub_navigation_route_ = this->create_publisher<istrorsx_core::msg::NavigationRoute>("/robot/navigation_route", 10);

    sub_gps_ = this->create_subscription<istrorsx_hw::msg::GpsData>(
        "/robot/gps_data", 10,
        [this](const istrorsx_hw::msg::GpsData::SharedPtr msg) { cb_gps(msg); });
    sub_planner_data_ = this->create_subscription<istrorsx_core::msg::PlannerData>(
        "/robot/planner_data", 10,
        [this](const istrorsx_core::msg::PlannerData::SharedPtr msg) { cb_planner_data(msg); });
    sub_navigation_point_set_ = this->create_subscription<istrorsx_core::msg::NavigationPointSet>(
        "/robot/navigation_point_set", 10,
        [this](const istrorsx_core::msg::NavigationPointSet::SharedPtr msg) { cb_navigation_point_set(msg); });

    srv_navmap_export_ = this->create_service<istrorsx_core::srv::NavMapExport>(
        "/robot/navmap_export",
        [this](const std::shared_ptr<istrorsx_core::srv::NavMapExport::Request> request,
               std::shared_ptr<istrorsx_core::srv::NavMapExport::Response> response) { handleNavMapExport(request, response); });

    // Port of gps_thread's pre-while(1) initialization (istro_rt2025.cpp):
    // prime the first navigation point from conf.navigationPath, once, at
    // startup -- not gated by a gps fix.
    if (conf.useNavigation) {
        navigation_next_point(conf.navigationPath, navigationPathPos_, navp_latitude_raw_, navp_longitude_raw_, navp_loadarea_next_, navp_idx_);
        navp_latitude_ = navp_latitude_raw_;
        navp_longitude_ = navp_longitude_raw_;
    }
}

// Splits the loaded navMapNode[]/navMapSegment[] graph into connected components and reports
// everything outside the largest one -- route planning can never reach it.
static void navmapCheckConnectivity(istrorsx_core::srv::NavMapExport::Response &response)
{
    std::vector<int> parent(navmap_node_cnt);
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&parent](int i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    for (int j = 0; j < navmap_segment_cnt; j++) {
        parent[find(navMapSegment[j].node1_idx)] = find(navMapSegment[j].node2_idx);
    }

    std::map<int, std::vector<int>> comp_nodes, comp_segments;
    for (int i = 0; i < navmap_node_cnt; i++) {
        comp_nodes[find(i)].push_back(i);
    }
    for (int j = 0; j < navmap_segment_cnt; j++) {
        comp_segments[find(navMapSegment[j].node1_idx)].push_back(j);
    }

    int main_root = -1;
    for (const auto &c : comp_nodes) {
        if ((main_root < 0) || (c.second.size() > comp_nodes[main_root].size())) {
            main_root = c.first;
        }
    }
    response.component_cnt = (int)comp_nodes.size();
    response.main_node_cnt = (main_root < 0) ? 0 : (int)comp_nodes[main_root].size();
    response.main_segment_cnt = (main_root < 0) ? 0 : (int)comp_segments[main_root].size();

    std::vector<std::pair<int, int>> others;    // (node count, root), largest first
    for (const auto &c : comp_nodes) {
        if (c.first != main_root) {
            others.push_back({(int)c.second.size(), c.first});
        }
    }
    std::sort(others.begin(), others.end(), std::greater<std::pair<int, int>>());

    std::set<long long> all_ways;
    int k = 0;
    for (const auto &o : others) {
        const std::vector<int> &nodes = comp_nodes[o.second];
        std::set<long long> ways;
        for (int j : comp_segments[o.second]) {
            ways.insert(navMapSegment[j].way_id);
        }
        std::ostringstream way_ids, node_ids;
        for (long long w : ways) {
            way_ids << (way_ids.tellp() > 0 ? "," : "") << w;
            all_ways.insert(w);
        }
        for (int i : nodes) {
            node_ids << (node_ids.tellp() > 0 ? "," : "") << navMapNode[i].node_id;
            response.disconnected_node_ids.push_back(navMapNode[i].node_id);
        }
        LOGM_WARN(loggerIstroNavigation2, "handleNavMapExport", "msg=\"navmap component not connected to the main graph!\""
            << ", component=" << ++k << ", node_cnt=" << nodes.size() << ", segment_cnt=" << comp_segments[o.second].size()
            << ", lat=" << ioff(navMapNode[nodes[0]].latitude, 7) << ", lon=" << ioff(navMapNode[nodes[0]].longitude, 7)
            << ", way_ids=\"" << way_ids.str() << "\", node_ids=\"" << node_ids.str() << "\"");
    }
    response.disconnected_way_ids.assign(all_ways.begin(), all_ways.end());
}

void NavigationNode::handleNavMapExport(const std::shared_ptr<istrorsx_core::srv::NavMapExport::Request> request,
    std::shared_ptr<istrorsx_core::srv::NavMapExport::Response> response)
{
    std::string name = request->map_name;
    if ((name.size() > 4) && (name.compare(name.size() - 4, 4, ".osm") == 0)) {
        name.resize(name.size() - 4);
    }
    response->osm_file = "conf/" + name + ".osm";
    response->kml_file = "out/" + name + ".kml";
    response->cpp_file = "out/" + name + ".cpp";
    response->success = false;

    if (name.empty() || (name.find('/') != std::string::npos) || (name.find("..") != std::string::npos)) {
        response->message = "invalid map_name (expected a plain name such as navmap-ba-sadjk)";
    } else if (access(response->osm_file.c_str(), R_OK) != 0) {
        response->message = "cannot read " + response->osm_file;
    } else {
        // navmap_load() replaces the map navigation is using -- keep a copy and put it back
        std::vector<navmap_node_t> nodes(navMapNode, navMapNode + navmap_node_cnt);
        std::vector<navmap_segment_t> segments(navMapSegment, navMapSegment + navmap_segment_cnt);

        if (navmap_load(response->osm_file) < 0) {
            response->message = "navmap_load() failed for " + response->osm_file;
        } else {
            response->node_cnt = navmap_node_cnt;
            response->segment_cnt = navmap_segment_cnt;
            if ((navmap_node_cnt > 0) && (navmap_segment_cnt > 0)) {
                navmapCheckConnectivity(*response);
            }
            if ((navmap_node_cnt <= 0) || (navmap_segment_cnt <= 0)) {
                response->message = "no node/segment loaded from " + response->osm_file;
            } else if (navmap_export_kml(response->kml_file) < 0) {
                response->message = "cannot write " + response->kml_file;
            } else if (navmap_export_data(response->cpp_file) < 0) {
                response->message = "cannot write " + response->cpp_file;
            } else {
                response->success = true;
                response->message = (response->component_cnt > 1)
                    ? "ok, " + std::to_string(response->component_cnt - 1) + " component(s) not connected to the main graph"
                    : "ok";
            }
        }

        std::copy(nodes.begin(), nodes.end(), navMapNode);
        std::copy(segments.begin(), segments.end(), navMapSegment);
        navmap_node_cnt = (int)nodes.size();
        navmap_segment_cnt = (int)segments.size();
    }

    if (response->success) {
        LOGM_INFO(loggerIstroNavigation2, "handleNavMapExport", "msg=\"navmap exported\", osm_file=\"" << response->osm_file
            << "\", kml_file=\"" << response->kml_file << "\", cpp_file=\"" << response->cpp_file
            << "\", node_cnt=" << response->node_cnt << ", segment_cnt=" << response->segment_cnt
            << ", component_cnt=" << response->component_cnt << ", main_node_cnt=" << response->main_node_cnt
            << ", main_segment_cnt=" << response->main_segment_cnt
            << ", navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt);
    } else {
        LOGM_ERROR(loggerIstroNavigation2, "handleNavMapExport", "msg=\"navmap export failed!\", map_name=\"" << request->map_name
            << "\", error=\"" << response->message << "\"");
    }
}

int NavigationNode::gps_writeData(int gps_fix, double gps_latitude_raw, double gps_longitude_raw)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_fix = gps_fix;
    pdata->gps_latitude_raw = gps_latitude_raw;
    pdata->gps_longitude_raw = gps_longitude_raw;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroNavigation2, "gps_writeData", "fix=" << gps_fix
        << ", latitude_raw=" << ioff(gps_latitude_raw, 6) << ", longitude_raw=" << ioff(gps_longitude_raw, 6));

    return 0;
}

// Port of gps_thread's own read side -- in the legacy single-process app,
// gps_thread read the physical device directly, not the shared DataSet; here
// the "physical device" is another process (gps_node), so the data crosses
// via a message + DataSet round-trip instead (see class comment). Named
// navigation_readData(), NOT loopReadData() -- that name is reserved for
// loop()'s own port in drive_node (istro_rt2025.cpp's loop_readData()),
// a different legacy function entirely. See doc/ai/05_migration_guide.md.
int NavigationNode::navigation_readData(int &gps_fix, double &gps_latitude_raw, double &gps_longitude_raw)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    gps_fix = pdata->gps_fix;
    gps_latitude_raw = pdata->gps_latitude_raw;
    gps_longitude_raw = pdata->gps_longitude_raw;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int NavigationNode::navigation_writeData(int navp_idx, double navp_dist, double navp_dist_raw, double navp_azimuth, double navp_azimuth_raw,
    double navp_maxdist, int navp_loadarea, double navp_latitude, double navp_longitude, double navp_latitude_raw, double navp_longitude_raw,
    int gps_ref, double gps_x, double gps_y)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->gps_navp_idx = navp_idx;
    pdata->gps_navp_dist = navp_dist;
    pdata->gps_navp_dist_raw = navp_dist_raw;
    pdata->gps_navp_azimuth = navp_azimuth;
    pdata->gps_navp_azimuth_raw = navp_azimuth_raw;
    pdata->gps_navp_maxdist = navp_maxdist;
    pdata->gps_navp_loadarea = navp_loadarea;
    pdata->gps_navp_latitude = navp_latitude;
    pdata->gps_navp_longitude = navp_longitude;
    pdata->gps_navp_latitude_raw = navp_latitude_raw;
    pdata->gps_navp_longitude_raw = navp_longitude_raw;

    pdata->gps_ref = gps_ref;
    pdata->gps_x = gps_x;
    pdata->gps_y = gps_y;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    LOGM_DEBUG(loggerIstroNavigation2, "navigation_writeData", "navp_idx=" << navp_idx
        << ", navp_dist=" << ioff(navp_dist, 3) << ", navp_azimuth=" << ioff(navp_azimuth, 2)
        << ", navp_loadarea=" << navp_loadarea << ", gps_ref=" << gps_ref
        << ", gps_x=" << ioff(gps_x, 2) << ", gps_y=" << ioff(gps_y, 2));

    return 0;
}

// Not a legacy port -- see navigation_node.hpp.
void NavigationNode::publishNavigationData(int navp_idx, double navp_dist, double navp_dist_raw, double navp_azimuth, double navp_azimuth_raw,
    double navp_maxdist, int navp_loadarea, double navp_latitude, double navp_longitude, double navp_latitude_raw, double navp_longitude_raw,
    int gps_ref, double gps_x, double gps_y)
{
    double tpub = timeBegin();

    istrorsx_core::msg::NavigationData msg;
    msg.navp_idx = navp_idx;
    msg.navp_dist = navp_dist;
    msg.navp_dist_raw = navp_dist_raw;
    msg.navp_azimuth = navp_azimuth;
    msg.navp_azimuth_raw = navp_azimuth_raw;
    msg.navp_maxdist = navp_maxdist;
    msg.navp_loadarea = navp_loadarea;
    msg.navp_latitude = navp_latitude;
    msg.navp_longitude = navp_longitude;
    msg.navp_latitude_raw = navp_latitude_raw;
    msg.navp_longitude_raw = navp_longitude_raw;
    msg.ref = gps_ref;
    msg.x = gps_x;
    msg.y = gps_y;

    pub_navigation_->publish(msg);
    timeEnd("istro::navigation_node.publishNavigationData", tpub);
}

void NavigationNode::publishNavigationRoute()
{
    double tpub = timeBegin();

    istrorsx_core::msg::NavigationRoute msg;

    // Temporary nodes sit at the tail of navMapNode[] (navmap_node_add() appends,
    // navmap_plan_delete() drops them again), so the first one marks where the
    // stable base graph ends.
    int base_node_count = navmap_node_cnt;
    for (int i = 0; i < navmap_node_cnt; i++) {
        if ((navMapNode[i].flags & NAVMAP_FLAG_TMP_PLAN) == 0) continue;
        if (msg.tmp_nodes.empty()) {
            base_node_count = i;
        }
        istrorsx_core::msg::NavMapNode n;
        n.node_id   = navMapNode[i].node_id;
        n.latitude  = navMapNode[i].latitude;
        n.longitude = navMapNode[i].longitude;
        n.flags     = navMapNode[i].flags;
        msg.tmp_nodes.push_back(n);
    }
    msg.base_node_count = base_node_count;

    if ((int)(base_node_count + msg.tmp_nodes.size()) != navmap_node_cnt) {
        LOGM_ERROR(loggerIstroNavigation2, "publishNavigationRoute", "msg=\"temporary nodes are not contiguous at the tail!\""
            << ", base_node_count=" << base_node_count << ", tmp_nodes=" << msg.tmp_nodes.size()
            << ", navmap_node_cnt=" << navmap_node_cnt);
    }

    for (int j = 0; j < navmap_segment_cnt; j++) {
        int flags = navMapSegment[j].flags;
        if ((flags & NAVMAP_FLAG_TMP_PLAN) > 0) {
            istrorsx_core::msg::NavMapSegment sg;
            sg.way_id    = navMapSegment[j].way_id;
            sg.node1_idx = navMapSegment[j].node1_idx;
            sg.node2_idx = navMapSegment[j].node2_idx;
            sg.flags     = flags;
            msg.tmp_segments.push_back(sg);
            continue;
        }
        if ((flags & NAVMAP_FLAG_ROUTE) > 0) {
            msg.route_segment_indices.push_back(j);
        }
        if ((flags & NAVMAP_FLAG_INVISIBLE) > 0) {
            msg.invisible_segment_indices.push_back(j);
        }
    }

    pub_navigation_route_->publish(msg);
    timeEnd("istro::navigation_node.publishNavigationRoute", tpub);
}

// Port of gps_thread's navigation-point/route-planning extension
// (istro_rt2025.cpp) -- one call = one legacy while(1) iteration's worth of
// this extension.
void NavigationNode::navigationTick()
{
    int gps_fix;
    double gps_latitude_raw, gps_longitude_raw;
    if (navigation_readData(gps_fix, gps_latitude_raw, gps_longitude_raw) < 0) {
        LOGM_ERROR(loggerIstroNavigation2, "navigationTick", "msg=\"navigation_readData() failed!\"");
        return;
    }

    // ---- reference point / gps_x,gps_y ----
    double gps_ref_latitude = ANGLE_NONE;
    double gps_ref_longitude = ANGLE_NONE;
    int gps_ref = navigation_ref_get(gps_ref_latitude, gps_ref_longitude);

    if (gps_fix > 0) {
        if (gps_ref <= 0) {
            LOGM_ERROR(loggerIstroNavigation2, "navigationTick", "msg=\"error: reference point not initialized!\"");
            return;
        }
        navigation_getXY(gps_latitude_raw, gps_longitude_raw, gps_x_, gps_y_);
    } else {
        gps_latitude_raw = ANGLE_NONE;
        gps_longitude_raw = ANGLE_NONE;
    }
    double gps_latitude = gps_latitude_raw;
    double gps_longitude = gps_longitude_raw;

    // coneseek_stop/coneseek_intlen (ISTRO_VISION_ORANGECONE mode only) --
    // cached from planner_node's PlannerData.msg via cb_planner_data(),
    // defaults to -1/-1 ("not set") until the first message arrives.
    int coneseek_stop = coneseek_stop_;
    int coneseek_intlen = coneseek_intlen_;

    if (gps_fix > 0) {
        // calculate distance and azimuth to next navigation point, check if it was passed
        if (conf.useNavigation) {
            // wait for unknown navigation coordinates - have to be scanned from a QR-code
            if ((navp_idx_ >= 0) && ((navp_latitude_raw_ >= ANGLE_OK) || (navp_longitude_raw_ >= ANGLE_OK))) {
                navigation_point_get(navp_idx_, navp_latitude_raw_, navp_longitude_raw_);
                navp_latitude_ = navp_latitude_raw_;
                navp_longitude_ = navp_longitude_raw_;
            }
            if ((navp_latitude_raw_ < ANGLE_OK) && (navp_longitude_raw_ < ANGLE_OK)) {
                geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw_, navp_longitude_raw_, gps_navp_dist_raw_, gps_navp_azimuth_raw_);
                gps_navp_dist_ = gps_navp_dist_raw_;
                gps_navp_azimuth_ = gps_navp_azimuth_raw_;
                int point_passed = gps_navp_dist_raw_ < NAVIGATION_DISTANCE_THRESHOLD;
                // do we need to approach this point very closely (un/loading area)?
                int navp_approach = navigation_approach(conf.navigationPath, navigationPathPos_ - 2);
                if (point_passed && navp_approach) {
                    // ak sme vo vzdialenosti do 10m a vidime kuzel, tak ignorujeme GPS vzdialenost
                    // a cakame na priblizenie dostatocne blizko ku kuzelu
                    if (coneseek_intlen < 0) {
                        // wait for passing the second threshold or detecting that the distance is greater than the previous one
                        point_passed = (gps_navp_dist_raw_ < NAVIGATION_DISTANCE_THRESHOLD2) ||
                                       (gps_navp_dist_raw_ >= gps_navp_mindist_ + 1.5);
                    } else {
                        point_passed = (coneseek_stop > 0);
                    }
                }
                if (point_passed) {
                    LOGM_INFO(loggerIstroNavigation2, "navigationTick", "msg=\"navigation point passed!\", pos=" << navigationPathPos_
                        << ", navp_dist=" << ioff(gps_navp_dist_raw_, 3) << ", navp_mindist=" << ioff(gps_navp_mindist_, 3)
                        << ", navp_approach=" << navp_approach << ", coneseek_stop=" << coneseek_stop);
                    navp_loadarea_ = navp_loadarea_next_;
                    navigation_next_point(conf.navigationPath, navigationPathPos_, navp_latitude_raw_, navp_longitude_raw_, navp_loadarea_next_, navp_idx_);
                    navp_latitude_ = navp_latitude_raw_;
                    navp_longitude_ = navp_longitude_raw_;
                    gps_navp_maxdist_ = -1;
                    gps_navp_mindist_ = -1;
                    if ((navp_latitude_raw_ < ANGLE_OK) && (navp_longitude_raw_ < ANGLE_OK)) {
                        geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_raw_, navp_longitude_raw_, gps_navp_dist_raw_, gps_navp_azimuth_raw_);
                        gps_navp_dist_ = gps_navp_dist_raw_;
                        gps_navp_azimuth_ = gps_navp_azimuth_raw_;
                    } else {
                        gps_navp_dist_ = gps_navp_dist_raw_ = -1;
                        gps_navp_azimuth_raw_ = gps_navp_azimuth_ = ANGLE_NONE;
                    }
                }
            } else {
                gps_navp_dist_ = gps_navp_dist_raw_ = -1;
                gps_navp_azimuth_raw_ = gps_navp_azimuth_ = ANGLE_NONE;
            }
        } else {
            gps_navp_dist_ = gps_navp_dist_raw_ = -1;
            gps_navp_azimuth_raw_ = gps_navp_azimuth_ = ANGLE_NONE;
        }
    }

    // calculate maxdist/mindist
    if (gps_navp_dist_raw_ >= 0) {
        if ((gps_navp_maxdist_ < 0) || (gps_navp_maxdist_ < gps_navp_dist_raw_)) {
            gps_navp_maxdist_ = gps_navp_dist_raw_;
        }
        if ((gps_navp_mindist_ < 0) || (gps_navp_mindist_ > gps_navp_dist_raw_)) {
            gps_navp_mindist_ = gps_navp_dist_raw_;
        }
    }

    // navigation point changed? recalculate route
    int nmap_flags = 0;
#ifndef ISTRO_NAVMAP_NO_ROUTING
    if (nmap_np_idx_ != navp_idx_) {
        if ((navp_idx_ >= 0) && (gps_fix > 0) && (navp_latitude_raw_ < ANGLE_OK) && (navp_longitude_raw_ < ANGLE_OK)) {
            nmap_flags += 20;
            nmap_np_idx_ = navp_idx_;
            // plan route
            LOGM_INFO(loggerIstroNavigation2, "navigationTick", "msg=\"nmap: planning new route...\", nmap_np_idx=" << nmap_np_idx_ << ", fix=" << gps_fix
                << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                << ", navp_latitude_raw=" << ioff(navp_latitude_raw_, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw_, 6));
            navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw_, navp_longitude_raw_);
        } else {    // if (navp_idx < 0)
            if (nmap_np_idx_ >= 0) {
                nmap_flags += 40;
                nmap_np_idx_ = -1;
                LOGM_INFO(loggerIstroNavigation2, "navigationTick", "msg=\"nmap: deleting route...\", nmap_np_idx=" << nmap_np_idx_);
                navmap_plan_delete();
            }
        }
    }
    // route matching: replace gps and navigation point coordinates with points on route
    if ((nmap_np_idx_ >= 0) && (gps_fix > 0)) {
        int sidx = -1;
        double llat, llon;

        nmap_flags += 1;

        double dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);

        if (dd2 > NAVMAP_REROUTE_DIST * NAVMAP_REROUTE_DIST) {
            nmap_flags += 10;
            LOGM_INFO(loggerIstroNavigation2, "navigationTick", "msg=\"nmap: rerouting...\", dd2=" << ioff(dd2, 3)
                << ", nmap_np_idx=" << nmap_np_idx_ << ", fix=" << gps_fix
                << ", gps_latitude_raw=" << ioff(gps_latitude_raw, 6) << ", gps_longitude_raw=" << ioff(gps_longitude_raw, 6)
                << ", navp_latitude_raw=" << ioff(navp_latitude_raw_, 6) << ", navp_longitude_raw=" << ioff(navp_longitude_raw_, 6));
            navmap_planRouteLL(gps_latitude_raw, gps_longitude_raw, navp_latitude_raw_, navp_longitude_raw_);

            dd2 = navmap_dist2PS_LL(gps_latitude_raw, gps_longitude_raw, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);
        }

        if ((sidx >= 0) && (llat < ANGLE_OK) && (llon < ANGLE_OK)) {
            nmap_flags += 2;
            gps_latitude = llat;
            gps_longitude = llon;
            navigation_getXY(gps_latitude, gps_longitude, gps_x_, gps_y_);
        }

        double nlat, nlon;

        int res = navmap_routeFwd(gps_latitude, gps_longitude, NAVMAP_ROUTEFWD_DIST, nlat, nlon);

        if ((res >= 0) && (nlat < ANGLE_OK) && (nlon < ANGLE_OK)) {
            nmap_flags += 4;
            double ndist, nazimuth;
            navp_latitude_ = nlat;
            navp_longitude_ = nlon;
            geoCalc.getDist(gps_latitude, gps_longitude, navp_latitude_, navp_longitude_, ndist, nazimuth);
            // distance to navigation point is real (could be replaced with route length), azimuth is changed
            gps_navp_azimuth_ = nazimuth;
        }
    }
#endif
    (void)nmap_flags;   // only used for logging in legacy; kept as a variable for 1:1 fidelity

    if (navigation_writeData(navp_idx_, gps_navp_dist_, gps_navp_dist_raw_, gps_navp_azimuth_, gps_navp_azimuth_raw_,
            gps_navp_maxdist_, navp_loadarea_, navp_latitude_, navp_longitude_, navp_latitude_raw_, navp_longitude_raw_,
            gps_ref, gps_x_, gps_y_) < 0) {
        LOGM_ERROR(loggerIstroNavigation2, "navigationTick", "msg=\"navigation_writeData() failed!\"");
        return;
    }

    publishNavigationData(navp_idx_, gps_navp_dist_, gps_navp_dist_raw_, gps_navp_azimuth_, gps_navp_azimuth_raw_,
        gps_navp_maxdist_, navp_loadarea_, navp_latitude_, navp_longitude_, navp_latitude_raw_, navp_longitude_raw_,
        gps_ref, gps_x_, gps_y_);

    publishNavigationRoute();
}

void NavigationNode::cb_gps(const istrorsx_hw::msg::GpsData::SharedPtr msg)
{
    if (gps_writeData(msg->fix ? 1 : 0, msg->latitude, msg->longitude) < 0) {
        LOGM_ERROR(loggerIstroNavigation2, "cb_gps", "msg=\"gps_writeData() failed!\"");
        return;
    }
    navigationTick();
}

void NavigationNode::cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg)
{
    coneseek_stop_ = msg->coneseek_stop;
    coneseek_intlen_ = msg->coneseek_intlen;
}

// Cross-process replacement for legacy loadarea_process()'s own direct
// navigation_point_set() call (istro_rt2025.cpp:2298): one process there
// meant one shared navigationPoint[]/navigationPointXY[], so process_thread's
// QR-scan result was instantly visible to gps_thread's own route matching.
// Here planner_node and navigation_node are separate processes with separate
// copies of that array, so the update has to arrive as a message and be
// re-applied locally -- navigation_point_set() itself is the same legacy
// function, unchanged (it also recomputes navigationPointXY[] via
// navigation_getXY(), which is why istro_main_navigation.cpp's own
// navigation_init() must have run first -- it has).
void NavigationNode::cb_navigation_point_set(const istrorsx_core::msg::NavigationPointSet::SharedPtr msg)
{
    if (navigation_point_set(msg->point_idx, msg->point_latitude, msg->point_longitude) < 0) {
        LOGM_ERROR(loggerIstroNavigation2, "cb_navigation_point_set", "msg=\"navigation_point_set() failed!\", point_idx=" << msg->point_idx);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_navigation(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroNavigation2, "main", "msg=\"istro_main_navigation() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);

    istro_close_navigation();

    rclcpp::shutdown();
    return 0;
}
