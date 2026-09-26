# Currently-planned route, published by navigation_node every navigationTick()
# (same cadence as NavigationData.msg). Everything navmap_planRouteLL() changes
# about the graph, so save_node can draw the same picture navigation_node sees.
#
# Base-graph indices (navmap_data.cpp entries) are identical in every process and
# never change at runtime; the temporary structures below are appended at the tail
# and dropped again by navmap_plan_delete(), so base indices stay stable.
#
# NAVMAP_FLAG_ROUTE_OPDIR is deliberately not carried: navmap uses it for route
# following only, never for drawing or KML export.

int32[] route_segment_indices        # base segments flagged NAVMAP_FLAG_ROUTE
int32[] invisible_segment_indices    # base segments flagged NAVMAP_FLAG_INVISIBLE
                                     # (hidden because a temporary segment replaces them)

# navmap_planRouteLL()'s temporary start/end connector structures -- at most 3
# nodes and 6 segments per plan. Appended in this order; flags are final (they
# already include TMP_PLAN, ROUTE, INVISIBLE as applicable).
int32           base_node_count      # navmap_node_cnt before tmp_nodes; receiver rejects
                                     # the message if its own count differs, since the
                                     # node indices below would then not line up
NavMapNode[]    tmp_nodes
NavMapSegment[] tmp_segments
