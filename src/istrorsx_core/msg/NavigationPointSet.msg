# Cross-process replacement for legacy process_thread's navigation_point_set()
# call (navig.cpp: navigation_point_set(point_idx, point_latitude,
# point_longitude)), made from loadarea_process() when a QR-scanned
# coordinate resolves a navigation point's real-world position. In legacy
# this was a same-process function call (process_thread calling straight
# into navig.cpp, both linked into one binary); now planner_node and
# navigation_node are separate processes, so it becomes a message.
# Published by planner_node, subscribed by navigation_node -- closes the gap
# flagged when navigation_node was originally built, see
# doc/ai/01_architecture.md's navigation_node section.

int32   point_idx        # index into navigationPoint[] (navig_data.cpp) to update
float64 point_latitude
float64 point_longitude
