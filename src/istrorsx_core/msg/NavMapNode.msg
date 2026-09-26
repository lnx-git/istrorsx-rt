# One navMapNode[] entry (navmap.h), for the temporary nodes navmap_planRouteLL()
# adds to the route graph. x/y are not carried: navmap_draw()/navmap_export_kml()
# use lat/lon only, and navmap_node_add() recomputes x/y itself.
int64   node_id
float64 latitude
float64 longitude
int32   flags
