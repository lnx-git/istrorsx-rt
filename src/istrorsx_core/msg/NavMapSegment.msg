# One navMapSegment[] entry (navmap.h), for navmap_planRouteLL()'s temporary
# segments. node1_idx/node2_idx index navMapNode[] -- valid in the receiver only
# after it has re-added the temporary nodes in the order they were published.
int64 way_id
int32 node1_idx
int32 node2_idx
int32 flags
