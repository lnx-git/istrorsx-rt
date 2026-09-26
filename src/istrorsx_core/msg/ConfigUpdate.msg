# Runtime conf change, published by the node that made it. Legacy had one global
# conf; here every node has its own copy, so the write has to be sent.
#
# Delta: update_velocity says whether the velocity_* fields carry a change
# (same shape as SpeedCommand's set_speed). Default QoS, no late-join recovery --
# all nodes start together. Rationale and rejected options: 01_architecture.md.
bool update_velocity
int32 velocity_fwd    # offset from VEL_ZERO (ctrlboard_defs.h), as on the -vf command line
int32 velocity_fwd2   # -1 = not set (legacy semantics)
int32 velocity_fwd3   # -1 = not set
int32 velocity_back   # negative offset from VEL_ZERO, as on -vb
