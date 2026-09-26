# Debug/visualization-only output of planner_node -- deliberately kept out
# of PlannerData.msg/DriveCommand.msg (driving-relevant data only). Carries
# the intermediate values legacy's Lidar::drawOutput() needs for its "lidar"
# overlay image (istro_rt2025.cpp save_thread) but which only ever existed
# as planner_node-private state (cb_lidar()'s lidar_process()/DegreeMap::find()
# output, plannerTick()'s wmodel.evalGrid()/find() output) -- never otherwise
# published, since PlannerData.msg/DriveCommand.msg only carry the final
# decision, not these intermediate bounds. The future save_node subscribes
# to this alongside LidarData.msg to reproduce Lidar::drawOutput() itself.
# See doc/ai/01_architecture.md's save_node section.

int64 image_number       # same value/tick as PlannerData.msg's own image_number -- lets save_node pair the two messages

int32 lidar_angle_min    # DegreeMap::find()'s widest free interval in the raw lidar dmap, start angle, -1 = none found
int32 lidar_angle_max    # ... end angle, -1 = none found
int32 lidar_stop         # Lidar::process()'s own near-obstacle stop flag

int32 process_angle_min  # DegreeMap::find()'s widest free interval in wmodel's fused obstacle grid, start angle, -1 = none found
int32 process_angle_max  # ... end angle, -1 = none found
