# One file save_node just wrote to disk -- see SaveEvent.msg for the batch
# this belongs to.

string tag            # legacy saveImage() tag: "camera"/"vision"/"nnpred"/"cdepth"/"rcamera"/"rvision"/"rnnpred"/"rcdepth"/"lidar"/"wmgrid"/"wmgrif"/"navmap_png"/"navmap_kml"
string filename        # out/-relative path, e.g. "out/rt2026_0002345_0000042_camera.jpg" -- matches legacy logp.cpp's own JSON output convention (OUT_IMG_DNAME + filename), so telemetry_node can pass it straight through to the frontend unmodified

# The clock this file is actually stamped with. For most tags this equals
# the SAME plannerTick() the rest of the batch in SaveEvent.msg came from
# -- but "wmgrid"/"wmgrif" are stamped with wmodel's own separate image
# counter instead (legacy save_thread's own comment: "storing image with
# different image_number, because pdata and wmodel are not correlated in
# time" -- wmodel.updateGrid() only advances its counter when the grid
# actually changes, not once per plannerTick()). Always read this field for
# correlation, never assume it matches a message-level image_number.
int64 image_number
