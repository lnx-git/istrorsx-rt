#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "system.h"
#include "camera_defs.h"

#ifdef ISTRO_CAMERA_REALSENSE
#include <librealsense2/rs.hpp>
#endif

using namespace cv;
using namespace std;

const int CAMERA_FRAME_WIDTH         = 848;
const int CAMERA_FRAME_HEIGHT        = 480;

/*
https://support.intelrealsense.com/hc/en-us/community/posts/360039243533-Resolution-Configuration-Options-on-Real-Sense-D435-for-Raspberry-Pi-4-4-GB-RAM
The RealSense SDK has a tool called rs-enumerate-devices that can list
the supported modes on the specific hardware that it is run on.
  16:9  ->   640x360, 480x270, 424x240 @ 6/15/30/60/90 FPS, ... 1280x720 @ 6/15/30 FPS,
   4:3  ->   640x480 @ 6/15/30/60/90 FPS
*/

/* CAMERA_DEPTH_FRAME_WIDTH/HEIGHT now come from camera_defs.h (istrobtx) --
   also needed by istrorsx_core's vision_depth.h, which can't include this
   RealSense-SDK-coupled header. */

class Camera {
private:
    int device_id;

public:
#ifdef ISTRO_CAMERA_REALSENSE
    rs2::pipeline pipe;
    rs2::frameset frames;
    rs2::colorizer color_map;
#endif
    
public:
    int init(int dev_id = -1);  // device_id: 0 = camera with lower serial_number; 1 = camera with higher serial_number; -1 = any camera
    void close(void);
    
    int getFrame(Mat& frame);
    int getFrameDepth(Mat& frame);
    // drawFrame() (below) is unchanged -- it never touched hardware state
    // either, but it's a no-op outside ISTRO_GUI (no calibration overlay
    // exists to extract), so it wasn't worth moving/duplicating.
    void drawFrame(const Mat& frame);
    // drawDepthFrame() moved to istrobtx's camera_defs.h/.cpp as the free
    // function camera_draw_depth_frame() -- it never touched Camera's own
    // hardware state (this->device_id became an explicit parameter), and
    // the future save_node needs to call it without linking librealsense2.
    // See doc/ai/05_migration_guide.md.
};

#endif
