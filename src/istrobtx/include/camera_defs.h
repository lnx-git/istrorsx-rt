#ifndef __CAMERA_DEFS_H__
#define __CAMERA_DEFS_H__

#include <opencv2/opencv.hpp>

/* Pure data + geometry extracted from istrorsx_hw's camera.h/.cpp so code
   that doesn't own a physical camera (istrorsx_core's vision_depth.h,
   and the future save_node for camera_draw_depth_frame() below) can use
   them without pulling in librealsense2 -- same split as lidar_defs.h/
   lidar.h. Originally named camera_types.h (constants only); merged with
   the later camera_draw_depth_frame() extraction into one camera_defs.h/
   .cpp pair, matching this project's one xxx_defs.h/.cpp naming
   convention (see lidar_defs.h's own note). */

const int CAMERA_DEPTH_FRAME_WIDTH   = 848;
const int CAMERA_DEPTH_FRAME_HEIGHT  = 480;

// 1:1 port of legacy Camera::drawDepthFrame() (istro_rt2025.cpp/camera.cpp,
// called from save_thread only), converted from a class method to a free
// function -- it never touched Camera's own hardware state (no
// this->pipe/frames/color_map access), only this->device_id (now an
// explicit parameter, used solely for the side JSON dump's own "dev_id"
// field below, not for image processing). It was only ever trapped inside
// the librealsense2-coupled Camera class by C++ file organization, not any
// real functional need -- moved here so the future save_node can render the
// "cdepth"/"rcdepth" debug images without linking librealsense2.
//
// Does two unrelated things, both kept (legacy 1:1): (a) appends a sampled
// (not full-resolution) JSON dump of raw depth values to a side file
// (out/camera_depth.json) -- deliberately sparse (a handful of rows/columns,
// not the whole frame) so it stays cheap enough to write on every call,
// meant to be visualizable as a lightweight depth-debug trace; (b) converts
// the raw uint16 depth Mat into the actual 8-bit rainbow-colorized "cdepth"/
// "rcdepth" image via convertTo()+equalizeHist()+applyColorMap(). istrorsx_hw's
// Camera class loses this method (see camera.h) -- camera_node never called
// it either (only capture+publish raw frames).
void camera_draw_depth_frame(const cv::Mat &frame, cv::Mat &image, long image_number, int device_id);

#endif
