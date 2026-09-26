#include <stdio.h>
#include "camera.h"
#if 0  // NOT_YET_MIGRATED: vision_depth.h/.cpp (VisionDepth, depth-image obstacle detection) not ported.
       // Only reachable from Camera::getFrameDepth()'s ISTRO_CAMERA_RS_DEPTH-undefined fallback path,
       // which is dead code here since ISTRO_CAMERA_RS_DEPTH is active (system.h) -- kept out entirely
       // rather than pulling in the header just to satisfy an unconditional #include. See
       // doc/ai/05_migration_guide.md.
#include "vision_depth.h"
#endif  // NOT_YET_MIGRATED
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerCamera, "Camera");

#ifdef ISTRO_CAMERA_REALSENSE

int Camera::init(int dev_id /*= -1*/) 
// device_id: 0 = camera with lower serial_number; 1 = camera with higher serial_number; -1 = any camera
{
    device_id = dev_id;

    // Initialize the context
    rs2::context ctx;

    // Detect available RealSense devices
    rs2::device_list devices = ctx.query_devices();

    // Not part of the original code -- added because pipe.start(cfg) below
    // does not fail fast when no device matches the requested config: it
    // blocks indefinitely instead (confirmed via debug logging around
    // pipe.start(), see doc/ai/03_progress.md), likely librealsense2's
    // hot-plug support (wait for a matching device to appear) rather than a
    // bug specific to this SDK version. The device count was already
    // available here (used below for the 2-camera serial-number pick), just
    // never checked -- checking it lets Camera::init() fail fast instead.
    // device_id==1 ("higher serial number" camera, see below) only makes
    // sense with at least 2 devices present; device_id==0/-1 just need one.
    if (devices.size() == 0) {
        LOGM_ERROR(loggerCamera, "init", "msg=\"no RealSense device detected!\"");
        return -1;
    }
    if ((device_id == 1) && (devices.size() < 2)) {
        LOGM_ERROR(loggerCamera, "init", "msg=\"device_id=1 requires at least 2 RealSense devices, found " << devices.size() << "!\"");
        return -1;
    }

    string device_sn = "";

    if ((device_id >= 0) && (devices.size() >= 2)) {
        string sn_a = devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        string sn_b = devices[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        // Deterministic regardless of query_devices() enumeration order --
        // device_id==0 always gets the lower serial, device_id==1 always
        // gets the higher one (previously only device_id==1 compared at
        // all; device_id==0 just took devices[0] as-is, so it could bind to
        // either physical camera depending on USB enumeration order, and
        // could end up colliding with device_id==1 on the same device).
        string sn_lo = (sn_a.compare(sn_b) <= 0) ? sn_a : sn_b;
        string sn_hi = (sn_a.compare(sn_b) <= 0) ? sn_b : sn_a;
        device_sn = (device_id > 0) ? sn_hi : sn_lo;
    }

    //Contruct a pipeline which abstracts the device
    //rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    if (!device_sn.empty()) {
        cfg.enable_device(device_sn);
    }

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, RS2_FORMAT_BGR8, 30);
#ifdef ISTRO_CAMERA_RS_DEPTH
    cfg.enable_stream(RS2_STREAM_DEPTH, CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT, RS2_FORMAT_Z16, 30);
#endif

    //Instruct pipeline to start streaming with the requested configuration
    LOGM_INFO(loggerCamera, "init", "msg=\"before pipe.start()\"");  // DEBUG: confirm exactly where init() blocks
    pipe.start(cfg);
    LOGM_INFO(loggerCamera, "init", "msg=\"after pipe.start()\"");  // DEBUG: confirm exactly where init() blocks

//    if (!cap0.isOpened()) {
//        LOGM_ERROR(loggerCamera, "init", "Could not initialize camera[0]!");
//        return -1;
//    }

#ifdef ISTRO_GUI        
    namedWindow( "Camera0", 0 ); 

    Mat img;
    img.create(640,480,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    imshow("Camera0", img);
#endif

    return 0;
}

void Camera::close(void)
{
}

int Camera::getFrame(Mat& frame)
{
    //Wait for all configured streams to produce a frame
    frames = pipe.wait_for_frames();

    //Get each frame
    rs2::frame color_frame = frames.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat color_mat(Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    resize(color_mat, color_mat, Size(640, 480), 0, 0, INTER_NEAREST);  // INTER_AREA is slow (70ms)

    color_mat.copyTo(frame);
    if( frame.empty() )
        return -1;
  
    return 0;
} 

int Camera::getFrameDepth(Mat& frame)
{
#ifdef ISTRO_CAMERA_RS_DEPTH
    //rs2::frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
    //Mat depth_mat(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

    rs2::frame depth_frame = frames.get_depth_frame();
    Mat depth_mat(Size(CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    
    // rescale to an unsupported resolution?
    // resize(depth_mat, depth_mat, Size(CAMERA_DEPTH_FRAME_WIDTH, CAMERA_DEPTH_FRAME_HEIGHT), 0, 0, INTER_NEAREST);

    depth_mat.copyTo(frame);
    if (!frame.empty()) {
        return 0;
    }
#else

#ifdef ISTRO_VISION_DEPTH
    VisionDepth::getTestData(frame);
    return 0;
#else
    if (!frame.empty()) {
        frame.release();
    }
#endif

#endif

    return -1;
} 

#else

Mat camera_img0;
Mat camera_img1;    
Mat camera_img2;    
Mat camera_img3;    
Mat camera_img4;    
Mat camera_img5;

int Camera::init(int dev_id /*= -1*/) { 
    device_id = dev_id;
    LOGM_WARN(loggerCamera, "init", "MOCK CAMERA implementation, DEBUG ONLY!!");

    //camera_img0 = imread("sample/0001032_cesta_oblacno.jpg", 1);
    //camera_img0 = imread("sample/0001060_ok.jpg", 1);  // no obstacles
    camera_img0 = imread("sample/0001126_cesta_vpravo.jpg", 1);  // obstacle on the left side
    //camera_img0 = imread("sample/0001210_trava_zelena2.jpg", 1);
    //camera_img0 = imread("sample/0009442_cesta_vlavo.jpg", 1); 
    if (!camera_img0.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #1!");
        return -1;
    }

/*
    camera_img1 = imread("sample/zbar-test.jpg", 1);  // QR-code test
    if (!camera_img1.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #2!");
        return -2;
    }

    camera_img2 = imread("sample/0034400_fisheye.jpg", 1);  // no obstacle
//    camera_img2 = imread("sample/0001060_ok.jpg", 1);  // no obstacle
    if (!camera_img2.data) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #3!");
        return -3;
    }
*/
/*
    camera_img1 = imread("sample/speetctl_test/0002200_camera.jpg");
    camera_img2 = imread("sample/speetctl_test/0002250_camera.jpg");
    camera_img3 = imread("sample/speetctl_test/0002302_camera.jpg");
    camera_img4 = imread("sample/speetctl_test/0002339_camera.jpg");
    camera_img5 = imread("sample/speetctl_test/0005022_camera.jpg");
*/
    camera_img1 = camera_img0;
    camera_img2 = imread("sample/2014689_kuzele.jpg");
    camera_img3 = imread("sample/2013222_kuzele.jpg");
    camera_img4 = imread("sample/kuzele_test/0031677_camera.jpg");
    //camera_img5 = imread("sample/2008241_kuzele.jpg"); 
    camera_img5 = imread("sample/0000021_cesta_proti_slnku.jpg");

    if ((!camera_img1.data) || (!camera_img2.data) || (!camera_img3.data) || (!camera_img4.data) || (!camera_img5.data)) {
        LOGM_ERROR(loggerCamera, "init", "error opening file #2-6!");
        return -3;
    }

#ifdef ISTRO_GUI        
    namedWindow( "Camera0", 0 ); 

    Mat img;
    img.create(640,480,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    imshow("Camera0", img);
#endif

    return 0; 
}

void Camera::close(void) 
{ 
} 

int getframe_cnt = 0;

int Camera::getFrame(Mat& frame) 
{ 
    msleep(100);
    if (device_id < 1) {
        if (getframe_cnt < 50) {
            frame = camera_img1;  // camera_img0;
        } else 
        if (getframe_cnt < 100) {
            frame = camera_img2;
        } else
        if (getframe_cnt < 150) {
            frame = camera_img3;
        } else 
        if (getframe_cnt < 200) {
            frame = camera_img4;
        } else {
            frame = camera_img5;
        }
        //getframe_cnt++;
    } else {
        //frame = camera_img5;
        frame = camera_img1;
    }

    return 0; 
}

int Camera::getFrameDepth(Mat& frame)
{
#ifdef ISTRO_VISION_DEPTH
    VisionDepth::getTestData(frame);
    return 0;
#else
    if (!frame.empty()) {
        frame.release();
    }
#endif
    return -1;
} 

#endif

void Camera::drawFrame(const Mat& frame)
{
#ifdef ISTRO_GUI    
    imshow("Camera0", frame);
#endif    
}

// Camera::drawDepthFrame() moved to istrobtx's camera_defs.h/.cpp as the
// free function camera_draw_depth_frame() -- see camera.h's declaration
// comment.
