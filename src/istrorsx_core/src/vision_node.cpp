#include "istrorsx_core/vision_node.hpp"

#include "config.h"
#include "threads.h"
#include "dataset.h"
#include "system.h"
#include "logger.h"
#include "mtime.h"
#include "istrorsx_core/istrobtx/istro_main_vision.h"
#include "istrorsx_core/istrobtx/vision.h"
#include "istrorsx_core/istrobtx/qrscan.h"
#ifdef ISTRO_VISIONN
#include "istrorsx_core/istrobtx/visionn.h"
#endif
#ifdef ISTRO_VISION_DEPTH
#include "istrorsx_core/istrobtx/vision_depth.h"
#endif

LOG_DEFINE(loggerIstroVision2, "istroVision");

// Legacy file-scope const, declared right before vision_thread() in
// istro_rt2025.cpp.
static const int DMAP_MIN_INTERVAL_LENGTH = 20;

static cv::Mat cameraFrameColorToMat(const istrorsx_hw::msg::CameraFrame::SharedPtr &msg)
{
    if ((msg->color_width <= 0) || (msg->color_height <= 0)) {
        return cv::Mat();
    }
    return cv::Mat(msg->color_height, msg->color_width, msg->color_cv_type,
        (void *)msg->color_data.data(), msg->color_step);
}

static cv::Mat cameraFrameDepthToMat(const istrorsx_hw::msg::CameraFrame::SharedPtr &msg)
{
    if ((msg->depth_width <= 0) || (msg->depth_height <= 0)) {
        return cv::Mat();
    }
    return cv::Mat(msg->depth_height, msg->depth_width, msg->depth_cv_type,
        (void *)msg->depth_data.data(), msg->depth_step);
}

VisionNode::VisionNode() : Node("vision_node")
{
    pub_vision_front_ = this->create_publisher<istrorsx_core::msg::VisionData>("/robot/vision_front_data", 10);
    pub_vision_rear_ = this->create_publisher<istrorsx_core::msg::VisionData>("/robot/vision_rear_data", 10);
    pub_vision_debug_front_ = this->create_publisher<istrorsx_core::msg::VisionDebugData>("/robot/vision_front_debug_data", 10);
    pub_vision_debug_rear_ = this->create_publisher<istrorsx_core::msg::VisionDebugData>("/robot/vision_rear_debug_data", 10);

    sub_camera_front_ = this->create_subscription<istrorsx_hw::msg::CameraFrame>(
        "/robot/camera_front_data", 10,
        [this](const istrorsx_hw::msg::CameraFrame::SharedPtr msg) { cb_camera_front(msg); });
    sub_camera_rear_ = this->create_subscription<istrorsx_hw::msg::CameraFrame>(
        "/robot/camera_rear_data", 10,
        [this](const istrorsx_hw::msg::CameraFrame::SharedPtr msg) { cb_camera_rear(msg); });
    sub_control_ = this->create_subscription<istrorsx_core::msg::VisionControl>(
        "/robot/vision_control", 10,
        [this](const istrorsx_core::msg::VisionControl::SharedPtr msg) { cb_control(msg); });
    sub_planner_data_ = this->create_subscription<istrorsx_core::msg::PlannerData>(
        "/robot/planner_data", 10,
        [this](const istrorsx_core::msg::PlannerData::SharedPtr msg) { cb_planner_data(msg); });
}

void VisionNode::cb_control(const istrorsx_core::msg::VisionControl::SharedPtr msg)
{
    // Port of vision_thread_qrscan_enable()/_disable()/processRear_enable()/
    // _disable() (istro_rt2025.cpp) -- there these were edge-triggered
    // (logged only on actual change); a plain assignment here is equivalent
    // since VisionControl is only ever published on an actual change too.
    qrscan_enabled_ = msg->qrscan_enabled;
    process_rear_enabled_ = msg->process_rear_enabled;

    LOGM_INFO(loggerIstroVision2, "cb_control", "qrscan_enabled=" << qrscan_enabled_ << ", process_rear_enabled=" << process_rear_enabled_);
}

void VisionNode::cb_planner_data(const istrorsx_core::msg::PlannerData::SharedPtr msg)
{
    process_dir_ = msg->process_dir;
}

int VisionNode::camera_writeData(const cv::Mat &camera_img, const cv::Mat &camera_depth)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->camera_img = camera_img;
    pdata->camera_depth = camera_depth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int VisionNode::camera2_writeData(const cv::Mat &camera_img, const cv::Mat &camera_depth)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    pdata->camera2_img = camera_img;
    pdata->camera2_depth = camera_depth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

// Port of vision_thread's own read side -- in the legacy single-process app,
// vision_thread read pdata->camera_img/camera_depth directly out of the
// DataSet slot capture_camera_thread had just filled; here the "capture
// thread" is a different process (camera_node), so the data crosses via a
// message + DataSet round-trip instead (see class comment).
// Named vision_readData()/vision2_readData() -- both are ports of the same
// legacy vision_thread, just its front-camera and rear-camera branches, so
// they follow DataSet's own camera_/camera2_ field-naming split rather than
// needing two differently-named threads.
int VisionNode::vision_readData(cv::Mat &camera_img, cv::Mat &camera_depth)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    camera_img = pdata->camera_img;
    camera_depth = pdata->camera_depth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

int VisionNode::vision2_readData(cv::Mat &camera_img, cv::Mat &camera_depth)
{
    DataSet *pdata;

    pdata = (DataSet *)threads.getData(THDATA_STATE_SHARED, THDATA_STATE_SHARED_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    camera_img = pdata->camera2_img;
    camera_depth = pdata->camera2_depth;

    threads.setData(pdata, THDATA_STATE_SHARED, 1);

    return 0;
}

// Not a legacy port -- see vision_node.hpp.
void VisionNode::publishVisionData(int camera_id, int64_t image_number, const DegreeMap &dmap, int angle_min, int angle_max,
    double qrscan_latitude, double qrscan_longitude)
{
    double tpub = timeBegin();

    istrorsx_core::msg::VisionData msg;
    msg.camera_id = camera_id;
    msg.image_number = image_number;

    for (int i = 0; i < DEGREE_MAP_COUNT; i++) {
        msg.dmap[i] = dmap.dmap[i];
        msg.dist[i] = dmap.dist[i];
        msg.maxd[i] = dmap.maxd[i];
    }

    msg.angle_min = angle_min;
    msg.angle_max = angle_max;
    msg.qrscan_latitude = qrscan_latitude;
    msg.qrscan_longitude = qrscan_longitude;

    if (camera_id == 0) {
        pub_vision_front_->publish(msg);
    } else {
        pub_vision_rear_->publish(msg);
    }
    timeEnd("istro::vision_node.publishVisionData", tpub);
}

static void matToDebugFields(const cv::Mat &m, int32_t &width, int32_t &height, int32_t &cv_type, int32_t &step, std::vector<uint8_t> &data)
{
    if (m.empty() || !m.isContinuous()) {
        width = 0;
        height = 0;
        cv_type = 0;
        step = 0;
        data.clear();
        return;
    }
    width = m.cols;
    height = m.rows;
    cv_type = m.type();
    step = (int32_t)m.step;
    size_t bytes = m.total() * m.elemSize();
    data.assign(m.data, m.data + bytes);
}

// Not a legacy port -- see vision_node.hpp.
void VisionNode::publishVisionDebugData(int camera_id, int64_t image_number, const cv::Mat &markers, const cv::Mat &epweight, const cv::Mat &elweight,
    const cv::Mat &camera_img_pred, int angle_min, int angle_max)
{
    double tpub = timeBegin();

    istrorsx_core::msg::VisionDebugData msg;
    msg.camera_id = camera_id;
    msg.image_number = image_number;

    matToDebugFields(markers, msg.markers_width, msg.markers_height, msg.markers_cv_type, msg.markers_step, msg.markers_data);
    matToDebugFields(epweight, msg.epweight_width, msg.epweight_height, msg.epweight_cv_type, msg.epweight_step, msg.epweight_data);
    matToDebugFields(elweight, msg.elweight_width, msg.elweight_height, msg.elweight_cv_type, msg.elweight_step, msg.elweight_data);
    matToDebugFields(camera_img_pred, msg.camera_img_pred_width, msg.camera_img_pred_height, msg.camera_img_pred_cv_type,
        msg.camera_img_pred_step, msg.camera_img_pred_data);

    msg.angle_min = angle_min;
    msg.angle_max = angle_max;

    if (camera_id == 0) {
        pub_vision_debug_front_->publish(msg);
    } else {
        pub_vision_debug_rear_->publish(msg);
    }
    timeEnd("istro::vision_node.publishVisionDebugData", tpub);
}

// Port of vision_thread's front-camera branch (istro_rt2025.cpp) -- one call
// = one legacy vision_thread iteration's worth of it. Legacy's "pdata2"
// prefetch optimization (send the next already-captured frame to the NN
// server while still finishing the current one's postprocessing) is not
// ported -- see vision_node.hpp class comment.
void VisionNode::visionTickFront()
{
    cv::Mat camera_img, camera_depth;
    if (vision_readData(camera_img, camera_depth) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickFront", "msg=\"vision_readData() failed!\"");
        return;
    }

    // process front camera image only when going forward (matches legacy's
    // "if (!vision_thread_processRear_enabled) {...}" gate)
    if (process_rear_enabled_) {
        return;
    }

    double t = timeBegin();

#ifdef ISTRO_VISIONN
    double t2 = timeBegin();
    if (visionn.send_imgpr(camera_img) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickFront", "msg=\"error: visionn.send_imgpr() failed!\"");
    }
    timeEnd("istro::vision_node.visionn_send", t2);
#endif

    double qrscan_latitude = ANGLE_NONE;
    double qrscan_longitude = ANGLE_NONE;
    if (conf.useQRScan && qrscan_enabled_) {
        double t3 = timeBegin();
        qrscan.scanGeo(camera_img, qrscan_latitude, qrscan_longitude);
        timeEnd("istro::vision_node.qrscan_process", t3);
    }

#ifdef ISTRO_VISION_DEPTH
    DegreeMap visiond_dmap;
    cv::Mat camera_depth_pred0, camera_depth_pred;
    cv::Mat visiond_markers, visiond_markersIM, visiond_epweight, visiond_elweight;
    double t4 = timeBegin();
    visiond.process(camera_depth, camera_depth_pred0, camera_depth_pred);
    visiond_vsn.eval(camera_depth_pred, visiond_markers, visiond_markersIM, visiond_epweight, visiond_elweight, visiond_dmap);
    timeEnd("istro::vision_node.visiond_process", t4);
#endif

    cv::Mat camera_img_pred;
#ifdef ISTRO_VISIONN
    double t5 = timeBegin();
    if (visionn.recv_imgpr(camera_img_pred) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickFront", "msg=\"error: visionn.recv_imgpr() failed!\"");
    }
    timeEnd("istro::vision_node.visionn_recv", t5);
#endif

    cv::Mat vision_markers, vision_markersIM, vision_epweight, vision_elweight;
    DegreeMap vision_dmap;
#ifdef ISTRO_VISIONN
    vision.eval(camera_img_pred, vision_markers, vision_markersIM, vision_epweight, vision_elweight, vision_dmap);
#else
    vision.eval(camera_img, vision_markers, vision_markersIM, vision_epweight, vision_elweight, vision_dmap);
#endif

#ifdef ISTRO_VISION_DEPTH
    double t6 = timeBegin();
    vision_dmap.apply(visiond_dmap);
    vision.applyEPWeight(vision_epweight, vision_elweight, visiond_epweight, visiond_elweight);
    vision.applyMarkers(vision_markers, visiond_markers);
    timeEnd("istro::vision_node.visiond_apply", t6);
#endif

    // process_dir (legacy: heading 45..135, "which direction should we go")
    // -- cached from planner_node's PlannerData.msg via cb_planner_data(),
    // defaults to 90 (straight ahead) until the first message arrives.
    int process_dir = process_dir_;
    int angle_min, angle_max;
    vision_dmap.find(DMAP_MIN_INTERVAL_LENGTH, process_dir, angle_min, angle_max);

    LOGM_INFO(loggerIstroVision2, "visionTickFront", "angle_min=" << angle_min << ", angle_max=" << angle_max
        << ", qrscan_latitude=" << ioff(qrscan_latitude, 6) << ", qrscan_longitude=" << ioff(qrscan_longitude, 6));
    timeEnd("istro::vision_node.process", t);

    publishVisionData(0, camera_image_number_, vision_dmap, angle_min, angle_max, qrscan_latitude, qrscan_longitude);
    publishVisionDebugData(0, camera_image_number_, vision_markers, vision_epweight, vision_elweight, camera_img_pred, angle_min, angle_max);
}

// Port of vision_thread's rear-camera branch (istro_rt2025.cpp). Legacy
// never runs QR scanning on the rear camera -- qrscan_latitude/longitude are
// always ANGLE_NONE on this topic (see VisionData.msg).
void VisionNode::visionTickRear()
{
    cv::Mat camera2_img, camera2_depth;
    if (vision2_readData(camera2_img, camera2_depth) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickRear", "msg=\"vision2_readData() failed!\"");
        return;
    }

    // process rear camera image only when going backward (matches legacy's
    // "if (vision_thread_processRear_enabled) {...}" gate)
    if (!process_rear_enabled_) {
        return;
    }

    double t = timeBegin();

#ifdef ISTRO_VISIONN
    double t2 = timeBegin();
    if (visionn.send_imgpr(camera2_img) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickRear", "msg=\"error: visionn.send_imgpr() failed(2)!\"");
    }
    timeEnd("istro::vision_node.visionn_send", t2);
#endif

#ifdef ISTRO_VISION_DEPTH
    DegreeMap visiond_dmap;
    cv::Mat camera_depth_pred0, camera2_depth_pred;
    cv::Mat visiond_markers, visiond_markersIM, visiond_epweight, visiond_elweight;
    double t3 = timeBegin();
    visiond.process(camera2_depth, camera_depth_pred0, camera2_depth_pred);
    visiond_vsn.eval(camera2_depth_pred, visiond_markers, visiond_markersIM, visiond_epweight, visiond_elweight, visiond_dmap);
    timeEnd("istro::vision_node.visiond_process", t3);
#endif

    cv::Mat camera2_img_pred;
#ifdef ISTRO_VISIONN
    double t4 = timeBegin();
    if (visionn.recv_imgpr(camera2_img_pred) < 0) {
        LOGM_ERROR(loggerIstroVision2, "visionTickRear", "msg=\"error: visionn.recv_imgpr() failed(2)!\"");
    }
    timeEnd("istro::vision_node.visionn_recv", t4);
#endif

    cv::Mat vision2_markers, vision2_markersIM, vision2_epweight, vision2_elweight;
    DegreeMap vision2_dmap;
#ifdef ISTRO_VISIONN
    vision.eval(camera2_img_pred, vision2_markers, vision2_markersIM, vision2_epweight, vision2_elweight, vision2_dmap);
#else
    vision.eval(camera2_img, vision2_markers, vision2_markersIM, vision2_epweight, vision2_elweight, vision2_dmap);
#endif

#ifdef ISTRO_VISION_DEPTH
    double t5 = timeBegin();
    vision2_dmap.apply(visiond_dmap);
    vision.applyEPWeight(vision2_epweight, vision2_elweight, visiond_epweight, visiond_elweight);
    vision.applyMarkers(vision2_markers, visiond_markers);
    timeEnd("istro::vision_node.visiond_apply", t5);
#endif

    // process_dir -- see visionTickFront().
    int process_dir = process_dir_;
    int angle_min, angle_max;
    vision2_dmap.find(DMAP_MIN_INTERVAL_LENGTH, process_dir, angle_min, angle_max);

    LOGM_INFO(loggerIstroVision2, "visionTickRear", "angle_min=" << angle_min << ", angle_max=" << angle_max);
    timeEnd("istro::vision_node.vision2_process", t);

    publishVisionData(1, camera2_image_number_, vision2_dmap, angle_min, angle_max, ANGLE_NONE, ANGLE_NONE);
    publishVisionDebugData(1, camera2_image_number_, vision2_markers, vision2_epweight, vision2_elweight, camera2_img_pred, angle_min, angle_max);
}

void VisionNode::cb_camera_front(const istrorsx_hw::msg::CameraFrame::SharedPtr msg)
{
    cv::Mat camera_img = cameraFrameColorToMat(msg);
    cv::Mat camera_depth = cameraFrameDepthToMat(msg);

    if (camera_writeData(camera_img, camera_depth) < 0) {
        LOGM_ERROR(loggerIstroVision2, "cb_camera_front", "msg=\"camera_writeData() failed!\"");
        return;
    }
    camera_image_number_ = msg->image_number;
    visionTickFront();
}

void VisionNode::cb_camera_rear(const istrorsx_hw::msg::CameraFrame::SharedPtr msg)
{
    cv::Mat camera_img = cameraFrameColorToMat(msg);
    cv::Mat camera_depth = cameraFrameDepthToMat(msg);

    if (camera2_writeData(camera_img, camera_depth) < 0) {
        LOGM_ERROR(loggerIstroVision2, "cb_camera_rear", "msg=\"camera2_writeData() failed!\"");
        return;
    }
    camera2_image_number_ = msg->image_number;
    visionTickRear();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (istro_main_vision(argc, argv) != 0) {
        LOGM_ERROR(loggerIstroVision2, "main", "msg=\"istro_main_vision() failed, exiting\"");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);

    istro_close_vision();

    rclcpp::shutdown();
    return 0;
}
