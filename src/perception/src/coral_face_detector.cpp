#include "coral_face_detector.h"

#include <algorithm>
// ROS
#include <ros/ros.h>

constexpr int DOWN_SCALING_FACTOR = 2;

CoralFaceDetector::CoralFaceDetector() :
    detector(ros::package::getPath("perception") + "/resources/ssd_mobilenet_v2_face_quant_postprocess_edgetpu.tflite", 0.7) {
    //detector(ros::package::getPath("perception") + "/resources/efficientdet_lite0_320_ptq_edgetpu.tflite", 0.7) {
}

void CoralFaceDetector::detect(const cv::Mat& /*img_gray*/, const cv::Mat& img_rgb, std::vector<cv::Rect>& face_detections) {
    cv::Mat img_rgb_s;
    cv::resize(img_rgb, img_rgb_s, cv::Size(320, 320), 0.0, 0.0, cv::INTER_LINEAR);
    std::vector<uint8_t> data;
    const size_t num_bytes = img_rgb_s.rows * img_rgb_s.cols * 3;
    data.resize(num_bytes);
    std::memcpy(&data[0], &img_rgb_s.at<uint8_t>(0, 0), num_bytes);
    const auto& coral_dets = detector.detect(data);
    // Convert detections.
    const float x_scale = static_cast<float>(img_rgb_s.cols);
    const float y_scale = static_cast<float>(img_rgb_s.rows);
    face_detections.clear();
    for (const auto& cd : coral_dets) {
        const float width = cd.width() * x_scale;
        const float height = cd.height() * y_scale;
        face_detections.emplace_back(cd.xmin * x_scale + (width / 2), cd.ymin * y_scale + (height / 2), width, height);
        //ROS_WARN_STREAM("Det: " << face_detections.back().x << ", " << face_detections.back().y << ": " << face_detections.back().width << ", " << face_detections.back().height);
    }
}
