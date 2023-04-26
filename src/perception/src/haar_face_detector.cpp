#include "haar_face_detector.h"

#include <algorithm>

constexpr int DOWN_SCALING_FACTOR = 2;

HaarFaceDetector::HaarFaceDetector() {
    detector.load(ros::package::getPath("perception") + "/resources/haarcascade_frontalface_default.xml");
}

void HaarFaceDetector::detect(const cv::Mat& img_gray, std::vector<cv::Rect>& face_detections) {
    cv::Mat img_gray_s;
    cv::resize(img_gray, img_gray_s, cv::Size(), 1.0 / DOWN_SCALING_FACTOR, 1.0 / DOWN_SCALING_FACTOR, cv::INTER_LINEAR);
    detector.detectMultiScale(img_gray_s, face_detections, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30 / DOWN_SCALING_FACTOR, 30 / DOWN_SCALING_FACTOR));
    for (auto& det : face_detections) {
        det.x = det.x * DOWN_SCALING_FACTOR;
        det.y = det.y * DOWN_SCALING_FACTOR;
        det.width = det.width * DOWN_SCALING_FACTOR;
        det.height = det.height * DOWN_SCALING_FACTOR;
    }
}
