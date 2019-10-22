#include "haar_face_detector.h"

HaarFaceDetector::HaarFaceDetector() {
    detector.load(ros::package::getPath("perception") + "/resources/haarcascade_frontalface_default.xml");
}

void HaarFaceDetector::detect(const cv::Mat& img_gray, std::vector<cv::Rect>& face_detections) {
    detector.detectMultiScale(img_gray, face_detections, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
}
