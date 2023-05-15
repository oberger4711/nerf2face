#pragma once

// ROS
#include <ros/package.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "face_detector.h"

class HaarFaceDetector : public FaceDetector {
public:
    HaarFaceDetector();
    void detect(const cv::Mat& img_gray, const cv::Mat& img_rgb, std::vector<cv::Rect>& face_detections) override;
private:
    cv::CascadeClassifier detector;
};
