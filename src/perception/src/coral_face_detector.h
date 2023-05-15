#pragma once

// ROS
#include <ros/package.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

#include "face_detector.h"
#include "coral/coraldetector/coraldetector.h"

class CoralFaceDetector : public FaceDetector {
public:
    CoralFaceDetector();
    void detect(const cv::Mat& img_gray, const cv::Mat& img_rgb, std::vector<cv::Rect>& face_detections) override;
private:
    coral::coraldetector::CoralDetector detector;
};
