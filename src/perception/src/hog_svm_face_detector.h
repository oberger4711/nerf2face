#pragma once

// Std
#include <cassert>
#include <vector>
#include <algorithm>
// ROS
#include <ros/console.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
// DLib
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include "face_detector.h"

class HogSvmFaceDetector : public FaceDetector {
public:
    HogSvmFaceDetector();
    void detect(const cv::Mat& img_gray, std::vector<cv::Rect>& face_detections) override;
private:
    dlib::frontal_face_detector detector;
};
