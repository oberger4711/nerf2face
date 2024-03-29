#pragma once

// Std
#include <vector>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

class FaceDetector {
public:
    virtual void detect(const cv::Mat& img_gray, const cv::Mat& img_rgb, std::vector<cv::Rect>& face_detections) = 0;
};
