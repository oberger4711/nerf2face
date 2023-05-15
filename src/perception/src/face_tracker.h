#pragma once

// Std
#include <memory>
#include <vector>
#include <chrono>
// ROS
#include <ros/ros.h>
// Boost
#include <boost/optional.hpp>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>

#include "face_detector.h"
#include "haar_face_detector.h"
#include "coral_face_detector.h"

enum class FaceDetectorImpl {
    HAAR,
    MOBILE_NET_V2
};

enum class FaceTrackerImpl {
    CSRT,
    KCF,
    MOSSE,
    NONE
};

class FaceTracker {
public:
    FaceTracker(const FaceDetectorImpl detector_impl, const FaceTrackerImpl tracker_impl);
    boost::optional<cv::Rect> findFace(const cv::Mat& img, const double timestamp);
    void reset();

private:
    enum class FaceTrackerState {
        DETECT,
        TRACK
    };

    std::unique_ptr<FaceDetector> make_detector(const FaceDetectorImpl detector_impl);
    cv::Ptr<cv::Tracker> make_tracker();
    boost::optional<cv::Rect> trackFace(const cv::Mat& img_gray);
    boost::optional<cv::Rect> detectFace(const cv::Mat& img_gray, const cv::Mat& img_rgb);
    
    FaceTrackerImpl tracker_impl;
    FaceTrackerState current_state;
    double ts_last_detection;
    std::unique_ptr<FaceDetector> detector;
    cv::Ptr<cv::Tracker> tracker;
};
