#pragma once

// ROS
#include <ros/ros.h>
// Boost
#include <boost/optional.hpp>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/tracking.hpp>

enum class FaceTrackerImpl {
    CSRT,
    KCF,
    MOSSE
};

class FaceTracker {
public:
    FaceTracker(const std::string&, const FaceTrackerImpl tracker_impl);
    boost::optional<cv::Rect> findFace(const cv::Mat& img);
    void reset();

private:
    enum class FaceTrackerState {
        DETECT,
        TRACK
    };

    cv::Ptr<cv::Tracker> make_tracker();
    boost::optional<cv::Rect> trackFace(const cv::Mat& img);
    boost::optional<cv::Rect> detectFace(const cv::Mat& img);
    
    FaceTrackerImpl tracker_impl;
    FaceTrackerState current_state;
    cv::CascadeClassifier detector;
    cv::Ptr<cv::Tracker> tracker;
};
