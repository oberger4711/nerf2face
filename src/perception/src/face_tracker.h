#pragma once

// ROS
#include <ros/ros.h>
// Boost
#include <boost/optional.hpp>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class FaceTracker {
public:
    FaceTracker(const std::string&);
    boost::optional<cv::Rect> findFace(const cv::Mat& img);
    void reset();

private:
    enum class FaceTrackerState {
        DETECT, TRACK
    };

    boost::optional<cv::Rect> trackFace(const cv::Mat& img);
    boost::optional<cv::Rect> detectFace(const cv::Mat& img);
    
    cv::CascadeClassifier face_detector;
    FaceTrackerState current_state;
};
