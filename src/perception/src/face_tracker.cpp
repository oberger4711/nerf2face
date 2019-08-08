#include "face_tracker.h"

FaceTracker::FaceTracker() {
    reset();
}

void FaceTracker::reset() {
    current_state = FaceTrackerState::DETECT;
}

boost::optional<cv::Rect> FaceTracker::findFace(const cv::Mat& img) {
    return boost::optional<cv::Rect>();
}

boost::optional<cv::Rect> FaceTracker::trackFace(const cv::Mat& img) {
    return boost::optional<cv::Rect>();
}

boost::optional<cv::Rect> FaceTracker::detectFace(const cv::Mat& img) {
    return boost::optional<cv::Rect>();
}
