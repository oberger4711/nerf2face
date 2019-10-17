#include "face_tracker.h"

FaceTracker::FaceTracker(const std::string& cascade_file_path, const FaceTrackerImpl tracker_impl) :
    tracker_impl(tracker_impl),
    current_state(FaceTrackerState::DETECT) {
    tracker = make_tracker();
    ROS_INFO_STREAM(cascade_file_path);
    detector.load(cascade_file_path);
    reset();
}

cv::Ptr<cv::Tracker> FaceTracker::make_tracker() {
    switch (tracker_impl) {
        case FaceTrackerImpl::CSRT:
            return cv::TrackerCSRT::create();
        case FaceTrackerImpl::KCF:
            return cv::TrackerKCF::create();
        case FaceTrackerImpl::MOSSE:
            return cv::TrackerMOSSE::create();
            break;
        default:
            ROS_WARN_STREAM("Requested tracker implementation " << static_cast<unsigned int>(tracker_impl) << " is not supported.");
            return cv::TrackerMOSSE::create();
    }
}

void FaceTracker::reset() {
    current_state = FaceTrackerState::DETECT;
}

boost::optional<cv::Rect> FaceTracker::findFace(const cv::Mat& img) {
    // To grayscale.
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, CV_BGR2GRAY);
    if (current_state == FaceTrackerState::TRACK) {
        auto tracked_face = trackFace(img_gray);
        if (tracked_face) {
            return tracked_face;
        }
        else {
            // Fall back to detection.
            current_state = FaceTrackerState::DETECT;
            ROS_INFO("Switch to DETECT.");
        }
    }
    if (current_state == FaceTrackerState::DETECT) {
        auto detected_face = detectFace(img_gray);
        if (detected_face) {
            // Start tracking.
            tracker = make_tracker();
            tracker->init(img_gray, static_cast<cv::Rect2d>(detected_face.get()));
            current_state = FaceTrackerState::TRACK;
            ROS_INFO("Switch to TRACK.");
        }
        return detected_face;
    }
    return boost::optional<cv::Rect>();
}

boost::optional<cv::Rect> FaceTracker::trackFace(const cv::Mat& img_gray) {
    cv::Rect2d tracked_face_double;
    const bool tracking_succeeded = tracker->update(img_gray, tracked_face_double);
    cv::Rect tracked_face = static_cast<cv::Rect>(tracked_face_double);
    if (tracking_succeeded) {
        return boost::optional<cv::Rect>(tracked_face);
    }
    else {
        return boost::optional<cv::Rect>();
    }
}

boost::optional<cv::Rect> FaceTracker::detectFace(const cv::Mat& img_gray) {
    std::vector<cv::Rect> face_detections;
    detector.detectMultiScale(img_gray, face_detections, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    const cv::Point img_center(img_gray.cols / 2, img_gray.rows / 2);
    boost::optional<cv::Rect> final_face_detection;
    float final_distance_2 = std::numeric_limits<float>::infinity();
    for (const auto& face_detection : face_detections) {
        if (!final_face_detection) {
            final_face_detection = face_detection;
        }
        else {
            const auto face_detection_center = (face_detection.br() + face_detection.tl()) * 0.5f;
            const auto distanceVector = img_center - face_detection_center;
            const float distance_2 = distanceVector.x * distanceVector.x + distanceVector.y * distanceVector.y;
            if (distance_2 < final_distance_2) {
                final_distance_2 = distance_2;
                final_face_detection = face_detection;
            }
        }
    }
    return final_face_detection;
}
