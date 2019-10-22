#include "face_tracker.h"

FaceTracker::FaceTracker(const FaceDetectorImpl detector_impl, const FaceTrackerImpl tracker_impl) :
    tracker_impl(tracker_impl),
    current_state(FaceTrackerState::DETECT) {
    detector = make_detector(detector_impl);
    tracker = make_tracker();
    reset();
}

std::unique_ptr<FaceDetector> FaceTracker::make_detector(const FaceDetectorImpl detector_impl) {
    switch (detector_impl) {
        case FaceDetectorImpl::HAAR:
            return std::unique_ptr<FaceDetector>(new HaarFaceDetector());
        case FaceDetectorImpl::HOG_SVM:
            return std::unique_ptr<FaceDetector>(new HogSvmFaceDetector());
        default:
            ROS_WARN_STREAM("Requested detector implementation " << static_cast<unsigned int>(detector_impl) << " is not supported.");
            return std::unique_ptr<FaceDetector>(new HaarFaceDetector());
    }
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
    //auto start_time = std::chrono::high_resolution_clock::now();
    detector->detect(img_gray, face_detections);
    //auto end_time = std::chrono::high_resolution_clock::now();
    //ROS_INFO_STREAM("detect() took " << (end_time - start_time) / std::chrono::milliseconds(1) << " ms.\n");
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
