#include "face_tracker.h"

FaceTracker::FaceTracker(const std::string& cascade_file_path) {
    ROS_INFO_STREAM(cascade_file_path);
    face_detector.load(cascade_file_path);
    reset();
}

void FaceTracker::reset() {
    current_state = FaceTrackerState::DETECT;
}

boost::optional<cv::Rect> FaceTracker::findFace(const cv::Mat& img) {
    ROS_INFO("Finding face.");
    // To grayscale.
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, CV_BGR2GRAY);
    ROS_INFO_STREAM("Img gray: " << img_gray.cols << " x " <<  img_gray.rows);
    std::vector<cv::Rect> face_detections;
    face_detector.detectMultiScale(img_gray, face_detections, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    const cv::Point img_center(img.cols / 2, img.rows / 2);
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

boost::optional<cv::Rect> FaceTracker::trackFace(const cv::Mat& img) {
    return boost::optional<cv::Rect>();
}

boost::optional<cv::Rect> FaceTracker::detectFace(const cv::Mat& img) {
    return boost::optional<cv::Rect>();
}
