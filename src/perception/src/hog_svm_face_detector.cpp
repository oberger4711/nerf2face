#include "hog_svm_face_detector.h"

HogSvmFaceDetector::HogSvmFaceDetector() :
    detector(dlib::get_frontal_face_detector()) {
}

void HogSvmFaceDetector::detect(const cv::Mat& img_gray_cv, std::vector<cv::Rect>& face_detections_cv) {
    // This just wraps the dlib detector with CV interface.
    assert(img_gray_cv.type() == 0); // Should be 8 bit grayscale mat.
    const auto img_gray_dlib = dlib::cv_image<unsigned char>(img_gray_cv); // CV to dlib image
    const auto face_detections_dlib = detector(img_gray_dlib);
    // Dlib rects to CV rects
    std::transform(
            face_detections_dlib.begin(),
            face_detections_dlib.end(),
            std::back_inserter(face_detections_cv),
            [](const dlib::rectangle& r) {
                return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
            });
}
