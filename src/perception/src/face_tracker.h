// Boost
#include <boost/optional.hpp>
// OpenCV
#include <opencv2/opencv.hpp>

class FaceTracker {
public:
    FaceTracker();
    boost::optional<cv::Rect> findFace(const cv::Mat& img);

private:
    enum class FaceTrackerState {
        DETECT, TRACK
    };

    boost::optional<cv::Rect> trackFace(const cv::Mat& img);
    boost::optional<cv::Rect> detectFace(const cv::Mat& img);
    
    FaceTrackerState current_state;
};
