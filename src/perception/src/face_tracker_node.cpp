#include "face_tracker_node.h"

FaceTrackerNode::FaceTrackerNode(ros::NodeHandle& nh) :
    nh(nh),
    it(nh),
    face_tracker(ros::package::getPath("perception") + "/resources/haarcascade_frontalface_default.xml", FaceTrackerImpl::MOSSE) {
    reconfigure_server.setCallback(std::bind(&FaceTrackerNode::handle_reconfigure, this, std::placeholders::_1, std::placeholders::_2));
    reset_service = nh.advertiseService("reset", &FaceTrackerNode::handle_reset, this);
    image_sub = nh.subscribe("image", 1, &FaceTrackerNode::handle_image, this);
    //image_sub = it.subscribe("image", 1, &FaceTrackerNode::handle_image, this, image_transport::TransportHints("compressed"));
    face_detection_pub = nh.advertise<perception_msgs::FaceDetectionStamped>("face_detection", 20);
    image_debug_pub = it.advertise("image_debug", 1);
}

void FaceTrackerNode::handle_reconfigure(perception::FaceTrackerConfig&, uint32_t) {
    // Update config and reset.
    this->config = config;
    ROS_INFO("Reconfigured.");
    reset();
}

bool FaceTrackerNode::handle_reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    reset();
    return true;
}

void FaceTrackerNode::reset() {
    // TODO: Set face tracker parameters.
    face_tracker.reset();
    ROS_INFO("Reset.");
}

void FaceTrackerNode::handle_image(const sensor_msgs::ImageConstPtr& img_msg) {
    // To CV mat.
    cv_bridge::CvImagePtr cv_img;
    cv_img = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    // Detect / track.
    const auto rect_or_empty = face_tracker.findFace(cv_img->image);
    //boost::optional<cv::Rect> rect_or_empty;
    // Send detection.
    perception_msgs::FaceDetectionStamped det_stamped_msg;
    det_stamped_msg.header.stamp = cv_img->header.stamp;
    auto& det_msg = det_stamped_msg.face_detection;
    // Normalize aabb.
    if (rect_or_empty) {
        const auto& rect = rect_or_empty.get();
        det_msg.x_center = (rect.tl().x + rect.width / 2.f) / cv_img->image.cols;
        det_msg.y_center = (rect.tl().y + rect.height / 2.f) / cv_img->image.rows;
        det_msg.width = rect.width / cv_img->image.cols;
        det_msg.height = rect.height / cv_img->image.rows;
        det_msg.detected = true;
    }
    else {
        det_msg.detected = false;
    }
    // Publish debug image (if anyone is interested in).
    if (true || image_debug_pub.getNumSubscribers() > 0) {
        auto& img = cv_img->image;
        if (rect_or_empty) {
            cv::rectangle(img, rect_or_empty.get(), cv::Scalar(255, 0, 0));
        }
        std_msgs::Header header;
        header.stamp = cv_img->header.stamp;
        auto debug_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        image_debug_pub.publish(debug_msg);
    }
    face_detection_pub.publish(det_stamped_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "face_tracker_node");
    ros::NodeHandle nh("~");
    FaceTrackerNode face_tracker_node(nh);
    ros::Rate loop_rate(50);
    ros::spin();
}
