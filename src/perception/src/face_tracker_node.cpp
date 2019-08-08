#include "face_tracker_node.h"

FaceTrackerNode::FaceTrackerNode(ros::NodeHandle& nh) :
    nh(nh),
    image_transport(nh) {
    reconfigure_server.setCallback(std::bind(&FaceTrackerNode::handle_reconfigure, this, std::placeholders::_1, std::placeholders::_2));
    reset_service = nh.advertiseService("reset", &FaceTrackerNode::handle_reset, this);
    image_sub = image_transport.subscribe("image", 1, &FaceTrackerNode::handle_image, this);
    face_detection_pub = nh.advertise<perception_msgs::FaceDetectionStamped>("face_detection", 20);
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
    ROS_INFO("Received image.");
    // To CV mat.
    cv_bridge::CvImagePtr cv_img;
    cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    // Detect / track.
    const auto rect_or_empty = face_tracker.findFace(cv_img->image);
    // Send detection.
    perception_msgs::FaceDetectionStamped det_stamped_msg;
    det_stamped_msg.header.stamp = cv_img->header.stamp;
    auto& det_msg = det_stamped_msg.face_detection;
    // Normalize aabb.
    if (rect_or_empty) {
        const auto& rect = rect_or_empty.get();
        det_msg.x_center = (rect.tl().x + rect.width / 2) / cv_img->image.cols;
        det_msg.y_center = (rect.tl().y + rect.height / 2) / cv_img->image.rows;
        det_msg.width = rect.width / cv_img->image.cols;
        det_msg.height = rect.height / cv_img->image.rows;
        det_msg.detected = true;
    }
    else {
        det_msg.detected = false;
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
