#include "face_tracker_node.h"

FaceTrackerNode::FaceTrackerNode(ros::NodeHandle& nh) :
    nh(nh),
    image_transport(nh) {
    image_subscriber = image_transport.subscribe("image", 1, &FaceTrackerNode::handleImage, this);
    face_detection_publisher = nh.advertise<perception_msgs::FaceDetectionStamped>("face_detection", 20);
}

void FaceTrackerNode::handleImage(const sensor_msgs::ImageConstPtr& img_msg) {
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
    face_detection_publisher.publish(det_stamped_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "face_tracker_node");
    ros::NodeHandle nh("~");
    FaceTrackerNode face_tracker_node(nh);
    ros::Rate loop_rate(50);
    ROS_INFO("Launched.");
    ros::spin();
}
