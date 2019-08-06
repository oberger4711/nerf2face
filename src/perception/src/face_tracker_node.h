#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <perception_msgs/FaceDetectionStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV
#include <opencv2/opencv.hpp>

#include "face_tracker.h"

class FaceTrackerNode {
public:
    FaceTrackerNode(ros::NodeHandle& nh);

private:
    void handleImage(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle& nh;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_subscriber;
    ros::Publisher face_detection_publisher;
    FaceTracker face_tracker;
};
