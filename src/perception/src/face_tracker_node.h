#pragma once

// Std
#include <functional>
// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <perception/FaceTrackerConfig.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <perception_msgs/FaceDetectionStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// Boost
#include <boost/optional.hpp>
// OpenCV
#include <opencv2/opencv.hpp>

#include "face_tracker.h"

class FaceTrackerNode {
public:
    FaceTrackerNode(ros::NodeHandle& nh);

private:
    void handle_reconfigure(perception::FaceTrackerConfig&, uint32_t);
    bool handle_reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void reset();

    void handle_image(const sensor_msgs::ImageConstPtr&);

    ros::NodeHandle& nh;
    image_transport::ImageTransport it;
    dynamic_reconfigure::Server<perception::FaceTrackerConfig> reconfigure_server;
    ros::ServiceServer reset_service;
    ros::Subscriber image_sub;
    ros::Publisher face_detection_pub;
    image_transport::Publisher image_debug_pub;
    FaceTracker face_tracker;
    perception::FaceTrackerConfig config;
};
