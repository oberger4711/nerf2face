#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

void handleImageCompressed(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    ROS_INFO("Received image.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "face_tracker_node");
    ros::NodeHandle nh("~");
    ros::Subscriber camera_image = nh.subscribe("image_compressed", 1, handleImageCompressed);
    ros::Rate loop_rate(50);
    ROS_INFO("Launched.");
    ros::spin();
}
