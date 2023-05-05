#!/bin/bash
rosinstall_generator ros_comm cv_bridge image_transport diagnostic_updater camera_info_manager dynamic_reconfigure compressed_image_transport --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall && \
wstool merge -t src melodic-ros_comm-wet.rosinstall && \
wstool update -t src && \
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster && \
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
