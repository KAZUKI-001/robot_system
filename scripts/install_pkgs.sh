#!/bin/bash
source /opt/ros/humble/setup.bash &&\
sudo apt update &&\
sudo apt install \
ros-$ROS_DISTRO-pcl-conversions \
ros-$ROS_DISTRO-teleop-twist-keyboard \
ros-$ROS_DISTRO-image-transport \
ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-v4l2-camera -y 


