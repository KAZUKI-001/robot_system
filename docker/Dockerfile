ARG ROS_DISTRO=humble
ARG BASE_IMAGE=osrf/ros:${ROS_DISTRO}-desktop-full
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update

#install necessary pc utility
COPY ./scripts/install_pc_util.sh /usr/local/bin/install_pc_util.sh
RUN chmod +x /usr/local/bin/install_pc_util.sh
RUN . /usr/local/bin/install_pc_util.sh
 
#install necessary ros2 pkgs
COPY ./scripts/install_pkgs.sh /usr/local/bin/install_pkgs.sh
RUN chmod +x /usr/local/bin/install_pkgs.sh
RUN . /usr/local/bin/install_pkgs.sh

WORKDIR /root/NEDO_ROS
