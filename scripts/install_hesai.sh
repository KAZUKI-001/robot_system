#!/bin/bash
#install boost
sudo apt-get update
sudo apt-get install libboost-all-dev -y
#install yaml
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
#clone driver ripo
cd ../third_party
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git src/HesaiLidar_ROS_2.0
#build
colcon build --symlink-install
. install/local_setup.bash
cd ../scripts
