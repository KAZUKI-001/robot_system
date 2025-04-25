#!/bin/bash
cd ../third_party/src
git clone https://github.com/agilexrobotics/ugv_sdk.git 
git clone https://github.com/agilexrobotics/ranger_ros2.git
cd ..
colcon build
source install/setup.bash
cd ../scripts
