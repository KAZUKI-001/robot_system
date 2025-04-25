#!/bin/bash
# スクリプトの絶対パスを取得
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# リポジトリの絶対パスを取得
WS_DIR="$(cd "$(dirname $SCRIPT_DIR)" && pwd)"
cd $WS_DIR/ros_ws
rm -rf build install log  && \
colcon build --packages-select vehicle_interface && \
source install/setup.bash && \
colcon build --packages-select nedo_localization && \
source install/setup.bash && \
colcon build 
cd $WS_DIR/scripts