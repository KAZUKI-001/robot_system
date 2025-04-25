#!/bin/bash
# スクリプトの絶対パスを取得
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# リポジトリの絶対パスを取得
WS_DIR="$(cd "$(dirname $SCRIPT_DIR)" && pwd)"
source $WS_DIR/third_party/install/setup.bash
source $WS_DIR/ros_ws/install/setup.bash
