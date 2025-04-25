#!/bin/bash

# 引数チェック
if [ "$#" -ne 3 ]; then
  echo "Usage: $1 <x> <y> <theta (in degrees)>"
fi

# 引数
x=$1
y=$2
theta_deg=$3

# 度をラジアンに変換
theta_rad=$(echo "$theta_deg * 3.14159265358979 / 180" | bc -l)

# 四元数の計算 (yaw = theta_rad)
qx=0.0
qy=0.0
qz=$(echo "s($theta_rad / 2)" | bc -l)
qw=$(echo "c($theta_rad / 2)" | bc -l)

# ROS 2 コマンドでメッセージを送信
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    position: {x: $x, y: $y, z: 0.0},
    orientation: {x: $qx, y: $qy, z: $qz, w: $qw}
  }
}" --once

