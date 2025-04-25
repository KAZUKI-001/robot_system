#!/bin/bash
xhost +local:

docker run   --rm   --net host   --gpus all   -e DISPLAY=$DISPLAY  \
        -v $HOME/.Xauthority:/root/.Xauthority   \
        -v ./bags:/tmp/input_bags   \
        -v ./preprocessed:/tmp/preprocessed   \
        koide3/direct_visual_lidar_calibration:humble   \
        ros2 run direct_visual_lidar_calibration preprocess -a -d \
        /tmp/input_bags /tmp/preprocessed

docker run   --rm   --net host   --gpus all   -e DISPLAY=$DISPLAY   \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v ./preprocessed:/tmp/preprocessed   mykoide:latest   \
        ros2 run direct_visual_lidar_calibration find_matches_superglue.py \
        /tmp/preprocessed

docker run   --rm   --net host   --gpus all   -e DISPLAY=$DISPLAY   \
        -v $HOME/.Xauthority:/root/.Xauthority   \
        -v ./bags:/tmp/input_bags   \
        -v ./preprocessed:/tmp/preprocessed   \
        koide3/direct_visual_lidar_calibration:humble   \
        ros2 run direct_visual_lidar_calibration initial_guess_auto \
        /tmp/preprocessed

docker run   --rm   --net host   --gpus all   -e DISPLAY=$DISPLAY   \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v ./preprocessed:/tmp/preprocessed   \
        koide3/direct_visual_lidar_calibration:humble   \
        ros2 run direct_visual_lidar_calibration calibrate \
        /tmp/preprocessed

