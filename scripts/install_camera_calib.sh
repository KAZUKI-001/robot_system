sudo apt update
sudo apt install ros-$ROS_DISTRO-camera-calibration-parsers \
                 ros-$ROS_DISTRO-camera-info-manager \
                 ros-$ROS_DISTRO-launch-testing-ament-cmake -y
git clone -b $ROS_DISTRO https://github.com/ros-perception/image_pipeline.git ../third_party/src/image_pipeline

cd ../third_party
colcon build --packages-select \
	tracetools_image_pipeline \
	camera_calibration \
	depth_image_proc \
	image_publisher \
	image_rotate \
	image_view \
	image_pipeline \
	stereo_image_proc
cd ../scripts
