#ifndef NEDO_LOCALIZATION__DETECTOR_NODE_HPP_
#define NEDO_LOCALIZATION__DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libudev.h>
#include <opencv2/opencv.hpp>
#include "vehicle_interface/msg/marker_point_with_id.hpp"
namespace nedo_localization{

    class DetectorNode : public rclcpp::Node
    {
        public:
            explicit DetectorNode(const rclcpp::NodeOptions & options);
        private:
            void capture_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
            void initialize();
            bool is_capture_device(const std::string& device_path);
            std::string get_camera_serial(int device_id);
            rclcpp::Publisher<vehicle_interface::msg::MarkerPointWithId>::SharedPtr publisher_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr capture_sub_;
    };

}


#endif //NEDO_LOCALIZATION__DETECTOR_NODE_HPP_