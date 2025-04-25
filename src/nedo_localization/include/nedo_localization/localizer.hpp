#ifndef NEDO_LOCALIZATION__LOCALIZER_HPP
#define NEDO_LOCALIZATION__LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>
namespace nedo_localization{
    class Localizer : public rclcpp::Node
    {
        public:
            explicit Localizer(const rclcpp::NodeOptions & options);
        private:
        void marker_pose_callback(const vehicle_interface::msg::PoseStampedArray::SharedPtr marker_poses);
        void load_map();
        void lookup_transform();
        void publish_pose();
        void localization();
        void localize_by_svd(geometry_msgs::msg::PoseStamped &robot_pose);
        void localize_by_two_points(geometry_msgs::msg::PoseStamped &robot_pose);
        void lpf_pose(geometry_msgs::msg::PoseStamped &current_pose);
        void best_fit_transform(
            const std::vector<Eigen::Vector2d>& A, const std::vector<Eigen::Vector2d>& B,geometry_msgs::msg::PoseStamped &return_transform);
        bool calculate_success_;
        bool has_previous_pose_;
        double lpf_rate_;
        geometry_msgs::msg::PoseStamped previous_pose_;
        rclcpp::Subscription<vehicle_interface::msg::PoseStampedArray>::SharedPtr marker_suber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_puber_;
        rclcpp::TimerBase::SharedPtr timer_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        geometry_msgs::msg::TransformStamped transform_;
        std::map<std::string,geometry_msgs::msg::PoseStamped> stored_poses_;
        std::map<std::string,geometry_msgs::msg::PoseStamped> world_map_;
    };
}

#endif //NEDO_LOCALIZATION__LOCALIZER_HPP