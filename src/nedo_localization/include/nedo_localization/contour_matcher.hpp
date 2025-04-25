#ifndef NEDO_LOCALIZATION__MATCHER_NODE_HPP_
#define NEDO_LOCALIZATION__MATCHER_NODE_HPP_

#include "nedo_localization/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <Eigen/Dense>
#include <nedo_localization/detection_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vehicle_interface/msg/markers.hpp>
#include <vehicle_interface/msg/projected_points.hpp>
#include <vehicle_interface/msg/pose_stamped_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace nedo_localization{

    class ContourMatcher :public rclcpp::Node
    {
        public:
            explicit ContourMatcher(const rclcpp::NodeOptions & options);
        
        private:
            void sync_callback(
                const vehicle_interface::msg::Markers::ConstSharedPtr &contours,
                const vehicle_interface::msg::ProjectedPoints::ConstSharedPtr &points
            );
            std::map<std::string,geometry_msgs::msg::PoseStamped> pre_markers_;
            bool marker_pose_ok(const geometry_msgs::msg::PoseStamped &marker_pose);
            void publish_marker_pose(vehicle_interface::msg::Markers &markers,const vehicle_interface::msg::ProjectedPoints & projected_points);

            inline Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points);
            rclcpp::Publisher<vehicle_interface::msg::PoseStampedArray>::SharedPtr marker_pose_puber_;
            message_filters::Subscriber<vehicle_interface::msg::Markers> marker_sub_;
            message_filters::Subscriber<vehicle_interface::msg::ProjectedPoints> points_sub_;
            std::shared_ptr<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
            vehicle_interface::msg::Markers,
            vehicle_interface::msg::ProjectedPoints
            >>> sync_;
    };

} //nedo_localization

#endif //NEDO_LOCALIZATION__MATCHER_NODE_HPP_