#ifndef NEDO_VIZ__SHOW_PROJECTION_NODE_HPP_
#define NEDO_VIZ__SHOW_PROJECTION_NODE_HPP_

#include "nedo_viz/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vehicle_interface/msg/markers.hpp>
#include <vehicle_interface/msg/projected_points.hpp>
#include <nedo_localization/detection_utils.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace nedo_viz
{

  class ShowProjectionNode : public rclcpp::Node
  {
  public:
    ShowProjectionNode(const rclcpp::NodeOptions & options);

  private:
    
    void sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image,
      const vehicle_interface::msg::Markers::ConstSharedPtr &markers,
      const vehicle_interface::msg::ProjectedPoints::ConstSharedPtr &points
      );

    void publish_image(
      const cv::Mat &frame, 
      const std::vector<nedo_localization::Marker> & markers,
      const vehicle_interface::msg::ProjectedPoints & points
      );

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_puber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<vehicle_interface::msg::Markers> marker_sub;
    message_filters::Subscriber<vehicle_interface::msg::ProjectedPoints> points_sub;


    std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        vehicle_interface::msg::Markers,
        vehicle_interface::msg::ProjectedPoints
        >>> sync_;
  };

}  // namespace nedo_viz

#endif  // NEDO_VIZ__SHOW_PROJECTION_NODE_HPP_
