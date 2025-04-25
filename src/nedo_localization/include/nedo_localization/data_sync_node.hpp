#ifndef NEDO_LOCALIZATION__DATA_SYNC_NODE_HPP_
#define NEDO_LOCALIZATION__DATA_SYNC_NODE_HPP_

#include "nedo_localization/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace nedo_localization
{

  class DataSyncNode : public rclcpp::Node
  {
  public:
    DataSyncNode(const rclcpp::NodeOptions & options);

  private:
    
    void sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points 
      );

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sync_image_puber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_point_puber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_;


    std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::PointCloud2
        >>> sync_;
  };
  

}  // namespace nedo_localization

#endif  // NEDO_LOCALIZATION__DATA_SYNC_NODE_HPP_
