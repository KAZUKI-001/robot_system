#include "nedo_localization/data_sync_node.hpp"

namespace nedo_localization{
    DataSyncNode::DataSyncNode(const rclcpp::NodeOptions &options)
    : Node("debug_image_node",options){
        //sync_image_puber_ = this->create_publisher<sensor_msgs::msg::Image>("sync_image",rclcpp::SystemDefaultsQoS());
        //sync_point_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sync_point",rclcpp::SystemDefaultsQoS());
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        sync_image_puber_ = this->create_publisher<sensor_msgs::msg::Image>("sync_image", qos);
        sync_point_puber_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sync_point", qos);

        image_sub_.subscribe(this,"capture");
        point_sub_.subscribe(this,"/livox/lidar");
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::PointCloud2>
            SyncPolicy;
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(50), image_sub_,point_sub_
        ));
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
        sync_->registerCallback(
            std::bind(&DataSyncNode::sync_callback, this , std::placeholders::_1,std::placeholders::_2)
        );

    }

    void DataSyncNode::sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &image,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points
    ){
        //RCLCPP_INFO(this->get_logger(),"publish data!");
        auto image_pub = *image;
        sync_image_puber_->publish(image_pub);
        auto point_pub = *points;
        sync_point_puber_->publish(point_pub);
    }
}//namespace nedo_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::DataSyncNode)