#include "nedo_localization/projection_node.hpp"
#include "nedo_localization/projection_util.hpp"

namespace nedo_localization
{
    ProjectionNode::ProjectionNode(const rclcpp::NodeOptions & options):
     Node("projection_node",options), tf_buffer_(get_clock()),tf_listener_(tf_buffer_)
    {
        initialize_ = false;
        point_suber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "sync_point", 
            rclcpp::SensorDataQoS(), 
            std::bind(&ProjectionNode::point_callback, this, std::placeholders::_1)
        );
        caminfo_suber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",rclcpp::SystemDefaultsQoS(),
            std::bind(&ProjectionNode::caminfo_callback,this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<vehicle_interface::msg::ProjectedPoints>("projected", rclcpp::SystemDefaultsQoS());
        camera_frame_name_ = "lidar";
        lidar_frame_name_ = "camera";
    }
    void ProjectionNode::caminfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo){
        if(caminfo_.width==0){//when caminfo is empty.
            caminfo_ = *caminfo;
        }
    }

    void ProjectionNode::point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
        auto start = std::chrono::high_resolution_clock::now();
        pcl::fromROSMsg(*cloud_msg,latest_scan_);
        if(!initialize_){
            try{
                camera_lidar_transform_ = lookup_transform(camera_frame_name_,lidar_frame_name_);
                initialize_ = false;
            }catch(tf2::TransformException &ex){
                initialize_ = false;
                return;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to lookup transform.",elapsed_time);
        if (!latest_scan_.empty()){
            std::vector<Eigen::RowVectorXd> projected_points = project_image_coordinate(caminfo_,camera_lidar_transform_,latest_scan_,true); 
            end = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            //RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to projection ",elapsed_time);
            auto pub_msg = create_projected_msg(projected_points);
            pub_msg.header = cloud_msg -> header;
            publisher_->publish(pub_msg);
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //RCLCPP_INFO(this->get_logger(),"took %ld milliseconds to projection projection callback.",elapsed_time);
    }
    geometry_msgs::msg::TransformStamped ProjectionNode::lookup_transform(const std::string &source_frame,const std::string &target_frame)
    {
        try{
            return tf_buffer_.lookupTransform(source_frame,target_frame,tf2::TimePointZero,tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
            throw ex;
        }
    }
}  // namespace nedo_localization


// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::ProjectionNode)