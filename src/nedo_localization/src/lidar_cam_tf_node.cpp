#include "nedo_localization/lidar_cam_tf_node.hpp"
using json = nlohmann::json;

namespace nedo_localization
{
    LidarCamTF::LidarCamTF(const rclcpp::NodeOptions & options)
    : Node("cam_lidar_tf",options),tf_broadcaster_(this) {
        this->declare_parameter<std::string>("json_file_path", "");
        load_transform_from_json();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarCamTF::publish_tf, this));
    }

    void LidarCamTF::load_transform_from_json(){
        std::string json_file_path = this->get_parameter("json_file_path").as_string();
        if (json_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "JSON file path is not set. Use the 'json_file_path' parameter.");
            rclcpp::shutdown();
            return;
        }

        std::ifstream file(json_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_file_path.c_str());
            rclcpp::shutdown();
            return;
        }

        try {
            json j;
            file >> j;
            auto T_lidar_camera = j["results"]["T_lidar_camera"];

            if (T_lidar_camera.size() != 7) {
                RCLCPP_ERROR(this->get_logger(), "T_lidar_camera must have 7 elements: [x, y, z, qx, qy, qz, qw]");
                rclcpp::shutdown();
                return;
            }

            transform_.transform.translation.x = T_lidar_camera[0];
            transform_.transform.translation.y = T_lidar_camera[1];
            transform_.transform.translation.z = T_lidar_camera[2];
            transform_.transform.rotation.x = T_lidar_camera[3];
            transform_.transform.rotation.y = T_lidar_camera[4];
            transform_.transform.rotation.z = T_lidar_camera[5];
            transform_.transform.rotation.w = T_lidar_camera[6];

            transform_.header.frame_id = "lidar";
            transform_.child_frame_id = "camera";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON file: %s", e.what());
            rclcpp::shutdown();
        }

    }

    void LidarCamTF::publish_tf(){
        transform_.header.stamp = this->get_clock()->now();
        tf_broadcaster_.sendTransform(transform_);
    }
}
// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::LidarCamTF)