#include "nedo_capture/nedo_cap_node.hpp"

namespace nedo_capture
{

NedoCapNode::NedoCapNode(const rclcpp::NodeOptions& options)
:Node("nedo_cap_node",options)
{
    //device = this->declare_parameter("device","/dev/video0");
    device = this->declare_parameter("device", "/dev/front_camera");
    frame_id = this->declare_parameter("frame_id","camera_frame");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    //auto qos = rclcpp::SystemDefaultsQoS();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("capture",qos);
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(20),
        std::bind(&NedoCapNode::timer_callback,this));

    // GStreamerパイプラインの定義 (MJPG, 4K, 60fps)
    //std::string pipeline = "v4l2src device=" + device + 
    //                       " ! image/jpeg, width=3840, height=2160, framerate=60/1 ! jpegdec ! videoconvert ! appsink";

    //std::string pipeline = "v4l2src device=" + device + 
    //                       " ! video/x-raw,format=YUY2,width=4416,height=1242,framerate=15/1 ! videoconvert ! appsink"; 
    
    // 追加：pipeline をパラメータから取得
    std::string pipeline;
    pipeline = this->declare_parameter("pipeline", "");
    if (pipeline.empty()) {
        if (device.find("left_camera") != std::string::npos || device.find("right_camera") != std::string::npos) {
            pipeline = "v4l2src device=" + device + " ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! appsink";
    } 
        else {
        pipeline = "v4l2src device=" + device + " ! video/x-raw,format=YUY2,width=4416,height=1242,framerate=15/1 ! videoconvert ! appsink";
    }
    }



                        
    cap.open(pipeline,cv::CAP_GSTREAMER);
    if(!cap.isOpened()){
        RCLCPP_ERROR(this->get_logger(),"Failed to opencamera.");
        RCLCPP_ERROR(this->get_logger(), "Pipeline used: %s", pipeline.c_str());
        return;
    }
}
void NedoCapNode::timer_callback(){
    if(cap.isOpened()){
        cv::Mat frame;
        cap >> frame;
        if(!frame.empty()){
            std_msgs::msg::Header header = std_msgs::msg::Header();
            header.frame_id = frame_id;
            header.stamp = this->now();
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            publisher_ -> publish(*msg);
        }
    }
}

NedoCapNode::~NedoCapNode()
{
    if(cap.isOpened()){
        cap.release();
    }
}

}  // namespace nedo_capture

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_capture::NedoCapNode)