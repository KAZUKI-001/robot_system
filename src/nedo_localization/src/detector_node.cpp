#include "nedo_localization/detection_utils.hpp"
#include "nedo_localization/detector_node.hpp"

namespace nedo_localization{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node",options)
    {
        //int duration = 500;
        publisher_ =  this->create_publisher<vehicle_interface::msg::MarkerPointWithId>("detected_points",10);
        capture_sub_ = this->create_subscription<sensor_msgs::msg::Image>("capture",rclcpp::SensorDataQoS(),std::bind(&DetectorNode::capture_callback,this,std::placeholders::_1));
        initialize();
    }

    void DetectorNode::capture_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        vehicle_interface::msg::MarkerPointWithId publish_msg;
        publish_msg.header.frame_id = "camera_"+cv_bridge::toCvShare(msg,"bgr8")->header.frame_id;
        publish_msg.header.stamp = cv_bridge::toCvShare(msg,"bgr8")->header.stamp;
        try{
            cv::Mat frame = cv_bridge::toCvShare(msg,"bgr8")->image;
            std::vector<std::vector<int>> pub_data;
            pub_data = detectMarkers(frame);
            for(const auto& point : pub_data){
                publish_msg.data.push_back(point.at(0));//id
                publish_msg.data.push_back(point.at(1));//x
                publish_msg.data.push_back(point.at(2));//y
            }
        }
        catch (cv_bridge::Exception&e){
            RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s",e.what());
        }
        if(publish_msg.data.size()>0){
            publisher_->publish(publish_msg);
        }
    }



    //create captures map
    void DetectorNode::initialize(){
        int search_camera_num = 10;
        for(int i=0; i<search_camera_num;i++){
            std::string serial = get_camera_serial(i);
            if (!serial.empty()) {
                std::cout << "Camera ID: " << i << ", Serial: " << serial << std::endl;
                RCLCPP_INFO(this->get_logger(),"Camera ID: %d, Serial: %s found.",i,serial.c_str());
                std::string capture_name = "camera_"+serial;
            }
            else{
                RCLCPP_INFO(this->get_logger(),"Camera ID: %d not found.",i);
            }
        }
    }

    bool DetectorNode::is_capture_device(const std::string& device_path){
        int fd = open(device_path.c_str(),O_RDWR);
        if(fd == -1){
            RCLCPP_ERROR(this->get_logger(),"Error opening device");
            return false;
        }
        struct v4l2_capability cap;
        if (ioctl(fd,VIDIOC_QUERYCAP,&cap) == -1){
            RCLCPP_ERROR(this->get_logger(),"Error querying device capabilities");
            close(fd);
            return false;
        }
        if(!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)){
            RCLCPP_WARN(this->get_logger(),"device is not a video capture device.");
            return false;
        }
        return true;
    }

    std::string DetectorNode::get_camera_serial(int device_id) {
        std::string device_path = "/dev/video" + std::to_string(device_id);
        struct udev *udev;
        struct udev_device *dev;
        struct udev_enumerate *enumerate;
        struct udev_list_entry *devices, *dev_list_entry;
        if(!is_capture_device(device_path)){
            return "";
        }

        udev = udev_new();
        if (!udev) {
            std::cerr << "Cannot create udev context" << std::endl;
            return "";
        }

        enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);
        devices = udev_enumerate_get_list_entry(enumerate);

        std::string serial_number = "";

        udev_list_entry_foreach(dev_list_entry, devices) {
            const char *path = udev_list_entry_get_name(dev_list_entry);
            dev = udev_device_new_from_syspath(udev, path);

            const char *devnode = udev_device_get_devnode(dev);
            if (devnode && std::string(devnode) == device_path) {
                struct udev_device *parent_dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
                if (parent_dev) {
                    const char *serial = udev_device_get_sysattr_value(parent_dev, "serial");
                    if (serial) {
                        serial_number = std::string(serial);
                    }
                }
                udev_device_unref(dev);
                break;
            }
            udev_device_unref(dev);
        }

        udev_enumerate_unref(enumerate);
        udev_unref(udev);

        if (serial_number.empty()) {
            std::cerr << "Error retrieving serial for device " << device_path << std::endl;
        }

        return serial_number;
    }
}

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::DetectorNode)