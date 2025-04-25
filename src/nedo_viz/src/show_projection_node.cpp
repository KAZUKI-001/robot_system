#include "nedo_viz/show_projection_node.hpp"
#include "rmw/qos_profiles.h"  // 追加
namespace nedo_viz
{

    ShowProjectionNode::ShowProjectionNode(const rclcpp::NodeOptions &options)
    : Node("debug_image_node",options){
        debug_img_puber_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image",rclcpp::SensorDataQoS());
        image_sub_.subscribe(this, "sync_image", rmw_qos_profile_sensor_data);      
        //image_sub_.subscribe(this,"sync_image");
        marker_sub.subscribe(this,"detected_contours");
        points_sub.subscribe(this,"projected");
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            vehicle_interface::msg::Markers,
            vehicle_interface::msg::ProjectedPoints>
            SyncPolicy;
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(50), image_sub_,marker_sub,points_sub
        ));
        //sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.005));
        sync_->registerCallback(
            std::bind(&ShowProjectionNode::sync_callback, this , std::placeholders::_1,std::placeholders::_2,std::placeholders::_3)
        );
    }

    void ShowProjectionNode::sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &image,
        const vehicle_interface::msg::Markers::ConstSharedPtr &markers,
        const vehicle_interface::msg::ProjectedPoints::ConstSharedPtr &points
    ){
        cv::Mat frame = cv_bridge::toCvCopy(image,"bgr8")->image;
        std::vector<nedo_localization::Marker> markers_std;
        vehicle_interface::msg::Markers marker_msg = *markers;
        nedo_localization::convert_msg_to(marker_msg,markers_std,nedo_localization::TO_MARKERS);
        publish_image(frame,markers_std,*points);
    }

    void ShowProjectionNode::publish_image(
      const cv::Mat &frame, 
      const std::vector<nedo_localization::Marker> & markers,
      const vehicle_interface::msg::ProjectedPoints & points
      ){
        cv::Mat pub_img;
        if(frame.empty()){
            pub_img = cv::Mat::zeros(cv::Size(1920,1080),CV_8UC3);
        }
        else{
            pub_img = frame;
        }
        if(!points.points.empty()){
            for(const auto & point :points.points){
                cv::Point2f img_point;
                img_point.x = point.u;
                img_point.y = point.v;
                bool inside_contour = false;
                
                for(const auto & marker : markers){
                    for(const auto & contour : marker.contours){
                        auto dist = cv::pointPolygonTest(contour,img_point,true);
                        if(dist>10){
                            double area = cv::contourArea(contour);
                            double centroid_dist = nedo_localization::compute_centroid_dist(marker);
                            bool judge_from_area_ok = nedo_localization::judge_from_contour_area(area,point);
                            bool judge_from_centroid_ok = nedo_localization::judge_from_centroid_dist(centroid_dist,point);
                            if(judge_from_area_ok && judge_from_centroid_ok){
                                inside_contour = true;
                            }
                        }
                    }
                }
                if(inside_contour){
                    cv::Point center;
                    center.x = point.u;
                    center.y = point.v;
                    cv::Scalar color(255,255,255);
                    cv::circle(pub_img,center,3,color,-1);
                }else{
                    cv::Point center;
                    center.x = point.u;
                    center.y = point.v;
                    // 奥行きに基づいて色相を調整（0~179の範囲でマッピング）
                    double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                    int hue = std::max(0, 179 - static_cast<int>(distance * 10));  // 奥が赤、手前が青

                    // HSVからBGRに変換して使用
                    cv::Mat hsv_color(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
                    cv::Mat bgr_color;
                    cv::cvtColor(hsv_color, bgr_color, cv::COLOR_HSV2BGR);

                    // 色をBGRで取得
                    cv::Scalar color(bgr_color.at<cv::Vec3b>(0, 0)[0], 
                                    bgr_color.at<cv::Vec3b>(0, 0)[1], 
                                    bgr_color.at<cv::Vec3b>(0, 0)[2]);
                    cv::circle(pub_img,center,3,color,-1);
                }
            }
        }
        if(!markers.empty()){
            for(const auto &marker : markers){
                for(const auto &contour : marker.contours){
                    const cv::Scalar color(150, 80, 255);
                    const int thickness = 2;          
                    const bool isClosed = true;       
                    cv::polylines(pub_img,contour,isClosed,color,thickness);
                }
            }
            //latest_markers_.clear();
        }
        cv::namedWindow("debug",cv::WINDOW_NORMAL);
        cv::resizeWindow("debug",1280,960);
        cv::imshow("debug",pub_img);
        cv::waitKey(2);
        //RCLCPP_INFO(this->get_logger(),"published!");
    }
}  // namespace nedo_viz

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_viz::ShowProjectionNode)
