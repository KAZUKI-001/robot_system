#include "nedo_localization/contour_matcher.hpp"

namespace nedo_localization{
    ContourMatcher::ContourMatcher(const rclcpp::NodeOptions & options)
    : Node("contour_matcher",options)
    {
        marker_pose_puber_=this->create_publisher<vehicle_interface::msg::PoseStampedArray>("/matched_marker",rclcpp::SensorDataQoS());

        marker_sub_.subscribe(this,"detected_contours");
        points_sub_.subscribe(this,"projected");
        typedef message_filters::sync_policies::ApproximateTime<
        vehicle_interface::msg::Markers,
        vehicle_interface::msg::ProjectedPoints>
        SyncPolicy;
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(50),marker_sub_,points_sub_
        ));
        //sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.005));
        sync_->registerCallback(
            std::bind(&ContourMatcher::sync_callback,this,std::placeholders::_1,std::placeholders::_2)
        );
    }

    void ContourMatcher::sync_callback(
        const vehicle_interface::msg::Markers::ConstSharedPtr &markers,
        const vehicle_interface::msg::ProjectedPoints::ConstSharedPtr &points
    ){
        auto time1 = rclcpp::Time(markers->header.stamp);
        auto time2 = rclcpp::Time(points->header.stamp);
        if (time1.nanoseconds() != 0 && time2.nanoseconds() != 0) {
            rclcpp::Duration diff = time1 - time2;
            RCLCPP_INFO(this->get_logger(), "Time difference (s): %.6f", diff.seconds());
        }
        vehicle_interface::msg::Markers latest_markers = *markers;
        publish_marker_pose(latest_markers,*points);
    }

    void ContourMatcher::publish_marker_pose(vehicle_interface::msg::Markers &markers,const vehicle_interface::msg::ProjectedPoints & projected_points){
        std::vector<Marker> detected_markers;
        convert_msg_to(markers,detected_markers,TO_MARKERS);
        if(!detected_markers.empty() && !projected_points.points.empty()){
            vehicle_interface::msg::PoseStampedArray pub_msg;
            for(const auto & marker : detected_markers){
                geometry_msgs::msg::PoseStamped tmp_pose;
                std::vector<Eigen::Vector3d> points_on_marker;
                //compute_distance of centroids of marker
                double centroid_dist = nedo_localization::compute_centroid_dist(marker);
                for(const auto & contour : marker.contours){
                    double area = cv::contourArea(contour);
                    for(const auto & point :projected_points.points){
                        cv::Point2f img_point;
                        img_point.x = point.u;
                        img_point.y = point.v;
                        auto dist = cv::pointPolygonTest(contour,img_point,true);
                        if(dist>0){
                            bool judge_from_area_ok = nedo_localization::judge_from_contour_area(area,point);
                            bool judge_from_centroid_ok = nedo_localization::judge_from_centroid_dist(centroid_dist,point);
                            Eigen::Vector3d inside_point;
                            if(judge_from_area_ok && judge_from_centroid_ok){
                                inside_point << point.x,point.y,point.z;
                                points_on_marker.push_back(inside_point);
                            }
                        }
                    }
                }
                if(!points_on_marker.empty()){
                    auto centroid = computeCentroid(points_on_marker);
                    tmp_pose.header.frame_id = "marker"+std::to_string(marker.id);
                    tmp_pose.header.stamp = this->now();
                    tmp_pose.pose.position.x = centroid(0);
                    tmp_pose.pose.position.y = centroid(1);
                    tmp_pose.pose.position.z = centroid(2);
                    double distance = std::sqrt(centroid(0)*centroid(0)+centroid(1)*centroid(1)+centroid(2)*centroid(2));
                    double up_low_D = compute_centroid_dist(marker);
                    //RCLCPP_INFO(this->get_logger(),"%f,%f",up_low_D,distance);
                    if(marker_pose_ok(tmp_pose)){
                        pub_msg.array.push_back(tmp_pose);
                        pre_markers_[tmp_pose.header.frame_id] = tmp_pose;
                    }
                }else{
                    //RCLCPP_WARN(this->get_logger(),"no point matched!");
                }
                if(!pub_msg.array.empty()){
                    pub_msg.header.frame_id = "map";
                    pub_msg.header.stamp = this->now();
                    //publish array of PoseStamped
                    marker_pose_puber_->publish(pub_msg);
                }
            }
        }
    }

    bool ContourMatcher::marker_pose_ok(const geometry_msgs::msg::PoseStamped &marker_pose){
        if(pre_markers_.find(marker_pose.header.frame_id) == pre_markers_.end()){
            //when new marker received
            RCLCPP_INFO(this->get_logger(),"new!!!");
            return true;
        }
        //calculate time difference
        const auto incoming_stamp = rclcpp::Time(marker_pose.header.stamp);
        const auto previous_stamp = rclcpp::Time(pre_markers_[marker_pose.header.frame_id].header.stamp);
        rclcpp::Duration duration = incoming_stamp - previous_stamp;
        //calculate max marker movement assuming robot's max speed as 0.6 m/s.
        double marker_dist_tolerance = duration.seconds()*0.8;
        const double x = marker_pose.pose.position.x;
        const double y = marker_pose.pose.position.y;
        const double prev_x = pre_markers_[marker_pose.header.frame_id].pose.position.x;
        const double prev_y = pre_markers_[marker_pose.header.frame_id].pose.position.y;
        double marker_movement = std::sqrt((x-prev_x)*(x-prev_x)+(y-prev_y)*(y-prev_y));
        if(marker_movement<marker_dist_tolerance){
            RCLCPP_INFO(this->get_logger(),"marker movement check ok!");
            return true;
        }else{
            RCLCPP_INFO(this->get_logger(),"marker movement check failed!");
            return false;
        }
    }

    inline Eigen::Vector3d ContourMatcher::computeCentroid(const std::vector<Eigen::Vector3d>& points){
        Eigen::Vector3d centroid(0.0,0.0,0.0);
        for (const auto & point : points){
            centroid += point;
        }
        if(!points.empty()){
            centroid /= static_cast<double>(points.size());
        }
        return centroid;
    }
}

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::ContourMatcher)