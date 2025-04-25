#include "nedo_localization/localizer.hpp"

namespace nedo_localization{
    Localizer::Localizer(const rclcpp::NodeOptions & options)
    : Node("localizer_node",options),tf_broadcaster_(this) {
        marker_suber_ = this->create_subscription<vehicle_interface::msg::PoseStampedArray>(
            "matched_marker",
            rclcpp::SensorDataQoS(),
            std::bind(&Localizer::marker_pose_callback,this,std::placeholders::_1)
            );
        pose_puber_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose",rclcpp::SystemDefaultsQoS());
        load_map();
        calculate_success_ = false;
        has_previous_pose_ = true;
        lpf_rate_ = this->declare_parameter<double>("lpf_rate",0.0);
    }

    void Localizer::marker_pose_callback(const vehicle_interface::msg::PoseStampedArray::SharedPtr marker_poses){
        for(const auto &marker_pose : marker_poses->array){
            stored_poses_[marker_pose.header.frame_id] = marker_pose;
        }
        localization();
    }

    void Localizer::localization(){
        calculate_success_ = true;
        geometry_msgs::msg::PoseStamped pub_msg;

        localize_by_svd(pub_msg);
        if(calculate_success_){
            pub_msg.header.frame_id="map";
            pub_msg.header.stamp = this->now();
            //lpf_pose(pub_msg);
            pose_puber_ ->publish(pub_msg);
            previous_pose_ = pub_msg;
            RCLCPP_INFO(this->get_logger(),"publish Pose !!");
        }{
            RCLCPP_WARN(this->get_logger(),"failed to calculate Pose!!");
        }
    }
    
    void Localizer::load_map(){

        std::vector<std::string> ids = this->declare_parameter<std::vector<std::string>>("LM_ids",std::vector<std::string>());
        std::vector<double> xs = this->declare_parameter<std::vector<double>>("LM_xs",std::vector<double>());
        std::vector<double> ys = this->declare_parameter<std::vector<double>>("LM_ys",std::vector<double>());
        if(xs.size() != ids.size() || ys.size() != ids.size()){
            RCLCPP_ERROR(this->get_logger(),"parameters are not defined correctly!!");
            return;
        }
        for(size_t i =0;i<ids.size();i++){
            RCLCPP_INFO(this->get_logger(),"id:%s,x:%f,y:%f",ids[i].c_str(),xs[i],ys[i]);
        }
        for(size_t i = 0; i<ids.size();i++){
            geometry_msgs::msg::PoseStamped landmark;
            landmark.header.stamp = this->now();
            landmark.pose.position.x=xs.at(i);
            landmark.pose.position.y=ys.at(i);
            world_map_[ids.at(i)] = landmark;
        }
    }

    //localization uing svd method
    void Localizer::localize_by_svd(geometry_msgs::msg::PoseStamped &robot_pose){
        std::vector<Eigen::Vector2d> detectVec;
        std::vector<Eigen::Vector2d> worldVec;
        for(const auto &landmark : world_map_){
            std::string id = landmark.first;
            if(stored_poses_.find(id) != stored_poses_.end()){
                detectVec.emplace_back(Eigen::Vector2d(stored_poses_[id].pose.position.x,stored_poses_[id].pose.position.y));
                worldVec.emplace_back(Eigen::Vector2d(world_map_[id].pose.position.x,world_map_[id].pose.position.y));
                RCLCPP_INFO(this->get_logger(),"stored...id:%s,x:%f,y:%f",id.c_str(),stored_poses_[id].pose.position.x,stored_poses_[id].pose.position.y);
                RCLCPP_INFO(this->get_logger(),"world...id:%s,x:%f,y:%f",id.c_str(),world_map_[id].pose.position.x,world_map_[id].pose.position.y);
            }
        }
        if(detectVec.size()>1){
            best_fit_transform(detectVec,worldVec,robot_pose);
        }else{
            calculate_success_ = false;
            return;
        }
        has_previous_pose_ = true;
    }
    
    void Localizer::lpf_pose(geometry_msgs::msg::PoseStamped &current_pose){
        if(has_previous_pose_){
           current_pose.pose.position.x = previous_pose_.pose.position.x*lpf_rate_ + current_pose.pose.position.x*(1-lpf_rate_);
           current_pose.pose.position.y = previous_pose_.pose.position.y*lpf_rate_ + current_pose.pose.position.y*(1-lpf_rate_);
           current_pose.pose.position.z = previous_pose_.pose.position.z*lpf_rate_ + current_pose.pose.position.z*(1-lpf_rate_);
        }
    }

    // Function to calculate the best fit transform in 2D
    void Localizer::best_fit_transform(
        const std::vector<Eigen::Vector2d>& A, const std::vector<Eigen::Vector2d>& B, geometry_msgs::msg::PoseStamped &return_transform) {
        
        if (A.size() != B.size() || A.size() < 2) {
            throw std::invalid_argument("Point sets must have the same size and contain at least 2 points.");
        }

        // Step 1: Compute centroids of A and B
        Eigen::Vector2d centroid_A = Eigen::Vector2d::Zero();
        Eigen::Vector2d centroid_B = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < A.size(); ++i) {
            centroid_A += A[i];
            centroid_B += B[i];
        }
        centroid_A /= A.size();
        centroid_B /= B.size();

        // Step 2: Subtract centroids to shift points to the origin
        std::vector<Eigen::Vector2d> AA(A.size());
        std::vector<Eigen::Vector2d> BB(B.size());
        for (size_t i = 0; i < A.size(); ++i) {
            AA[i] = A[i] - centroid_A;
            BB[i] = B[i] - centroid_B;
        }

        // Step 3: Compute the covariance matrix H
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < A.size(); ++i) {
            H += AA[i] * BB[i].transpose();
        }

        // Step 4: Perform SVD on the covariance matrix H
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        // Step 5: Compute the rotation matrix R
        Eigen::Matrix2d R = V * U.transpose();

        // Handle the case where the determinant of R is negative
        if (R.determinant() < 0) {
            V.col(1) *= -1;  // Flip the second column of V
            R = V * U.transpose();
        }

        // Step 6: Compute the translation vector t
        Eigen::Vector2d t = centroid_B - R * centroid_A;

        // Return the rotation matrix and translation vector
        return_transform.pose.position.x = t(0);
        return_transform.pose.position.y = t(1);
        auto theta = std::atan2(R(1,0),R(0,0));
        tf2::Quaternion q;
        q.setRPY(0,0,theta);
        tf2::convert(q, return_transform.pose.orientation);
    }
} //nedo_localization

// コンポーネントとして登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_localization::Localizer)