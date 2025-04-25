#include "nedo_localization/detection_utils.hpp"
namespace nedo_localization{
    const int TO_MARKERS = 1;
    const int TO_MSG = 0;
    // 赤色領域を抽出する関数
    cv::Mat extractRedArea(const cv::Mat& image) {
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

        for (int y = 0; y < image.rows; y++) {
            for (int x = 0; x < image.cols; x++) {
                cv::Vec3b color = image.at<cv::Vec3b>(y, x);
                int R = color[2];
                int G = color[1];
                int B = color[0];
                int total = R + G + B;

                if (total > 0) {
                    float r_ratio = static_cast<float>(R) / total;
                    float g_ratio = static_cast<float>(G) / total;
                    float b_ratio = static_cast<float>(B) / total;

                    if (r_ratio >= 0.46 && g_ratio <= 0.32 && b_ratio <= 0.32) {
                        mask.at<uchar>(y, x) = 255;
                    }
                }
            }
        }
        return mask;
    }

    // 形状を判定する関数
    int classifyShape(const std::vector<cv::Point>& contour) {
        int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
        for (const cv::Point& p : contour) {
            min_x = cv::min(min_x, p.x);
            max_x = cv::max(max_x, p.x);
            min_y = cv::min(min_y, p.y);
            max_y = cv::max(max_y, p.y);
        }

        int width = max_x - min_x;
        int height = max_y - min_y;
        double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
        double area_ratio = cv::contourArea(contour)/static_cast<double>(width)*static_cast<double>(height);

        // 1. 円の判定
        if (aspect_ratio >= 0.7 && aspect_ratio <= 1.5 &&area_ratio>0.70) {
            return 2; // 円
        }

        // 2. 三角形の判定（円でなかった場合）
        if (height > width && aspect_ratio>0.2) {
            int mid_y = min_y + height / 2;
            double upper_area = 0.0, lower_area = 0.0;
            std::vector<cv::Point> upperPart, lowerPart;

            // 輪郭を上下に分割
            for (const cv::Point& p : contour) {
                if (p.y < mid_y) {
                    upperPart.push_back(p);
                }
                else {
                    lowerPart.push_back(p);
                }
            }

            if (!upperPart.empty()) {
                upper_area = contourArea(upperPart);
            }
            if (!lowerPart.empty()) {
                lower_area = contourArea(lowerPart);
            }

            return (upper_area < lower_area) ? 0 : 1; // 0: 上向き三角形, 1: 下向き三角形
        }

        return -1; // 未分類
    }

    // 2つの図形で識別するランドマークのパターン判別関数
    int identifyMarkerPattern(std::vector<int>& xCoordinates, std::vector<int>& yCoordinates, std::vector<int>& types) {
        if (xCoordinates.size() != 2 || yCoordinates.size() != 2 || types.size() != 2) return -1;

        if (abs(xCoordinates[0] - xCoordinates[1]) > 50) {
            return -1;
       }

        std::vector<std::pair<int, int>> yTypePairs;
        for (size_t i = 0; i < yCoordinates.size(); i++) {
            yTypePairs.push_back(std::make_pair(yCoordinates[i], types[i]));
        }
        sort(yTypePairs.begin(), yTypePairs.end());

        std::vector<int> sortedTypes;
        for (const auto& pair : yTypePairs) {
            sortedTypes.push_back(pair.second);
        }

        if (sortedTypes == std::vector<int>{0, 0}) return 0;
        if (sortedTypes == std::vector<int>{0, 1}) return 1;
        if (sortedTypes == std::vector<int>{0, 2}) return 2;
        if (sortedTypes == std::vector<int>{1, 0}) return 3;
        if (sortedTypes == std::vector<int>{1, 1}) return 4;
        if (sortedTypes == std::vector<int>{1, 2}) return 5;
        if (sortedTypes == std::vector<int>{2, 0}) return 6;
        if (sortedTypes == std::vector<int>{2, 1}) return 7;
        if (sortedTypes == std::vector<int>{2, 2}) return 8;

        return -1;
    }

    // マーカーを検出する関数
    std::vector<std::vector<int>> detectMarkers(cv::Mat& frame) {
        cv::Mat mask = extractRedArea(frame);
        std::vector<std::vector<int>> ret_id_points;

        std::vector<std::vector<cv::Point>> contours;
        findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<int> xCoordinates;
        std::vector<int> yCoordinates;
        std::vector<int> types;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area <= 150.0) {
                continue; // 面積が150以下の場合はスキップ
            }

            int shape_type = classifyShape(contours[i]);
            if (shape_type != -1) {
                cv::Moments M = moments(contours[i]);
                int cX = static_cast<int>(M.m10 / M.m00);
                int cY = static_cast<int>(M.m01 / M.m00);

                xCoordinates.push_back(cX);
                yCoordinates.push_back(cY);
                types.push_back(shape_type);
            }
        }

        for (size_t i = 0; i < xCoordinates.size(); i++) {
            for (size_t j = i + 1; j < xCoordinates.size(); j++) {
                std::vector<int> xSubset = { xCoordinates[i], xCoordinates[j] };
                std::vector<int> ySubset = { yCoordinates[i], yCoordinates[j] };
                std::vector<int> typeSubset = { types[i], types[j] };

                int landmarkID = identifyMarkerPattern(xSubset, ySubset, typeSubset);
                if (landmarkID != -1) {
                    // 上部の図形の重心座標を出力
                    int upperIndex = (ySubset[0] < ySubset[1]) ? 0 : 1;
                    ret_id_points.push_back({ landmarkID, xSubset[upperIndex], ySubset[upperIndex] });
                }
            }
        }

        return ret_id_points;
    }

    // マーカーの輪郭抽出をする関数
    std::vector<Marker> detectContours(cv::Mat& frame) {
        cv::Mat mask = extractRedArea(frame);
        std::vector<Marker> ret_marker_contours;
        std::vector<std::vector<cv::Point>> contours;
        findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<int> xCoordinates;
        std::vector<int> yCoordinates;
        std::vector<int> types;
        std::vector<std::vector<cv::Point>> procd_contours;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area <= 150.0) {
                continue; // 面積が150以下の場合はスキップ
            }

            int shape_type = classifyShape(contours[i]);
            if (shape_type != -1) {
                cv::Moments M = moments(contours[i]);
                int cX = static_cast<int>(M.m10 / M.m00);
                int cY = static_cast<int>(M.m01 / M.m00);

                xCoordinates.push_back(cX);
                yCoordinates.push_back(cY);
                types.push_back(shape_type);
                procd_contours.push_back(contours[i]);
            }
        }

        for (size_t i = 0; i < xCoordinates.size(); i++) {
            for (size_t j = i + 1; j < xCoordinates.size(); j++) {
                std::vector<int> xSubset = { xCoordinates[i], xCoordinates[j] };
                std::vector<int> ySubset = { yCoordinates[i], yCoordinates[j] };
                std::vector<int> typeSubset = { types[i], types[j] };

                int landmarkID = identifyMarkerPattern(xSubset, ySubset, typeSubset);
                if (landmarkID != -1) {
                    // 上部の図形の重心座標を出力
                    Marker marker;
                    marker.id = landmarkID;
                    marker.contours.push_back(procd_contours[i]);
                    marker.contours.push_back(procd_contours[j]);
                    ret_marker_contours.push_back(marker);
                    
                }
            }
        }

        return ret_marker_contours;
    }

    double est_max_dist(double area, double f, double a){
        double r = 0.15;
        return r*std::sqrt(1+3.141592653589793*f*f/area)+a/area+1.0;
    }

    double est_min_dist(double area, double f, double b){
        double r = 0.15;
        return r*std::sqrt(1+3.141592653589793*f*f/area)-b/area-1.0;
    }

    bool judge_from_contour_area(double area,const vehicle_interface::msg::PointImage &point){
        double distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        double min_dist = nedo_localization::est_min_dist(area,2200.0,4000.0);
        double max_dist = nedo_localization::est_max_dist(area,2200.0,6500.0);
        if(min_dist<distance && distance <max_dist){
            return true;
        }
        return false;
    }

    bool judge_from_centroid_dist(double centroid_dist,const vehicle_interface::msg::PointImage &point){
        double distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        double max_dist = 2000.0/centroid_dist;
        double min_dist = 1000.0/centroid_dist;
        if(min_dist<distance && distance <max_dist){
            return true;
        }
        return false;
    }

    double compute_centroid_dist(const Marker marker){
        ////computeCentroid on image
        auto up = marker.contours.at(0);
        auto low = marker.contours.at(1);
        cv::Moments up_M = cv::moments(up);
        cv::Moments low_M = cv::moments(low);
        double up_cx = up_M.m10/up_M.m00;
        double up_cy = up_M.m01/up_M.m00;
        double low_cx = low_M.m10/low_M.m00;
        double low_cy = low_M.m01/low_M.m00;
        return std::sqrt((up_cx-low_cx)*(up_cx-low_cx)+(up_cy-low_cy)*(up_cy-low_cy));
    }

    
    void convert_msg_to(vehicle_interface::msg::Markers &ros_msg, std::vector<Marker> &markers,int cvt_to){
        if(cvt_to == TO_MSG){
            //check marker existance.
            if(markers.empty()){
                return;
            }
            //push back markers
            for(Marker marker : markers){
                vehicle_interface::msg::MarkerWithContour converted_marker;
                converted_marker.id = marker.id;
                for(std::vector<cv::Point> contour :marker.contours){
                    vehicle_interface::msg::IntPointArray converted_contour;
                    for(cv::Point point : contour){
                        vehicle_interface::msg::IntPoint converted_point;
                        converted_point.x = point.x;
                        converted_point.y = point.y;
                        converted_contour.points.push_back(converted_point);
                    }
                    converted_marker.contours.push_back(converted_contour);
                }
                ros_msg.markers.push_back(converted_marker);
            }
        }
        else if(cvt_to == TO_MARKERS){
            //check marker existance.
            if(ros_msg.markers.empty()){
                return;
            }
            //push back markers
            for(vehicle_interface::msg::MarkerWithContour marker : ros_msg.markers){
                Marker converted_marker;
                converted_marker.id = marker.id;
                for(vehicle_interface::msg::IntPointArray contour :marker.contours){
                    std::vector<cv::Point> converted_contour;
                    for(vehicle_interface::msg::IntPoint point : contour.points){
                        cv::Point converted_point;
                        converted_point.x = point.x;
                        converted_point.y = point.y;
                        converted_contour.push_back(converted_point);
                    }
                    converted_marker.contours.push_back(converted_contour);
                }
                markers.push_back(converted_marker);
            }
        }
    }
} //namespace nedo_localization