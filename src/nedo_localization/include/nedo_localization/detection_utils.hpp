#ifndef NEDO_CAPTURE__DETECTOR_UTIL_HPP_
#define NEDO_CAPTURE__DETECTOR_UTIL_HPP_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <vehicle_interface/msg/markers.hpp>
#include <vehicle_interface/msg/point_image.hpp>

namespace nedo_localization{
    extern const int TO_MSG;
    extern const int TO_MARKERS;
    struct Marker
    {
        int id;
        std::vector<std::vector<cv::Point>> contours;
    };

    cv::Point2f getCentroid(std::vector<cv::Point>& contour);

    bool isTriangle(std::vector<cv::Point>& contour, double minArea);

    bool isUpwardTriangle(std::vector<cv::Point>& contour, cv::Point& centroid);

    bool isDownwardTriangle(std::vector<cv::Point>& contour, cv::Point& centroid);

    bool isCircle(std::vector<cv::Point>& contour, double minArea, cv::Point& centroid);

    int identifyMarkerPattern(std::vector<int>& xCoordinates, std::vector<int>& yCoordinates, std::vector<int>& types);

    double est_max_dist(double area, double f, double a);

    double est_min_dist(double area, double f, double b);

    bool judge_from_contour_area(double area,const vehicle_interface::msg::PointImage &point);

    bool judge_from_centroid_dist(double centroid_dist,const vehicle_interface::msg::PointImage &point);

    double compute_centroid_dist(const Marker marker);

    void convert_msg_to(vehicle_interface::msg::Markers &ros_msg, std::vector<Marker> &markers,int cvt_to);

    std::vector<std::vector<int>> detectMarkers(cv::Mat& frame);
    std::vector<Marker> detectContours(cv::Mat& frame);
} //namespace nedo_localization

#endif //NEDO_CAPTURE__DETECTOR_UTIL_HPP_