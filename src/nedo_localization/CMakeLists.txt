cmake_minimum_required(VERSION 3.8)
project(nedo_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vehicle_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(message_filters)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

pkg_check_modules(LIBUDEV REQUIRED libudev)
include_directories(${LIBUDEV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/sorter_localization)

add_library(projection_util SHARED src/projection_util.cpp)
add_library(detection_util SHARED src/detection_utils.cpp)

# detection_util

target_include_directories(detection_util
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${LIBUDEV_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(detection_util
  OpenCV
  vehicle_interface
)

# projection_util

target_include_directories(projection_util
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(projection_util
  geometry_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  OpenCV
  vehicle_interface
)

# contour_node_component
add_library(contour_node_component SHARED src/contour_node.cpp)
target_link_libraries(contour_node_component detection_util ${LIBUDEV_LIBRARIES})
target_compile_features(contour_node_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(contour_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(contour_node_component
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(contour_node_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(contour_node_component PLUGIN "nedo_localization::ContourNode" EXECUTABLE contour_node)

# contour_matcher_component
add_library(contour_matcher_component SHARED src/contour_matcher_node.cpp)
target_link_libraries(contour_matcher_component detection_util)
target_compile_features(contour_matcher_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(contour_matcher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(contour_matcher_component
  rclcpp
  geometry_msgs
  message_filters
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(contour_matcher_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(contour_matcher_component PLUGIN "nedo_localization::ContourMatcher" EXECUTABLE contour_matcher)

# data_sync_node_component
add_library(data_sync_node_component SHARED src/data_sync_node.cpp)
target_link_libraries(data_sync_node_component detection_util)
target_compile_features(data_sync_node_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(data_sync_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(data_sync_node_component
  rclcpp
  sensor_msgs
  message_filters
  rclcpp_components
)
target_compile_definitions(data_sync_node_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(data_sync_node_component PLUGIN "nedo_localization::DataSyncNode" EXECUTABLE data_sync_node)

# detector_node_component
add_library(detector_node_component SHARED src/detector_node.cpp)
target_compile_features(detector_node_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(detector_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${LIBUDEV_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detector_node_component
  detection_util
  ${LIBUDEV_LIBRARIES}
)
ament_target_dependencies(detector_node_component
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(detector_node_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(detector_node_component PLUGIN "nedo_localization::DetectorNode" EXECUTABLE detector_node)

# projection_node_component
add_library(projection_node_component SHARED src/projection_node.cpp)
target_link_libraries(projection_node_component projection_util)
target_compile_features(projection_node_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(projection_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(projection_node_component
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_msgs
  pcl_conversions
  geometry_msgs
  tf2_ros
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(projection_node_component PRIVATE "NEDO_CAPTURE_BUILDING_LIBRARY")
rclcpp_components_register_node(projection_node_component PLUGIN "nedo_localization::ProjectionNode" EXECUTABLE projection_node)

# lidar_cam_tf_component
add_library(lidar_cam_tf_component SHARED src/lidar_cam_tf_node.cpp)
target_link_libraries(lidar_cam_tf_component detection_util)
target_compile_features(lidar_cam_tf_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(lidar_cam_tf_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lidar_cam_tf_component
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  rclcpp_components
)
target_compile_definitions(lidar_cam_tf_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(lidar_cam_tf_component PLUGIN "nedo_localization::LidarCamTF" EXECUTABLE lidar_cam_tf)

# localizer_component
add_library(localizer_component SHARED src/localizer.cpp)
target_compile_features(localizer_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(localizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(localizer_component
  rclcpp
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp_components
  vehicle_interface
)
target_compile_definitions(localizer_component PRIVATE "NEDO_LOCALIZATION_BUILDING_LIBRARY")
rclcpp_components_register_node(localizer_component PLUGIN "nedo_localization::Localizer" EXECUTABLE localizer_node)

# installs
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS detection_util projection_util contour_node_component detector_node_component projection_node_component lidar_cam_tf_component contour_matcher_component data_sync_node_component localizer_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Config.cmake
  NAMESPACE nedo_localization::
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(OpenCV vehicle_interface)
ament_export_libraries(
  detection_util
  projection_util
  contour_node_component
  detector_node_component
  projection_node_component
  lidar_cam_tf_component
  contour_matcher_component
  data_sync_node_component
  localizer_component
)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
