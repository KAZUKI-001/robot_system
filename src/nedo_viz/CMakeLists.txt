cmake_minimum_required(VERSION 3.8)
project(nedo_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters)
find_package(vehicle_interface REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nedo_localization REQUIRED)

add_library(show_projection_node_component SHARED src/show_projection_node.cpp)
target_compile_features(show_projection_node_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(show_projection_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(show_projection_node_component
 nedo_localization::detection_util
 )

ament_target_dependencies(
  show_projection_node_component
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  message_filters
  vehicle_interface
  rclcpp_components
  Eigen3
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(show_projection_node_component PRIVATE "NEDO_VIZ_BUILDING_LIBRARY")

rclcpp_components_register_node(
  show_projection_node_component
  PLUGIN "nedo_viz::ShowProjectionNode"
  EXECUTABLE show_projection_node
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS show_projection_node_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  show_projection_node_component
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
