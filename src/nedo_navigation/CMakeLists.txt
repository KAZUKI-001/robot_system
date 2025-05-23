cmake_minimum_required(VERSION 3.8)
project(nedo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# add executable
add_executable(nedo_controller src/nedo_controller.cpp)
ament_target_dependencies(nedo_controller rclcpp geometry_msgs)


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

# install targets
install(TARGETS
  nedo_controller
  DESTINATION lib/${PROJECT_NAME}
)
#install(PROGRAMS
  #test/test.bash
  #test/test_gui.bash
  #DESTINATION lib/${PROJECT_NAME}
#)

#ament_auto_package(
  #INSTALL_TO_SHARE
  #config
  #launch
  #test
#)

ament_package()
