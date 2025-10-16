cmake_minimum_required(VERSION 3.8)
project(kinect2_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Create SLAM node
add_executable(slam_node
  src/slam_node.cpp
)

target_include_directories(slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include
)

ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
)

# Install
install(TARGETS
  slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
