cmake_minimum_required(VERSION 3.8)
project(kinect2_simple_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Manual libfreenect2 setup
set(freenect2_INCLUDE_DIRS "/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/include")
set(freenect2_LIBRARIES "/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib/libfreenect2.so")

# Create executable
add_executable(kinect2_simple_publisher_node
  src/kinect2_simple_publisher.cpp
)

target_include_directories(kinect2_simple_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${freenect2_INCLUDE_DIRS}
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include
)

ament_target_dependencies(kinect2_simple_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(kinect2_simple_publisher_node
  ${freenect2_LIBRARIES}
  pthread
)

# Install
install(TARGETS
  kinect2_simple_publisher_node
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
