cmake_minimum_required(VERSION 3.8)
project(cv_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv_pipeline_node 
  src/cv_pipeline_node.cpp
)

ament_target_dependencies(cv_pipeline_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(cv_pipeline_node
  ${OpenCV_LIBRARIES}
  curl
)

install(TARGETS
  cv_pipeline_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  python/
  DESTINATION share/${PROJECT_NAME}/python
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
