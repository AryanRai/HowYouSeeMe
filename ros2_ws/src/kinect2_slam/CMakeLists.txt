cmake_minimum_required(VERSION 3.8)
project(kinect2_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ORB-SLAM3
set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ORB_SLAM3")
include_directories(
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  ${ORB_SLAM3_ROOT_DIR}/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
)

# Create SLAM node (placeholder - not used currently)
# add_executable(slam_node
#   src/slam_node.cpp
# )

# target_include_directories(slam_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   /opt/ros/jazzy/include/cv_bridge
#   /opt/ros/jazzy/include
# )

# ament_target_dependencies(slam_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   cv_bridge
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   OpenCV
# )

# ORB-SLAM3 Node
add_executable(orb_slam3_node
  src/orb_slam3_node.cpp
)

# Add OpenGL definitions to fix Pangolin compatibility
target_compile_definitions(orb_slam3_node PRIVATE
  GL_GLEXT_PROTOTYPES
  HAVE_GLEW
)

target_include_directories(orb_slam3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include
)

target_link_libraries(orb_slam3_node
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  Eigen3::Eigen
  GL
  GLEW
  /usr/local/lib/libpango_display.so
  /usr/local/lib/libpango_windowing.so
  /usr/local/lib/libpango_opengl.so
  /usr/local/lib/libpango_core.so
  -lpthread
)

ament_target_dependencies(orb_slam3_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  orb_slam3_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  kinect2_slam/tsdf_integrator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tsdf_integrator
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
