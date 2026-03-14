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
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find nlohmann_json for JSON parsing
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found, will use header-only version")
    include(FetchContent)
    FetchContent_Declare(json URL
        https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
    FetchContent_MakeAvailable(json)
endif()

# Rerun C++ SDK - Disabled due to Arrow build complexity
# 
# Status: The Rerun C++ logger code is complete and ready in src/rerun_logger_cpp_node.cpp
# 
# Issue: Building Rerun C++ SDK requires Apache Arrow C++, which has complex build dependencies.
# - Rerun SDK 0.20.2: Built for Arrow 14.0, incompatible with system Arrow 23.0 (ABI mismatch)
# - Rerun SDK 0.30.2: Pre-built bundle still has Arrow version conflicts
# - Building Arrow from source: Fails during compilation (see build logs)
#
# The C++ logger would provide:
# - RGB/depth image logging with downsampling
# - ORB-SLAM3 pose and trajectory visualization
# - TSDF point cloud streaming (max 50k points)
# - World state objects with labels and colors
# - Semantic marker visualization
# - Recording to .rrd files for playback
#
# Current solution: Use RViz2 for visualization (already integrated and working)
#
# Future options:
# 1. Wait for Rerun SDK with better Arrow compatibility
# 2. Use Python rerun-sdk (simpler, no C++ build needed)
# 3. Build Arrow C++ successfully and retry
#
# To re-enable: Uncomment the FetchContent code below and rebuild

set(BUILD_RERUN_LOGGER FALSE)
message(STATUS "Rerun C++ logger disabled (Arrow build complexity)")
message(STATUS "C++ logger code preserved in src/rerun_logger_cpp_node.cpp for future use")
message(STATUS "Use RViz2 for visualization")

# # Rerun C++ SDK 0.30.2 - Let it build its own Arrow
# include(FetchContent)
# 
# message(STATUS "Downloading Rerun C++ SDK 0.30.2 (will build Arrow)...")
# 
# set(RERUN_DOWNLOAD_AND_BUILD_ARROW ON CACHE BOOL "Let Rerun build Arrow" FORCE)
# 
# FetchContent_Declare(
#     rerun_sdk
#     URL https://github.com/rerun-io/rerun/releases/download/0.30.2/rerun_cpp_sdk.zip
#     DOWNLOAD_EXTRACT_TIMESTAMP TRUE
# )
# FetchContent_MakeAvailable(rerun_sdk)
# 
# if(EXISTS "${rerun_sdk_SOURCE_DIR}/CMakeLists.txt")
#     set(BUILD_RERUN_LOGGER TRUE)
#     message(STATUS "Rerun C++ SDK 0.30.2 will build with bundled Arrow")
# else()
#     set(BUILD_RERUN_LOGGER FALSE)
#     message(WARNING "Failed to download Rerun C++ SDK")
# endif()

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
  curl
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

# Rerun C++ Logger Node (optional, only if Rerun SDK and Arrow are installed)
if(BUILD_RERUN_LOGGER)
    add_executable(rerun_logger_cpp_node
      src/rerun_logger_cpp_node.cpp
    )

    target_include_directories(rerun_logger_cpp_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(rerun_logger_cpp_node
      rerun_sdk
      nlohmann_json::nlohmann_json
    )

    ament_target_dependencies(rerun_logger_cpp_node
      rclcpp
      sensor_msgs
      geometry_msgs
      visualization_msgs
      std_msgs
    )
    
    install(TARGETS rerun_logger_cpp_node
      DESTINATION lib/${PROJECT_NAME})
    
    message(STATUS "Rerun C++ logger will be built")
else()
    message(STATUS "Rerun C++ logger will NOT be built (dependencies not found)")
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  orb_slam3_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts are installed via setup.py entry_points
# No manual installation needed here

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
