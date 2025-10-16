/**
 * Simple Kinect v2 ROS2 Publisher
 * Uses libfreenect2 directly (like Protonect) to publish ROS2 topics
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>

class Kinect2SimplePublisher : public rclcpp::Node
{
public:
  Kinect2SimplePublisher() : Node("kinect2_simple_publisher")
  {
    // Declare parameters
    this->declare_parameter("base_name", "kinect2");
    this->declare_parameter("fps_limit", 30.0);
    this->declare_parameter("use_opengl", true);
    
    std::string base_name = this->get_parameter("base_name").as_string();
    double fps_limit = this->get_parameter("fps_limit").as_double();
    bool use_opengl = this->get_parameter("use_opengl").as_bool();
    
    // Create publishers for SD resolution (512x424) - most reliable
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + base_name + "/sd/image_color", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + base_name + "/sd/image_depth", 10);
    ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + base_name + "/sd/image_ir", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/" + base_name + "/sd/camera_info", 10);
    
    RCLCPP_INFO(this->get_logger(), "Initializing Kinect v2...");
    
    // Initialize libfreenect2
    if (!initKinect(use_opengl)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Kinect");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Kinect initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Publishing to /%s/sd/* topics", base_name.c_str());
    
    // Create timer for publishing frames
    int timer_ms = static_cast<int>(1000.0 / fps_limit);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_ms),
      std::bind(&Kinect2SimplePublisher::publishFrames, this));
    
    frame_count_ = 0;
  }
  
  ~Kinect2SimplePublisher()
  {
    if (device_ && device_) {
      device_->stop();
      device_->close();
    }
    RCLCPP_INFO(this->get_logger(), "Kinect stopped");
  }

private:
  bool initKinect(bool use_opengl)
  {
    try {
      // Set logger level
      libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
      
      // Enumerate devices
      freenect2_ = std::make_unique<libfreenect2::Freenect2>();
      int num_devices = freenect2_->enumerateDevices();
      
      if (num_devices == 0) {
        RCLCPP_ERROR(this->get_logger(), "No Kinect devices found");
        return false;
      }
      
      std::string serial = freenect2_->getDefaultDeviceSerialNumber();
      RCLCPP_INFO(this->get_logger(), "Found Kinect with serial: %s", serial.c_str());
      
      // Create pipeline
      libfreenect2::PacketPipeline *pipeline = nullptr;
      
      if (use_opengl) {
        RCLCPP_INFO(this->get_logger(), "Trying OpenGL pipeline...");
        pipeline = new libfreenect2::OpenGLPacketPipeline();
      } else {
        RCLCPP_INFO(this->get_logger(), "Using CPU pipeline...");
        pipeline = new libfreenect2::CpuPacketPipeline();
      }
      
      // Open device
      device_ = freenect2_->openDevice(serial, pipeline);
      
      if (!device_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device");
        return false;
      }
      
      // Create frame listener
      int types = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
      listener_ = std::make_unique<libfreenect2::SyncMultiFrameListener>(types);
      
      device_->setColorFrameListener(listener_.get());
      device_->setIrAndDepthFrameListener(listener_.get());
      
      // Start device
      if (!device_->start()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Kinect device");
        return false;
      }
      
      RCLCPP_INFO(this->get_logger(), "Device serial: %s", device_->getSerialNumber().c_str());
      RCLCPP_INFO(this->get_logger(), "Device firmware: %s", device_->getFirmwareVersion().c_str());
      
      return true;
      
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during Kinect initialization: %s", e.what());
      return false;
    }
  }
  
  void publishFrames()
  {
    if (!device_ || !device_) {
      return;
    }
    
    try {
      libfreenect2::FrameMap frames;
      
      // Wait for new frame with timeout
      if (!listener_->waitForNewFrame(frames, 1000)) {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for frames");
        return;
      }
      
      // Get frames
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      
      // Create timestamp
      auto now = this->now();
      
      // Publish RGB (convert BGRX to RGB)
      if (rgb) {
        cv::Mat rgb_mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        cv::Mat rgb_converted;
        cv::cvtColor(rgb_mat, rgb_converted, cv::COLOR_BGRA2RGB);
        
        auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", rgb_converted).toImageMsg();
        rgb_msg->header.stamp = now;
        rgb_msg->header.frame_id = "kinect2_rgb_optical_frame";
        rgb_pub_->publish(*rgb_msg);
      }
      
      // Publish IR
      if (ir) {
        cv::Mat ir_mat(ir->height, ir->width, CV_32FC1, ir->data);
        auto ir_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", ir_mat).toImageMsg();
        ir_msg->header.stamp = now;
        ir_msg->header.frame_id = "kinect2_ir_optical_frame";
        ir_pub_->publish(*ir_msg);
      }
      
      // Publish Depth
      if (depth) {
        cv::Mat depth_mat(depth->height, depth->width, CV_32FC1, depth->data);
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_mat).toImageMsg();
        depth_msg->header.stamp = now;
        depth_msg->header.frame_id = "kinect2_ir_optical_frame";
        depth_pub_->publish(*depth_msg);
      }
      
      // Publish camera info
      auto camera_info = sensor_msgs::msg::CameraInfo();
      camera_info.header.stamp = now;
      camera_info.header.frame_id = "kinect2_ir_optical_frame";
      camera_info.height = 424;
      camera_info.width = 512;
      camera_info_pub_->publish(camera_info);
      
      // Release frames
      listener_->release(frames);
      
      frame_count_++;
      if (frame_count_ % 150 == 0) {
        RCLCPP_INFO(this->get_logger(), "Published %d frames", frame_count_);
      }
      
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing frames: %s", e.what());
    }
  }
  
  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Kinect objects
  std::unique_ptr<libfreenect2::Freenect2> freenect2_;
  libfreenect2::Freenect2Device *device_ = nullptr;
  std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener_;
  
  // Stats
  int frame_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Kinect2SimplePublisher>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
