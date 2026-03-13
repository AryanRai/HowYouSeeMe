// ORB-SLAM3 ROS2 Node for Kinect v2 RGB-D
// RGB-D only mode (no IMU) for stable tracking without calibration

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

#include "System.h"

class OrbSlam3Node : public rclcpp::Node
{
public:
    OrbSlam3Node() : Node("orb_slam3_node")
    {
        // Declare parameters
        this->declare_parameter("voc_file", "");
        this->declare_parameter("settings_file", "");
        
        std::string voc_file = this->get_parameter("voc_file").as_string();
        std::string settings_file = this->get_parameter("settings_file").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", voc_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings file: %s", settings_file.c_str());
        
        // Initialize ORB-SLAM3 system in RGB-D mode (no IMU)
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            voc_file, settings_file, ORB_SLAM3::System::RGBD, true);
        
        // Subscribers for RGB-D
        rgb_sub_.subscribe(this, "/kinect2/hd/image_color");
        depth_sub_.subscribe(this, "/kinect2/hd/image_depth_rect");
        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        // Synchronizer for RGB-D
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(
            std::bind(&OrbSlam3Node::rgbdCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGB-D node initialized (no IMU)");
    }
    
    ~OrbSlam3Node()
    {
        if (slam_system_) {
            slam_system_->Shutdown();
        }
    }

private:
    void rgbdCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        // Convert ROS images to OpenCV
        cv_bridge::CvImageConstPtr cv_rgb, cv_depth;
        try {
            cv_rgb = cv_bridge::toCvShare(rgb_msg, "bgr8");
            cv_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;
        
        
        // Track frame (RGB-D only, no IMU)
        Sophus::SE3f Tcw = slam_system_->TrackRGBD(
            cv_rgb->image, cv_depth->image, timestamp);
        
        // Publish pose
        Eigen::Vector3f t = Tcw.translation();
        if (t.norm() > 0.001) {  // Check if pose is valid
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = rgb_msg->header.stamp;
            pose_msg.header.frame_id = "map";
            
            // Convert SE3 to ROS Pose (invert to get camera pose in world frame)
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Matrix3f R = Twc.rotationMatrix();
            Eigen::Vector3f t_world = Twc.translation();
            
            pose_msg.pose.position.x = t_world(0);
            pose_msg.pose.position.y = t_world(1);
            pose_msg.pose.position.z = t_world(2);
            
            Eigen::Quaternionf q(R);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            
            pose_pub_->publish(pose_msg);
        }
    }
    
    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbSlam3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
