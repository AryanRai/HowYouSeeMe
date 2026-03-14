/**
 * Rerun C++ Logger Node for HowYouSeeMe
 * Logs all ROS2 topics to Rerun for visualization
 * No Python/NumPy dependency issues!
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rerun.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class RerunLoggerNode : public rclcpp::Node
{
public:
    RerunLoggerNode() 
        : Node("rerun_logger_cpp"),
          rec_("howyouseeme")  // Initialize with default name, will be updated
    {
        // Parameters
        this->declare_parameter("recording_name", "howyouseeme");
        this->declare_parameter("save_path", "/tmp/howyouseeme.rrd");
        this->declare_parameter("rgb_downsample", 2);
        this->declare_parameter("pc_max_points", 50000);
        
        std::string name = this->get_parameter("recording_name").as_string();
        std::string save_path = this->get_parameter("save_path").as_string();
        rgb_downsample_ = this->get_parameter("rgb_downsample").as_int();
        pc_max_points_ = this->get_parameter("pc_max_points").as_int();
        
        // Spawn and save Rerun recording
        rec_.spawn().exit_on_failure();
        rec_.save(save_path).exit_on_failure();
        
        RCLCPP_INFO(this->get_logger(), "Rerun C++ logger initialized: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording to: %s", save_path.c_str());
        
        // QoS for best effort
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        // Subscriptions
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/kinect2/hd/image_color", qos,
            std::bind(&RerunLoggerNode::rgb_callback, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/kinect2/hd/image_depth_rect", qos,
            std::bind(&RerunLoggerNode::depth_callback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose", qos,
            std::bind(&RerunLoggerNode::pose_callback, this, std::placeholders::_1));
        
        tsdf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/tsdf/pointcloud", qos,
            std::bind(&RerunLoggerNode::tsdf_callback, this, std::placeholders::_1));
        
        world_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/semantic/world_state", qos,
            std::bind(&RerunLoggerNode::world_state_callback, this, std::placeholders::_1));
        
        markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/semantic/markers", qos,
            std::bind(&RerunLoggerNode::markers_callback, this, std::placeholders::_1));
        
        // Status timer
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RerunLoggerNode::log_status, this));
        
        RCLCPP_INFO(this->get_logger(), "Rerun C++ logger ready");
    }

private:
    void set_time(const std_msgs::msg::Header& header)
    {
        double timestamp_sec = header.stamp.sec + header.stamp.nanosec * 1e-9;
        rec_.set_time_seconds("ros_time", timestamp_sec);
    }
    
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        msg_counts_["rgb"]++;
        try {
            set_time(msg->header);
            
            // Direct conversion without cv_bridge (rgb8 format)
            if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
                RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                return;
            }
            
            // Downsample if needed (simple skip-pixel downsampling)
            if (rgb_downsample_ > 1) {
                size_t new_width = msg->width / rgb_downsample_;
                size_t new_height = msg->height / rgb_downsample_;
                std::vector<uint8_t> downsampled(new_width * new_height * 3);
                
                for (size_t y = 0; y < new_height; y++) {
                    for (size_t x = 0; x < new_width; x++) {
                        size_t src_idx = ((y * rgb_downsample_) * msg->width + (x * rgb_downsample_)) * 3;
                        size_t dst_idx = (y * new_width + x) * 3;
                        downsampled[dst_idx] = msg->data[src_idx];
                        downsampled[dst_idx + 1] = msg->data[src_idx + 1];
                        downsampled[dst_idx + 2] = msg->data[src_idx + 2];
                    }
                }
                
                rec_.log("camera/rgb", rerun::Image::from_rgb24(
                    downsampled,
                    {new_width, new_height}
                ));
            } else {
                rec_.log("camera/rgb", rerun::Image::from_rgb24(
                    msg->data,
                    {static_cast<size_t>(msg->width), static_cast<size_t>(msg->height)}
                ));
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "RGB log error: %s", e.what());
        }
    }
    
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        msg_counts_["depth"]++;
        try {
            set_time(msg->header);
            
            // Depth is 16UC1 format - direct access without cv_bridge
            if (rgb_downsample_ > 1) {
                // Downsample depth image
                int new_width = msg->width / rgb_downsample_;
                int new_height = msg->height / rgb_downsample_;
                std::vector<uint16_t> downsampled(new_width * new_height);
                
                for (int y = 0; y < new_height; y++) {
                    for (int x = 0; x < new_width; x++) {
                        int src_x = x * rgb_downsample_;
                        int src_y = y * rgb_downsample_;
                        int src_idx = src_y * msg->width + src_x;
                        downsampled[y * new_width + x] = 
                            reinterpret_cast<const uint16_t*>(msg->data.data())[src_idx];
                    }
                }
                
                rec_.log("camera/depth", rerun::DepthImage(
                    downsampled.data(),
                    {static_cast<size_t>(new_width), static_cast<size_t>(new_height)}
                ).with_meter(1000.0f));
            } else {
                rec_.log("camera/depth", rerun::DepthImage(
                    reinterpret_cast<const uint16_t*>(msg->data.data()),
                    {static_cast<size_t>(msg->width), static_cast<size_t>(msg->height)}
                ).with_meter(1000.0f));
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Depth log error: %s", e.what());
        }
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        msg_counts_["pose"]++;
        try {
            set_time(msg->header);
            
            auto& p = msg->pose.position;
            auto& q = msg->pose.orientation;
            
            // Log transform
            rec_.log("robot/pose", rerun::Transform3D(
                rerun::Vec3D{p.x, p.y, p.z},
                rerun::Quaternion::from_xyzw(q.x, q.y, q.z, q.w)
            ));
            
            // Log trajectory point
            rec_.log("robot/trajectory", rerun::Points3D({{p.x, p.y, p.z}})
                .with_colors({{0, 200, 100}})
                .with_radii({0.03f})
            );
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Pose log error: %s", e.what());
        }
    }
    
    void tsdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        msg_counts_["tsdf"]++;
        try {
            set_time(msg->header);
            
            // Extract points
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            
            std::vector<rerun::Position3D> points;
            points.reserve(std::min(msg->width * msg->height, 
                static_cast<uint32_t>(pc_max_points_)));
            
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                    points.push_back(rerun::Position3D{*iter_x, *iter_y, *iter_z});
                    if (points.size() >= pc_max_points_) break;
                }
            }
            
            if (!points.empty()) {
                rec_.log("map/tsdf", rerun::Points3D(points)
                    .with_colors({{180, 180, 220}})
                    .with_radii({0.015f})
                );
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "TSDF log error: %s", e.what());
        }
    }
    
    void world_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        msg_counts_["world"]++;
        try {
            auto data = json::parse(msg->data);
            
            std::vector<rerun::Position3D> points;
            std::vector<rerun::Color> colors;
            std::vector<std::string> labels;
            
            // Log robot position
            if (data.contains("robot") && data["robot"].is_object()) {
                auto robot = data["robot"];
                if (robot.contains("position") && robot["position"].is_array() && 
                    robot["position"].size() >= 3) {
                    auto pos = robot["position"];
                    rec_.log("world/robot", rerun::Points3D({{
                        pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>()
                    }})
                        .with_colors({{0, 255, 0}})
                        .with_radii({0.1f})
                        .with_labels({"Robot"})
                    );
                }
            }
            
            // Log objects
            if (data.contains("objects") && data["objects"].is_object()) {
                for (auto& [obj_id, obj] : data["objects"].items()) {
                    if (!obj.is_object()) continue;
                    if (!obj.contains("position") || !obj["position"].is_array() ||
                        obj["position"].size() < 3) continue;
                    
                    auto pos = obj["position"];
                    points.push_back(rerun::Position3D{
                        pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>()
                    });
                    
                    std::string label = obj.value("label", "?");
                    float conf = obj.value("confidence", 0.0f);
                    int count = obj.value("count", 1);
                    labels.push_back(label + " (" + std::to_string(conf).substr(0, 4) + 
                        ") x" + std::to_string(count));
                    
                    // Color by source
                    std::string source = obj.value("source", "");
                    if (source.find("face") != std::string::npos || 
                        source.find("emotion") != std::string::npos) {
                        colors.push_back(rerun::Color(50, 200, 255));
                    } else if (source.find("yolo") != std::string::npos) {
                        colors.push_back(rerun::Color(255, 220, 50));
                    } else {
                        colors.push_back(rerun::Color(200, 200, 200));
                    }
                }
            }
            
            // Log people
            if (data.contains("people") && data["people"].is_object()) {
                for (auto& [person_id, person] : data["people"].items()) {
                    if (!person.is_object()) continue;
                    if (!person.contains("position") || !person["position"].is_array() ||
                        person["position"].size() < 3) continue;
                    
                    auto pos = person["position"];
                    points.push_back(rerun::Position3D{
                        pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>()
                    });
                    
                    std::string label = person.value("label", "person");
                    float conf = person.value("confidence", 0.0f);
                    labels.push_back(label + " (" + std::to_string(conf).substr(0, 4) + ")");
                    colors.push_back(rerun::Color(255, 100, 100));
                }
            }
            
            if (!points.empty()) {
                rec_.log("world/objects", rerun::Points3D(points)
                    .with_colors(colors)
                    .with_labels(labels)
                    .with_radii({0.06f})
                );
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "World state log error: %s", e.what());
        }
    }
    
    void markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        msg_counts_["markers"]++;
        try {
            std::vector<rerun::Position3D> points;
            std::vector<std::string> labels;
            std::vector<rerun::Color> colors;
            
            for (const auto& marker : msg->markers) {
                if (marker.type != visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
                    continue;
                }
                
                auto& p = marker.pose.position;
                points.push_back(rerun::Position3D{p.x, p.y, p.z});
                labels.push_back(marker.text);
                colors.push_back(rerun::Color(
                    static_cast<uint8_t>(marker.color.r * 255),
                    static_cast<uint8_t>(marker.color.g * 255),
                    static_cast<uint8_t>(marker.color.b * 255)
                ));
            }
            
            if (!points.empty()) {
                rec_.log("world/markers", rerun::Points3D(points)
                    .with_labels(labels)
                    .with_colors(colors)
                    .with_radii({0.05f})
                );
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Markers log error: %s", e.what());
        }
    }
    
    void log_status()
    {
        RCLCPP_INFO(this->get_logger(),
            "Messages - RGB: %d, Depth: %d, Pose: %d, TSDF: %d, World: %d, Markers: %d",
            msg_counts_["rgb"], msg_counts_["depth"], msg_counts_["pose"],
            msg_counts_["tsdf"], msg_counts_["world"], msg_counts_["markers"]);
    }
    
    rerun::RecordingStream rec_;
    int rgb_downsample_;
    int pc_max_points_;
    
    std::map<std::string, int> msg_counts_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr world_state_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RerunLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
