#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

class CVPipelineNode : public rclcpp::Node
{
public:
    CVPipelineNode() : Node("cv_pipeline_node")
    {
        // Parameters
        this->declare_parameter("rgb_topic", "/kinect2/qhd/image_color");
        this->declare_parameter("depth_topic", "/kinect2/qhd/image_depth");
        this->declare_parameter("max_fps", 5.0);
        this->declare_parameter("python_env", "/home/aryan/anaconda3/envs/howyouseeme");
        
        rgb_topic_ = this->get_parameter("rgb_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        max_fps_ = this->get_parameter("max_fps").as_double();
        python_env_ = this->get_parameter("python_env").as_string();
        
        processing_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / max_fps_));
        
        RCLCPP_INFO(this->get_logger(), "CV Pipeline Node initialized");
        RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max FPS: %.1f", max_fps_);
        
        // Synchronized subscribers for RGB and depth
        rgb_sub_.subscribe(this, rgb_topic_);
        depth_sub_.subscribe(this, depth_topic_);
        
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&CVPipelineNode::imageCallback, this, 
                                         std::placeholders::_1, std::placeholders::_2));
        
        // Publishers
        result_pub_ = this->create_publisher<std_msgs::msg::String>("/cv_pipeline/results", 10);
        viz_pub_ = image_transport::create_publisher(this, "/cv_pipeline/visualization");
        
        // Model request subscriber
        model_request_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cv_pipeline/model_request", 10,
            std::bind(&CVPipelineNode::modelRequestCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Ready for model requests on /cv_pipeline/model_request");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_processing_time_ < processing_interval_) {
            return;
        }
        last_processing_time_ = now;
        
        try {
            RCLCPP_INFO_ONCE(this->get_logger(), "✅ Image callback working - receiving images!");
            
            cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            latest_rgb_ = rgb_ptr->image.clone();
            latest_depth_ = depth_ptr->image.clone();
            latest_rgb_stamp_ = rgb_msg->header.stamp;
            
            // Process any pending requests
            processPendingRequests();
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    void modelRequestCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Model request: %s", msg->data.c_str());
        
        // Parse simple format: "model_name:param1=value1,param2=value2"
        std::string request = msg->data;
        size_t colon_pos = request.find(':');
        
        std::string model_name = (colon_pos != std::string::npos) ? 
                                 request.substr(0, colon_pos) : request;
        std::string params = (colon_pos != std::string::npos) ? 
                            request.substr(colon_pos + 1) : "";
        
        pending_requests_.push_back({model_name, params});
    }
    
    void processPendingRequests()
    {
        if (pending_requests_.empty()) {
            return;
        }
        
        if (latest_rgb_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Pending requests but no images yet. Waiting for Kinect...");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Processing %zu pending request(s)", pending_requests_.size());
        
        for (const auto& request : pending_requests_) {
            processModelRequest(request.model, request.params);
        }
        
        pending_requests_.clear();
    }
    
    void processModelRequest(const std::string& model_name, const std::string& params)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "Processing with model: %s", model_name.c_str());
        
        // Save images to temp files for Python worker
        std::string temp_dir = "/tmp/cv_pipeline";
        system(("mkdir -p " + temp_dir).c_str());
        
        std::string rgb_path = temp_dir + "/rgb_latest.jpg";
        std::string depth_path = temp_dir + "/depth_latest.png";
        
        cv::imwrite(rgb_path, latest_rgb_);
        cv::imwrite(depth_path, latest_depth_);
        
        // Call Python worker
        std::string python_script = getPythonScriptPath(model_name);
        std::string command = python_env_ + "/bin/python " + python_script + 
                            " --rgb " + rgb_path + 
                            " --depth " + depth_path +
                            " --params \"" + params + "\"" +
                            " --model-size tiny" +  // Use tiny model for 4GB GPU
                            " 2>&1";
        
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());
        
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute model");
            return;
        }
        
        // Read output
        char buffer[256];
        std::string result;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(this->get_logger(), "Model %s completed in %ld ms", 
                   model_name.c_str(), duration.count());
        
        // Publish result
        auto result_msg = std_msgs::msg::String();
        result_msg.data = result;
        result_pub_->publish(result_msg);
        
        // Publish visualization
        publishVisualization(model_name, result);
    }
    
    void publishVisualization(const std::string& model_name, const std::string& result)
    {
        if (latest_rgb_.empty()) return;
        
        cv::Mat viz = latest_rgb_.clone();
        
        // Add overlay
        cv::putText(viz, "Model: " + model_name, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        
        // Add timestamp
        cv::putText(viz, "Processing complete", cv::Point(10, viz.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        auto viz_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz).toImageMsg();
        viz_msg->header.stamp = latest_rgb_stamp_;
        viz_pub_.publish(viz_msg);
    }
    
    std::string getPythonScriptPath(const std::string& model_name)
    {
        std::string base_path = "/home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/src/cv_pipeline/python/";
        return base_path + model_name + "_worker.py";
    }
    
    struct ModelRequest {
        std::string model;
        std::string params;
    };
    
    // Members
    std::string rgb_topic_;
    std::string depth_topic_;
    double max_fps_;
    std::string python_env_;
    std::chrono::milliseconds processing_interval_;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<Synchronizer> sync_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    image_transport::Publisher viz_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_request_sub_;
    
    cv::Mat latest_rgb_;
    cv::Mat latest_depth_;
    rclcpp::Time latest_rgb_stamp_;
    std::chrono::steady_clock::time_point last_processing_time_;
    
    std::vector<ModelRequest> pending_requests_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CVPipelineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
