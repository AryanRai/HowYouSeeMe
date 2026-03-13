#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <cmath>  // For M_PI constant

class BlueLilyIMUNode : public rclcpp::Node
{
public:
    BlueLilyIMUNode() : Node("bluelily_imu_node")
    {
        // Declare parameters
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        // CHECKLIST ITEM 8: Frame ID must be 'imu_link' for ORB-SLAM3 and TF2 chain
        this->declare_parameter("frame_id", "imu_link");
        
        // Get parameters
        port_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // CHECKLIST ITEM 17: QoS must be BEST_EFFORT for 800Hz sensor data
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();  // REQUIRED: prevents queue backlog at 800Hz
        
        // CHECKLIST ITEM 1: Topic name must be exactly /imu/data for ORB-SLAM3
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/bluelily/state", 10);
        
        // Open serial port
        if (!openSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "BlueLily IMU bridge started on %s", port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to /imu/data with frame_id: %s", frame_id_.c_str());
        
        // CHECKLIST ITEM 16: No blocking sleeps - using 1ms timer for 800Hz+ capability
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&BlueLilyIMUNode::readSerialData, this));
    }
    
    ~BlueLilyIMUNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool openSerialPort()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            return false;
        }
        
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_);
            return false;
        }
        
        // Set baud rate
        speed_t speed = B115200;
        switch (baud_rate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B115200; break;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        // 8N1 mode
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
        
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_);
            return false;
        }
        
        // CHECKLIST ITEM 15: Flush serial buffer on startup to remove stale data
        tcflush(serial_fd_, TCIOFLUSH);
        
        return true;
    }
    
    void readSerialData()
    {
        if (serial_fd_ < 0) {
            // CHECKLIST ITEM 14: Attempt reconnection if serial port is closed
            if (!reconnect_attempted_) {
                RCLCPP_WARN(this->get_logger(), "Serial port closed, attempting reconnection...");
                if (openSerialPort()) {
                    RCLCPP_INFO(this->get_logger(), "Reconnected to %s", port_.c_str());
                    reconnect_attempted_ = false;
                } else {
                    reconnect_attempted_ = true;
                }
            }
            return;
        }
        
        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);
        
        if (n > 0) {
            buffer[n] = '\0';
            serial_buffer_ += std::string(buffer);
            
            // Process complete lines
            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);
                
                // Skip comments
                if (line.empty() || line[0] == '#') continue;
                
                // CHECKLIST ITEM 6: Capture timestamp immediately on receipt, before parsing
                auto stamp = this->now();
                
                // CHECKLIST ITEM 14: Error handling - catch parse errors without crashing
                try {
                    processLine(line, stamp);
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "Parse error: %s", e.what());
                }
            }
        } else if (n < 0) {
            // CHECKLIST ITEM 14: Handle serial errors with reconnection
            RCLCPP_ERROR(this->get_logger(), "Serial read error, closing port");
            close(serial_fd_);
            serial_fd_ = -1;
            reconnect_attempted_ = false;
        }
    }
    
    void processLine(const std::string& line, const rclcpp::Time& stamp)
    {
        std::istringstream iss(line);
        std::string type;
        std::getline(iss, type, ',');
        
        if (type == "IMU") {
            parseIMU(iss, stamp);
        } else if (type == "TEMP") {
            parseTemperature(iss, stamp);
        } else if (type == "STATE") {
            parseState(iss, stamp);
        } else if (type == "HEARTBEAT") {
            // Just log heartbeat
            RCLCPP_DEBUG(this->get_logger(), "Heartbeat received");
        }
    }
    
    void parseIMU(std::istringstream& iss, const rclcpp::Time& stamp)
    {
        // Format: IMU,timestamp,seq,ax,ay,az,gx,gy,gz
        // CHECKLIST ITEM 13: BlueLily firmware sends data already in m/s² and rad/s (verified in BLUELILY_ROS2_CHANGES.md)
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Invalid IMU message format");
            return;
        }
        
        try {
            auto msg = sensor_msgs::msg::Imu();
            
            // CHECKLIST ITEM 5: Use ROS2 node clock captured at receipt, not parsing time
            msg.header.stamp = stamp;
            
            // CHECKLIST ITEM 8: Frame ID must be 'imu_link' for TF2 chain and ORB-SLAM3
            msg.header.frame_id = frame_id_;  // VERIFY OK: Set to 'imu_link' in constructor
            
            // CHECKLIST ITEM 3 & 12: Parse accelerometer (already in m/s² from BlueLily firmware)
            // VERIFY OK: BlueLily ROS2Bridge.cpp sends data in m/s² (see BLUELILY_ROS2_CHANGES.md)
            msg.linear_acceleration.x = std::stod(tokens[2]);
            msg.linear_acceleration.y = std::stod(tokens[3]);
            msg.linear_acceleration.z = std::stod(tokens[4]);
            
            // CHECKLIST ITEM 3 & 11: Parse gyroscope (already in rad/s from BlueLily firmware)
            // VERIFY OK: BlueLily ROS2Bridge.cpp sends data in rad/s (see BLUELILY_ROS2_CHANGES.md)
            msg.angular_velocity.x = std::stod(tokens[5]);
            msg.angular_velocity.y = std::stod(tokens[6]);
            msg.angular_velocity.z = std::stod(tokens[7]);
            
            // CHECKLIST ITEM 4: Set covariances - must NOT be all zeros
            // Using MPU6500 typical values for ORB-SLAM3 (Kalibr will refine these in Phase 1)
            
            // Initialize all to zero first
            for (int i = 0; i < 9; i++) {
                msg.linear_acceleration_covariance[i] = 0.0;
                msg.angular_velocity_covariance[i] = 0.0;
                msg.orientation_covariance[i] = 0.0;
            }
            
            // CHECKLIST ITEM 3: Set orientation_covariance[0] = -1 (no orientation computed)
            msg.orientation_covariance[0] = -1.0;
            
            // CHECKLIST ITEM 4: Accelerometer covariance (diagonal only)
            // MPU6500 typical: 0.0001 m²/s⁴
            msg.linear_acceleration_covariance[0] = 0.0001;  // x
            msg.linear_acceleration_covariance[4] = 0.0001;  // y
            msg.linear_acceleration_covariance[8] = 0.0001;  // z
            
            // CHECKLIST ITEM 4: Gyroscope covariance (diagonal only)
            // MPU6500 typical: 0.000001 rad²/s²
            msg.angular_velocity_covariance[0] = 0.000001;  // x
            msg.angular_velocity_covariance[4] = 0.000001;  // y
            msg.angular_velocity_covariance[8] = 0.000001;  // z
            
            // CHECKLIST ITEM 2: Publishing sensor_msgs/msg/Imu (VERIFY OK)
            imu_pub_->publish(msg);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing IMU data: %s", e.what());
        }
    }
    
    void parseTemperature(std::istringstream& iss, const rclcpp::Time& stamp)
    {
        // Format: TEMP,timestamp,seq,temperature
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 3) return;
        
        try {
            auto msg = sensor_msgs::msg::Temperature();
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id_;
            msg.temperature = std::stod(tokens[2]);
            msg.variance = 0.1;  // Estimated
            
            temp_pub_->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing temperature: %s", e.what());
        }
    }
    
    void parseState(std::istringstream& iss, const rclcpp::Time& stamp)
    {
        // Format: STATE,timestamp,seq,state_name
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 3) return;
        
        auto msg = std_msgs::msg::String();
        msg.data = tokens[2];
        state_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "BlueLily state: %s", tokens[2].c_str());
    }
    
    // Member variables
    std::string port_;
    int baud_rate_;
    std::string frame_id_;
    int serial_fd_ = -1;
    std::string serial_buffer_;
    bool reconnect_attempted_ = false;  // For reconnection logic
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlueLilyIMUNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
