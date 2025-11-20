#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <vector>

class BlueLilyIMUNode : public rclcpp::Node
{
public:
    BlueLilyIMUNode() : Node("bluelily_imu_node")
    {
        // Declare parameters
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "bluelily_imu");
        
        // Get parameters
        port_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/bluelily/state", 10);
        
        // Open serial port
        if (!openSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "BlueLily IMU bridge started on %s", port_.c_str());
        
        // Create timer for reading serial data
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
        
        return true;
    }
    
    void readSerialData()
    {
        if (serial_fd_ < 0) return;
        
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
                
                processLine(line);
            }
        }
    }
    
    void processLine(const std::string& line)
    {
        std::istringstream iss(line);
        std::string type;
        std::getline(iss, type, ',');
        
        if (type == "IMU") {
            parseIMU(iss);
        } else if (type == "TEMP") {
            parseTemperature(iss);
        } else if (type == "STATE") {
            parseState(iss);
        } else if (type == "HEARTBEAT") {
            // Just log heartbeat
            RCLCPP_DEBUG(this->get_logger(), "Heartbeat received");
        }
    }
    
    void parseIMU(std::istringstream& iss)
    {
        // Format: IMU,timestamp,seq,ax,ay,az,gx,gy,gz
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
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;
            
            // Parse accelerometer (m/s²)
            msg.linear_acceleration.x = std::stod(tokens[2]);
            msg.linear_acceleration.y = std::stod(tokens[3]);
            msg.linear_acceleration.z = std::stod(tokens[4]);
            
            // Parse gyroscope (rad/s)
            msg.angular_velocity.x = std::stod(tokens[5]);
            msg.angular_velocity.y = std::stod(tokens[6]);
            msg.angular_velocity.z = std::stod(tokens[7]);
            
            // Set covariances (unknown, so set to -1)
            for (int i = 0; i < 9; i++) {
                msg.linear_acceleration_covariance[i] = 0.0;
                msg.angular_velocity_covariance[i] = 0.0;
                msg.orientation_covariance[i] = -1.0;  // No orientation data
            }
            
            // Set diagonal covariances (estimated)
            msg.linear_acceleration_covariance[0] = 0.01;  // x
            msg.linear_acceleration_covariance[4] = 0.01;  // y
            msg.linear_acceleration_covariance[8] = 0.01;  // z
            
            msg.angular_velocity_covariance[0] = 0.001;  // x
            msg.angular_velocity_covariance[4] = 0.001;  // y
            msg.angular_velocity_covariance[8] = 0.001;  // z
            
            imu_pub_->publish(msg);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing IMU data: %s", e.what());
        }
    }
    
    void parseTemperature(std::istringstream& iss)
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
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;
            msg.temperature = std::stod(tokens[2]);
            msg.variance = 0.1;  // Estimated
            
            temp_pub_->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing temperature: %s", e.what());
        }
    }
    
    void parseState(std::istringstream& iss)
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
