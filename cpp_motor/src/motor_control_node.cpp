#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

#define ACCEL_LIMIT 15.0  
#define LOOP_RATE   50ms  

class RobustMotorDriver : public rclcpp::Node {
public:
    RobustMotorDriver() : Node("motor_control_node") {
        
        declare_parameter("serial_port", "/dev/ttyAMA0");
        declare_parameter("linear_scale", 190.0);
        declare_parameter("angular_scale", 21.0);
        declare_parameter("cmd_timeout_ms", 500.0);

        serial_port_    = get_parameter("serial_port").as_string();
        linear_scale_   = get_parameter("linear_scale").as_double();
        angular_scale_  = get_parameter("angular_scale").as_double();
        cmd_timeout_ms_ = get_parameter("cmd_timeout_ms").as_double();

        motor_polarities_ = {1, -1, 1, -1}; 
        last_cmd_time_ = this->now(); 

        if (!openSerial()) {
            RCLCPP_FATAL(get_logger(), "Failed to open Serial Port: %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobustMotorDriver::cmdVelCallback, this, std::placeholders::_1)
        );

        control_timer_ = create_wall_timer(
            LOOP_RATE, std::bind(&RobustMotorDriver::controlLoop, this)
        );

        RCLCPP_INFO(get_logger(), "Robust Motor Node Started on %s", serial_port_.c_str());
    }

    ~RobustMotorDriver() {
        stopAll();
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    int serial_fd_{-1};
    std::string serial_port_;
    double linear_scale_, angular_scale_, cmd_timeout_ms_;
    std::vector<int> motor_polarities_;

    double target_rpm_[4]{0.0, 0.0, 0.0, 0.0};
    double current_rpm_[4]{0.0, 0.0, 0.0, 0.0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_cmd_time_;
    std::mutex mutex_;

    bool openSerial() {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        termios tty{};
        tcgetattr(serial_fd_, &tty);
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD); 
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;              
        tty.c_cflag &= ~PARENB;          
        tty.c_cflag &= ~CSTOPB;         
        tty.c_cflag &= ~CRTSCTS;         

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        tty.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &tty);
        return true;
    }

    void sendCommand(int id, int rpm) {
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer),
            "{\"T\":10010,\"id\":%d,\"cmd\":%d,\"act\":3}\n", id, rpm);
        
        if (serial_fd_ >= 0) {
            write(serial_fd_, buffer, len);
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_time_ = this->now(); // Updates watchdog time

        double linear_rpm  = msg->linear.x  * linear_scale_;
        double angular_rpm = msg->angular.z * angular_scale_;

        double left_raw  = linear_rpm - angular_rpm;
        double right_raw = linear_rpm + angular_rpm;

        target_rpm_[0] = motor_polarities_[0] * left_raw;
        target_rpm_[1] = motor_polarities_[1] * right_raw;
        target_rpm_[2] = motor_polarities_[2] * left_raw;
        target_rpm_[3] = motor_polarities_[3] * right_raw;
    }

    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);

        auto time_diff = this->now() - last_cmd_time_;
        if (time_diff.seconds() * 1000.0 > cmd_timeout_ms_) {
            for(int i=0; i<4; i++) target_rpm_[i] = 0.0;
        }
        for (int i = 0; i < 4; ++i) {

            current_rpm_[i] = smoothStep(current_rpm_[i], target_rpm_[i]);

            sendCommand(i + 1, static_cast<int>(current_rpm_[i]));
        }
    }

    double smoothStep(double current, double target) {
        double diff = target - current;
        if (std::abs(diff) < ACCEL_LIMIT) {
            return target; 
        }
        if (target > current) return current + ACCEL_LIMIT;
        else return current - ACCEL_LIMIT;
    }

    void stopAll() {
        for (int i = 1; i <= 4; ++i) sendCommand(i, 0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobustMotorDriver>());
    rclcpp::shutdown();
    return 0;
}
