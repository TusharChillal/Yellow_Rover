/**
 * ROBUST MOTOR CONTROLLER - JSON Protocol + Ramping
 */

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

// Tuning Parameters
#define ACCEL_LIMIT 15.0  // RPM change per loop (Higher = Snappier, Lower = Smoother)
#define LOOP_RATE   50ms  // 20Hz Control Loop

class RobustMotorDriver : public rclcpp::Node {
public:
    RobustMotorDriver() : Node("motor_control_node") {
        
        // --- 1. CONFIGURATION ---
        declare_parameter("serial_port", "/dev/ttyAMA0");
        declare_parameter("linear_scale", 190.0);
        declare_parameter("angular_scale", 21.0);
        declare_parameter("cmd_timeout_ms", 500.0);

        serial_port_    = get_parameter("serial_port").as_string();
        linear_scale_   = get_parameter("linear_scale").as_double();
        angular_scale_  = get_parameter("angular_scale").as_double();
        cmd_timeout_ms_ = get_parameter("cmd_timeout_ms").as_double();

        // Polarities based on your reference: Left (1,3) are +, Right (2,4) are -
        motor_polarities_ = {1, -1, 1, -1}; 

        // --- 2. TIME FIX (Initialize Clock) ---
        last_cmd_time_ = this->now(); 

        // --- 3. SERIAL SETUP ---
        if (!openSerial()) {
            RCLCPP_FATAL(get_logger(), "Failed to open Serial Port: %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        // --- 4. ROS INTERFACE ---
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobustMotorDriver::cmdVelCallback, this, std::placeholders::_1)
        );

        // Control Loop (Handles Ramping & Sending)
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
    // --- VARIABLES ---
    int serial_fd_{-1};
    std::string serial_port_;
    double linear_scale_, angular_scale_, cmd_timeout_ms_;
    std::vector<int> motor_polarities_;

    // RPM State
    double target_rpm_[4]{0.0, 0.0, 0.0, 0.0};
    double current_rpm_[4]{0.0, 0.0, 0.0, 0.0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_cmd_time_;
    std::mutex mutex_;

    // --- SERIAL ---
    bool openSerial() {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        termios tty{};
        tcgetattr(serial_fd_, &tty);
        
        // Set Baud Rate to 115200
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD); // Enable Receiver
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;              // 8-bit characters
        tty.c_cflag &= ~PARENB;          // No parity
        tty.c_cflag &= ~CSTOPB;          // 1 Stop bit
        tty.c_cflag &= ~CRTSCTS;         // No Hardware flow control

        // Raw mode (disable software flow control/formatting)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        tty.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &tty);
        return true;
    }

    void sendCommand(int id, int rpm) {
        char buffer[64];
        // Your JSON Protocol
        int len = snprintf(buffer, sizeof(buffer),
            "{\"T\":10010,\"id\":%d,\"cmd\":%d,\"act\":3}\n", id, rpm);
        
        if (serial_fd_ >= 0) {
            write(serial_fd_, buffer, len);
        }
    }

    // --- CALLBACKS ---
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_time_ = this->now(); // Updates watchdog time

        double linear_rpm  = msg->linear.x  * linear_scale_;
        double angular_rpm = msg->angular.z * angular_scale_;

        // Differential Drive Math
        double left_raw  = linear_rpm - angular_rpm;
        double right_raw = linear_rpm + angular_rpm;

        // Apply Polarity and Store Targets
        // Mapping: 0->ID1, 1->ID2, 2->ID3, 3->ID4
        target_rpm_[0] = motor_polarities_[0] * left_raw;
        target_rpm_[1] = motor_polarities_[1] * right_raw;
        target_rpm_[2] = motor_polarities_[2] * left_raw;
        target_rpm_[3] = motor_polarities_[3] * right_raw;
    }

    // --- CONTROL LOOP (20Hz) ---
    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);

        // 1. WATCHDOG CHECK
        auto time_diff = this->now() - last_cmd_time_;
        if (time_diff.seconds() * 1000.0 > cmd_timeout_ms_) {
            // Stop if timeout
            for(int i=0; i<4; i++) target_rpm_[i] = 0.0;
        }

        // 2. RAMPING & SENDING
        for (int i = 0; i < 4; ++i) {
            // Smoothly move current RPM towards target RPM
            current_rpm_[i] = smoothStep(current_rpm_[i], target_rpm_[i]);
            
            // Send command (Motor IDs are 1-based: i+1)
            sendCommand(i + 1, static_cast<int>(current_rpm_[i]));
        }
    }

    double smoothStep(double current, double target) {
        double diff = target - current;
        if (std::abs(diff) < ACCEL_LIMIT) {
            return target; // Close enough, jump to target
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