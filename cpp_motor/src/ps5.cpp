#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"       
#include "geometry_msgs/msg/twist.hpp"   
#include <vector>
#include <string>

#define AXIS_LEFT_STICK_X 0
#define AXIS_L2_TRIGGER 5
#define AXIS_R2_TRIGGER 2
#define BUTTON_L1 4
#define BUTTON_R1 5

class Ps5TeleopNode : public rclcpp::Node
{
public:
    Ps5TeleopNode() : Node("ps5_teleop_node")
    {
        this->declare_parameter<double>("max_linear_speed", 1.0);  
        this->declare_parameter<double>("max_angular_speed", 1.0); 
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Ps5TeleopNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PS5 C++ Teleop Node Started.");
        RCLCPP_INFO(this->get_logger(), "Controls: R2=Fwd, L2=Rev, LeftStick=Turn");
        RCLCPP_INFO(this->get_logger(), "L1/R1 = Adjust Max Speed. Current: %.1f m/s", max_linear_speed_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Joy message has too few axes or buttons. Waiting...");
            return;
        }

        if (prev_buttons_.empty())
        {
            prev_buttons_.resize(msg->buttons.size(), 0);
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        double r2_pressure = (msg->axes[AXIS_R2_TRIGGER] + 1.0) / 2.0;
        double l2_pressure = (msg->axes[AXIS_L2_TRIGGER] + 1.0) / 2.0;

        // Combine them: R2 positive, L2 negative
        twist_msg.linear.x = (r2_pressure - l2_pressure) * max_linear_speed_;

        double left_stick_x = msg->axes[AXIS_LEFT_STICK_X];
        if (std::abs(left_stick_x) > 0.1) 
        {
            twist_msg.angular.z = left_stick_x * max_angular_speed_;
        }
        else
        {
            twist_msg.angular.z = 0.0;
        }

        if (msg->buttons[BUTTON_R1] == 1 && prev_buttons_[BUTTON_R1] == 0)
        {
            max_linear_speed_ = std::min(1.0, max_linear_speed_ + 0.1); // Cap at 1.0 m/s
            RCLCPP_INFO(this->get_logger(), "Max linear speed INCREASED to: %.1f m/s", max_linear_speed_);
        }
        if (msg->buttons[BUTTON_L1] == 1 && prev_buttons_[BUTTON_L1] == 0)
        {
            max_linear_speed_ = std::max(0.1, max_linear_speed_ - 0.1); // Floor at 0.1 m/s
            RCLCPP_INFO(this->get_logger(), "Max linear speed DECREASED to: %.1f m/s", max_linear_speed_);
        }
        cmd_vel_pub_->publish(twist_msg);

        prev_buttons_ = msg->buttons;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    double max_linear_speed_;
    double max_angular_speed_;
    std::vector<int> prev_buttons_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ps5TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
