//  ros2 run teleop_joy teleop_joy 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>

// Configuration functions
double linear(double x) {
    return x;
}

double two_exponential(double x) {
    return (pow(2, fabs(x)) - 1) * copysign(1, x);
}

double log2(double x) {
    return (log2(fabs(x)) - 1) * copysign(1, x);
}

double x_power_two(double x) {
    return pow(x, 2) * copysign(1, x);
}

double x_power_three(double x) {
    return pow(x, 3);
}

// Configuration setup
std::map<std::string, double(*)(double)> trigger_curve_dict = {
    {"linear", linear},
    {"two_exponential", two_exponential},
    {"log2", log2},
    {"x_power_two", x_power_two},
    {"x_power_three", x_power_three}
};

// Configuration
const bool mac_keybind = true;
const double max_linear_x = 0.31; // m/s
const double max_angular_z = 2 * M_PI; // rad/s

class TeleopJoy : public rclcpp::Node {
public:
    TeleopJoy(bool mac_keybind = false)
        : Node("teleop_joy_gta_node") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TeleopJoy::joy_tf, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "max_linear_x = %f m/s", max_linear_x);
        RCLCPP_INFO(this->get_logger(), "max_angular_z = %f rad/s", max_angular_z);
    }

private:
    void joy_tf(const sensor_msgs::msg::Joy::SharedPtr data) {
        double x = data->axes[0]; // -1 to 1
        double y = data->axes[1]; // -1 to 1
        double theta = data->axes[2]; // -1 to 1

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = -x;
        twist.linear.y = y;
        twist.angular.z = theta;

        cmd_vel_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "KeyboardInterrupt: juiting.." << std::endl;
        exit(0);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopJoy>();
    signal(SIGINT, signal_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}