#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DriveTurtleBot : public rclcpp::Node // MODIFY NAME
{
public:
    DriveTurtleBot() : Node("drive_node") // MODIFY NAME
    {
        this->declare_parameter<int>("given_radius", 3);
        this->get_parameter("given_radius", given_radius);

        drive_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&DriveTurtleBot::timer_callback, this));
    }
    void timer_callback()
    {
        auto direction = geometry_msgs::msg::Twist();

        direction.linear.x = a * exp(1 * 1);
        direction.angular.z = 5.0;
        drive_publisher_->publish(direction);
    RCLCPP_INFO(this->get_logger(), "Driving Robot... Current given_radius: %d", given_radius);
        if (a <= given_radius)
        {
            a = a + 0.5;
        }
    }

private:
    int given_radius;
    double a = 0;
    double time_elapsed_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveTurtleBot>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}