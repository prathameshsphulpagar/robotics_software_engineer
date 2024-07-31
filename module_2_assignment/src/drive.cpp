#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DriveTurtleBot : public rclcpp::Node // MODIFY NAME
{
public:
    DriveTurtleBot() : Node("drive_node") // MODIFY NAME
    {
        drive_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&DriveTurtleBot::timer_callback, this));
    }
    void timer_callback()
    {
        auto direction = geometry_msgs::msg::Twist();

        direction.linear.x = 2;
        direction.angular.z = 0.6;
        RCLCPP_INFO(this->get_logger(),"Driving Robot...");
        drive_publisher_->publish(direction);
    }

private:
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