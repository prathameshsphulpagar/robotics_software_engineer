#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
int i = 0;
class Task3 : public rclcpp::Node // MODIFY NAME
{
public:
    Task3() : Node("Task3_name") // MODIFY NAME
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        turtle1_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&Task3::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto direction = geometry_msgs::msg::Twist();
        if (i % 20 == 0)
        {
            moving_forward = !moving_forward; // Toggle the direction
        }

        if (moving_forward)
        {
            direction.linear.x = 0.5; // Move forward
        }
        else
        {
            direction.linear.x = -0.5; // Move backward
        }

        turtle1_cmd_pub_->publish(direction);
        i++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_cmd_pub_;
    bool moving_forward = true; 
    int i = 0;            
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Task3>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}