#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
int i = 0;
class Task3 : public rclcpp::Node // MODIFY NAME
{
public:
    Task3() : Node("Task3_name") // MODIFY NAME
    {
        this->declare_parameter<float>("angularz",0.2);
        this->get_parameter("angularz", angularz);


        this->declare_parameter<float>("linearx",0.2);
        this->get_parameter("linearx", linearx);

        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        turtle1_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        std::random_device rd;
        gen = std::mt19937(rd());
        dis = std::uniform_real_distribution<>(-4.0, 4.0);

        timer_ = this->create_wall_timer(500ms, std::bind(&Task3::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto direction = geometry_msgs::msg::Twist();
        direction.linear.x = (linearx + dis(gen));
        direction.angular.z = (angularz + dis(gen));
        turtle1_cmd_pub_->publish(direction);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_cmd_pub_;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
    bool moving_forward = true;
    int i = 0;
    float angularz,linearx;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Task3>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}