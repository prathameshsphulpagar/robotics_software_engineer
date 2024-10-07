#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>

class WallFollowingNode : public rclcpp::Node // MODIFY NAME
{
public:
    WallFollowingNode() : Node("wall_following_node") // MODIFY NAME
    {
        // Subscribe to the Lidar topic
        Subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&WallFollowingNode::wall_callback, this, std::placeholders::_1));
            // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }

private:

    void wall_callback(const sensor_msgs::msg::LaserScan msg)
    {

    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Subscription_;

    // Publisher for the velocity command
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowingNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}