#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("lidar_sensing_node") // MODIFY NAME
    {
        Subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
            "/scan",10,
            std::bind(&MyCustomNode::lidarCallback,this,std::placeholders::_1));
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg) const {
        float front_segment = *std::min_element(lidar_msg->ranges.begin() + 170 , lidar_msg->ranges.begin() + 190); // 340-360
        float right_segment = *std::min_element(lidar_msg->ranges.begin() + 80 , lidar_msg->ranges.begin() + 100); // 80-100
        float left_segment = *std::min_element(lidar_msg->ranges.begin() + 260 , lidar_msg->ranges.begin() + 280); // 260-280

        RCLCPP_INFO(this->get_logger(), "Front: '%f' Right: '%f' Left '%f'",
        front_segment,
        right_segment,
        left_segment
        );
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}