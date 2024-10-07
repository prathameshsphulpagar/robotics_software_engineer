#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>

float right_distance_wall_following = 3.00, threshold_value = 0.2;
double Kp, Ki, Kd;
float previous_error, integral, dt;

class WallFollowingNode : public rclcpp::Node // MODIFY NAME
{
public:
    WallFollowingNode() : Node("wall_following_node") // MODIFY NAME
    {
        // Subscribe to the Lidar topic
        LidarSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/scan/distance", 10, std::bind(&WallFollowingNode::wall_callback, this, std::placeholders::_1));
        ImuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_data/out", 10, std::bind(&WallFollowingNode::imu_callback, this, std::placeholders::_1));
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        Kp = 0.06;
        Ki = 0.0;
        Kd = 0.6;
        previous_error = 0.0;
        integral = 0.0;
        dt = 0.7; // Adjust as necessary
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu msg)
    {
        imu_x = msg.orientation.x;
        imu_y = msg.orientation.y;
        imu_z = msg.orientation.z;
        imu_w = msg.orientation.w;
    }
    void wall_callback(const geometry_msgs::msg::Twist msg)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        // Calculate error
        double error = right_distance_wall_following - msg.linear.z;

        // Calculate PID terms
        double P = Kp * error;
        integral += Ki * error * dt;
        double D1 = ((error - previous_error) / dt);
        double D = Kd * D1;

        // PID output
        double pid_output = P + integral + D;
        // Restrict to max/min
        if (pid_output > 1)
            pid_output = 1;
        else if (pid_output < -1)
            pid_output = -1;

        // Use IMU data to adjust the PID output (e.g., add some correction based on yaw)
        double yaw_correction = -0.1 * imu_z; // Example correction
        cmd_msg.angular.z = pid_output;       //+ yaw_correction;

        // Set linear speed
        cmd_msg.linear.x = 0.4; // Adjust as necessary

        // Publish the command
        velocity_publisher_->publish(cmd_msg);

        // Update previous error
        previous_error = error;
    }
    // veriable declare
    float imu_x, imu_y, imu_z, imu_w;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr LidarSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ImuSubscription_;

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