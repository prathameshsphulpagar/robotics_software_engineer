#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <chrono>

class ImuVelocityCalculator : public rclcpp::Node
{
public:
    ImuVelocityCalculator() : Node("imu_velocity_calculator")
    {
        // Subscribe to the IMU topic
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data/out", 10, std::bind(&ImuVelocityCalculator::imu_callback, this, std::placeholders::_1));

        // Publisher for velocity topic
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu_data/out/velocity", 10);

        // Initialize previous time
        prev_time_ = this->now();
        
        // Initialize velocity variables to 0
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = 0.0;
    }

private:
    // Callback function to handle incoming IMU messages
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Get current time
        rclcpp::Time current_time = this->now();

        // Calculate the time difference (delta_t) between current and previous message
        double delta_t = (current_time - prev_time_).seconds();

        // Extract linear acceleration from the IMU message
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // Update velocities using numerical integration (v = v0 + a * delta_t)
        vx_ += ax * delta_t;
        vy_ += ay * delta_t;
        vz_ += az * delta_t;

        // Log the calculated velocities
        RCLCPP_INFO(this->get_logger(), "Velocity -> X: %.4f m/s, Y: %.4f m/s, Z: %.4f m/s", vx_, vy_, vz_);

        // Create a Vector3Stamped message to publish velocity
        geometry_msgs::msg::Vector3Stamped velocity_msg;
        velocity_msg.header.stamp = current_time;
        velocity_msg.header.frame_id = "base_link";  // Frame of reference for the IMU data
        velocity_msg.vector.x = vx_;
        velocity_msg.vector.y = vy_;
        velocity_msg.vector.z = vz_;

        // Publish the velocity message
        velocity_publisher_->publish(velocity_msg);

        // Update the previous time
        prev_time_ = current_time;
    }

    // ROS2 subscription to IMU data
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    // Publisher for the calculated velocity
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher_;

    // Velocity variables
    double vx_, vy_, vz_;

    // Previous time to compute delta_t
    rclcpp::Time prev_time_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Create the node and spin
    rclcpp::spin(std::make_shared<ImuVelocityCalculator>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
