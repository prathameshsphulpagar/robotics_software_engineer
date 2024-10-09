#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>

int rightFlag = 1, leftFlag = 0;
float wallFollowingDistance = 2.00, frontObstacleThreshold_ = (wallFollowingDistance * 1.50);
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

        // timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(10), // Call every 100ms (10 Hz)
        // std::bind(&WallFollowingNode::PID_wallFollowing, this));
        Kp = 0.3;
        Ki = 0.0;
        Kd = 3.0;
        previous_error = 0.0;
        integral = 0.0;
        dt = 1; // Adjust as necessary
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
        wallDistance.linear.x = msg.linear.x;
        wallDistance.linear.y = msg.linear.y;
        wallDistance.linear.z = msg.linear.z;

        wallDistance.angular.x = msg.angular.x;
        wallDistance.angular.y = msg.angular.y;
        wallDistance.angular.z = msg.angular.z;
        PID_wallFollowing();
    }

    void PID_wallFollowing()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        // if ((wallDistance.linear.x < frontObstacleThreshold_) && (wallDistance.linear.x > (frontObstacleThreshold_ * 1.30)))
        // {
        //     // Obstacle in front, decide to move left or right
        //     if (wallDistance.linear.y < wallDistance.linear.z)
        //     {
        //         // Move to the right if left obstacle is closer
        //         cmd_msg.angular.z = 1.0; // Adjust value as necessary for turning right
        //     }
        //     else
        //     {
        //         // Move to the left if right obstacle is closer
        //         cmd_msg.angular.z = -1.0; // Adjust value as necessary for turning left
        //     }

        //     // Set linear speed for evasive maneuver
        //     cmd_msg.linear.x = 0.1; // Slow down when avoiding obstacle

        //     // Publish the evasive command
        //     // velocity_publisher_->publish(cmd_msg);
        //     return; // Skip the rest of the code
        // }
        if (rightFlag == 1 && leftFlag == 0)
        {
            if (wallDistance.linear.z < (wallFollowingDistance + movementThreshold_) && wallDistance.linear.z > (wallFollowingDistance - movementThreshold_))
            {

                // Calculate error
                double error = wallFollowingDistance - wallDistance.linear.z;

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
                cmd_msg.linear.x = 0.5; // Adjust as necessary

                // Publish the command
                // velocity_publisher_->publish(cmd_msg);

                // Update previous error
                previous_error = error;
                if ((wallDistance.linear.x < (frontObstacleThreshold_ * 1.50)) && (wallDistance.linear.x > 0))
                {
                    // Obstacle in front, decide to move left or right
                    if ((wallDistance.linear.x < (frontObstacleThreshold_)) && (wallDistance.linear.x > 0))
                    {
                        if (wallDistance.linear.y < wallDistance.linear.z)
                        {
                            // Move to the right if left obstacle is closer
                            cmd_msg.angular.z = 1.0; // Adjust value as necessary for turning right
                        }
                        else
                        {
                            // Move to the left if right obstacle is closer
                            cmd_msg.angular.z = -1.0; // Adjust value as necessary for turning left
                        }
                    }
                    // Set linear speed for evasive maneuver
                    cmd_msg.linear.x = 0.1; // Slow down when avoiding obstacle
                }
            }
            else
            {
                cmd_msg.linear.x = 0;
                cmd_msg.linear.y = 0;
                cmd_msg.linear.z = 0;

                // velocity_publisher_->publish(cmd_msg);
            }
        }
        else if (leftFlag == 1 && rightFlag == 0)
        {
            if (wallDistance.linear.y < (wallFollowingDistance + movementThreshold_) && wallDistance.linear.y > (wallFollowingDistance - movementThreshold_))
            {

                // Calculate error
                double error = wallFollowingDistance - wallDistance.linear.y;

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
                // double yaw_correction = -0.1 * imu_z; // Example correction
                cmd_msg.angular.z = -pid_output; //+ yaw_correction;

                // Set linear speed
                cmd_msg.linear.x = 0.5;

                // Publish the command
                // velocity_publisher_->publish(cmd_msg);

                // Update previous error
                previous_error = error;
                if ((wallDistance.linear.x < (frontObstacleThreshold_ * 1.50)) && (wallDistance.linear.x > 0))
                {
                    // Obstacle in front, decide to move left or right
                    if ((wallDistance.linear.x < (frontObstacleThreshold_)) && (wallDistance.linear.x > 0))
                    {
                        if (wallDistance.linear.y < wallDistance.linear.z)
                        {
                            // Move to the right if left obstacle is closer
                            cmd_msg.angular.z = 1.0; // Adjust value as necessary for turning right
                        }
                        else
                        {
                            // Move to the left if right obstacle is closer
                            cmd_msg.angular.z = -1.0; // Adjust value as necessary for turning left
                        }
                    }
                    // Set linear speed for evasive maneuver
                    cmd_msg.linear.x = 0.1; // Slow down when avoiding obstacle
                }
            }
            else
            {
                cmd_msg.linear.x = 0;
                cmd_msg.linear.y = 0;
                cmd_msg.linear.z = 0;

                // velocity_publisher_->publish(cmd_msg);
            }
        }
        else
        {
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = 0;
            cmd_msg.linear.z = 0;

            // velocity_publisher_->publish(cmd_msg);
        }
        velocity_publisher_->publish(cmd_msg);
    }

    float movementThreshold_ = 2.00;

    // veriable declare
    float imu_x, imu_y, imu_z, imu_w;
    geometry_msgs::msg::Twist wallDistance; // Declare wallDistance with its explicit type
    rclcpp::TimerBase::SharedPtr timer_;

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