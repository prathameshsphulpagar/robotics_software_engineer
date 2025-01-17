#include "lqr_node.hpp"
#include <angles/angles.h> // Include the angles library

LqrNode::LqrNode() : Node("lqr_node"), end_controller(false), odom_received_(false), current_waypoint(0) {
    // Initialize parameters
    dt_ = 0.1;
    tolerance = 0.1;
    max_linear_velocity = 0.5;
    max_angular_velocity = 1.0;

    // Initialize LQR cost matrices
    Q_ = Eigen::Matrix3d::Identity() * 0.8;
    R_ = Eigen::Matrix2d::Identity() * 0.8;

    // Initialize LQR controller
    int horizon = 10;
    lqr_ = std::make_unique<LQR>(Q_, R_, horizon);

    // Initialize waypoints
    waypoints_ = {
        State(1.0, 1.0, 0.0),
        State(2.0, 2.0, 0.0),
        State(3.0, 3.0, 0.0)
    };

    // Log the initialized waypoints
    RCLCPP_INFO(this->get_logger(), "Waypoints initialized.");

    // Initialize ROS subscribers and publishers
    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));
    control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)), std::bind(&LqrNode::controlLoopCallback, this));
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    actual_state_ = State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    odom_received_ = true;
}

void LqrNode::controlLoopCallback() {
    if (!odom_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry message...");
        return;
    }
    if (end_controller) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        control_loop_timer_->cancel();
        return;
    }

    if (current_waypoint >= static_cast<int>(waypoints_.size())) {
        end_controller = true;
        publishVelocity(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Reached final waypoint.");
        return;
    }

    State goal_state = waypoints_[current_waypoint];
    state_error_ << goal_state.x - actual_state_.x, goal_state.y - actual_state_.y, angles::normalize_angle(goal_state.theta - actual_state_.theta);

    RCLCPP_INFO(this->get_logger(), "Current waypoint: %d, Goal state: x=%.2f, y=%.2f, theta=%.2f", current_waypoint, goal_state.x, goal_state.y, goal_state.theta);
    RCLCPP_INFO(this->get_logger(), "State error: x=%.2f, y=%.2f, theta=%.2f", state_error_(0), state_error_(1), state_error_(2));

    if (state_error_.head<2>().norm() < tolerance) {
        current_waypoint++;
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %d, moving to next waypoint.", current_waypoint);
        return;
    }

    double yaw = actual_state_.theta;
    double v = control_input_.v;
    Eigen::Matrix3d A = lqr_->getA(yaw, v, dt_);
    Eigen::MatrixXd B = lqr_->getB(yaw, dt_);
    lqr_->updateMatrices(A, B);

    RCLCPP_INFO(this->get_logger(), "Calling computeRiccati with A and B matrices.");
    lqr_->computeRiccati(B, A);

    Eigen::Vector2d optimal_input = lqr_->computeOptimalInput(state_error_);

    // Adjust the direction of movement if the robot is moving backward
    if (optimal_input(0) < 0) {
        optimal_input(0) = -optimal_input(0);
        optimal_input(1) = angles::normalize_angle(optimal_input(1) + M_PI);
    }

    double v_cmd = std::clamp(optimal_input(0), 0.0, max_linear_velocity); // Ensure non-negative linear velocity
    double w_cmd = std::clamp(optimal_input(1), -max_angular_velocity, max_angular_velocity);

    RCLCPP_INFO(this->get_logger(), "Publishing velocity: v=%.2f, w=%.2f", v_cmd, w_cmd);
    publishVelocity(v_cmd, w_cmd);
}


void LqrNode::publishVelocity(double v, double w) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = v;
    msg.angular.z = w;
    control_input_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Velocity command published: linear=%.2f, angular=%.2f", v, w);
}

void LqrNode::optimiseHeading(std::vector<State>& waypoints) {
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        double dx = waypoints[i + 1].x - waypoints[i].x;
        double dy = waypoints[i + 1].y - waypoints[i].y;
        waypoints[i].theta = std::atan2(dy, dx);
    }
    waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
}

void LqrNode::angleNormalisation(double& angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LqrNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}