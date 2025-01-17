#include "lqr_node.hpp"

LqrNode::LqrNode() : Node("lqr_node"), end_controller(false), odom_received_(false), current_waypoint(0) {
    // Initialize parameters
    dt_ = 0.1;
    tolerance = 0.1;
    max_linear_velocity = 0.5;
    max_angular_velocity = 1.0;

    // Initialize LQR cost matrices
    Q_ = Eigen::Matrix3d::Identity();
    R_ = Eigen::Matrix2d::Identity();

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
    // Update the actual state with the received odometry data
    actual_state_.x = msg->pose.pose.position.x;
    actual_state_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, actual_state_.theta);

    odom_received_ = true;

    // Log the received odometry data
    RCLCPP_INFO(this->get_logger(), "Odometry received: x=%.2f, y=%.2f, theta=%.2f", actual_state_.x, actual_state_.y, actual_state_.theta);
}

void LqrNode::controlLoopCallback() {
    if (!odom_received_ || end_controller) {
        RCLCPP_WARN(this->get_logger(), "Odometry not received or controller ended.");
        return;
    }

    if (current_waypoint >= static_cast<int>(waypoints_.size())) {
        end_controller = true;
        publishVelocity(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Reached final waypoint.");
        return;
    }

    State goal_state = waypoints_[current_waypoint];
    state_error_ << goal_state.x - actual_state_.x, goal_state.y - actual_state_.y, goal_state.theta - actual_state_.theta;

    RCLCPP_INFO(this->get_logger(), "Current waypoint: %d, Goal state: x=%.2f, y=%.2f, theta=%.2f", current_waypoint, goal_state.x, goal_state.y, goal_state.theta);
    RCLCPP_INFO(this->get_logger(), "State error: x=%.2f, y=%.2f, theta=%.2f", state_error_(0), state_error_(1), state_error_(2));

    if (state_error_.norm() < tolerance) {
        current_waypoint++;
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %d, moving to next waypoint.", current_waypoint);
        return;
    }

    double yaw = actual_state_.theta;
    double v = control_input_.v;
    lqr_->updateMatrices(lqr_->getA(yaw, v, dt_), lqr_->getB(yaw, dt_));
    Eigen::Vector2d optimal_input = lqr_->computeOptimalInput(state_error_);

    double v_cmd = std::clamp(-max_linear_velocity,optimal_input(0),  max_linear_velocity);
    double w_cmd = std::clamp(-max_angular_velocity,optimal_input(1),  max_angular_velocity);

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
    for (auto& waypoint : waypoints) {
        double dx = waypoint.x - actual_state_.x;
        double dy = waypoint.y - actual_state_.y;
        waypoint.theta = std::atan2(dy, dx);
    }
}

void LqrNode::angleNormalisation(double& angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LqrNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}