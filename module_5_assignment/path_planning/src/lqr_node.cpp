#include <angles/angles.h>
#include <cmath>
#include <lqr_node.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>

input input_old = input(0.0);

LqrNode::LqrNode()
    : Node("lqr_node"), dt_(0.03), tolerance(0.8), end_controller(false),
      max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false) {

    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));
    control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
        std::bind(&LqrNode::controlLoopCallback, this));

    Q_ << 0.8, 0.0, 0.0, 0.0, 0.0, 0.8;
    R_ << 0.8, 0.0, 0.8;
    lqr_ = std::make_unique<LQR<Q_, R_, 100>>();

    waypoints_ = {State(0, 0, M_PI / 4), State(1, 1, M_PI / 2),
                  State(1, 2, M_PI), State(0, 3, 3 * M_PI / 2),
                  State(-1, 4, M_PI), State(-2, 3, -M_PI / 2),
                  State(-3, 2, M_PI), State(-3, 1, M_PI / 2)};
    actual_state_ = State(0, 0, 0);

    optimiseHeading(waypoints_);
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

}

void LqrNode::publishVelocity(double v,double w){

}

void LqrNode::optimiseHeading(std::vector<State> &waypoints){

}

void LqrNode::controlLoopCallback() {

}

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    auto controller = std::make_shared<LqrNode>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}