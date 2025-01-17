#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <algorithm>

// State of the mobile robot
struct State {
  double x;
  double y;
  double theta;

  State() {}
  State(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

// Input of the mobile robot
struct Input {
  double v; // linear velocity
  double w; // angular velocity

  Input() {}
  Input(double v_, double w_) : v(v_), w(w_) {}
};

class MobileRobotController : public rclcpp::Node {
  public:
    MobileRobotController() : Node("mobile_robot_controller") {
      // Subscribe to robot pose topic
      robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MobileRobotController::robotPoseCallback, this, std::placeholders::_1));

      // Publish control input topic
      control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      // Initialize state model matrix A
      A_ << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

      // Initialize state cost matrix Q
      Q_ << 0.05, 0, 0,
            0, 0.03, 0,
            0, 0, 0.04;
            
      // Initialize input cost matrix R
      R_ << 0.01, 0,
            0, 0.01;

      // Set desired state (x, y, theta)
      desired_state_ = {3.0, 1.0, M_PI/2};

      // Set control loop time step
      dt_ = 0.03;

      // Set goal tolerance
      tolerance = 0.3;

      // Initialize control loop flag
      end_controller = false;

      // Set maximum linear and angular velocities
      max_linear_velocity = 2.0;
      max_angular_velocity = M_PI/2;

      // Start control loop timer
      control_loop_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_), std::bind(&MobileRobotController::controlLoopCallback, this));
    }

  private:
    // Callback to get current state of robot pose
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Extract the orientation quaternion from the message
      tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      // Convert the quaternion to roll, pitch, and yaw angles
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);

      // Update the actual state of the robot with the current position and yaw angle
      actual_state_ = State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    }

    // Function to compute the input model matrix B based on the current yaw angle and time step
    Eigen::Matrix<double, 3, 2> getB(double yaw, double dt)
    {
      Eigen::Matrix<double, 3, 2> B; // Input model
      B << std::cos(yaw)*dt, 0,
           std::sin(yaw)*dt, 0,
           0, dt;

      return B;
    }

    // Function to publish velocity commands
    void pubVel(double v, double w)
    {
      geometry_msgs::msg::Twist vel;
      vel.linear.x = v;
      vel.angular.z = w;
      control_input_pub_->publish(vel);
    }

    // LQR control algorithm to compute optimal control inputs
    Input LQR(Eigen::Vector3d state_error, Eigen::Matrix3d Q, Eigen::Matrix2d R, Eigen::Matrix3d A, Eigen::Matrix<double, 3, 2> B)
    {     
      uint8_t N = 50; // number of iterations
      std::vector<Eigen::MatrixXd> P(N+1);
      Eigen::MatrixXd Qf = Q;
      P[N] = Qf;

      // Backward iteration to compute P matrices
      for (uint8_t i = N; i >= 1; --i) {
        auto Y = R + B.transpose() * P[i] * B;
        // Compute the SVD decomposition of Y
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Compute the pseudo-inverse of Y
        Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

        P[i-1] = Q + A.transpose() * P[i] * A - (A.transpose() * P[i] * B) * Yinv * (B.transpose() * P[i] * A);
      }

      std::vector<Eigen::MatrixXd> K(N);
      std::vector<Eigen::Vector2d> u(N);
      
      // Forward iteration to compute K matrices and control inputs
      for (uint8_t i = 0; i <= N-1; ++i){
        auto Y = R + B.transpose() * P[i+1] * B;
        // Compute the SVD decomposition of Y
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Compute the pseudo-inverse of Y
        Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

        K[i] = Yinv * B.transpose() * P[i+1] * A;
        u[i] = -K[i] * state_error;
      }

      // Return the optimal control input
      Input u_optimal = {u[N-1](0), u[N-1](1)};
      return u_optimal;
    }

    // Callback for control loop timer
    void controlLoopCallback()
    {
      if (end_controller == false) { // if robot has not reached goal
        // Compute the input model matrix B
        Eigen::Matrix<double, 3, 2> B = getB(actual_state_.theta, dt_);
        // Get the actual and desired states as Eigen vectors
        Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y, actual_state_.theta);
        Eigen::Vector3d x_desired(desired_state_.x, desired_state_.y, desired_state_.theta);
        // Compute the state error
        state_error_ = x_actual - x_desired;

        // Compute the optimal control input using LQR
        Input u = LQR(state_error_, Q_, R_, A_, B);
        // Clamp the control inputs to the maximum velocities
        u.v = std::clamp(u.v, -max_linear_velocity, max_linear_velocity);
        u.w = std::clamp(u.w, -max_angular_velocity, max_angular_velocity);

        // Publish the control inputs
        pubVel(u.v, u.w);

        // Check if the state error is within the tolerance
        double state_error_magnitude = state_error_.norm();
        if (state_error_magnitude < tolerance) {
          // Stop the robot and end the controller
          pubVel(0, 0);
          end_controller = true;
        }
      }
      else {
        // Log that the goal has been reached and cancel the control loop timer
        RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
        control_loop_timer_->cancel();
        return;
      }
    }

    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_input_pub_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // LQR Matrices
    Eigen::Matrix3d A_; // State model
    Eigen::Matrix3d Q_; // State cost
    Eigen::Matrix2d R_; // Input cost
    Eigen::Vector3d state_error_;

    double dt_; // sample time
    double tolerance; // goal tolerance
    bool end_controller;
    double max_linear_velocity;
    double max_angular_velocity;

    // Current robot state and desired state
    State actual_state_;
    State desired_state_;
};

int main(int argc, char **argv) {
  try
  {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create and spin the controller node
    auto controller = std::make_shared<MobileRobotController>();
    rclcpp::spin(controller);
    // Shutdown ROS 2
    rclcpp::shutdown();
  }
  catch(const std::exception& e)
  {
    // Log any exceptions
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mobile_robot_controller"), "Error: " << e.what());
  }

  return 0;
}