#include "lqr_lib.hpp"

LQR::LQR(StateMatrix const& Q, InputMatrix const& R, int horizon): Q_(Q), R_(R), horizon_(horizon) {
    A_ = StateMatrix::Zero(3, 3);
    B_ = InputMatrix::Zero(3, 2);
    P_ = Q_;
    K_ = StateMatrix::Zero(B_.cols(), A_.rows());
}

LQR::StateMatrix LQR::getA(double yaw, double v, double dt) {
    StateMatrix A(3, 3);
    A << 1, 0, -v * sin(yaw) * dt,
         0, 1,  v * cos(yaw) * dt,
         0, 0, 1;
    return A;
}

LQR::InputMatrix LQR::getB(double yaw, double dt) {
    InputMatrix B(3, 2);
    B << cos(yaw) * dt, 0,
         dt * sin(yaw), 0,
         0, dt;
    return B;
}

void LQR::updateMatrices(StateMatrix const& A, InputMatrix const& B) {
    A_ = A;
    B_ = B;
}

void LQR::computeRiccati(InputMatrix B,StateMatrix A)
{

    P_ = Q_;
    B_ = B_;
    A_ = A_;
    std::cout << "Initial P_: \n" << P_ << std::endl;
    std::cout << "Initial A_: \n" << A_ << std::endl;
    std::cout << "Initial B_: \n" << B_ << std::endl;
for (int i = horizon_; i > 0; --i) {
    Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

    P_ = Q_ + A_.transpose() * P_ * A_ - (A_.transpose() * P_ * B_) * Yinv * (B_.transpose() * P_ * A_);
}

Eigen::MatrixXd Y = R_ + B_.transpose() * P_ * B_;
Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
K_ = Yinv * B_.transpose() * P_ * A_;
    std::cout << "Computed gain matrix K: \n" << K_ << std::endl;

}
LQR::InputVector LQR::computeOptimalInput(StateVector const& state_error) {
    InputVector u = -K_ * state_error;
        std::cout << "Computed optimal input: \n" << u << std::endl;

    return u;
}
