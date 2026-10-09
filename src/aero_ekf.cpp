#include "volleyball_tracker/aero_ekf.hpp"
#include <cmath>

namespace volleyball_tracker {

AeroEKF::AeroEKF() {
    // Measurement matrix: measures [x, y, z]
    H_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    setNoiseParams(0.01, 0.5, 0.05);
    setAerodynamicParams(0.27, 0.105, 0.25, 1.225);
}

void AeroEKF::setAerodynamicParams(double mass_kg, double radius_m, double drag_coeff, double air_density) {
    double area = M_PI * radius_m * radius_m;
    alpha_ = (air_density * drag_coeff * area) / (2.0 * mass_kg);
}

void AeroEKF::setNoiseParams(double q_pos, double q_vel, double r_pos) {
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (q_pos * q_pos);
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q_vel * q_vel);

    R_ = Eigen::Matrix3d::Identity() * (r_pos * r_pos);
}

void AeroEKF::initialize(const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_vel) {
    state_.head<3>() = init_pos;
    state_.tail<3>() = init_vel;

    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 0.1;
    P_.block<3, 3>(3, 3) *= 10.0;

    is_initialized_ = true;
}

void AeroEKF::reset() {
    is_initialized_ = false;
    state_.setZero();
    P_.setIdentity();
}

Eigen::Matrix<double, 6, 1> AeroEKF::stateDynamics(const Eigen::Matrix<double, 6, 1>& x) const {
    Eigen::Matrix<double, 6, 1> x_dot;
    Eigen::Vector3d v = x.tail<3>();
    double speed = v.norm();

    x_dot.head<3>() = v;
    x_dot(3) = -alpha_ * speed * v.x();
    x_dot(4) = -alpha_ * speed * v.y();
    x_dot(5) = -alpha_ * speed * v.z() - g_;

    return x_dot;
}

Eigen::Matrix<double, 6, 6> AeroEKF::computeJacobian(const Eigen::Matrix<double, 6, 1>& x) const {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
    // d(pos_dot)/d(vel) = I
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d v = x.tail<3>();
    double speed = std::max(v.norm(), 1e-3);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double delta = (i == j) ? 1.0 : 0.0;
            F(3 + i, 3 + j) = -alpha_ * ((v(i) * v(j)) / speed + speed * delta);
        }
    }
    return F;
}

void AeroEKF::predict(double dt) {
    if (!is_initialized_ || dt <= 0.0) return;

    // Runge-Kutta 4th Order numerical integration
    Eigen::Matrix<double, 6, 1> k1 = stateDynamics(state_);
    Eigen::Matrix<double, 6, 1> k2 = stateDynamics(state_ + 0.5 * dt * k1);
    Eigen::Matrix<double, 6, 1> k3 = stateDynamics(state_ + 0.5 * dt * k2);
    Eigen::Matrix<double, 6, 1> k4 = stateDynamics(state_ + dt * k3);

    state_ += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // Covariance propagation: Phi = I + F*dt
    Eigen::Matrix<double, 6, 6> F = computeJacobian(state_);
    Eigen::Matrix<double, 6, 6> Phi = Eigen::Matrix<double, 6, 6>::Identity() + F * dt;

    P_ = Phi * P_ * Phi.transpose() + Q_ * dt;
}

bool AeroEKF::update(const Eigen::Vector3d& measurement, double gate_threshold) {
    if (!is_initialized_) {
        initialize(measurement);
        return true;
    }

    // Innovation
    Eigen::Vector3d y = measurement - H_ * state_;
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;

    // Chi-Square / Mahalanobis distance gating
    double d2 = y.transpose() * S.inverse() * y;
    if (d2 > (gate_threshold * gate_threshold)) {
        return false; // Outlier rejected
    }

    // Kalman gain
    Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    // State & covariance update
    state_ += K * y;
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    P_ = (I - K * H_) * P_;

    return true;
}

} // namespace volleyball_tracker
