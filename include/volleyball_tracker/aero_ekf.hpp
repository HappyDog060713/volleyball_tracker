#pragma once

#include <Eigen/Dense>

namespace volleyball_tracker {

class AeroEKF {
public:
    AeroEKF();
    ~AeroEKF() = default;

    void initialize(const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_vel = Eigen::Vector3d::Zero());
    void predict(double dt);
    bool update(const Eigen::Vector3d& measurement, double gate_threshold = 4.0);

    Eigen::Vector3d getPosition() const { return state_.head<3>(); }
    Eigen::Vector3d getVelocity() const { return state_.tail<3>(); }
    const Eigen::Matrix<double, 6, 1>& getState() const { return state_; }
    const Eigen::Matrix<double, 6, 6>& getCovariance() const { return P_; }

    void setAerodynamicParams(double mass_kg, double radius_m, double drag_coeff, double air_density);
    void setNoiseParams(double q_pos, double q_vel, double r_pos);

    bool isInitialized() const { return is_initialized_; }
    void reset();

private:
    Eigen::Matrix<double, 6, 1> stateDynamics(const Eigen::Matrix<double, 6, 1>& x) const;
    Eigen::Matrix<double, 6, 6> computeJacobian(const Eigen::Matrix<double, 6, 1>& x) const;

    Eigen::Matrix<double, 6, 1> state_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 6> P_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> Q_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 3, 3> R_{Eigen::Matrix<double, 3, 3>::Identity()};
    Eigen::Matrix<double, 3, 6> H_{Eigen::Matrix<double, 3, 6>::Zero()};

    double alpha_{0.0028}; // (rho * Cd * A) / (2 * m)
    double g_{9.80665};    // gravity (m/s^2)
    bool is_initialized_{false};
};

} // namespace volleyball_tracker
