#pragma once

#include <vector>
#include <Eigen/Dense>

namespace volleyball_tracker {

struct TrajectoryPredictionResult {
    Eigen::Vector3d landing_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d landing_vel{Eigen::Vector3d::Zero()};
    double time_to_landing{0.0};
    bool is_in_court{false};
    bool is_valid{false};
    std::vector<Eigen::Vector3d> trajectory_points;
};

class TrajectoryPredictor {
public:
    TrajectoryPredictor();
    ~TrajectoryPredictor() = default;

    void setCourtDimensions(double x_min, double x_max, double y_min, double y_max);
    void setAerodynamics(double alpha, double g = 9.80665);

    TrajectoryPredictionResult predictLanding(const Eigen::Vector3d& current_pos,
                                              const Eigen::Vector3d& current_vel,
                                              double target_z = 0.0,
                                              double max_time = 5.0,
                                              double sim_dt = 0.01);

private:
    double alpha_{0.0028};
    double g_{9.80665};
    double court_x_min_{-4.5};
    double court_x_max_{4.5};
    double court_y_min_{0.0};
    double court_y_max_{18.0};
};

} // namespace volleyball_tracker
