#include "volleyball_tracker/trajectory_predictor.hpp"

namespace volleyball_tracker {

TrajectoryPredictor::TrajectoryPredictor() {}

void TrajectoryPredictor::setCourtDimensions(double x_min, double x_max, double y_min, double y_max) {
    court_x_min_ = x_min;
    court_x_max_ = x_max;
    court_y_min_ = y_min;
    court_y_max_ = y_max;
}

void TrajectoryPredictor::setAerodynamics(double alpha, double g) {
    alpha_ = alpha;
    g_ = g;
}

TrajectoryPredictionResult TrajectoryPredictor::predictLanding(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    double target_z,
    double max_time,
    double sim_dt)
{
    TrajectoryPredictionResult res;
    res.is_valid = false;

    // Only predict when moving or above target
    if (current_pos.z() < target_z) {
        return res;
    }

    Eigen::Vector3d pos = current_pos;
    Eigen::Vector3d vel = current_vel;
    double t = 0.0;

    res.trajectory_points.push_back(pos);

    while (t < max_time) {
        double speed = vel.norm();
        Eigen::Vector3d acc = -alpha_ * speed * vel - Eigen::Vector3d(0, 0, g_);

        pos += vel * sim_dt + 0.5 * acc * sim_dt * sim_dt;
        vel += acc * sim_dt;
        t += sim_dt;

        res.trajectory_points.push_back(pos);

        if (pos.z() <= target_z) {
            // Hit ground or target interception plane
            res.landing_pos = pos;
            res.landing_vel = vel;
            res.time_to_landing = t;
            res.is_valid = true;

            // Check if inside court boundary
            res.is_in_court = (pos.x() >= court_x_min_ && pos.x() <= court_x_max_ &&
                               pos.y() >= court_y_min_ && pos.y() <= court_y_max_);
            break;
        }
    }

    return res;
}

} // namespace volleyball_tracker
