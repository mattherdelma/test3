#pragma once
//
// Constant-velocity Kalman filter in image space, matching the formulation used
// by SORT / ByteTrack.
//
// State  x = [cx, cy, a, h, vx, vy, va, vh]   (8-dim)
//   cx,cy : box center      a : aspect ratio (w/h)     h : height
//   v*    : respective time derivatives
// Measurement z = [cx, cy, a, h]   (4-dim)
//
// Process / measurement noise scale with the object height, so large/near
// objects are allowed to move faster — the standard ByteTrack heuristic.
//
#include <Eigen/Dense>
#include <utility>

namespace rtp {

class KalmanFilter {
public:
    using StateMean  = Eigen::Matrix<float, 1, 8>;
    using StateCov   = Eigen::Matrix<float, 8, 8>;
    using Measure    = Eigen::Matrix<float, 1, 4>;
    using MeasureCov = Eigen::Matrix<float, 4, 4>;
    using StateDist  = std::pair<StateMean, StateCov>;

    KalmanFilter();

    // Create a track from an initial measurement [cx, cy, a, h].
    StateDist initiate(const Measure& measurement) const;

    // Advance the state one step (mutates mean/cov in place).
    void predict(StateMean& mean, StateCov& covariance) const;

    // Project the state distribution into measurement space.
    std::pair<Measure, MeasureCov> project(const StateMean& mean,
                                           const StateCov& covariance) const;

    // Correct the state with a new measurement.
    StateDist update(const StateMean& mean, const StateCov& covariance,
                     const Measure& measurement) const;

private:
    Eigen::Matrix<float, 8, 8> motion_mat_;   // F
    Eigen::Matrix<float, 4, 8> update_mat_;    // H
    float std_weight_position_;
    float std_weight_velocity_;
};

} // namespace rtp
