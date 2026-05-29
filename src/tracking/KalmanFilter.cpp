#include "KalmanFilter.h"

namespace rtp {

KalmanFilter::KalmanFilter()
    : std_weight_position_(1.f / 20.f), std_weight_velocity_(1.f / 160.f) {
    motion_mat_ = Eigen::Matrix<float, 8, 8>::Identity();
    // Constant-velocity: position_{t+1} = position_t + velocity_t.
    for (int i = 0; i < 4; ++i) motion_mat_(i, i + 4) = 1.f;
    update_mat_ = Eigen::Matrix<float, 4, 8>::Zero();
    for (int i = 0; i < 4; ++i) update_mat_(i, i) = 1.f;
}

KalmanFilter::StateDist KalmanFilter::initiate(const Measure& z) const {
    StateMean mean = StateMean::Zero();
    mean.leftCols<4>() = z;                 // velocities start at zero

    const float h = z(0, 3);
    Eigen::Matrix<float, 1, 8> std;
    std << 2 * std_weight_position_ * h,
           2 * std_weight_position_ * h,
           1e-2f,
           2 * std_weight_position_ * h,
           10 * std_weight_velocity_ * h,
           10 * std_weight_velocity_ * h,
           1e-5f,
           10 * std_weight_velocity_ * h;

    StateCov cov = std.array().square().matrix().asDiagonal();
    return {mean, cov};
}

void KalmanFilter::predict(StateMean& mean, StateCov& cov) const {
    const float h = mean(0, 3);
    Eigen::Matrix<float, 1, 8> std;
    std << std_weight_position_ * h,
           std_weight_position_ * h,
           1e-2f,
           std_weight_position_ * h,
           std_weight_velocity_ * h,
           std_weight_velocity_ * h,
           1e-5f,
           std_weight_velocity_ * h;
    const StateCov Q = std.array().square().matrix().asDiagonal();

    mean = mean * motion_mat_.transpose();
    cov  = motion_mat_ * cov * motion_mat_.transpose() + Q;
}

std::pair<KalmanFilter::Measure, KalmanFilter::MeasureCov>
KalmanFilter::project(const StateMean& mean, const StateCov& cov) const {
    const float h = mean(0, 3);
    Eigen::Matrix<float, 1, 4> std;
    std << std_weight_position_ * h,
           std_weight_position_ * h,
           1e-1f,
           std_weight_position_ * h;
    const MeasureCov R = std.array().square().matrix().asDiagonal();

    const Measure    proj_mean = mean * update_mat_.transpose();
    const MeasureCov proj_cov  =
        update_mat_ * cov * update_mat_.transpose() + R;
    return {proj_mean, proj_cov};
}

KalmanFilter::StateDist KalmanFilter::update(const StateMean& mean,
                                             const StateCov& cov,
                                             const Measure& z) const {
    auto [proj_mean, proj_cov] = project(mean, cov);

    // Kalman gain via the (4x4) projected covariance: K = P Hᵀ S⁻¹.
    const Eigen::Matrix<float, 8, 4> PHt = cov * update_mat_.transpose();
    const Eigen::Matrix<float, 8, 4> K   = PHt * proj_cov.inverse();

    const Measure   innovation = z - proj_mean;
    const StateMean new_mean   = mean + (innovation * K.transpose());
    const StateCov  new_cov    = cov - K * proj_cov * K.transpose();
    return {new_mean, new_cov};
}

} // namespace rtp
