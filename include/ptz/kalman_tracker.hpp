// kalman_tracker.hpp - Constant-acceleration motion model for a tracked target.
//
// State vector (6D), in image-plane / pixel units:
//   [ x, y, vx, vy, ax, ay ]
//
// The process model (constant acceleration) and the measurement model
// (direct observation of position) are both LINEAR, so the optimal estimator
// here is a standard linear Kalman filter. The class is named "EKF-ready":
// predict()/update() are written in the generic matrix form so that a
// nonlinear measurement model (e.g. observing a target in angular gimbal
// coordinates) can be dropped in by overriding the Jacobian H and the
// measurement function h(x) without touching the rest of the pipeline.
//
// The key product for the controller is predictAhead(dt): the target's
// position projected dt seconds into the future, used to compensate the
// total system latency (capture + inference + actuation) and remove the
// phase lag of pure feedback control.
#ifndef PTZ_KALMAN_TRACKER_HPP
#define PTZ_KALMAN_TRACKER_HPP

#include "ptz/matrix.hpp"

namespace ptz {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

class KalmanTracker {
public:
    static constexpr std::size_t N = 6;  // state dimension
    static constexpr std::size_t M = 2;  // measurement dimension

    // process_noise_psd : spectral density of the jerk (acceleration noise),
    //                     larger -> filter trusts the model less, tracks
    //                     maneuvers faster but is noisier.
    // measurement_noise  : variance of the pixel position measurement.
    explicit KalmanTracker(double process_noise_psd = 50.0,
                           double measurement_noise = 4.0);

    // Seed the filter from the first detection. Velocity/acceleration start
    // at zero with a large covariance so the first few updates converge fast.
    void init(const Point2D& measurement);

    bool initialized() const { return initialized_; }

    // Advance the state estimate by dt seconds (no measurement).
    void predict(double dt);

    // Fuse a new pixel-position measurement.
    void update(const Point2D& measurement);

    // Convenience: predict(dt) then update(z). Typical per-frame call.
    void step(double dt, const Point2D& measurement);

    // Non-mutating projection of the *position* dt seconds ahead of the
    // current estimate. This is the control set-point for latency
    // compensation. Pass the total estimated loop delay as dt.
    Point2D predictAhead(double dt) const;

    Point2D position() const { return {x_(0, 0), x_(1, 0)}; }
    Point2D velocity() const { return {x_(2, 0), x_(3, 0)}; }
    Point2D acceleration() const { return {x_(4, 0), x_(5, 0)}; }
    double speed() const;  // magnitude of the velocity vector (px/s)

    void setProcessNoise(double psd) { process_noise_psd_ = psd; }
    void setMeasurementNoise(double r);

private:
    static Matrix<N, N> transition(double dt);
    Matrix<N, N> processNoise(double dt) const;

    Matrix<N, 1> x_;   // state estimate
    Matrix<N, N> P_;   // state covariance
    Matrix<M, N> H_;   // measurement matrix
    Matrix<M, M> R_;   // measurement noise covariance
    double process_noise_psd_;
    bool initialized_ = false;
};

}  // namespace ptz

#endif  // PTZ_KALMAN_TRACKER_HPP
