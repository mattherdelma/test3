#include "ptz/kalman_tracker.hpp"

#include <cmath>

namespace ptz {

// State index layout: x=0, y=1, vx=2, vy=3, ax=4, ay=5.
namespace {
constexpr std::size_t IX = 0, IY = 1, IVX = 2, IVY = 3, IAX = 4, IAY = 5;
}

KalmanTracker::KalmanTracker(double process_noise_psd, double measurement_noise)
    : process_noise_psd_(process_noise_psd) {
    H_ = Matrix<M, N>::Zero();
    H_(0, IX) = 1.0;
    H_(1, IY) = 1.0;
    setMeasurementNoise(measurement_noise);
    P_ = Matrix<N, N>::Identity() * 1.0e3;
}

void KalmanTracker::setMeasurementNoise(double r) {
    R_ = Matrix<M, M>::Zero();
    R_(0, 0) = r;
    R_(1, 1) = r;
}

void KalmanTracker::init(const Point2D& measurement) {
    x_ = Matrix<N, 1>::Zero();
    x_(IX, 0) = measurement.x;
    x_(IY, 0) = measurement.y;
    // Large initial uncertainty on the unobserved (velocity/accel) states.
    P_ = Matrix<N, N>::Identity() * 1.0e3;
    P_(IX, IX) = R_(0, 0);
    P_(IY, IY) = R_(1, 1);
    initialized_ = true;
}

Matrix<KalmanTracker::N, KalmanTracker::N> KalmanTracker::transition(double dt) {
    Matrix<N, N> F = Matrix<N, N>::Identity();
    const double half_dt2 = 0.5 * dt * dt;
    // x  += vx*dt + 0.5*ax*dt^2
    F(IX, IVX) = dt;
    F(IX, IAX) = half_dt2;
    // y  += vy*dt + 0.5*ay*dt^2
    F(IY, IVY) = dt;
    F(IY, IAY) = half_dt2;
    // vx += ax*dt
    F(IVX, IAX) = dt;
    // vy += ay*dt
    F(IVY, IAY) = dt;
    return F;
}

// Continuous white-noise jerk model, discretised. Each axis triplet
// (pos, vel, acc) gets the standard block:
//   [ dt^5/20  dt^4/8  dt^3/6 ]
//   [ dt^4/8   dt^3/3  dt^2/2 ] * psd
//   [ dt^3/6   dt^2/2  dt     ]
Matrix<KalmanTracker::N, KalmanTracker::N> KalmanTracker::processNoise(double dt) const {
    const double q = process_noise_psd_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double pp = dt5 / 20.0;
    const double pv = dt4 / 8.0;
    const double pa = dt3 / 6.0;
    const double vv = dt3 / 3.0;
    const double va = dt2 / 2.0;
    const double aa = dt;

    Matrix<N, N> Q = Matrix<N, N>::Zero();
    // X axis block {IX, IVX, IAX}
    Q(IX, IX) = pp;  Q(IX, IVX) = pv;  Q(IX, IAX) = pa;
    Q(IVX, IX) = pv; Q(IVX, IVX) = vv; Q(IVX, IAX) = va;
    Q(IAX, IX) = pa; Q(IAX, IVX) = va; Q(IAX, IAX) = aa;
    // Y axis block {IY, IVY, IAY}
    Q(IY, IY) = pp;  Q(IY, IVY) = pv;  Q(IY, IAY) = pa;
    Q(IVY, IY) = pv; Q(IVY, IVY) = vv; Q(IVY, IAY) = va;
    Q(IAY, IY) = pa; Q(IAY, IVY) = va; Q(IAY, IAY) = aa;

    return Q * q;
}

void KalmanTracker::predict(double dt) {
    if (!initialized_ || dt <= 0.0) return;
    const Matrix<N, N> F = transition(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + processNoise(dt);
}

void KalmanTracker::update(const Point2D& measurement) {
    if (!initialized_) {
        init(measurement);
        return;
    }
    Matrix<M, 1> z;
    z(0, 0) = measurement.x;
    z(1, 0) = measurement.y;

    const Matrix<M, 1> y = z - H_ * x_;                       // innovation
    const Matrix<M, M> S = H_ * P_ * H_.transpose() + R_;     // innovation cov
    const Matrix<N, M> K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    x_ = x_ + K * y;
    const Matrix<N, N> I = Matrix<N, N>::Identity();
    const Matrix<N, N> KH = K * H_;
    // Joseph-stabilised covariance update keeps P symmetric/PSD.
    const Matrix<N, N> ImKH = I - KH;
    P_ = ImKH * P_ * ImKH.transpose() + K * R_ * K.transpose();
}

void KalmanTracker::step(double dt, const Point2D& measurement) {
    predict(dt);
    update(measurement);
}

Point2D KalmanTracker::predictAhead(double dt) const {
    if (!initialized_) return {x_(IX, 0), x_(IY, 0)};
    const Matrix<N, N> F = transition(dt);
    const Matrix<N, 1> xp = F * x_;
    return {xp(IX, 0), xp(IY, 0)};
}

double KalmanTracker::speed() const {
    const double vx = x_(IVX, 0);
    const double vy = x_(IVY, 0);
    return std::sqrt(vx * vx + vy * vy);
}

}  // namespace ptz
