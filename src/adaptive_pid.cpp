#include "ptz/adaptive_pid.hpp"

#include <algorithm>
#include <cmath>

namespace ptz {

namespace {
double clampMag(double v, double limit) {
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}
}  // namespace

AdaptivePID::AdaptivePID(const PidConfig& cfg) : cfg_(cfg) {}

void AdaptivePID::reset() {
    ax_ = AxisState{};
    ay_ = AxisState{};
    integral_reset_ = false;
}

double AdaptivePID::computeAxis(AxisState& s, double error, double dt, double kp,
                                double ki, double kd, bool clear_integral) {
    if (clear_integral) s.integral = 0.0;

    // Derivative on error, low-pass filtered.
    double raw_deriv = 0.0;
    if (s.has_prev && dt > 0.0) {
        raw_deriv = (error - s.prev_error) / dt;
    }
    const double a = std::clamp(cfg_.derivative_lpf, 0.0, 1.0);
    s.deriv_filt = a * s.deriv_filt + (1.0 - a) * raw_deriv;

    // Integrate then clamp (anti-windup).
    s.integral += error * dt;
    s.integral = clampMag(s.integral, cfg_.integral_limit);

    const double out = kp * error + ki * s.integral + kd * s.deriv_filt;

    s.prev_error = error;
    s.has_prev = true;
    return out;
}

Point2D AdaptivePID::compute(const Point2D& error, double dt, double target_speed) {
    integral_reset_ = false;

    const double err_mag = std::sqrt(error.x * error.x + error.y * error.y);

    // --- Error dead-zone: suppress micro-jitter near centre. ---
    if (err_mag <= cfg_.deadzone) {
        // Decay the integral gently so it does not linger while idling.
        ax_.integral *= 0.9;
        ay_.integral *= 0.9;
        ax_.prev_error = error.x;
        ay_.prev_error = error.y;
        ax_.has_prev = ay_.has_prev = true;
        return {0.0, 0.0};
    }

    // --- Detect a sudden direction reversal via the error "jerk". ---
    bool clear_integral = false;
    if (dt > 0.0 && ax_.has_prev && ay_.has_prev) {
        const double rate_x = (error.x - ax_.prev_error) / dt;
        const double rate_y = (error.y - ay_.prev_error) / dt;
        const double jerk_x = (rate_x - ax_.prev_rate) / dt;
        const double jerk_y = (rate_y - ay_.prev_rate) / dt;
        const double jerk_mag = std::sqrt(jerk_x * jerk_x + jerk_y * jerk_y);
        if (jerk_mag > cfg_.reset_jerk_threshold) {
            clear_integral = true;
            integral_reset_ = true;
        }
        ax_.prev_rate = rate_x;
        ay_.prev_rate = rate_y;
    }

    // --- Gain scheduling. ---
    // Speed boost: faster target -> higher Kp/Kd for snappier tracking.
    const double speed_factor =
        1.0 + cfg_.speed_boost * std::min(target_speed / cfg_.speed_ref, 1.0);

    // Near-zone damping: small error -> ramp Kp/Kd down to near_scale.
    double near_factor = 1.0;
    if (err_mag < cfg_.near_zone) {
        const double t = err_mag / cfg_.near_zone;  // 0 at centre, 1 at edge
        near_factor = cfg_.near_scale + (1.0 - cfg_.near_scale) * t;
    }

    const double kp = cfg_.kp * speed_factor * near_factor;
    const double kd = cfg_.kd * speed_factor * near_factor;
    const double ki = cfg_.ki;  // integral gain left unscheduled

    Point2D out;
    out.x = computeAxis(ax_, error.x, dt, kp, ki, kd, clear_integral);
    out.y = computeAxis(ay_, error.y, dt, kp, ki, kd, clear_integral);

    // --- Output saturation. ---
    out.x = clampMag(out.x, cfg_.output_limit);
    out.y = clampMag(out.y, cfg_.output_limit);
    return out;
}

}  // namespace ptz
