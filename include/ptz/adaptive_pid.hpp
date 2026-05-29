// adaptive_pid.hpp - Gain-scheduled 2D PID controller for gimbal pan/tilt.
//
// Operates on the pixel-error vector (set-point minus measured target
// position) and returns a control increment for the two gimbal axes.
//
// Adaptive behaviour:
//   * Gain scheduling: Kp and Kd are scaled UP with target speed (fast
//     responses for fast targets) and scaled DOWN inside a "near zone"
//     around zero error (gentle settling, no overshoot).
//   * Error dead-zone: when |error| <= deadzone the output is forced to
//     zero, killing the high-frequency micro-jitter when the target is
//     already centred.
//   * Integral anti-windup: the integral term is clamped, and it is reset
//     when a sudden change in the error rate (target reversing direction)
//     is detected, preventing the stale integral from causing overshoot.
//   * Derivative low-pass: the derivative term is filtered to avoid
//     amplifying measurement noise.
#ifndef PTZ_ADAPTIVE_PID_HPP
#define PTZ_ADAPTIVE_PID_HPP

#include "ptz/kalman_tracker.hpp"  // for Point2D

namespace ptz {

struct PidConfig {
    // Base gains (per axis, identical for pan/tilt by default).
    double kp = 0.6;
    double ki = 0.05;
    double kd = 0.12;

    // Gain scheduling vs. target speed (px/s). At speed >= speed_ref the
    // proportional/derivative gains are multiplied by (1 + speed_boost).
    double speed_ref = 400.0;
    double speed_boost = 0.8;

    // "Near zone": within this error magnitude (px) Kp/Kd ramp down towards
    // near_scale to damp settling and prevent overshoot.
    double near_zone = 30.0;
    double near_scale = 0.4;

    // Error dead-zone (px). |error| <= deadzone => zero output.
    double deadzone = 2.0;

    // Derivative low-pass smoothing factor in [0,1]; higher = smoother/slower.
    double derivative_lpf = 0.6;

    // Anti-windup: clamp on the magnitude of the integral accumulator (per axis).
    double integral_limit = 80.0;

    // Output saturation: max control increment magnitude per axis.
    double output_limit = 60.0;

    // Integral reset trigger: if the magnitude of the change in the error
    // rate (px/s^2, i.e. error "jerk") exceeds this, the target is treated
    // as having reversed direction and the integral is cleared.
    double reset_jerk_threshold = 1.5e4;
};

class AdaptivePID {
public:
    explicit AdaptivePID(const PidConfig& cfg = PidConfig{});

    // error          : set-point - measurement, in pixels.
    // dt             : control period in seconds.
    // target_speed   : magnitude of the target velocity (px/s), used for
    //                  gain scheduling. Pass KalmanTracker::speed().
    // Returns the control increment to apply to the gimbal (per-axis, px-equivalent).
    Point2D compute(const Point2D& error, double dt, double target_speed);

    void reset();

    // Whether the most recent compute() detected a direction reversal and
    // cleared the integral (useful for telemetry/diagnostics).
    bool integralWasReset() const { return integral_reset_; }

    const PidConfig& config() const { return cfg_; }
    void setConfig(const PidConfig& cfg) { cfg_ = cfg; }

private:
    struct AxisState {
        double integral = 0.0;
        double prev_error = 0.0;
        double prev_rate = 0.0;
        double deriv_filt = 0.0;
        bool has_prev = false;
    };

    double computeAxis(AxisState& s, double error, double dt, double kp,
                       double ki, double kd, bool clear_integral);

    PidConfig cfg_;
    AxisState ax_;
    AxisState ay_;
    bool integral_reset_ = false;
};

}  // namespace ptz

#endif  // PTZ_ADAPTIVE_PID_HPP
