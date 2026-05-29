// servo_controller.hpp - Cognition & control layer for PTZ visual servoing.
//
// Wires the four building blocks into one per-frame pipeline:
//
//   vision detections
//        |
//        v
//   TargetSelector  --(which target)-->  KalmanTracker  --(predictAhead)-->
//        |                                                       |
//        |                                            latency-compensated
//        |                                              set-point error
//        v                                                       v
//   AdaptivePID  --(raw increment)-->  TrajectoryPlanner  --(smooth increment)
//                                                                |
//                                                                v
//                                                       gimbal command
//
// Error convention: error = (predicted target position) - (frame centre).
// A positive error means the target is to the right/below centre; the
// returned increment drives the gimbal command in that direction so the
// target is pulled back to the centre. Flip the sign per axis at the driver
// if your gimbal's positive direction is inverted.
#ifndef PTZ_SERVO_CONTROLLER_HPP
#define PTZ_SERVO_CONTROLLER_HPP

#include <vector>

#include "ptz/adaptive_pid.hpp"
#include "ptz/kalman_tracker.hpp"
#include "ptz/target_selector.hpp"
#include "ptz/trajectory_planner.hpp"

namespace ptz {

struct ServoConfig {
    Point2D frame_center{640.0, 360.0};  // image centre (set-point), px
    double system_latency = 0.08;        // total loop delay estimate, s
    PidConfig pid{};
    TrajectoryConfig trajectory{};
    SelectorConfig selector{};
    double process_noise_psd = 50.0;     // Kalman jerk PSD
    double measurement_noise = 4.0;      // Kalman pixel variance
};

struct ControlOutput {
    bool has_target = false;
    std::int64_t target_id = -1;
    Point2D increment{};         // smoothed command increment for this tick
    Point2D predicted_target{};  // latency-compensated target position
    Point2D raw_pid{};           // PID output before smoothing (diagnostics)
    double target_speed = 0.0;   // estimated target speed (px/s)
    bool integral_reset = false; // PID cleared its integral this tick
};

class ServoController {
public:
    explicit ServoController(const ServoConfig& cfg = ServoConfig{});

    // Run one control cycle. `dt` is the time since the previous call (s).
    ControlOutput update(const std::vector<TargetObservation>& detections,
                         double dt);

    void reset();

    const ServoConfig& config() const { return cfg_; }

    // Direct access for tuning / telemetry.
    KalmanTracker& tracker() { return tracker_; }
    AdaptivePID& pid() { return pid_; }
    TrajectoryPlanner& planner() { return planner_; }
    TargetSelector& selector() { return selector_; }

private:
    ServoConfig cfg_;
    KalmanTracker tracker_;
    AdaptivePID pid_;
    TrajectoryPlanner planner_;
    TargetSelector selector_;
    std::int64_t active_id_ = -1;
};

}  // namespace ptz

#endif  // PTZ_SERVO_CONTROLLER_HPP
