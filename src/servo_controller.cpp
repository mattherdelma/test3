#include "ptz/servo_controller.hpp"

namespace ptz {

ServoController::ServoController(const ServoConfig& cfg)
    : cfg_(cfg),
      tracker_(cfg.process_noise_psd, cfg.measurement_noise),
      pid_(cfg.pid),
      planner_(cfg.trajectory),
      selector_(cfg.selector) {}

void ServoController::reset() {
    tracker_ = KalmanTracker(cfg_.process_noise_psd, cfg_.measurement_noise);
    pid_.reset();
    planner_.reset();
    selector_.reset();
    active_id_ = -1;
}

ControlOutput ServoController::update(
    const std::vector<TargetObservation>& detections, double dt) {
    ControlOutput out;

    // 1) Decide which target to track (with anti-flicker hysteresis).
    TargetObservation selected;
    const std::int64_t id = selector_.select(detections, cfg_.frame_center, &selected);
    if (id < 0) {
        // No target: keep the gimbal command still, decay PID state.
        active_id_ = -1;
        return out;
    }
    out.has_target = true;
    out.target_id = id;

    // 2) On a target switch, re-seed the tracker and the PID so we do not
    //    carry stale motion/integral from the previous target.
    if (id != active_id_) {
        tracker_ = KalmanTracker(cfg_.process_noise_psd, cfg_.measurement_noise);
        tracker_.init(selected.position);
        pid_.reset();
        active_id_ = id;
    }

    // 3) Fuse the new measurement into the motion model.
    tracker_.step(dt, selected.position);

    // 4) Predict where the target will be after the total loop latency; this
    //    is the set-point that cancels feedback phase lag.
    const Point2D predicted = tracker_.predictAhead(cfg_.system_latency);
    out.predicted_target = predicted;
    out.target_speed = tracker_.speed();

    // 5) Error of the predicted target relative to the frame centre.
    const Point2D error{predicted.x - cfg_.frame_center.x,
                        predicted.y - cfg_.frame_center.y};

    // 6) Adaptive PID -> raw control increment.
    const Point2D raw = pid_.compute(error, dt, out.target_speed);
    out.raw_pid = raw;
    out.integral_reset = pid_.integralWasReset();

    // 7) Bezier + dynamics smoothing -> feasible command increment.
    out.increment = planner_.smoothIncrement(raw, dt);
    return out;
}

}  // namespace ptz
