// trajectory_planner.hpp - Bezier-based smoothing of the control command.
//
// The raw PID output is a per-tick increment that can jump in steps. Feeding
// a step straight to a servo causes mechanical shock (gear wear) and visible
// image jitter. This planner shapes the command into a smooth reference that:
//
//   * is C1-continuous (velocity continuous) by anchoring the start tangent
//     of a cubic Bezier to the gimbal's current command velocity, so there
//     is never an instantaneous velocity discontinuity (no step);
//   * eases in and out (the Bezier velocity profile is bell-shaped);
//   * respects physical dynamics: the command velocity and acceleration are
//     hard-clamped to the gimbal's limits, so the trajectory is always
//     dynamically feasible.
//
// Both an online (receding-horizon) smoother for the live control loop and
// offline cubic-Bezier path-sampling helpers (useful for planning/plotting)
// are provided.
#ifndef PTZ_TRAJECTORY_PLANNER_HPP
#define PTZ_TRAJECTORY_PLANNER_HPP

#include <vector>

#include "ptz/kalman_tracker.hpp"  // for Point2D

namespace ptz {

struct TrajectoryConfig {
    double horizon = 0.15;     // Bezier ease horizon (s). Larger = smoother/slower.
    double max_velocity = 800.0;   // px/s command velocity limit.
    double max_acceleration = 6000.0;  // px/s^2 command acceleration limit.
};

class TrajectoryPlanner {
public:
    explicit TrajectoryPlanner(const TrajectoryConfig& cfg = TrajectoryConfig{});

    // Initialise the internal command state to the gimbal's current absolute
    // position (and zero velocity). Call once at start / after a reset.
    void setState(const Point2D& position);
    void reset();

    // Online smoothing. Given the PID control increment for this tick, treat
    // (current command + increment) as the new target and return the
    // dynamically-feasible, Bezier-shaped increment to actually send to the
    // gimbal this tick.
    Point2D smoothIncrement(const Point2D& increment, double dt);

    // Same, but the caller supplies the absolute target command instead of an
    // increment.
    Point2D follow(const Point2D& target_absolute, double dt);

    Point2D position() const { return pos_; }
    Point2D velocity() const { return vel_; }

    // --- Offline cubic-Bezier helpers (free of internal state) ---

    // Evaluate a cubic Bezier at parameter u in [0,1].
    static Point2D cubicBezier(const Point2D& p0, const Point2D& p1,
                               const Point2D& p2, const Point2D& p3, double u);

    // Sample a C1 ease curve from `start` (moving at start_velocity) to `end`
    // (at rest) into `samples` points over the given duration. Returns the
    // sampled positions; handy for visualising or pre-planning a move.
    static std::vector<Point2D> planEaseCurve(const Point2D& start,
                                              const Point2D& start_velocity,
                                              const Point2D& end, double duration,
                                              int samples);

private:
    double smoothAxis(double& pos, double& vel, double target, double dt) const;

    TrajectoryConfig cfg_;
    Point2D pos_{};
    Point2D vel_{};
};

}  // namespace ptz

#endif  // PTZ_TRAJECTORY_PLANNER_HPP
