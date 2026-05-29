#include "ptz/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

namespace ptz {

namespace {
double clampMag(double v, double limit) {
    return std::max(-limit, std::min(limit, v));
}
}  // namespace

TrajectoryPlanner::TrajectoryPlanner(const TrajectoryConfig& cfg) : cfg_(cfg) {}

void TrajectoryPlanner::setState(const Point2D& position) {
    pos_ = position;
    vel_ = {0.0, 0.0};
}

void TrajectoryPlanner::reset() {
    pos_ = {0.0, 0.0};
    vel_ = {0.0, 0.0};
}

// Receding-horizon cubic-Bezier shaping for a single axis.
//   P0 = current position, tangent anchored to current velocity (C1)
//   P3 = target,           tangent zero (ease to rest)
// We read the Bezier's velocity at the first step, then enforce the
// acceleration and velocity limits before integrating the position.
double TrajectoryPlanner::smoothAxis(double& pos, double& vel, double target,
                                     double dt) const {
    if (dt <= 0.0) return 0.0;
    const double T = std::max(cfg_.horizon, dt);
    const double u = std::min(dt / T, 1.0);

    const double p0 = pos;
    const double p1 = pos + vel * (T / 3.0);  // start tangent = current velocity
    const double p2 = target;                 // end tangent -> 0
    const double p3 = target;

    // Derivative of the cubic Bezier w.r.t. parameter u, scaled to per-second.
    const double omu = 1.0 - u;
    const double dB = 3.0 * omu * omu * (p1 - p0) + 6.0 * omu * u * (p2 - p1) +
                      3.0 * u * u * (p3 - p2);
    double v_des = dB / T;

    // Dynamics constraint: limit acceleration, then velocity.
    const double dv_max = cfg_.max_acceleration * dt;
    const double dv = clampMag(v_des - vel, dv_max);
    vel += dv;
    vel = clampMag(vel, cfg_.max_velocity);

    const double old_pos = pos;
    pos += vel * dt;
    return pos - old_pos;
}

Point2D TrajectoryPlanner::follow(const Point2D& target_absolute, double dt) {
    Point2D inc;
    inc.x = smoothAxis(pos_.x, vel_.x, target_absolute.x, dt);
    inc.y = smoothAxis(pos_.y, vel_.y, target_absolute.y, dt);
    return inc;
}

Point2D TrajectoryPlanner::smoothIncrement(const Point2D& increment, double dt) {
    const Point2D target{pos_.x + increment.x, pos_.y + increment.y};
    return follow(target, dt);
}

Point2D TrajectoryPlanner::cubicBezier(const Point2D& p0, const Point2D& p1,
                                       const Point2D& p2, const Point2D& p3,
                                       double u) {
    u = std::clamp(u, 0.0, 1.0);
    const double omu = 1.0 - u;
    const double b0 = omu * omu * omu;
    const double b1 = 3.0 * omu * omu * u;
    const double b2 = 3.0 * omu * u * u;
    const double b3 = u * u * u;
    return {b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y};
}

std::vector<Point2D> TrajectoryPlanner::planEaseCurve(const Point2D& start,
                                                     const Point2D& start_velocity,
                                                     const Point2D& end,
                                                     double duration, int samples) {
    std::vector<Point2D> path;
    if (samples < 2) samples = 2;
    path.reserve(static_cast<std::size_t>(samples));

    const double T = std::max(duration, 1e-6);
    // Control points: start tangent follows the current velocity (C1),
    // end tangent is zero so the move eases to rest.
    const Point2D p0 = start;
    const Point2D p1{start.x + start_velocity.x * (T / 3.0),
                     start.y + start_velocity.y * (T / 3.0)};
    const Point2D p2 = end;
    const Point2D p3 = end;

    for (int i = 0; i < samples; ++i) {
        const double u = static_cast<double>(i) / (samples - 1);
        path.push_back(cubicBezier(p0, p1, p2, p3, u));
    }
    return path;
}

}  // namespace ptz
