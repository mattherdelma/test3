// test_control.cpp - Lightweight sanity tests for the PTZ control stack.
#include <cmath>
#include <cstdio>
#include <vector>

#include "ptz/adaptive_pid.hpp"
#include "ptz/kalman_tracker.hpp"
#include "ptz/target_selector.hpp"
#include "ptz/trajectory_planner.hpp"

static int g_failures = 0;

static void check(bool cond, const char* name) {
    std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond) ++g_failures;
}

static void test_kalman_predicts_constant_velocity() {
    ptz::KalmanTracker kf(/*psd=*/10.0, /*meas_noise=*/1.0);
    const double dt = 1.0 / 60.0;
    const double vx = 500.0, vy = -200.0;
    ptz::Point2D p{100.0, 300.0};

    // Feed a constant-velocity track for 1 s.
    for (int i = 0; i < 60; ++i) {
        p.x += vx * dt;
        p.y += vy * dt;
        kf.step(dt, p);
    }

    // Estimated velocity should match the truth within a small tolerance.
    const ptz::Point2D v = kf.velocity();
    check(std::fabs(v.x - vx) < 25.0, "kalman velocity x converges");
    check(std::fabs(v.y - vy) < 25.0, "kalman velocity y converges");

    // Predict 0.1 s ahead and compare to the analytic future position.
    const double ahead = 0.1;
    const ptz::Point2D pred = kf.predictAhead(ahead);
    const double exp_x = p.x + vx * ahead;
    const double exp_y = p.y + vy * ahead;
    check(std::fabs(pred.x - exp_x) < 8.0, "kalman predictAhead x");
    check(std::fabs(pred.y - exp_y) < 8.0, "kalman predictAhead y");
}

static void test_pid_deadzone() {
    ptz::AdaptivePID pid;  // default deadzone = 2 px
    const ptz::Point2D out = pid.compute({1.0, 1.0}, 1.0 / 60.0, 0.0);
    check(out.x == 0.0 && out.y == 0.0, "pid deadzone suppresses tiny error");

    const ptz::Point2D out2 = pid.compute({50.0, 0.0}, 1.0 / 60.0, 0.0);
    check(out2.x > 0.0, "pid responds to real error");
}

static void test_pid_output_limit() {
    ptz::PidConfig c;
    c.output_limit = 10.0;
    ptz::AdaptivePID pid(c);
    ptz::Point2D out{};
    for (int i = 0; i < 50; ++i) out = pid.compute({1000.0, 0.0}, 1.0 / 60.0, 0.0);
    check(std::fabs(out.x) <= 10.0 + 1e-9, "pid output saturates at limit");
}

static void test_bezier_endpoints() {
    const ptz::Point2D p0{0, 0}, p1{1, 5}, p2{8, 2}, p3{10, 10};
    const ptz::Point2D a = ptz::TrajectoryPlanner::cubicBezier(p0, p1, p2, p3, 0.0);
    const ptz::Point2D b = ptz::TrajectoryPlanner::cubicBezier(p0, p1, p2, p3, 1.0);
    check(std::fabs(a.x - p0.x) < 1e-9 && std::fabs(a.y - p0.y) < 1e-9,
          "bezier hits start at u=0");
    check(std::fabs(b.x - p3.x) < 1e-9 && std::fabs(b.y - p3.y) < 1e-9,
          "bezier hits end at u=1");
}

static void test_trajectory_no_step() {
    // A large step command must not produce an instantaneous large increment;
    // the smoother ramps it up subject to the acceleration limit.
    ptz::TrajectoryConfig c;
    c.max_acceleration = 5000.0;
    ptz::TrajectoryPlanner tp(c);
    tp.setState({0.0, 0.0});
    const double dt = 1.0 / 60.0;
    const ptz::Point2D inc = tp.smoothIncrement({500.0, 0.0}, dt);
    // First-tick velocity is bounded by max_accel*dt, so increment is small.
    check(std::fabs(inc.x) < 500.0, "trajectory does not pass a step through");
    check(std::fabs(inc.x) <= c.max_acceleration * dt * dt + 1e-6,
          "trajectory respects acceleration limit on first tick");
}

static void test_selector_hysteresis() {
    ptz::SelectorConfig c;
    c.switch_hold_frames = 5;
    c.switch_margin = 0.1;
    ptz::TargetSelector sel(c);
    const ptz::Point2D center{640, 360};

    // Frame 1: only target 1 (off-centre) -> selected.
    std::vector<ptz::TargetObservation> dets = {{1, {500, 360}, 30000.0, 1.0}};
    check(sel.select(dets, center) == 1, "selector picks the sole target");

    // Now a clearly better target 2 (centred, larger) appears. It beats the
    // incumbent well past switch_margin, but must NOT win immediately.
    dets = {{1, {500, 360}, 30000.0, 1.0}, {2, {640, 360}, 90000.0, 1.0}};
    std::int64_t id = sel.select(dets, center);
    check(id == 1, "selector holds incumbent on first challenge");

    // After enough sustained frames it should switch.
    for (int i = 0; i < c.switch_hold_frames; ++i) id = sel.select(dets, center);
    check(id == 2, "selector switches after sustained better target");
}

int main() {
    test_kalman_predicts_constant_velocity();
    test_pid_deadzone();
    test_pid_output_limit();
    test_bezier_endpoints();
    test_trajectory_no_step();
    test_selector_hysteresis();

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
