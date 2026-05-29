// demo.cpp - Closed-loop simulation of the PTZ servo control stack.
//
// Simulates a target moving across the frame (with a mid-run direction
// reversal and a second decoy target appearing) and prints the controller's
// per-frame behaviour: selection, latency-compensated prediction, and the
// smoothed gimbal command.
#include <cmath>
#include <cstdio>
#include <vector>

#include "ptz/servo_controller.hpp"

int main() {
    ptz::ServoConfig cfg;
    cfg.frame_center = {640.0, 360.0};
    cfg.system_latency = 0.08;  // 80 ms total loop delay

    ptz::ServoController controller(cfg);
    controller.planner().setState({0.0, 0.0});  // gimbal command starts at 0

    const double dt = 1.0 / 60.0;  // 60 FPS
    const int frames = 240;

    std::printf("%4s  %8s %8s  %8s %8s  %8s %8s  %6s %s\n", "frm", "tgt_x",
                "tgt_y", "pred_x", "pred_y", "cmd_x", "cmd_y", "id", "note");

    // Primary target sweeps right, then reverses at frame 120.
    ptz::Point2D primary{200.0, 360.0};
    double vx = 600.0;  // px/s

    for (int f = 0; f < frames; ++f) {
        if (f == 120) vx = -600.0;  // sudden direction reversal
        primary.x += vx * dt;
        primary.y = 360.0 + 80.0 * std::sin(f * 0.05);

        std::vector<ptz::TargetObservation> dets;
        dets.push_back({/*id=*/1, primary, /*area=*/30000.0, /*conf=*/0.95});

        // A larger decoy target appears in the second half, off to the side.
        if (f >= 150) {
            ptz::Point2D decoy{900.0, 300.0};
            dets.push_back({/*id=*/2, decoy, /*area=*/55000.0, /*conf=*/0.9});
        }

        const ptz::ControlOutput out = controller.update(dets, dt);

        const char* note = "";
        if (out.integral_reset) note = "<-- integral reset (reversal)";
        if (f == 150) note = "decoy appears";

        if (f % 10 == 0 || out.integral_reset) {
            std::printf("%4d  %8.1f %8.1f  %8.1f %8.1f  %8.2f %8.2f  %6lld %s\n",
                        f, primary.x, primary.y, out.predicted_target.x,
                        out.predicted_target.y, out.increment.x, out.increment.y,
                        static_cast<long long>(out.target_id), note);
        }
    }

    std::printf("\nDone. Final gimbal command position: (%.1f, %.1f)\n",
                controller.planner().position().x,
                controller.planner().position().y);
    return 0;
}
