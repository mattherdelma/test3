// target_selector.hpp - Priority-based multi-target selection.
//
// Scores each candidate detection by a weighted combination of:
//   * proximity to the frame centre (inverse distance), and
//   * visible area (bigger / closer targets score higher).
//
// To avoid high-frequency flapping between similarly-scored targets, a switch
// away from the currently tracked target only happens when a challenger beats
// it by a relative margin AND sustains that lead for a number of consecutive
// frames (hysteresis).
#ifndef PTZ_TARGET_SELECTOR_HPP
#define PTZ_TARGET_SELECTOR_HPP

#include <cstdint>
#include <vector>

#include "ptz/kalman_tracker.hpp"  // for Point2D

namespace ptz {

struct TargetObservation {
    std::int64_t id = -1;   // stable track id from the vision layer
    Point2D position{};     // pixel position of the target centre
    double area = 0.0;      // visible bounding-box area (px^2)
    double confidence = 1.0;  // detector confidence in [0,1]
};

struct SelectorConfig {
    double weight_distance = 1.0;   // weight on the proximity term
    double weight_area = 0.6;       // weight on the visible-area term
    double distance_scale = 200.0;  // px; sets how fast the proximity term decays
    double area_scale = 40000.0;    // px^2; normaliser for the area term
    double switch_margin = 0.15;    // challenger must beat current by this fraction
    int switch_hold_frames = 5;     // ...for this many consecutive frames
};

class TargetSelector {
public:
    explicit TargetSelector(const SelectorConfig& cfg = SelectorConfig{});

    // Pick the target to track this frame. `frame_center` is the image
    // centre (the natural set-point). Returns the id of the selected target,
    // or -1 if there are no candidates. The chosen observation, when present,
    // is written to `out_selected`.
    std::int64_t select(const std::vector<TargetObservation>& candidates,
                        const Point2D& frame_center,
                        TargetObservation* out_selected = nullptr);

    std::int64_t currentId() const { return current_id_; }
    void reset();

    double scoreOf(const TargetObservation& t, const Point2D& frame_center) const;

private:
    SelectorConfig cfg_;
    std::int64_t current_id_ = -1;
    std::int64_t challenger_id_ = -1;
    int challenger_streak_ = 0;
};

}  // namespace ptz

#endif  // PTZ_TARGET_SELECTOR_HPP
