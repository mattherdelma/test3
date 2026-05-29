#include "ptz/target_selector.hpp"

#include <cmath>

namespace ptz {

TargetSelector::TargetSelector(const SelectorConfig& cfg) : cfg_(cfg) {}

void TargetSelector::reset() {
    current_id_ = -1;
    challenger_id_ = -1;
    challenger_streak_ = 0;
}

double TargetSelector::scoreOf(const TargetObservation& t,
                               const Point2D& frame_center) const {
    const double dx = t.position.x - frame_center.x;
    const double dy = t.position.y - frame_center.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    // Proximity term: inverse-distance, smoothly bounded to avoid blow-up at
    // the centre. 1 at the centre, decaying with distance.
    const double proximity = 1.0 / (1.0 + dist / cfg_.distance_scale);

    // Visible-area term, normalised and saturated at 1.
    double area_term = t.area / cfg_.area_scale;
    if (area_term > 1.0) area_term = 1.0;

    const double raw =
        cfg_.weight_distance * proximity + cfg_.weight_area * area_term;
    return raw * t.confidence;
}

std::int64_t TargetSelector::select(const std::vector<TargetObservation>& candidates,
                                    const Point2D& frame_center,
                                    TargetObservation* out_selected) {
    if (candidates.empty()) {
        reset();
        return -1;
    }

    // Best candidate overall, plus the score/observation of the currently
    // tracked target (if it is still present).
    const TargetObservation* best = nullptr;
    double best_score = -1.0;
    const TargetObservation* current = nullptr;
    double current_score = -1.0;

    for (const auto& t : candidates) {
        const double s = scoreOf(t, frame_center);
        if (s > best_score) {
            best_score = s;
            best = &t;
        }
        if (t.id == current_id_) {
            current = &t;
            current_score = s;
        }
    }

    const TargetObservation* chosen = best;

    if (current != nullptr) {
        // We already track a still-visible target. Only switch if a different
        // candidate beats it by the margin for enough consecutive frames.
        if (best != nullptr && best->id != current_id_ &&
            best_score > current_score * (1.0 + cfg_.switch_margin)) {
            if (best->id == challenger_id_) {
                ++challenger_streak_;
            } else {
                challenger_id_ = best->id;
                challenger_streak_ = 1;
            }
            if (challenger_streak_ >= cfg_.switch_hold_frames) {
                chosen = best;  // promote the challenger
            } else {
                chosen = current;  // hold the incumbent (hysteresis)
            }
        } else {
            // Incumbent still best (or close enough); clear any challenger.
            challenger_id_ = -1;
            challenger_streak_ = 0;
            chosen = current;
        }
    } else {
        // No incumbent (first frame or it disappeared): take the best now.
        challenger_id_ = -1;
        challenger_streak_ = 0;
    }

    current_id_ = chosen->id;
    if (out_selected != nullptr) *out_selected = *chosen;
    return current_id_;
}

}  // namespace ptz
