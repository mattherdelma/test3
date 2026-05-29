#pragma once
//
// A single tracked target (track-let) managed by the ByteTracker.
//
#include "KalmanFilter.h"
#include "../common/Types.h"

#include <vector>

namespace rtp {

enum class TrackState { New, Tracked, Lost, Removed };

class STrack {
public:
    STrack(const Box& tlwh, float score, int class_id);

    // ByteTrack lifecycle hooks.
    void activate(KalmanFilter& kf, int frame_id);
    void reActivate(const STrack& det, int frame_id, bool new_id);
    void update(const STrack& det, int frame_id);
    void markLost()    { state_ = TrackState::Lost; }
    void markRemoved() { state_ = TrackState::Removed; }

    // Batch prediction across many tracks shares one filter instance.
    static void multiPredict(std::vector<STrack*>& tracks, KalmanFilter& kf);

    // Current estimate as a top-left/width-height box (image pixels).
    Box tlwh() const;
    const Box& detBox()   const { return det_tlwh_; }
    float score()         const { return score_; }
    int   classId()       const { return class_id_; }
    int   trackId()       const { return track_id_; }
    int   frameId()       const { return frame_id_; }
    int   startFrame()    const { return start_frame_; }
    bool  isActivated()   const { return is_activated_; }
    TrackState state()    const { return state_; }
    void  setState(TrackState s) { state_ = s; }

private:
    static int nextId();
    // Convert tlwh <-> measurement [cx, cy, a, h].
    static KalmanFilter::Measure toXYAH(const Box& tlwh);

    KalmanFilter::StateMean mean_{};
    KalmanFilter::StateCov  cov_{};

    Box   det_tlwh_;          // last associated detection box
    float score_;
    int   class_id_;

    int  track_id_{0};
    int  frame_id_{0};
    int  start_frame_{0};
    int  tracklet_len_{0};
    bool is_activated_{false};
    TrackState state_{TrackState::New};

    KalmanFilter* kf_{nullptr};
};

} // namespace rtp
