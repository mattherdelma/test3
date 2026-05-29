#pragma once
//
// ByteTrack multi-object tracker.
//
// The defining idea of ByteTrack: associate in two rounds. High-confidence
// detections are matched first against all active tracks; then the *low*-
// confidence detections (normally discarded) are matched against the tracks
// still unmatched, recovering objects under occlusion / motion blur without
// spawning false tracks.
//
//   Reference: Zhang et al., "ByteTrack: Multi-Object Tracking by Associating
//   Every Detection Box", ECCV 2022.
//
#include "STrack.h"
#include "KalmanFilter.h"
#include "LinearAssignment.h"
#include "../common/Types.h"

#include <vector>

namespace rtp {

struct ByteTrackConfig {
    float track_thresh  = 0.5f;   // split high/low detections at this score
    float high_thresh   = 0.6f;   // score needed to *start* a new track
    float match_thresh  = 0.8f;   // IoU gate (as 1-IoU cost) for round 1
    int   track_buffer  = 30;     // frames a lost track survives before removal
    float frame_rate    = 30.f;
};

class BYTETracker {
public:
    explicit BYTETracker(ByteTrackConfig cfg = {});

    // Feed one frame of detections; returns the currently-active tracks.
    std::vector<STrack> update(const Detections& detections);

private:
    // IoU distance (1 - IoU) cost matrix between two track sets.
    static std::vector<std::vector<float>> iouDistance(
        const std::vector<STrack*>& a, const std::vector<STrack*>& b);

    // IoU association that stays correct when either side is empty (an empty
    // cost matrix cannot encode the column count, so handle it explicitly).
    static Assignment associate(const std::vector<STrack*>& tracks,
                                const std::vector<STrack*>& dets, float thresh);

    static std::vector<STrack*> jointPtrs(std::vector<STrack>& a,
                                          std::vector<STrack>& b);
    static void removeDuplicates(std::vector<STrack>& a, std::vector<STrack>& b);

    ByteTrackConfig cfg_;
    int   frame_id_{0};
    int   max_time_lost_;
    KalmanFilter kf_;

    std::vector<STrack> tracked_;   // confirmed, currently tracked
    std::vector<STrack> lost_;      // temporarily lost
    std::vector<STrack> removed_;   // terminated this frame
};

} // namespace rtp
