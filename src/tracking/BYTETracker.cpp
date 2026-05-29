#include "BYTETracker.h"
#include "LinearAssignment.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace rtp {

namespace {

// Detection STracks have track_id 0 (never activated); use their raw det box.
// Activated tracks expose their Kalman-predicted box via tlwh().
Box boxOf(const STrack* t) {
    return t->trackId() == 0 ? t->detBox() : t->tlwh();
}

std::vector<STrack> jointStracks(const std::vector<STrack>& a,
                                 const std::vector<STrack>& b) {
    std::unordered_set<int> seen;
    std::vector<STrack> res;
    res.reserve(a.size() + b.size());
    for (const auto& t : a) { seen.insert(t.trackId()); res.push_back(t); }
    for (const auto& t : b) if (!seen.count(t.trackId())) res.push_back(t);
    return res;
}

std::vector<STrack> subStracks(const std::vector<STrack>& a,
                               const std::vector<STrack>& b) {
    std::unordered_set<int> drop;
    for (const auto& t : b) drop.insert(t.trackId());
    std::vector<STrack> res;
    for (const auto& t : a) if (!drop.count(t.trackId())) res.push_back(t);
    return res;
}

} // namespace

BYTETracker::BYTETracker(ByteTrackConfig cfg) : cfg_(cfg) {
    max_time_lost_ = static_cast<int>(cfg_.frame_rate / 30.f * cfg_.track_buffer);
}

std::vector<std::vector<float>> BYTETracker::iouDistance(
    const std::vector<STrack*>& a, const std::vector<STrack*>& b) {
    std::vector<std::vector<float>> cost(a.size(),
                                         std::vector<float>(b.size(), 0.f));
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Box ba = boxOf(a[i]);
        for (std::size_t j = 0; j < b.size(); ++j) {
            cost[i][j] = 1.f - iou(ba, boxOf(b[j]));   // IoU distance
        }
    }
    return cost;
}

Assignment BYTETracker::associate(const std::vector<STrack*>& tracks,
                                  const std::vector<STrack*>& dets,
                                  float thresh) {
    if (tracks.empty() || dets.empty()) {
        Assignment a;
        for (int i = 0; i < static_cast<int>(tracks.size()); ++i)
            a.unmatched_rows.push_back(i);
        for (int j = 0; j < static_cast<int>(dets.size()); ++j)
            a.unmatched_cols.push_back(j);
        return a;
    }
    return linearAssignment(iouDistance(tracks, dets), thresh);
}

void BYTETracker::removeDuplicates(std::vector<STrack>& a,
                                   std::vector<STrack>& b) {
    std::vector<char> rm_a(a.size(), 0), rm_b(b.size(), 0);
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (std::size_t j = 0; j < b.size(); ++j) {
            if (iou(a[i].tlwh(), b[j].tlwh()) > 0.85f) {
                const int dur_a = a[i].frameId() - a[i].startFrame();
                const int dur_b = b[j].frameId() - b[j].startFrame();
                if (dur_a > dur_b) rm_b[j] = 1; else rm_a[i] = 1;
            }
        }
    }
    std::vector<STrack> na, nb;
    for (std::size_t i = 0; i < a.size(); ++i) if (!rm_a[i]) na.push_back(a[i]);
    for (std::size_t j = 0; j < b.size(); ++j) if (!rm_b[j]) nb.push_back(b[j]);
    a.swap(na);
    b.swap(nb);
}

std::vector<STrack> BYTETracker::update(const Detections& objects) {
    ++frame_id_;

    std::vector<STrack> activated, refind, newly_lost, newly_removed;

    // --- 1. Split detections by confidence --------------------------------
    std::vector<STrack> det_high, det_low;
    for (const auto& o : objects) {
        if (o.score >= cfg_.track_thresh)
            det_high.emplace_back(o.box, o.score, o.class_id);
        else if (o.score > 0.1f)
            det_low.emplace_back(o.box, o.score, o.class_id);
    }

    // --- 2. Partition existing tracks -------------------------------------
    std::vector<STrack*> unconfirmed, tracked;
    for (auto& t : tracked_)
        (t.isActivated() ? tracked : unconfirmed).push_back(&t);

    // --- 3. Predict pool = tracked + lost ---------------------------------
    std::vector<STrack*> pool = tracked;
    for (auto& t : lost_) pool.push_back(&t);
    STrack::multiPredict(pool, kf_);

    // --- 4. First association: pool vs high detections --------------------
    std::vector<STrack*> high_ptrs;
    for (auto& d : det_high) high_ptrs.push_back(&d);
    auto a1 = associate(pool, high_ptrs, cfg_.match_thresh);

    for (auto [it, id] : a1.matches) {
        STrack* trk = pool[it];
        STrack& det = det_high[id];
        if (trk->state() == TrackState::Tracked) {
            trk->update(det, frame_id_);
            activated.push_back(*trk);
        } else {
            trk->reActivate(det, frame_id_, /*new_id=*/false);
            refind.push_back(*trk);
        }
    }

    std::vector<STrack*> r_tracked;          // still-tracked, unmatched in r1
    for (int it : a1.unmatched_rows)
        if (pool[it]->state() == TrackState::Tracked) r_tracked.push_back(pool[it]);

    std::vector<STrack*> remaining_high;     // high dets left for unconfirmed
    for (int id : a1.unmatched_cols) remaining_high.push_back(&det_high[id]);

    // --- 5. Second association: r_tracked vs LOW detections --------------
    std::vector<STrack*> low_ptrs;
    for (auto& d : det_low) low_ptrs.push_back(&d);
    auto a2 = associate(r_tracked, low_ptrs, 0.5f);

    for (auto [it, id] : a2.matches) {
        STrack* trk = r_tracked[it];
        STrack& det = det_low[id];
        if (trk->state() == TrackState::Tracked) {
            trk->update(det, frame_id_);
            activated.push_back(*trk);
        } else {
            trk->reActivate(det, frame_id_, false);
            refind.push_back(*trk);
        }
    }
    for (int it : a2.unmatched_rows) {
        STrack* trk = r_tracked[it];
        if (trk->state() != TrackState::Lost) {
            trk->markLost();
            newly_lost.push_back(*trk);
        }
    }

    // --- 6. Unconfirmed tracks vs leftover high detections ---------------
    auto a3 = associate(unconfirmed, remaining_high, 0.7f);
    for (auto [iu, id] : a3.matches) {
        unconfirmed[iu]->update(*remaining_high[id], frame_id_);
        activated.push_back(*unconfirmed[iu]);
    }
    for (int iu : a3.unmatched_rows) {
        unconfirmed[iu]->markRemoved();
        newly_removed.push_back(*unconfirmed[iu]);
    }

    // --- 7. Initialize new tracks from strong leftover detections --------
    for (int id : a3.unmatched_cols) {
        STrack* det = remaining_high[id];
        if (det->score() < cfg_.high_thresh) continue;
        det->activate(kf_, frame_id_);
        activated.push_back(*det);
    }

    // --- 8. Age out lost tracks ------------------------------------------
    for (auto& t : lost_) {
        if (frame_id_ - t.frameId() > max_time_lost_) {
            t.markRemoved();
            newly_removed.push_back(t);
        }
    }

    // --- 9. Rebuild member state -----------------------------------------
    tracked_ = jointStracks(activated, refind);
    lost_    = subStracks(lost_, tracked_);
    for (auto& t : newly_lost) lost_.push_back(t);
    lost_    = subStracks(lost_, newly_removed);
    removeDuplicates(tracked_, lost_);

    std::vector<STrack> output;
    for (const auto& t : tracked_)
        if (t.isActivated()) output.push_back(t);
    return output;
}

} // namespace rtp
