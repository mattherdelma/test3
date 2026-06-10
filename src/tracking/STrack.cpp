#include "STrack.h"

#include <atomic>
#include <tuple>

namespace rtp {

namespace {
std::atomic<int> g_track_id_count{0};
}

int STrack::nextId() {
    return g_track_id_count.fetch_add(1, std::memory_order_relaxed) + 1;
}

STrack::STrack(const Box& tlwh, float score, int class_id)
    : det_tlwh_(tlwh), score_(score), class_id_(class_id) {}

KalmanFilter::Measure STrack::toXYAH(const Box& t) {
    KalmanFilter::Measure z;
    z << t.x + t.w * 0.5f,        // cx
         t.y + t.h * 0.5f,        // cy
         t.w / t.h,               // aspect ratio
         t.h;                     // height
    return z;
}

Box STrack::tlwh() const {
    // mean = [cx, cy, a, h, ...]; reconstruct w = a*h.
    const float cx = mean_(0, 0), cy = mean_(0, 1);
    const float a  = mean_(0, 2), h  = mean_(0, 3);
    const float w  = a * h;
    return {cx - w * 0.5f, cy - h * 0.5f, w, h};
}

void STrack::activate(KalmanFilter& kf, int frame_id) {
    kf_ = &kf;
    track_id_ = nextId();
    std::tie(mean_, cov_) = kf.initiate(toXYAH(det_tlwh_));

    tracklet_len_ = 0;
    state_        = TrackState::Tracked;
    is_activated_ = (frame_id == 1);   // first frame: immediately confirmed
    frame_id_     = frame_id;
    start_frame_  = frame_id;
}

void STrack::reActivate(const STrack& det, int frame_id, bool new_id) {
    std::tie(mean_, cov_) = kf_->update(mean_, cov_, toXYAH(det.det_tlwh_));
    tracklet_len_ = 0;
    state_        = TrackState::Tracked;
    is_activated_ = true;
    frame_id_     = frame_id;
    score_        = det.score_;
    class_id_     = det.class_id_;
    det_tlwh_     = det.det_tlwh_;
    if (new_id) track_id_ = nextId();
}

void STrack::update(const STrack& det, int frame_id) {
    frame_id_ = frame_id;
    ++tracklet_len_;
    std::tie(mean_, cov_) = kf_->update(mean_, cov_, toXYAH(det.det_tlwh_));
    state_        = TrackState::Tracked;
    is_activated_ = true;
    score_        = det.score_;
    class_id_     = det.class_id_;
    det_tlwh_     = det.det_tlwh_;
}

void STrack::multiPredict(std::vector<STrack*>& tracks, KalmanFilter& kf) {
    for (STrack* t : tracks) {
        // A non-tracked target has no observed velocity to trust this step.
        if (t->state_ != TrackState::Tracked) t->mean_(0, 7) = 0.f;
        kf.predict(t->mean_, t->cov_);
        t->kf_ = &kf;
    }
}

} // namespace rtp
