//
// Lightweight assertions (no GTest dependency) for the tracking stack:
// linear assignment, the Kalman filter, and the ByteTrack association logic.
//
#include "tracking/BYTETracker.h"
#include "tracking/LinearAssignment.h"
#include "tracking/KalmanFilter.h"
#include "common/Types.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);            \
            ++g_failures;                                                      \
        }                                                                      \
    } while (0)

using namespace rtp;

// ---- Linear assignment -----------------------------------------------------
void testAssignmentIdentity() {
    std::printf("test: linear assignment picks the diagonal\n");
    // Diagonal is cheap (0.1), off-diagonal expensive (0.9). Optimal = i->i.
    std::vector<std::vector<float>> cost = {
        {0.1f, 0.9f, 0.9f},
        {0.9f, 0.1f, 0.9f},
        {0.9f, 0.9f, 0.1f},
    };
    auto a = linearAssignment(cost, 0.5f);
    CHECK(a.matches.size() == 3);
    for (auto [r, c] : a.matches) CHECK(r == c);
    CHECK(a.unmatched_rows.empty());
    CHECK(a.unmatched_cols.empty());
}

void testAssignmentRectangularAndThreshold() {
    std::printf("test: rectangular cost + gating threshold\n");
    // 2 rows, 3 cols. Row 0 best->col1, row 1 best->col0. col2 unmatched.
    std::vector<std::vector<float>> cost = {
        {0.8f, 0.2f, 0.95f},
        {0.3f, 0.7f, 0.95f},
    };
    auto a = linearAssignment(cost, 0.5f);
    CHECK(a.matches.size() == 2);
    CHECK(a.unmatched_cols.size() == 1 && a.unmatched_cols[0] == 2);

    // Tighten the gate so only the very cheap pair survives.
    auto b = linearAssignment(cost, 0.25f);
    CHECK(b.matches.size() == 1);
    CHECK(b.matches[0].first == 0 && b.matches[0].second == 1);
}

void testAssignmentEmpty() {
    std::printf("test: empty inputs are safe\n");
    auto a = linearAssignment({}, 0.5f);
    CHECK(a.matches.empty());
    std::vector<std::vector<float>> rows_no_cols = {{}, {}};
    auto b = linearAssignment(rows_no_cols, 0.5f);
    CHECK(b.matches.empty());
    CHECK(b.unmatched_rows.size() == 2);
}

// ---- Kalman filter ---------------------------------------------------------
void testKalmanConvergesToConstantVelocity() {
    std::printf("test: Kalman tracks constant-velocity motion\n");
    KalmanFilter kf;
    KalmanFilter::Measure z0;
    z0 << 100.f, 100.f, 0.5f, 50.f;   // cx, cy, aspect, height
    auto [mean, cov] = kf.initiate(z0);

    // Feed measurements moving +10 in x and +5 in y each step.
    const float vx = 10.f, vy = 5.f;
    for (int step = 1; step <= 20; ++step) {
        kf.predict(mean, cov);
        KalmanFilter::Measure z;
        z << 100.f + vx * step, 100.f + vy * step, 0.5f, 50.f;
        std::tie(mean, cov) = kf.update(mean, cov, z);
    }
    // Estimated velocity should approach the true velocity.
    CHECK(std::fabs(mean(0, 4) - vx) < 1.0f);
    CHECK(std::fabs(mean(0, 5) - vy) < 1.0f);

    // One more pure prediction should extrapolate forward by ~velocity.
    const float px = mean(0, 0);
    kf.predict(mean, cov);
    CHECK(mean(0, 0) > px);
}

// ---- ByteTrack end to end --------------------------------------------------
Detection mk(float x, float y, float w, float h, float score, int cls = 0) {
    return Detection{Box{x, y, w, h}, score, cls};
}

void testByteTrackStableId() {
    std::printf("test: ByteTrack keeps a stable id for a moving object\n");
    BYTETracker tracker;

    int established_id = -1;
    for (int f = 0; f < 15; ++f) {
        Detections dets = { mk(50.f + f * 8, 60.f, 40.f, 80.f, 0.9f) };
        auto tracks = tracker.update(dets);
        if (f >= 2) {                       // allow a couple frames to confirm
            CHECK(tracks.size() == 1);
            if (!tracks.empty()) {
                if (established_id < 0) established_id = tracks[0].trackId();
                CHECK(tracks[0].trackId() == established_id);
            }
        }
    }
    CHECK(established_id > 0);
}

void testByteTrackRecoversAfterMiss() {
    std::printf("test: ByteTrack survives a brief miss (occlusion)\n");
    BYTETracker tracker;
    int id = -1;

    // Establish a track over several frames.
    for (int f = 0; f < 6; ++f) {
        auto tracks = tracker.update({ mk(100.f + f * 6, 100.f, 40.f, 80.f, 0.9f) });
        if (!tracks.empty()) id = tracks[0].trackId();
    }
    CHECK(id > 0);

    // Two frames with no detections (object occluded) -> track goes lost.
    tracker.update({});
    tracker.update({});

    // Reappears near the predicted position -> should reclaim the SAME id.
    auto tracks = tracker.update({ mk(100.f + 8 * 6, 100.f, 40.f, 80.f, 0.9f) });
    CHECK(tracks.size() == 1);
    if (!tracks.empty()) CHECK(tracks[0].trackId() == id);
}

void testByteTrackLowScoreNoNewTrack() {
    std::printf("test: low-confidence detection alone does not start a track\n");
    BYTETracker tracker;
    // Score below high_thresh: must not spawn a confirmed track.
    auto t0 = tracker.update({ mk(10.f, 10.f, 30.f, 60.f, 0.2f) });
    CHECK(t0.empty());
    auto t1 = tracker.update({ mk(10.f, 10.f, 30.f, 60.f, 0.2f) });
    CHECK(t1.empty());
}

} // namespace

int main() {
    testAssignmentIdentity();
    testAssignmentRectangularAndThreshold();
    testAssignmentEmpty();
    testKalmanConvergesToConstantVelocity();
    testByteTrackStableId();
    testByteTrackRecoversAfterMiss();
    testByteTrackLowScoreNoNewTrack();

    if (g_failures == 0) {
        std::printf("\nAll tracking tests passed.\n");
        return 0;
    }
    std::printf("\n%d check(s) failed.\n", g_failures);
    return 1;
}
