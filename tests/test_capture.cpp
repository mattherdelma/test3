//
// DXGI Desktop Duplication smoke test (Windows only).
//
// Runs the real capture path end-to-end: initialize duplication, spin the
// capture thread, pull a batch of frames off the lock-free queue, and validate
// that each frame carries a valid device surface of the expected size. Also
// reports the achieved capture rate.
//
// This needs an interactive desktop session and a GPU, so on a headless / no-
// display machine it SKIPS (returns 0) rather than failing the suite.
//
#include "capture/DxgiCapturer.h"

#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    using namespace rtp;
    int failures = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);            \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

    DxgiCapturer capturer;
    try {
        capturer.initialize(/*adapter=*/0, /*output=*/0);
    } catch (const std::exception& e) {
        std::printf("SKIP: desktop duplication unavailable (%s)\n", e.what());
        return 0;   // environment can't run this test; not a failure
    }

    std::printf("Initialized capture: %dx%d\n", capturer.width(), capturer.height());
    CHECK(capturer.width()  > 0);
    CHECK(capturer.height() > 0);

    FrameQueue queue;
    capturer.start(queue);

    constexpr int kWant = 30;
    int got = 0;
    const auto t0 = Clock::now();
    const auto deadline = t0 + std::chrono::seconds(5);

    uint64_t last_id = 0;
    bool ids_monotonic = true;

    while (got < kWant && Clock::now() < deadline) {
        auto frame = queue.pop();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        CHECK(frame->valid());
        CHECK(frame->dptr != nullptr);
        CHECK(frame->width  == capturer.width());
        CHECK(frame->height == capturer.height());
        CHECK(frame->pitch  >= static_cast<std::size_t>(frame->width) * 4);
        if (got > 0 && frame->frame_id <= last_id) ids_monotonic = false;
        last_id = frame->frame_id;

        // Return the surface to the pool, exactly as the inference stage would.
        capturer.release(frame->pool_slot);
        ++got;
    }
    capturer.stop();

    const double secs = std::chrono::duration<double>(Clock::now() - t0).count();
    CHECK(ids_monotonic);

    if (got == 0) {
        // A perfectly static desktop yields only timeouts; treat as a skip.
        std::printf("SKIP: no frames produced (static desktop?) timeouts=%llu\n",
                    static_cast<unsigned long long>(capturer.stats().timeouts.load()));
        return failures ? 1 : 0;
    }

    std::printf("Captured %d frames in %.2fs (%.1f fps), dropped=%llu\n",
                got, secs, got / secs,
                static_cast<unsigned long long>(
                    capturer.stats().dropped_full_queue.load()));

    if (failures == 0) {
        std::printf("\nDXGI capture smoke test passed.\n");
        return 0;
    }
    std::printf("\n%d check(s) failed.\n", failures);
    return 1;
}
