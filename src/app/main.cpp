//
// End-to-end pipeline: DXGI capture -> SPSC queue -> YOLOv8 (TensorRT) ->
// ByteTrack. The capture thread and the inference/tracking thread are separate;
// they share only the lock-free queue and the capture pool's release() call.
//
#include "../capture/DxgiCapturer.h"
#include "../inference/YoloV8.h"
#include "../tracking/BYTETracker.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }
} // namespace

#if defined(_WIN32)

int main(int argc, char** argv) {
    const std::string engine = argc > 1 ? argv[1] : "yolov8s.fp16.engine";
    std::signal(SIGINT, onSignal);

    using namespace rtp;

    DxgiCapturer capturer;
    capturer.initialize(/*adapter=*/0, /*output=*/0);
    std::printf("Capturing %dx%d desktop\n", capturer.width(), capturer.height());

    FrameQueue queue;
    YoloV8 detector(engine, YoloConfig{/*conf=*/0.25f, /*nms=*/0.45f});
    BYTETracker tracker(ByteTrackConfig{/*track_thresh=*/0.5f, /*high=*/0.6f});

    capturer.start(queue);

    // Inference + tracking loop on the consumer thread (here: main).
    uint64_t processed = 0;
    auto report_t0 = Clock::now();

    while (!g_stop.load(std::memory_order_acquire)) {
        auto frame = queue.pop();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }

        const auto t_infer0 = Clock::now();
        Detections dets = detector.detect(frame->dptr, frame->width,
                                          frame->height, frame->pitch);
        // The captured surface is fully consumed -> return it to the pool.
        capturer.release(frame->pool_slot);

        std::vector<STrack> tracks = tracker.update(dets);
        const auto t_now = Clock::now();

        const double infer_ms =
            std::chrono::duration<double, std::milli>(t_now - t_infer0).count();
        const double latency_ms =
            std::chrono::duration<double, std::milli>(t_now - frame->captured_at)
                .count();

        if (++processed % 60 == 0) {
            const double secs =
                std::chrono::duration<double>(t_now - report_t0).count();
            std::printf("[frame %llu] dets=%zu tracks=%zu | infer=%.2fms "
                        "e2e=%.2fms fps=%.1f dropped=%llu\n",
                        static_cast<unsigned long long>(frame->frame_id),
                        dets.size(), tracks.size(), infer_ms, latency_ms,
                        60.0 / secs,
                        static_cast<unsigned long long>(
                            capturer.stats().dropped_full_queue.load()));
            report_t0 = t_now;
        }

        // Hand `tracks` to whatever consumes the result (overlay, control loop…)
        (void)tracks;
    }

    capturer.stop();
    std::printf("Stopped. Total captured=%llu\n",
                static_cast<unsigned long long>(capturer.stats().captured.load()));
    return 0;
}

#else  // non-Windows: DXGI is unavailable; build the inference/tracking stages.

int main() {
    std::fprintf(stderr,
                 "DXGI Desktop Duplication is Windows-only.\n"
                 "On this platform the capture stage is disabled; the TensorRT\n"
                 "detector and ByteTrack tracker still build and can be driven\n"
                 "from a file/camera source. See README.md.\n");
    return 0;
}

#endif
