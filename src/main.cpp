// SPDX-License-Identifier: MIT
//
// Low-latency robot visual-servo closed-loop — integration framework.
//
//   ┌────────────┐  Frame*   ┌────────────┐  Result  ┌────────────┐  Packet*
//   │  Capture   │──q_frames─▶│ Inference  │──q_infer─▶│  Control   │──serial──▶ MCU
//   │ (core 6)   │           │ (core N)   │          │ (core 7)   │
//   └────────────┘           └────────────┘          └─────┬──────┘
//        ▲ Frame pool             (vision)                 │ Packet pool
//        └──── recycle ◀──────────────────────────────────┘
//
// Threads communicate ONLY through lock-free SPSC queues, and ALL hot-path
// objects come from pre-allocated memory pools — no new/malloc after startup.
//
// Capture and Control are pinned to dedicated physical cores (6 and 7) and run
// at real-time priority to protect the <10 ms end-to-end budget.

#include "rvs/lockfree_spsc_queue.hpp"
#include "rvs/memory_pool.hpp"
#include "rvs/serial_gateway.hpp"
#include "rvs/state_machine.hpp"
#include "rvs/thread_affinity.hpp"
#include "rvs/types.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

namespace rvs {

// ---- Core assignment (AMD: leave SMT siblings of 6/7 isolated via isolcpus) -
constexpr unsigned kCaptureCore   = 6;
constexpr unsigned kControlCore    = 7;
constexpr unsigned kInferenceCore  = 5;  // best-effort; inference is the slack
constexpr unsigned kGatewayCore    = 4;  // serial IO thread

// ---- Pool / queue depths ----------------------------------------------------
constexpr std::size_t kFramePoolSize  = 8;   // a few frames in flight at once
constexpr std::size_t kPacketPoolSize = 16;
constexpr std::size_t kFrameQDepth    = 8;    // power of two
constexpr std::size_t kInferQDepth    = 8;
constexpr std::size_t kPacketQDepth   = 16;

// Forward decl: wire encoder defined after the control thread for readability.
std::uint16_t encode(const ControlPacket& p,
                     std::array<std::uint8_t, kSerialFrameMax>& buf);

// Shared system context: pools + queues live here so all threads see them.
struct System {
    MemoryPool<Frame, kFramePoolSize>          frame_pool;
    MemoryPool<ControlPacket, kPacketPoolSize> packet_pool;

    using FrameHandle  = MemoryPool<Frame, kFramePoolSize>::Handle;
    using PacketHandle = MemoryPool<ControlPacket, kPacketPoolSize>::Handle;

    // Hand-off queues carry pool *handles* (move-only) so ownership flows with
    // the data and the slot is auto-returned when the consumer is done.
    SpscQueue<FrameHandle, kFrameQDepth>     q_frames;   // capture -> inference
    SpscQueue<InferenceResult, kInferQDepth> q_infer;    // inference -> control
    SpscQueue<PacketHandle, kPacketQDepth>   q_recycle;  // (unused demo hook)

    SerialGateway gateway;

    std::atomic<bool> running{true};

    // ---- latency telemetry (relaxed; for reporting only) ----
    std::atomic<std::uint64_t> ticks{0};
    std::atomic<std::uint64_t> e2e_micros_sum{0};
    std::atomic<std::uint64_t> e2e_micros_max{0};
};

// ============================ Capture thread ================================
// Stands in for a real camera grab (V4L2/SDK DMA into the pooled buffer).
void capture_thread(System& sys) {
    ThreadAffinity::configure(kCaptureCore, RtPriority::Realtime);

    std::uint64_t seq = 0;
    constexpr auto period = std::chrono::microseconds(2000);  // ~500 fps grab
    auto next = now();

    while (sys.running.load(std::memory_order_acquire)) {
        next += period;

        auto frame = sys.frame_pool.acquire();  // pooled, no malloc
        if (frame) {
            frame->sequence    = seq++;
            frame->captured_at = now();
            // (real driver would DMA pixels here; we leave the buffer as-is)

            // Hand ownership downstream; if inference is behind, drop the frame
            // (its handle is destroyed -> slot returns to the pool).
            if (!sys.q_frames.push(std::move(frame))) {
                // dropped: frame handle destructs here, recycling the slot
            }
        }

        std::this_thread::sleep_until(next);
    }
}

// =========================== Inference thread ===============================
// Stands in for the perception model. Produces a normalised target error.
void inference_thread(System& sys) {
    ThreadAffinity::configure(kInferenceCore, RtPriority::High);

    System::FrameHandle frame;
    double phase = 0.0;

    while (sys.running.load(std::memory_order_acquire)) {
        if (!sys.q_frames.pop(frame)) {
            std::this_thread::yield();  // nothing to do; don't spin a core hot
            continue;
        }

        // --- fake detector: target orbits the image centre ---
        phase += 0.01;
        InferenceResult r;
        r.sequence     = frame->sequence;
        r.captured_at  = frame->captured_at;
        r.target_valid = true;
        r.target_x     = 0.3f * static_cast<float>(std::sin(phase));
        r.target_y     = 0.3f * static_cast<float>(std::cos(phase));
        r.confidence   = 0.95f;
        r.inferred_at  = now();

        // frame handle goes out of scope at next loop iteration -> slot freed
        sys.q_infer.push(r);  // small POD, fine to drop if control is behind
    }
}

// ============================ Control thread ================================
// The heartbeat: runs the FSM and drives the serial gateway. Deterministic
// fixed-rate loop, pinned + real-time so jitter stays bounded.
void control_thread(System& sys, ControlStateMachine& fsm) {
    ThreadAffinity::configure(kControlCore, RtPriority::Realtime);

    constexpr auto period = std::chrono::microseconds(1000);  // 1 kHz control
    auto next = now();

    InferenceResult latest;
    bool have_target = false;

    while (sys.running.load(std::memory_order_acquire)) {
        next += period;

        // Drain to the freshest inference result (we only care about latest).
        InferenceResult r;
        while (sys.q_infer.pop(r)) { latest = r; have_target = true; }

        // Borrow a packet from the pool, run the FSM, transmit.
        auto pkt = sys.packet_pool.acquire();
        if (pkt && fsm.tick(have_target, latest, *pkt)) {
            // Serialize into a wire frame and hand to the async gateway.
            SerialFrame sf;
            sf.len = encode(*pkt, sf.data);
            sys.gateway.send(sf);  // wait-free enqueue; never blocks the loop

            // End-to-end latency: capture -> control issue (when we have a target).
            if (have_target && pkt->captured_at.time_since_epoch().count() != 0) {
                const auto us = static_cast<std::uint64_t>(
                    micros_between(pkt->captured_at, pkt->issued_at));
                sys.ticks.fetch_add(1, std::memory_order_relaxed);
                sys.e2e_micros_sum.fetch_add(us, std::memory_order_relaxed);
                std::uint64_t prev = sys.e2e_micros_max.load(std::memory_order_relaxed);
                while (us > prev && !sys.e2e_micros_max.compare_exchange_weak(
                                        prev, us, std::memory_order_relaxed)) {}
            }
        }
        // pkt handle returns to the pool here. have_target stays latched so the
        // controller keeps acting on the most recent perception between frames.

        std::this_thread::sleep_until(next);
    }
}

// Pack a ControlPacket into the on-wire byte frame (newline-delimited demo).
// Layout: [flags][seq:2][axes:2*kNumAxes] little-endian.
std::uint16_t encode(const ControlPacket& p, std::array<std::uint8_t, kSerialFrameMax>& buf) {
    std::uint16_t n = 0;
    buf[n++] = p.flags;
    buf[n++] = static_cast<std::uint8_t>(p.sequence & 0xFF);
    buf[n++] = static_cast<std::uint8_t>((p.sequence >> 8) & 0xFF);
    for (std::size_t i = 0; i < kNumAxes; ++i) {
        const std::int16_t v = p.axis_setpoint[i];
        buf[n++] = static_cast<std::uint8_t>(v & 0xFF);
        buf[n++] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    }
    buf[n++] = '\n';
    return n;
}

}  // namespace rvs

int main(int argc, char** argv) {
    using namespace rvs;

    auto sys = std::make_unique<System>();   // big pools: heap-alloc ONCE here
    ControlStateMachine fsm;

    // Optionally open the serial link to the MCU (best-effort for the demo).
    SerialConfig scfg;
    if (argc > 1) scfg.device = argv[1];
    const bool serial_up = sys->gateway.start(scfg, static_cast<int>(kGatewayCore));
    std::printf("[rvs] serial gateway: %s (%s)\n",
                serial_up ? "up" : "down (running headless)", scfg.device.c_str());

    // Launch the pipeline.
    std::thread t_cap (capture_thread,   std::ref(*sys));
    std::thread t_inf (inference_thread, std::ref(*sys));
    std::thread t_ctl (control_thread,   std::ref(*sys), std::ref(fsm));

    // Drive the demo state machine from the "operator".
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    fsm.post(Command::StartTracking);
    std::printf("[rvs] state -> %s\n", to_string(fsm.state()));

    // Run for a few seconds, then report latency and shut down cleanly.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    const std::uint64_t n = sys->ticks.load();
    const double avg = n ? static_cast<double>(sys->e2e_micros_sum.load()) / n : 0.0;
    std::printf("[rvs] FSM state=%s ticks=%llu  e2e avg=%.1f us  max=%llu us\n",
                to_string(fsm.state()),
                static_cast<unsigned long long>(n), avg,
                static_cast<unsigned long long>(sys->e2e_micros_max.load()));
    std::printf("[rvs] serial: sent=%llu recv=%llu tx_drop=%llu rx_drop=%llu\n",
                (unsigned long long)sys->gateway.bytes_sent(),
                (unsigned long long)sys->gateway.bytes_recv(),
                (unsigned long long)sys->gateway.tx_dropped(),
                (unsigned long long)sys->gateway.rx_dropped());

    sys->running.store(false, std::memory_order_release);
    t_cap.join();
    t_inf.join();
    t_ctl.join();
    sys->gateway.stop();

    std::puts("[rvs] shutdown complete");
    return 0;
}
