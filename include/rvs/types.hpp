// SPDX-License-Identifier: MIT
//
// Core data types shared across the three layers of the visual-servo loop:
//   - Vision perception layer  (capture -> inference)
//   - Servo control layer      (inference -> control)
//   - Hardware gateway layer   (control -> serial -> Teensy/Arduino)
//
// Every payload carries capture-relative timestamps so the end-to-end latency
// budget (target: < 10 ms) can be measured at any stage without extra plumbing.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace rvs {

// A single steady-clock time point. We standardise on steady_clock because we
// only ever care about elapsed durations, never wall-clock time.
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline TimePoint now() noexcept { return Clock::now(); }

inline double micros_between(TimePoint a, TimePoint b) noexcept {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

// ---------------------------------------------------------------------------
// Vision perception layer payloads
// ---------------------------------------------------------------------------

// Fixed image geometry for the pre-allocated buffer pool. Sized for a typical
// industrial mono camera ROI; adjust to match the sensor. Keeping this a
// compile-time constant lets the memory pool allocate fixed-size slabs.
constexpr std::uint32_t kFrameWidth   = 640;
constexpr std::uint32_t kFrameHeight  = 480;
constexpr std::uint32_t kFrameChannels = 1;  // grayscale; use 3 for BGR
constexpr std::size_t   kFrameBytes =
    static_cast<std::size_t>(kFrameWidth) * kFrameHeight * kFrameChannels;

// A raw frame owned by the memory pool. The pixel buffer is part of the object
// so the whole thing comes from a single pre-allocated slab — no per-frame
// allocation ever happens on the hot path.
struct Frame {
    std::uint64_t              sequence = 0;     // monotonically increasing id
    TimePoint                  captured_at{};    // stamped by the capture thread
    std::uint32_t              width    = kFrameWidth;
    std::uint32_t              height   = kFrameHeight;
    std::uint32_t              channels = kFrameChannels;
    std::array<std::uint8_t, kFrameBytes> pixels{};

    void reset() noexcept {
        sequence = 0;
        captured_at = {};
        // pixel buffer is overwritten by the capture stage, no need to clear
    }
};

// ---------------------------------------------------------------------------
// Servo control layer payloads
// ---------------------------------------------------------------------------

// Result handed from the inference stage to the control stage. Carries the
// originating frame's timestamps so latency can be attributed end-to-end.
struct InferenceResult {
    std::uint64_t sequence = 0;
    TimePoint     captured_at{};   // copied from the source Frame
    TimePoint     inferred_at{};   // stamped when inference completed

    bool   target_valid = false;   // did the detector find the target?
    float  target_x     = 0.0f;    // normalised image coords [-1, 1]
    float  target_y     = 0.0f;
    float  confidence   = 0.0f;
};

// Number of independently driven servo axes (e.g. pan/tilt + focus).
constexpr std::size_t kNumAxes = 4;

// A control command destined for the microcontroller. This is the unit that
// flows through the serial TX queue and is also drawn from a memory pool.
struct ControlPacket {
    std::uint64_t sequence = 0;
    TimePoint     captured_at{};   // carried through from the frame
    TimePoint     issued_at{};     // stamped when control computed the command

    std::array<std::int16_t, kNumAxes> axis_setpoint{};  // servo targets
    std::uint8_t  flags = 0;       // bitfield: bit0 = enable, bit1 = estop, ...

    void reset() noexcept {
        sequence = 0;
        captured_at = {};
        issued_at = {};
        axis_setpoint.fill(0);
        flags = 0;
    }
};

// Raw inbound frame from the microcontroller (telemetry / acks). Fixed size so
// it can live in a pool and in the lock-free RX queue.
constexpr std::size_t kSerialFrameMax = 64;
struct SerialFrame {
    TimePoint                              received_at{};
    std::uint16_t                          len = 0;
    std::array<std::uint8_t, kSerialFrameMax> data{};
};

}  // namespace rvs
