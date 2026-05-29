// SPDX-License-Identifier: MIT
//
// Main-loop finite state machine for the visual-servo controller.
//
// The control thread is the system's heartbeat. On every tick it:
//   1. drains the latest inference result (vision layer -> control layer),
//   2. runs the behaviour for the current state,
//   3. emits at most one ControlPacket to the serial gateway (hardware layer).
//
// States:
//   Standby        - holds the servos safe/disabled, waits for a command.
//   VisualTracking - closed-loop: drive servos to centre the detected target.
//   AutoTestSeq    - replays a scripted motion sequence (commissioning/QA).
//   Fault          - latched safe state after an error; requires explicit reset.
//
// Transitions are driven by external Commands (operator/HMI) and by internal
// guards (e.g. loss of target, end of test sequence, e-stop).
#pragma once

#include "types.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace rvs {

enum class State : std::uint8_t {
    Standby = 0,
    VisualTracking,
    AutoTestSeq,
    Fault,
};

inline const char* to_string(State s) noexcept {
    switch (s) {
        case State::Standby:        return "Standby";
        case State::VisualTracking: return "VisualTracking";
        case State::AutoTestSeq:    return "AutoTestSeq";
        case State::Fault:          return "Fault";
    }
    return "?";
}

// External commands posted into the FSM (e.g. from an HMI thread). Cheap enough
// to pass through a small lock-free queue or an atomic; kept as a plain enum.
enum class Command : std::uint8_t {
    None = 0,
    EnterStandby,
    StartTracking,
    StartAutoTest,
    EStop,        // immediate -> Fault, servos disabled
    ResetFault,   // Fault -> Standby once the cause is cleared
};

// A simple PD law for the visual servo. Gains are intentionally conservative;
// tune per axis. Operates on normalised image error [-1, 1] -> servo delta.
struct ServoLaw {
    float kp = 220.0f;   // proportional gain (counts per unit image error)
    float kd = 18.0f;    // derivative gain
    std::int16_t limit = 1500;  // per-tick slew limit (counts)

    float prev_err_x = 0.0f;
    float prev_err_y = 0.0f;

    void reset() noexcept { prev_err_x = prev_err_y = 0.0f; }

    // Compute pan/tilt deltas from the latest target error.
    void compute(float err_x, float err_y, std::int16_t& d_pan,
                 std::int16_t& d_tilt) noexcept {
        const float dx = err_x - prev_err_x;
        const float dy = err_y - prev_err_y;
        prev_err_x = err_x;
        prev_err_y = err_y;
        d_pan  = clamp(kp * err_x + kd * dx);
        d_tilt = clamp(kp * err_y + kd * dy);
    }

private:
    std::int16_t clamp(float v) const noexcept {
        if (v >  limit) v =  limit;
        if (v < -limit) v = -limit;
        return static_cast<std::int16_t>(v);
    }
};

// Scripted motion for the AutoTestSeq state: a ring of setpoints stepped on a
// fixed dwell. Encodes a classic box/diagonal sweep for mechanical validation.
class TestSequence {
public:
    struct Step { std::array<std::int16_t, kNumAxes> setpoint; std::uint32_t dwell_ticks; };

    void load_default() {
        steps_ = {{
            {{{    0,    0, 0, 0}}, 200},
            {{{ 6000,    0, 0, 0}}, 200},
            {{{ 6000, 6000, 0, 0}}, 200},
            {{{-6000, 6000, 0, 0}}, 200},
            {{{-6000,-6000, 0, 0}}, 200},
            {{{    0,    0, 0, 0}}, 200},
        }};
        count_ = 6;
        reset();
    }

    void reset() noexcept { idx_ = 0; ticks_ = 0; }
    bool done() const noexcept { return idx_ >= count_; }

    // Advance one tick; returns the current setpoint to command.
    const std::array<std::int16_t, kNumAxes>& tick() noexcept {
        if (idx_ >= count_) return steps_[count_ ? count_ - 1 : 0].setpoint;
        if (++ticks_ >= steps_[idx_].dwell_ticks) { ticks_ = 0; ++idx_; }
        const std::size_t cur = idx_ < count_ ? idx_ : count_ - 1;
        return steps_[cur].setpoint;
    }

private:
    std::array<Step, 16> steps_{};
    std::size_t          count_ = 0;
    std::size_t          idx_   = 0;
    std::uint32_t        ticks_ = 0;
};

// Configuration / tunables for the FSM behaviour.
struct FsmConfig {
    // If no valid target arrives within this many ticks, tracking gives up and
    // falls back to Standby (prevents runaway on detector dropout).
    std::uint32_t track_timeout_ticks = 100;
};

// The state machine itself. Pure logic: it consumes an InferenceResult and the
// pending Command, and produces a ControlPacket via the supplied sink. It does
// no I/O and no allocation, so it is trivially unit-testable.
class ControlStateMachine {
public:
    explicit ControlStateMachine(FsmConfig cfg = {}) : cfg_(cfg) {
        test_seq_.load_default();
    }

    State state() const noexcept { return state_.load(std::memory_order_relaxed); }

    // Post an external command (thread-safe single slot; latest wins).
    void post(Command c) noexcept { pending_.store(c, std::memory_order_release); }

    // One control tick.
    //  have_target / result : latest inference output (have_target=false if none)
    //  out                  : packet to populate and send (only if returns true)
    // Returns true if `out` should be transmitted this tick.
    bool tick(bool have_target, const InferenceResult& result,
              ControlPacket& out) noexcept {
        handle_command();
        const State s = state_.load(std::memory_order_relaxed);
        switch (s) {
            case State::Standby:        return run_standby(out);
            case State::VisualTracking: return run_tracking(have_target, result, out);
            case State::AutoTestSeq:    return run_autotest(out);
            case State::Fault:          return run_fault(out);
        }
        return false;
    }

private:
    void transition(State next) noexcept {
        if (state_.load(std::memory_order_relaxed) == next) return;
        // entry actions
        switch (next) {
            case State::VisualTracking: law_.reset(); since_target_ = 0; break;
            case State::AutoTestSeq:    test_seq_.reset();               break;
            default: break;
        }
        state_.store(next, std::memory_order_release);
    }

    void handle_command() noexcept {
        const Command c = pending_.exchange(Command::None, std::memory_order_acquire);
        switch (c) {
            case Command::None: break;
            case Command::EStop:        transition(State::Fault);          break;
            case Command::ResetFault:
                if (state_.load(std::memory_order_relaxed) == State::Fault)
                    transition(State::Standby);
                break;
            case Command::EnterStandby:  transition(State::Standby);        break;
            case Command::StartTracking:
                if (state_.load(std::memory_order_relaxed) != State::Fault)
                    transition(State::VisualTracking);
                break;
            case Command::StartAutoTest:
                if (state_.load(std::memory_order_relaxed) != State::Fault)
                    transition(State::AutoTestSeq);
                break;
        }
    }

    // ----------------------------- behaviours ------------------------------

    bool run_standby(ControlPacket& out) noexcept {
        // Hold position, servos disabled. Emit a heartbeat so the MCU knows we
        // are alive and stays in hold rather than failsafe.
        out.reset();
        out.flags = 0;  // enable bit clear
        out.issued_at = now();
        return true;
    }

    bool run_tracking(bool have_target, const InferenceResult& r,
                      ControlPacket& out) noexcept {
        if (!have_target || !r.target_valid) {
            if (++since_target_ > cfg_.track_timeout_ticks) {
                transition(State::Standby);  // detector lost -> safe
            }
            return false;  // no command this tick; servos hold last setpoint
        }
        since_target_ = 0;

        std::int16_t d_pan = 0, d_tilt = 0;
        law_.compute(r.target_x, r.target_y, d_pan, d_tilt);

        out.reset();
        out.flags = 0x01;  // enable
        out.axis_setpoint[0] = d_pan;
        out.axis_setpoint[1] = d_tilt;
        out.sequence    = r.sequence;
        out.captured_at = r.captured_at;  // carry latency timestamp through
        out.issued_at   = now();
        return true;
    }

    bool run_autotest(ControlPacket& out) noexcept {
        const auto& sp = test_seq_.tick();
        out.reset();
        out.flags = 0x01;  // enable
        out.axis_setpoint = sp;
        out.issued_at = now();
        if (test_seq_.done()) transition(State::Standby);
        return true;
    }

    bool run_fault(ControlPacket& out) noexcept {
        // Latched safe state: command zero, enable cleared, set e-stop flag so
        // the MCU also latches off. Keep heartbeating.
        out.reset();
        out.flags = 0x02;  // estop bit
        out.issued_at = now();
        return true;
    }

    FsmConfig            cfg_;
    std::atomic<State>   state_{State::Standby};
    std::atomic<Command> pending_{Command::None};

    ServoLaw      law_;
    TestSequence  test_seq_;
    std::uint32_t since_target_ = 0;
};

}  // namespace rvs
