// SPDX-License-Identifier: MIT
//
// Lightweight self-tests for the system-level building blocks. No framework:
// failures abort via assert and the binary returns non-zero on the first one.
#include "rvs/lockfree_spsc_queue.hpp"
#include "rvs/memory_pool.hpp"
#include "rvs/state_machine.hpp"

#include <cassert>
#include <cstdio>
#include <thread>
#include <vector>

using namespace rvs;

// ---- SPSC queue: single-threaded ordering + full/empty semantics -----------
static void test_spsc_basic() {
    SpscQueue<int, 4> q;  // usable capacity = 3
    assert(q.size_approx() == 0);
    assert(q.push(1));
    assert(q.push(2));
    assert(q.push(3));
    assert(!q.push(4));   // full
    int v = 0;
    assert(q.pop(v) && v == 1);
    assert(q.pop(v) && v == 2);
    assert(q.push(4));    // space again
    assert(q.pop(v) && v == 3);
    assert(q.pop(v) && v == 4);
    assert(!q.pop(v));    // empty
    std::puts("[ok] spsc basic");
}

// ---- SPSC queue: concurrent producer/consumer carries every item exactly once
static void test_spsc_threaded() {
    SpscQueue<std::uint64_t, 1024> q;
    constexpr std::uint64_t N = 2'000'000;
    std::uint64_t sum_consumed = 0;

    std::thread prod([&] {
        for (std::uint64_t i = 1; i <= N; ++i) {
            while (!q.push(i)) { /* spin until space */ }
        }
    });
    std::thread cons([&] {
        std::uint64_t got = 0, val = 0;
        while (got < N) {
            if (q.pop(val)) { sum_consumed += val; ++got; }
        }
    });
    prod.join();
    cons.join();

    const std::uint64_t expected = N * (N + 1) / 2;
    assert(sum_consumed == expected);
    std::puts("[ok] spsc threaded (2M items, checksum match)");
}

// ---- Memory pool: exhaustion, recycling, no double-allocation --------------
static void test_memory_pool() {
    MemoryPool<int, 3> pool;
    assert(pool.free_count() == 3);

    auto a = pool.acquire(10);
    auto b = pool.acquire(20);
    auto c = pool.acquire(30);
    assert(a && b && c);
    assert(*a == 10 && *b == 20 && *c == 30);
    assert(pool.in_use() == 3);

    auto d = pool.acquire(40);
    assert(!d);  // exhausted -> null handle, no throw, no malloc

    b.reset();   // return one slot
    assert(pool.free_count() == 1);
    auto e = pool.acquire(50);
    assert(e && *e == 50);
    assert(pool.free_count() == 0);
    std::puts("[ok] memory pool exhaustion + recycle");
}

// ---- Memory pool: distinct addresses while live (no aliasing) --------------
static void test_memory_pool_distinct() {
    MemoryPool<std::uint64_t, 8> pool;
    std::vector<MemoryPool<std::uint64_t, 8>::Handle> held;
    for (int i = 0; i < 8; ++i) held.push_back(pool.acquire(i));
    for (std::size_t i = 0; i < held.size(); ++i)
        for (std::size_t j = i + 1; j < held.size(); ++j)
            assert(held[i].get() != held[j].get());
    std::puts("[ok] memory pool distinct addresses");
}

// ---- Memory pool: hammer acquire/release from two threads ------------------
static void test_memory_pool_threaded() {
    MemoryPool<std::uint64_t, 64> pool;
    std::atomic<bool> go{false};
    auto worker = [&] {
        while (!go.load()) {}
        for (int i = 0; i < 200000; ++i) {
            auto h = pool.acquire(static_cast<std::uint64_t>(i));
            if (h) { volatile std::uint64_t x = *h; (void)x; }  // h frees here
        }
    };
    std::thread t1(worker), t2(worker);
    go.store(true);
    t1.join();
    t2.join();
    assert(pool.free_count() == 64);  // everything returned, no leaks
    std::puts("[ok] memory pool threaded (MPMC, all slots returned)");
}

// ---- FSM: command-driven transitions and tracking behaviour ----------------
static void test_state_machine() {
    ControlStateMachine fsm;
    ControlPacket out;
    InferenceResult r;
    r.target_valid = true; r.target_x = 0.5f; r.target_y = -0.5f;
    r.captured_at = now();

    assert(fsm.state() == State::Standby);

    fsm.post(Command::StartTracking);
    bool tx = fsm.tick(true, r, out);
    assert(fsm.state() == State::VisualTracking);
    assert(tx && (out.flags & 0x01));            // enabled, commanding
    assert(out.axis_setpoint[0] != 0);           // pan responds to error

    fsm.post(Command::EStop);
    tx = fsm.tick(true, r, out);
    assert(fsm.state() == State::Fault);
    assert(tx && (out.flags & 0x02));            // estop latched

    fsm.post(Command::StartTracking);            // ignored while faulted
    fsm.tick(true, r, out);
    assert(fsm.state() == State::Fault);

    fsm.post(Command::ResetFault);
    fsm.tick(false, r, out);
    assert(fsm.state() == State::Standby);
    std::puts("[ok] state machine transitions");
}

// ---- FSM: tracking times out to Standby when the target is lost ------------
static void test_tracking_timeout() {
    FsmConfig cfg; cfg.track_timeout_ticks = 5;
    ControlStateMachine fsm(cfg);
    ControlPacket out;
    InferenceResult r;

    fsm.post(Command::StartTracking);
    fsm.tick(false, r, out);
    assert(fsm.state() == State::VisualTracking);
    for (int i = 0; i < 10; ++i) fsm.tick(false, r, out);  // no target
    assert(fsm.state() == State::Standby);
    std::puts("[ok] tracking timeout fallback");
}

int main() {
    test_spsc_basic();
    test_spsc_threaded();
    test_memory_pool();
    test_memory_pool_distinct();
    test_memory_pool_threaded();
    test_state_machine();
    test_tracking_timeout();
    std::puts("ALL TESTS PASSED");
    return 0;
}
