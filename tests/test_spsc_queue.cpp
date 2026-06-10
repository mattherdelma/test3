//
// Concurrency test for the wait-free SPSC ring buffer: one producer thread and
// one consumer thread, mirroring the capture -> inference hand-off. Verifies
// that every item arrives exactly once, in order, with no loss or duplication.
//
// Portable (std::thread); builds and runs under VS2019 just as it does on Linux.
//
#include "common/SPSCQueue.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>
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
} // namespace

int main() {
    using rtp::SPSCQueue;

    // ---- 1. Single-threaded FIFO + full/empty semantics ------------------
    std::printf("test: SPSC FIFO order and full/empty boundaries\n");
    {
        SPSCQueue<int, 4> q;                 // capacity 4 -> holds 3 items
        CHECK(q.empty());
        CHECK(q.push(1));
        CHECK(q.push(2));
        CHECK(q.push(3));
        CHECK(!q.push(4));                   // full (one slot reserved)
        CHECK(q.size_approx() == 3);
        CHECK(q.pop().value() == 1);
        CHECK(q.pop().value() == 2);
        CHECK(q.push(4));                    // space freed up
        CHECK(q.pop().value() == 3);
        CHECK(q.pop().value() == 4);
        CHECK(!q.pop().has_value());         // empty
        CHECK(q.empty());
    }

    // ---- 2. Producer/consumer stress: nothing lost or reordered ----------
    std::printf("test: SPSC producer/consumer stress (1M items)\n");
    {
        constexpr uint64_t N = 1'000'000;
        SPSCQueue<uint64_t, 1024> q;
        std::atomic<bool> producer_done{false};

        std::thread producer([&] {
            for (uint64_t i = 0; i < N; ) {
                if (q.push(i)) ++i;          // spin on backpressure
                else std::this_thread::yield();
            }
            producer_done.store(true, std::memory_order_release);
        });

        uint64_t expected = 0;
        bool ordered = true;
        while (expected < N) {
            auto v = q.pop();
            if (!v) { std::this_thread::yield(); continue; }
            if (*v != expected) ordered = false; // strict FIFO expected
            ++expected;
        }
        producer.join();

        CHECK(ordered);
        CHECK(expected == N);
        CHECK(producer_done.load(std::memory_order_acquire));
        CHECK(q.empty());
    }

    if (g_failures == 0) {
        std::printf("\nAll SPSC queue tests passed.\n");
        return 0;
    }
    std::printf("\n%d check(s) failed.\n", g_failures);
    return 1;
}
