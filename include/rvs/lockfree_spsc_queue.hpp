// SPDX-License-Identifier: MIT
//
// Lock-free single-producer / single-consumer (SPSC) bounded ring buffer.
//
// This is the work-horse for inter-stage hand-off in the pipeline:
//   capture  --[SpscQueue<Frame*>]-->  inference
//   inference --[SpscQueue<InferenceResult>]--> control
//   control  --[SpscQueue<ControlPacket*>]--> serial gateway
//
// Design notes:
//   * Wait-free push/pop in the common case: exactly one atomic load-acquire and
//     one atomic store-release per operation, no CAS, no locks.
//   * Producer and consumer indices live on separate cache lines (alignas 64)
//     so the two threads never thrash the same line (false sharing kills the
//     latency budget on AMD Zen where the L2 is per-core).
//   * Each side caches the *other* side's index and only re-reads the shared
//     atomic when its local cache says the queue looks full/empty. This removes
//     the cross-core coherency traffic from the steady state.
//   * Capacity must be a power of two so index wrapping is a single AND.
#pragma once

#include <atomic>
#include <cstddef>
#include <new>
#include <type_traits>
#include <utility>

namespace rvs {

#if defined(__cpp_lib_hardware_interference_size)
inline constexpr std::size_t kCacheLine = std::hardware_destructive_interference_size;
#else
inline constexpr std::size_t kCacheLine = 64;  // AMD Zen / x86 cache line
#endif

template <typename T, std::size_t Capacity>
class SpscQueue {
    static_assert(Capacity >= 2, "Capacity must be at least 2");
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of two");

public:
    SpscQueue() = default;
    ~SpscQueue() {
        // Drain any leftover constructed elements so their destructors run.
        T tmp;
        while (pop(tmp)) { /* discard */ }
    }

    SpscQueue(const SpscQueue&)            = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;

    // Producer side. Returns false if the queue is full (caller must back off
    // or drop — never blocks). Accepts both lvalue and rvalue via forwarding.
    template <typename U>
    bool push(U&& value) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t next = (tail + 1) & kMask;

        // Fast path: use the cached head; only refresh from the shared atomic
        // when the cache says we are full.
        if (next == cached_head_) {
            cached_head_ = head_.load(std::memory_order_acquire);
            if (next == cached_head_) {
                return false;  // genuinely full
            }
        }

        // Construct in place in the (currently empty) slot.
        // GCC 13 emits a false-positive -Wstringop-overflow here because the
        // storage array sits after the atomic indices; the placement-new is
        // in-bounds (index masked to [0, Capacity)). Suppress only this line.
#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
        ::new (&storage_[tail]) T(std::forward<U>(value));
#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop
#endif
        tail_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns false if the queue is empty. Moves the element out.
    bool pop(T& out) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);

        if (head == cached_tail_) {
            cached_tail_ = tail_.load(std::memory_order_acquire);
            if (head == cached_tail_) {
                return false;  // genuinely empty
            }
        }

        out = std::move(slot(head));
        slot(head).~T();  // slot becomes empty again, ready for reuse
        head_.store((head + 1) & kMask, std::memory_order_release);
        return true;
    }

    // Approximate size — safe to call from either side, but only a hint because
    // the other index may move concurrently. Useful for telemetry/back-pressure.
    std::size_t size_approx() const noexcept {
        const std::size_t t = tail_.load(std::memory_order_acquire);
        const std::size_t h = head_.load(std::memory_order_acquire);
        return (t - h) & kMask;
    }

    static constexpr std::size_t capacity() noexcept { return Capacity - 1; }

private:
    static constexpr std::size_t kMask = Capacity - 1;

    T&       slot(std::size_t i) noexcept { return *std::launder(reinterpret_cast<T*>(&storage_[i])); }

    using Storage = std::aligned_storage_t<sizeof(T), alignof(T)>;

    // Producer-owned line: the tail index plus the consumer index it caches.
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};
    std::size_t cached_head_ = 0;

    // Consumer-owned line: the head index plus the producer index it caches.
    alignas(kCacheLine) std::atomic<std::size_t> head_{0};
    std::size_t cached_tail_ = 0;

    // The ring storage on its own line(s).
    alignas(kCacheLine) Storage storage_[Capacity];
};

}  // namespace rvs
