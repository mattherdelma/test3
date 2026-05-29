#pragma once
//
// Bounded, wait-free single-producer / single-consumer ring buffer.
//
// One capture thread pushes, one inference thread pops. Because there is exactly
// one writer and one reader, correctness only needs two atomic indices with
// acquire/release ordering — no locks, no CAS loops on the hot path.
//
// The buffer holds `Capacity` slots but can store at most `Capacity - 1` items
// so that the empty and full states are distinguishable.
//
#include <atomic>
#include <array>
#include <cstddef>
#include <optional>
#include <utility>

namespace rtp {

template <typename T, std::size_t Capacity>
class SPSCQueue {
    static_assert(Capacity >= 2, "Capacity must be at least 2");

public:
    SPSCQueue() = default;
    SPSCQueue(const SPSCQueue&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;

    // Producer side. Returns false if the queue is full (item not stored).
    bool push(T value) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = increment(head);
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // full
        }
        slots_[head] = std::move(value);
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns std::nullopt if the queue is empty.
    std::optional<T> pop() {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return std::nullopt; // empty
        }
        T value = std::move(slots_[tail]);
        tail_.store(increment(tail), std::memory_order_release);
        return value;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    // Approximate count (may race; for diagnostics only).
    std::size_t size_approx() const {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        return (h + Capacity - t) % Capacity;
    }

private:
    static std::size_t increment(std::size_t i) { return (i + 1) % Capacity; }

    // Pad indices onto separate cache lines to avoid false sharing between
    // the producer's head and the consumer's tail.
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::array<T, Capacity> slots_{};
};

} // namespace rtp
