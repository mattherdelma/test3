// SPDX-License-Identifier: MIT
//
// Fixed-capacity object / memory pool.
//
// Purpose: on the real-time path we must NEVER call new/malloc (page faults,
// lock contention inside the allocator, and heap fragmentation all blow the
// 10 ms budget). Every Frame, ControlPacket and SerialFrame is drawn from a
// pool that pre-allocates all of its storage up front.
//
// Concurrency: the free list is a lock-free Treiber stack of slot indices.
// It is MPMC-safe (the capture thread may acquire while the control thread
// releases the previous frame), and uses a version-tagged head to defeat the
// ABA problem. acquire()/release() are wait-free under no contention and
// lock-free under contention — no syscalls, no blocking.
//
// Ownership: acquire() hands back a PooledPtr<T>, a unique-ptr-like RAII handle
// that returns the slot to the pool on destruction. Move-only, zero overhead.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace rvs {

template <typename T, std::size_t Capacity>
class MemoryPool;

// RAII handle returned by MemoryPool::acquire(). Behaves like a unique_ptr but
// "deletes" by returning the slot to its originating pool.
template <typename T, std::size_t Capacity>
class PooledPtr {
public:
    PooledPtr() noexcept = default;
    PooledPtr(std::nullptr_t) noexcept {}

    PooledPtr(PooledPtr&& o) noexcept
        : pool_(o.pool_), ptr_(o.ptr_), index_(o.index_) {
        o.pool_ = nullptr; o.ptr_ = nullptr;
    }
    PooledPtr& operator=(PooledPtr&& o) noexcept {
        if (this != &o) {
            reset();
            pool_ = o.pool_; ptr_ = o.ptr_; index_ = o.index_;
            o.pool_ = nullptr; o.ptr_ = nullptr;
        }
        return *this;
    }

    PooledPtr(const PooledPtr&)            = delete;
    PooledPtr& operator=(const PooledPtr&) = delete;

    ~PooledPtr() { reset(); }

    T*       get()        const noexcept { return ptr_; }
    T*       operator->() const noexcept { return ptr_; }
    T&       operator*()  const noexcept { return *ptr_; }
    explicit operator bool() const noexcept { return ptr_ != nullptr; }

    void reset() noexcept {
        if (ptr_) {
            pool_->release_index(index_);
            pool_ = nullptr;
            ptr_  = nullptr;
        }
    }

private:
    friend class MemoryPool<T, Capacity>;
    PooledPtr(MemoryPool<T, Capacity>* pool, T* ptr, std::uint32_t index) noexcept
        : pool_(pool), ptr_(ptr), index_(index) {}

    MemoryPool<T, Capacity>* pool_ = nullptr;
    T*                       ptr_  = nullptr;
    std::uint32_t            index_ = 0;
};

template <typename T, std::size_t Capacity>
class MemoryPool {
    static_assert(Capacity > 0, "Capacity must be > 0");
    static_assert(Capacity < 0xFFFFFFFFu, "index is 32-bit");

public:
    using Handle = PooledPtr<T, Capacity>;
    static constexpr std::uint32_t kNil = 0xFFFFFFFFu;  // empty-list sentinel

    MemoryPool() noexcept {
        // Build the initial free list: 0 -> 1 -> 2 -> ... -> Capacity-1 -> nil.
        for (std::uint32_t i = 0; i < Capacity; ++i) {
            next_[i].store((i + 1 < Capacity) ? (i + 1) : kNil,
                           std::memory_order_relaxed);
        }
        head_.store(pack(0, 0), std::memory_order_relaxed);
        free_count_.store(Capacity, std::memory_order_relaxed);
    }

    ~MemoryPool() = default;
    MemoryPool(const MemoryPool&)            = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;

    // Acquire a slot and construct a T in place with the given args. Returns a
    // null handle if the pool is exhausted (caller decides policy: drop, retry).
    template <typename... Args>
    Handle acquire(Args&&... args) noexcept {
        const std::uint32_t idx = pop_index();
        if (idx == kNil) return Handle{};  // exhausted — no allocation, no throw
        T* obj = ::new (&storage_[idx]) T(std::forward<Args>(args)...);
        return Handle{this, obj, idx};
    }

    std::size_t capacity()   const noexcept { return Capacity; }
    std::size_t free_count() const noexcept {
        return free_count_.load(std::memory_order_relaxed);
    }
    std::size_t in_use()     const noexcept { return Capacity - free_count(); }

private:
    friend class PooledPtr<T, Capacity>;

    // Pack a (version, index) pair into a single 64-bit word for atomic CAS.
    // The version counter makes the head value unique on every push, so a
    // popped-then-repushed slot cannot masquerade as an unchanged head (ABA).
    static constexpr std::uint64_t pack(std::uint32_t version, std::uint32_t index) noexcept {
        return (static_cast<std::uint64_t>(version) << 32) | index;
    }
    static constexpr std::uint32_t idx_of(std::uint64_t v) noexcept {
        return static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
    }
    static constexpr std::uint32_t ver_of(std::uint64_t v) noexcept {
        return static_cast<std::uint32_t>(v >> 32);
    }

    std::uint32_t pop_index() noexcept {
        std::uint64_t old_head = head_.load(std::memory_order_acquire);
        for (;;) {
            const std::uint32_t idx = idx_of(old_head);
            if (idx == kNil) return kNil;  // empty
            const std::uint32_t nxt = next_[idx].load(std::memory_order_relaxed);
            const std::uint64_t new_head = pack(ver_of(old_head) + 1, nxt);
            if (head_.compare_exchange_weak(old_head, new_head,
                                            std::memory_order_acquire,
                                            std::memory_order_acquire)) {
                free_count_.fetch_sub(1, std::memory_order_relaxed);
                return idx;
            }
            // old_head reloaded by CAS; loop retries
        }
    }

    void push_index(std::uint32_t idx) noexcept {
        std::uint64_t old_head = head_.load(std::memory_order_relaxed);
        for (;;) {
            next_[idx].store(idx_of(old_head), std::memory_order_relaxed);
            const std::uint64_t new_head = pack(ver_of(old_head) + 1, idx);
            if (head_.compare_exchange_weak(old_head, new_head,
                                            std::memory_order_release,
                                            std::memory_order_relaxed)) {
                free_count_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
    }

    // Called by PooledPtr on destruction: run T's destructor, return the slot.
    void release_index(std::uint32_t idx) noexcept {
        std::launder(reinterpret_cast<T*>(&storage_[idx]))->~T();
        push_index(idx);
    }

    using Storage = std::aligned_storage_t<sizeof(T), alignof(T)>;

    // Pre-allocated, contiguous backing store — the only allocation we ever do
    // for these objects, and it happens once at construction.
    Storage                    storage_[Capacity];
    std::atomic<std::uint32_t> next_[Capacity];

    alignas(64) std::atomic<std::uint64_t> head_{0};
    std::atomic<std::size_t>               free_count_{0};
};

}  // namespace rvs
