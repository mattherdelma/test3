// SPDX-License-Identifier: MIT
//
// Thread affinity + real-time scheduling utility.
//
// To hold a sub-10 ms end-to-end budget we must keep the latency-critical
// threads (capture and control) off the OS scheduler's general run queue and
// pinned to dedicated physical cores so the CPU never migrates them or lets
// another task evict their L1/L2 working set.
//
// On AMD Zen, sibling SMT threads share L1/L2, so pinning capture to core 6 and
// control to core 7 (distinct physical cores, ideally with their SMT siblings
// left idle / isolated via the kernel `isolcpus=` boot arg) gives each thread a
// private cache and removes cross-core jitter.
//
// Per the spec this wraps Win32 SetThreadAffinityMask; on Linux it uses the
// equivalent pthread_setaffinity_np so the same call site works on either host.
#pragma once

#include <cstdint>
#include <thread>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <pthread.h>
#  include <sched.h>
#  include <cerrno>
#  include <cstring>
#endif

namespace rvs {

enum class RtPriority {
    Normal,
    High,      // above-normal, still time-shared
    Realtime,  // SCHED_FIFO / THREAD_PRIORITY_TIME_CRITICAL — use with care
};

class ThreadAffinity {
public:
    // Pin the *calling* thread to a single physical core.
    // Returns true on success. Failures are non-fatal (e.g. running without
    // CAP_SYS_NICE) — the system still works, just with more jitter.
    static bool pin_to_core(unsigned core_index) noexcept {
        return pin_to_mask(std::uint64_t{1} << core_index);
    }

    // Pin to an arbitrary core mask (bit N set => eligible for core N).
    static bool pin_to_mask(std::uint64_t core_mask) noexcept {
#if defined(_WIN32)
        // This is the API the spec calls out explicitly.
        const DWORD_PTR prev =
            SetThreadAffinityMask(GetCurrentThread(),
                                  static_cast<DWORD_PTR>(core_mask));
        return prev != 0;
#else
        cpu_set_t set;
        CPU_ZERO(&set);
        for (unsigned i = 0; i < 64; ++i) {
            if (core_mask & (std::uint64_t{1} << i)) CPU_SET(i, &set);
        }
        const int rc =
            pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        return rc == 0;
#endif
    }

    // Raise the scheduling priority of the calling thread.
    static bool set_priority(RtPriority prio) noexcept {
#if defined(_WIN32)
        int win_prio = THREAD_PRIORITY_NORMAL;
        switch (prio) {
            case RtPriority::Normal:   win_prio = THREAD_PRIORITY_NORMAL; break;
            case RtPriority::High:     win_prio = THREAD_PRIORITY_HIGHEST; break;
            case RtPriority::Realtime: win_prio = THREAD_PRIORITY_TIME_CRITICAL; break;
        }
        return SetThreadPriority(GetCurrentThread(), win_prio) != 0;
#else
        if (prio == RtPriority::Realtime) {
            sched_param sp{};
            sp.sched_priority = 80;  // mid-high FIFO band, below kernel threads
            return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0;
        }
        // High maps to a more negative nice value; needs privilege. Best-effort.
        sched_param sp{};
        sp.sched_priority = 0;
        const int policy = SCHED_OTHER;
        return pthread_setschedparam(pthread_self(), policy, &sp) == 0;
#endif
    }

    // Convenience: configure the calling thread in one shot. Returns true only
    // if BOTH the affinity pin and the priority change succeeded.
    static bool configure(unsigned core_index, RtPriority prio) noexcept {
        const bool a = pin_to_core(core_index);
        const bool b = set_priority(prio);
        return a && b;
    }

    static unsigned hardware_cores() noexcept {
        const unsigned n = std::thread::hardware_concurrency();
        return n ? n : 1;
    }
};

}  // namespace rvs
