# Low-Latency Robot Visual-Servo Closed-Loop System

An end-to-end framework that fuses the **vision perception**, **hardware
gateway**, and **servo control** layers into a single closed loop with a target
end-to-end latency budget of **< 10 ms**.

The design is built around three principles that keep jitter and tail latency
bounded:

1. **Lock-free, allocation-free hot path** — threads exchange data through
   wait-free SPSC queues and draw every object from pre-allocated pools. After
   startup there is **no `new`/`malloc`** on the control path.
2. **Core isolation** — the latency-critical capture and control threads are
   pinned to dedicated physical cores at real-time priority so the OS scheduler
   never migrates them or evicts their cache.
3. **Non-blocking I/O** — all serial traffic to the microcontroller is offloaded
   to a dedicated gateway thread; the control loop only touches lock-free queues.

## Architecture

```
 ┌────────────┐  Frame*   ┌────────────┐  Result  ┌────────────┐  Packet
 │  Capture   │──q_frames─▶│ Inference  │──q_infer─▶│  Control   │──serial──▶ MCU
 │ (core 6,RT)│           │ (core 5)   │          │ (core 7,RT)│   (async)
 └────────────┘           └────────────┘          └─────┬──────┘
       ▲ Frame pool            (vision)                  │ Packet pool
       └──── recycle ◀─────────────────────────────────-┘
```

| Stage        | Thread          | Core | Priority  | Rate     | Source/Sink                         |
|--------------|-----------------|------|-----------|----------|-------------------------------------|
| Acquisition  | `capture_thread`| 6    | Realtime  | ~500 fps | Camera DMA → `Frame` pool           |
| Inference    | `inference_thread`| 5  | High      | per-frame| `Frame` → `InferenceResult`         |
| Control      | `control_thread`| 7    | Realtime  | 1 kHz    | FSM → `ControlPacket` → gateway     |
| Serial I/O   | gateway thread  | 4    | Normal    | poll-driven | lock-free TX/RX ↔ Teensy/Arduino |

## Highlighted deliverables

| Component                         | File                                  |
|-----------------------------------|---------------------------------------|
| **Lock-free SPSC queue**          | `include/rvs/lockfree_spsc_queue.hpp` |
| **Memory / object pool template** | `include/rvs/memory_pool.hpp`         |
| **Thread-affinity utility**       | `include/rvs/thread_affinity.hpp`     |
| **Main-loop state machine**       | `include/rvs/state_machine.hpp`       |
| Async serial gateway              | `include/rvs/serial_gateway.hpp`      |
| Integration framework             | `src/main.cpp`                        |
| Shared data types                 | `include/rvs/types.hpp`               |

### Lock-free SPSC queue
Bounded power-of-two ring buffer. Wait-free in the common case (one
load-acquire + one store-release, no CAS). Producer/consumer indices sit on
separate cache lines (`alignas(64)`) and each side caches the other's index to
remove cross-core coherency traffic from the steady state — critical on AMD Zen
where L2 is per-core.

### Memory pool
Fixed-capacity pool with a lock-free Treiber-stack free list. The head is
version-tagged to defeat ABA, making `acquire`/`release` MPMC-safe (capture may
acquire while control releases). `acquire()` returns a move-only `PooledPtr`
RAII handle that recycles the slot on destruction. Exhaustion returns a null
handle — never throws, never allocates.

### Thread affinity
`ThreadAffinity::configure(core, priority)` pins the calling thread and raises
its scheduling class. Uses `SetThreadAffinityMask` on Windows and
`pthread_setaffinity_np` + `SCHED_FIFO` on Linux behind one API.

### Main-loop state machine
States: `Standby`, `VisualTracking`, `AutoTestSeq`, `Fault`. Transitions are
driven by external `Command`s (HMI/e-stop) and internal guards (target loss
timeout, end of test sequence). The FSM is pure logic — no I/O, no allocation —
so it is fully unit-testable. The visual-servo law is a per-axis PD controller
with slew limiting.

## Build & run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/rvs_tests          # unit tests (queue, pool, FSM)
./build/rvs_demo [device]  # integration demo; optional serial device path
ctest --test-dir build     # run tests via CTest
```

The demo runs the full pipeline for a few seconds against a synthetic detector
and reports measured end-to-end (capture → control-issue) latency.

> **Note on cores/priority:** pinning to cores 6/7 and `SCHED_FIFO` require the
> cores to exist and the process to have `CAP_SYS_NICE` (e.g. `isolcpus=6,7` and
> running as root or with the capability granted). Affinity/priority failures
> are non-fatal — the system still runs, just with more jitter.

## Tuning notes

- Boot the host with `isolcpus=6,7 nohz_full=6,7 rcu_nocbs=6,7` to evict the
  kernel scheduler and timer ticks from the control cores.
- Leave the SMT siblings of cores 6/7 idle (or disable SMT) so the realtime
  threads own their L1/L2.
- Adjust `kFrameWidth/Height`, pool depths, and queue depths in
  `types.hpp`/`main.cpp` to match the sensor and the in-flight depth you need.
- Replace the newline deframer in `serial_gateway.hpp` with COBS + CRC for a
  production wire protocol.
