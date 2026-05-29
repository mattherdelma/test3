// SPDX-License-Identifier: MIT
//
// Asynchronous, non-blocking serial gateway to the Teensy/Arduino motor MCU.
//
// The control loop must NEVER block on I/O. So all serial traffic is funnelled
// through two lock-free SPSC queues and serviced by a dedicated gateway thread:
//
//     control thread  --tx_queue--> [gateway thread] --write()--> MCU
//     control thread  <--rx_queue-- [gateway thread] <--read()---  MCU
//
// The gateway thread owns the file descriptor exclusively, uses poll() so it
// sleeps when idle (no busy-wait stealing a core), and uses a non-blocking fd
// so a slow/absent MCU can never stall it. From the control loop's point of
// view, send() and try_receive() are wait-free queue ops.
//
// A tiny COBS-style framing is left to the caller; this class moves byte frames.
#pragma once

#include "lockfree_spsc_queue.hpp"
#include "thread_affinity.hpp"
#include "types.hpp"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#if !defined(_WIN32)
#  include <fcntl.h>
#  include <poll.h>
#  include <termios.h>
#  include <unistd.h>
#endif

namespace rvs {

struct SerialConfig {
    std::string device   = "/dev/ttyACM0";  // Teensy/Arduino CDC device
    int         baudrate = 1000000;          // 1 Mbaud over USB CDC
};

class SerialGateway {
public:
    static constexpr std::size_t kTxDepth = 256;
    static constexpr std::size_t kRxDepth = 256;

    SerialGateway() = default;
    ~SerialGateway() { stop(); }

    SerialGateway(const SerialGateway&)            = delete;
    SerialGateway& operator=(const SerialGateway&) = delete;

    // Open the port and launch the gateway thread. core < 0 => no pinning.
    bool start(const SerialConfig& cfg, int pin_core = -1) {
        if (running_.load(std::memory_order_acquire)) return true;
        if (!open_port(cfg)) return false;
        pin_core_ = pin_core;
        running_.store(true, std::memory_order_release);
        worker_ = std::thread(&SerialGateway::run, this);
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) worker_.join();
        close_port();
    }

    // ---- Control-loop facing API (wait-free, never blocks) ----------------

    // Enqueue a frame for transmission. Returns false if TX queue is full
    // (caller may drop the command — stale setpoints are worse than none).
    bool send(const SerialFrame& frame) noexcept { return tx_.push(frame); }

    // Pop one inbound frame if available.
    bool try_receive(SerialFrame& out) noexcept { return rx_.pop(out); }

    bool connected() const noexcept { return fd_valid(); }

    // Telemetry counters (relaxed — for dashboards, not control decisions).
    std::uint64_t tx_dropped() const noexcept { return tx_dropped_.load(std::memory_order_relaxed); }
    std::uint64_t rx_dropped() const noexcept { return rx_dropped_.load(std::memory_order_relaxed); }
    std::uint64_t bytes_sent() const noexcept { return bytes_sent_.load(std::memory_order_relaxed); }
    std::uint64_t bytes_recv() const noexcept { return bytes_recv_.load(std::memory_order_relaxed); }

private:
    // ----------------------- platform fd helpers ---------------------------
#if defined(_WIN32)
    bool fd_valid() const noexcept { return false; }  // Windows path TODO (overlapped IO)
    bool open_port(const SerialConfig&) { return false; }
    void close_port() {}
    void run() {}
#else
    int  fd_ = -1;
    bool fd_valid() const noexcept { return fd_ >= 0; }

    static speed_t to_speed(int baud) {
        switch (baud) {
            case 115200:  return B115200;
            case 230400:  return B230400;
            case 460800:  return B460800;
            case 500000:  return B500000;
            case 921600:  return B921600;
            case 1000000: return B1000000;
            case 2000000: return B2000000;
            default:      return B1000000;
        }
    }

    bool open_port(const SerialConfig& cfg) {
        fd_ = ::open(cfg.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) { close_port(); return false; }
        cfmakeraw(&tio);                 // 8N1, no echo, no canonical processing
        const speed_t sp = to_speed(cfg.baudrate);
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cc[VMIN]  = 0;             // non-blocking semantics
        tio.c_cc[VTIME] = 0;
        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) { close_port(); return false; }
        ::tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close_port() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    void run() {
        if (pin_core_ >= 0) {
            ThreadAffinity::pin_to_core(static_cast<unsigned>(pin_core_));
        }
        SerialFrame out_frame;       // current outbound frame being drained
        std::size_t out_off = 0;     // how many bytes of it are written
        bool have_out = false;

        std::uint8_t in_buf[512];
        SerialFrame  in_frame;       // accumulates inbound bytes into a frame

        while (running_.load(std::memory_order_acquire)) {
            // Pull a new TX frame if we are idle and one is queued.
            if (!have_out) {
                have_out = tx_.pop(out_frame);
                out_off  = 0;
            }

            pollfd pfd{};
            pfd.fd     = fd_;
            pfd.events = POLLIN | (have_out ? POLLOUT : 0);
            // 1 ms timeout: bounds latency of reacting to new TX work while
            // still sleeping the thread when the link is quiet.
            const int pr = ::poll(&pfd, 1, 1);
            if (pr < 0) {
                if (errno == EINTR) continue;
                break;  // fatal fd error
            }

            if (pr > 0 && (pfd.revents & POLLOUT) && have_out) {
                const ssize_t n =
                    ::write(fd_, out_frame.data.data() + out_off,
                            out_frame.len - out_off);
                if (n > 0) {
                    out_off += static_cast<std::size_t>(n);
                    bytes_sent_.fetch_add(static_cast<std::uint64_t>(n),
                                          std::memory_order_relaxed);
                    if (out_off >= out_frame.len) have_out = false;  // done
                } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    have_out = false;  // drop on hard error
                }
            }

            if (pr > 0 && (pfd.revents & POLLIN)) {
                const ssize_t n = ::read(fd_, in_buf, sizeof(in_buf));
                if (n > 0) {
                    bytes_recv_.fetch_add(static_cast<std::uint64_t>(n),
                                          std::memory_order_relaxed);
                    deframe(in_buf, static_cast<std::size_t>(n), in_frame);
                }
            }
        }
    }

    // Minimal newline-delimited deframer. Replace with COBS/CRC for production.
    void deframe(const std::uint8_t* buf, std::size_t n, SerialFrame& acc) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::uint8_t b = buf[i];
            if (b == '\n' || acc.len >= kSerialFrameMax) {
                if (acc.len > 0) {
                    acc.received_at = now();
                    if (!rx_.push(acc)) {
                        rx_dropped_.fetch_add(1, std::memory_order_relaxed);
                    }
                    acc.len = 0;
                }
                if (b == '\n') continue;
            }
            acc.data[acc.len++] = b;
        }
    }
#endif

    // ----------------------------- state -----------------------------------
    SpscQueue<SerialFrame, kTxDepth> tx_;  // control -> gateway
    SpscQueue<SerialFrame, kRxDepth> rx_;  // gateway -> control

    std::thread       worker_;
    std::atomic<bool> running_{false};
    int               pin_core_ = -1;

    std::atomic<std::uint64_t> tx_dropped_{0};
    std::atomic<std::uint64_t> rx_dropped_{0};
    std::atomic<std::uint64_t> bytes_sent_{0};
    std::atomic<std::uint64_t> bytes_recv_{0};
};

}  // namespace rvs
